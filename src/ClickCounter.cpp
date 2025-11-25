#include "ClickCounter.h"
#include "StatusLed.h"

#include <cstdio>
#include <cstring>
#include <climits>
#include <cmath>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_err.h>

bool ClickCounter::s_isrServiceInstalled = false;

#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
struct IntervalStatsRec {
  uint32_t magic;
  float meanMs;
  float varianceMs2;
  uint32_t sampleCount;
  uint32_t crc32;
};
#endif

void ClickCounter::begin(uint8_t pinClick, bool simulate) {
  _pin = pinClick;
  _simulate = simulate;
  _motion = MotionState::IDLE;
  _lastMotion = MotionState::IDLE;
  _pos = 0;
  _epoch = 0;
  _panic = false;
  _lastPersistPos = 0;
  _lastPersistMs = millis();
  _lastPersistLevelLow = false;
  _sensorExpectedLow = false;
  _sensorLiveLow = false;
  _sensorPersisted = false;
  _simSensorLow = false;
  _overshootLogged = false;
  _lastActiveDirection = MotionState::IDLE;
  _tailHoldUntil = 0;
  _edgePhase = 0;
  _edgeCountIsr = 0;
  _lastIsrUs = 0;

  if (!_statusLed) {
    pinMode(PIN_CLICK_DEBUG, OUTPUT);
    digitalWrite(PIN_CLICK_DEBUG, LOW);
  }

  _prefsOpen = _prefs.begin(NAMESPACE, false);
  if (_prefsOpen) {
    loadFromNvs();
    _lastPersistPos = _pos;
    _lastPersistLevelLow = _sensorExpectedLow;
  } else {
    _end = DEFAULT_END;
    _pos = 0;
    _sensorExpectedLow = false;
    _sensorPersisted = false;
    _lastPersistLevelLow = false;
  }

  _calibrationActive = false;
  _calibOpenSet = false;
  _calibClosedSet = false;
  _calibEntryPos = _pos;
  _calibEntryEnd = _end;
  _calibOpenRaw = 0;
  _calibClosedRaw = 0;

#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
  _lastAcceptedMs = 0;
  _intervalCount = 0;
  _intervalCount = 0;
  _intervalIndex = 0;
  _timeError = 0.0f;
  for (uint8_t i = 0; i < INTERVAL_WINDOW; ++i) {
    _intervalWindow[i] = 0;
  }
  applyIntervalDefaults();
  loadIntervalStats();
  seedIntervalWindow(intervalSeedValue());
#endif

  if (_simulate) {
    _lastSimTickMs = millis();
    _sensorLiveLow = _sensorExpectedLow;
    _simSensorLow = _sensorExpectedLow;
    mirrorSensorLevel();
  } else {
    attachHardwareIsr();
    refreshLiveLevel();
    if (!_sensorPersisted) {
      _sensorExpectedLow = _sensorLiveLow;
      _sensorPersisted = true;
      _lastPersistLevelLow = _sensorExpectedLow;
    }
    mirrorSensorLevel();
  }
}

void ClickCounter::setLogger(LogFn logger) {
  _log = logger;
}

void ClickCounter::setDebugLogger(LogFn logger) {
  _debugLog = logger;
}

void ClickCounter::setStatusLed(StatusLed* led) {
  _statusLed = led;
  if (_statusLed) {
    _statusLed->onDriveLevel(!_sensorLiveLow);
  }
}

void ClickCounter::setMotion(MotionState s) {
  if (_motion == MotionState::IDLE && s != MotionState::IDLE) {
    prepareForMotion();
    _lastActiveDirection = s;
    _tailHoldUntil = 0;
  } else if (_motion != MotionState::IDLE && s == MotionState::IDLE) {
    _tailHoldUntil = millis() + TAIL_HOLD_MS;
#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
    persistIntervalStats();
#endif
  }
  if (s != MotionState::IDLE) {
    _lastActiveDirection = s;
  }
  _motion = s;
}

void ClickCounter::setSimulation(bool simulate) {
  if (_simulate == simulate) return;

  if (!_simulate) {
    detachHardwareIsr();
  }

  _simulate = simulate;

  if (_simulate) {
    _lastSimTickMs = millis();
    _sensorLiveLow = _sensorExpectedLow;
    _simSensorLow = _sensorExpectedLow;
    _lastActiveDirection = MotionState::IDLE;
    _tailHoldUntil = 0;
    mirrorSensorLevel();
  } else {
    attachHardwareIsr();
    refreshLiveLevel();
    if (!_sensorPersisted) {
      _sensorExpectedLow = _sensorLiveLow;
      _sensorPersisted = true;
    }
    _lastActiveDirection = MotionState::IDLE;
    _tailHoldUntil = 0;
    mirrorSensorLevel();
  }

  _edgePhase = 0;
#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
  _lastAcceptedMs = 0;
  _timeError = 0.0f;
  seedIntervalWindow(intervalSeedValue());
#endif
}

void ClickCounter::update(bool allowBeyondLimits) {
  if (_simulate) {
    simulateTicks();
  } else {
    drainHardwareEdges();
  }

  bool moving = (_motion != MotionState::IDLE);
  bool wasMoving = (_lastMotion != MotionState::IDLE);

  if (!allowBeyondLimits) {
    bool overshoot = (_pos < 0) || (_pos > _end);
    if (overshoot && !_overshootLogged) {
      String msg;
      msg.reserve(80);
      msg += F("[CLICK] Motion overshoot detected (pos=");
      msg += _pos;
      msg += F(", end=");
      msg += _end;
      msg += F(")");
      logMessage(msg);
      _overshootLogged = true;
    } else if (!overshoot && _overshootLogged) {
      _overshootLogged = false;
    }

    if (_pos < -2 || _pos > _end + 2) {
      _panic = true;
    }
    if (_pos < -1) _pos = -1;
    if (_pos > _end + 1) _pos = _end + 1;
  } else {
    if (_pos < SET_MIN_POS) _pos = SET_MIN_POS;
    if (_pos > SET_MAX_POS) _pos = SET_MAX_POS;
    _overshootLogged = false;
  }

  if (!moving && wasMoving) {
    persistPos(true);
  }

  _lastMotion = _motion;
}

void ClickCounter::beginCalibration() {
  _calibrationActive = true;
  _calibEntryPos = _pos;
  _calibEntryEnd = _end;
  _calibOpenSet = false;
  _calibClosedSet = false;
  _calibOpenRaw = 0;
  _calibClosedRaw = 0;
}

void ClickCounter::setOpenHere() {
  int32_t beforeEnd = _end;
  int32_t raw = _pos;
  if (_calibrationActive) {
    _calibOpenSet = true;
    _calibOpenRaw = raw;
  }

  shiftCoordinateFrame(raw, _calibrationActive);

  if (_calibrationActive) {
    _calibOpenRaw = 0;
    recomputeSpanFromMarks();
  }

  clampCalibrationRange();
  _panic = false;

  if (_end != beforeEnd) persistEnd();
  persistPos(true);
}

void ClickCounter::setClosedHere() {
  if (_pos < 0) _pos = 0;

  if (_calibrationActive) {
    _calibClosedSet = true;
    _calibClosedRaw = _pos;
  }

  int32_t beforeEnd = _end;
  _end = _pos;

  if (_calibrationActive) {
    recomputeSpanFromMarks();
  }

  clampCalibrationRange();
  _panic = false;

  if (_end != beforeEnd) persistEnd();
  persistPos(true);
}

void ClickCounter::finalizeCalibration() {
  recomputeSpanFromMarks();
  clampCalibrationRange();

  if (_pos < 0) _pos = 0;
  if (_pos > _end) _pos = _end;

  _panic = false;
  _calibrationActive = false;
  _calibOpenSet = false;
  _calibClosedSet = false;

  persistEnd();
  persistPos(true);
}

void ClickCounter::clearPanic() {
  _panic = false;
}

bool ClickCounter::canOpen() const {
  return _pos > 0;
}

bool ClickCounter::canClose() const {
  return _pos < _end;
}

bool ClickCounter::panic() const {
  return _panic;
}

int32_t ClickCounter::position() const {
  return _pos;
}

int32_t ClickCounter::end() const {
  return _end;
}

void ClickCounter::forcePersist() {
  persistPos(true);
}

void ClickCounter::simulateTicks() {
  const unsigned long now = millis();
  const unsigned long period = 200;  // 5 Hz simulated click stream

  if (_motion == MotionState::IDLE) {
    _lastSimTickMs = now;
    _sensorLiveLow = _simSensorLow;
    mirrorSensorLevel();
    return;
  }

  if (now < _lastSimTickMs) {
    _lastSimTickMs = now;
  }

  uint32_t pendingEdges = 0;
  while ((unsigned long)(now - _lastSimTickMs) >= period) {
    _lastSimTickMs += period;
    pendingEdges += 2;  // one high + one low per click
  }

  if (pendingEdges) {
    processEdgeBatch(pendingEdges);
  }
}

void ClickCounter::drainHardwareEdges() {
  // 1. Check if the signal has settled.
  // If an interrupt happened recently, we might be in a transition or bounce.
  // We wait until the line is stable for at least 30ms before processing.
  // This acts as a "Software Debounce" / "State Verification" window.
  uint32_t nowUs = micros();
  noInterrupts();
  uint32_t lastUs = _lastIsrUs;
  uint32_t edges = _edgeCountIsr;
  interrupts();

  if (edges == 0) return;

  // If the last edge was less than 30ms ago, defer processing.
  // We want to verify the final state with digitalRead, so we need stability.
  if ((uint32_t)(nowUs - lastUs) < 30000) {
    return;
  }

  // 2. Verify State
  // We have 'edges' pending.
  // If edges is ODD, we expect the state to have TOGGLED.
  // If edges is EVEN, we expect the state to be the SAME.
  int currentLevel = digitalRead(_pin);
  bool currentLow = (currentLevel == LOW);
  
  bool expectedLow = _sensorLiveLow;
  if (edges % 2 != 0) {
    expectedLow = !expectedLow;
  }

  if (currentLow != expectedLow) {
    // DISCREPANCY DETECTED
    // The ISR count does not match the physical pin state.
    // This implies either Ghost Edges (noise) or Missed Edges.
    
    if (edges % 2 != 0) {
      // We expected a toggle, but state is same.
      // Most likely a single Ghost Edge (Noise Spike).
      // Action: Ignore the edge(s) to match the steady state.
      // If edges=1, we make it 0.
      // If edges=3, we make it 2 (or 0).
      // Safest: Reduce edges by 1 to cancel the toggle.
      if (edges > 0) edges--;
      
      if (_debugLog) {
        String dbg;
        dbg.reserve(40);
        dbg += F("[CLKDBG] GHOST EDGE detected. Edges=");
        dbg += (edges + 1); // Original
        dbg += F("->");
        dbg += edges;
        debugLog(dbg);
      }
    } else {
      // We expected same state, but state toggled.
      // We missed an edge!
      // Action: Add an edge to catch up.
      edges++;
      
      if (_debugLog) {
        String dbg;
        dbg.reserve(40);
        dbg += F("[CLKDBG] MISSED EDGE detected. Edges=");
        dbg += (edges - 1); // Original
        dbg += F("->");
        dbg += edges;
        debugLog(dbg);
      }
    }
  }

  // 3. Commit changes
  // We clear the ISR counter, but we might have modified 'edges' locally.
  // If we deferred, we wouldn't be here.
  noInterrupts();
  // Careful: New edges might have arrived while we were thinking?
  // But we checked 'lastUs' stability, so unlikely to have new valid edges.
  // However, to be safe, we subtract the ORIGINAL 'edges' count from the global counter
  // and then process the ADJUSTED 'edges' count.
  // Actually, simpler: just clear global, and if new ones came, they are new.
  // But if we "deferred" earlier, we didn't clear.
  // Here we ARE processing. So we clear what we read.
  // But wait, if we modify 'edges', we are diverging from ISR truth.
  // The ISR counter should be cleared of the *read* amount.
  // If we decide to *ignore* an edge, we just don't pass it to processEdgeBatch.
  if (_edgeCountIsr >= edges) {
      _edgeCountIsr = 0; // Clear all. 
      // (Assuming no new edges came in the last few microseconds. 
      //  If they did, we might lose them or count them next time?
      //  If we clear 0, we lose nothing.
      //  If we clear _edgeCountIsr, we clear everything.)
      // Correct logic:
      // We read 'edges' (local var) from '_edgeCountIsr' at start.
      // We assume no new edges since then (because of stability check).
      // So clearing _edgeCountIsr is safe.
  } else {
      // This case (ISR < edges) is impossible unless we messed up.
      _edgeCountIsr = 0;
  }
  interrupts();

  processEdgeBatch(edges);
}

void ClickCounter::processEdgeBatch(uint32_t edges) {
  if (!edges) return;

  bool anyTailHoldUsed = false;

  for (uint32_t i = 0; i < edges; ++i) {
    _edgePhase ^= 1;
    _sensorLiveLow = !_sensorLiveLow;
    mirrorSensorLevel();

    if (_edgePhase == 0) {
      bool usedTailHold = false;
      MotionState dir = computeEffectiveDirection(&usedTailHold);

#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
      bool intervalRejected = false;
      if (dir == MotionState::CLOSING || dir == MotionState::OPENING) {
        unsigned long acceptMs = millis();
        if (_lastAcceptedMs != 0) {
          uint32_t deltaMs = static_cast<uint32_t>(acceptMs - _lastAcceptedMs);
          bool haveMedian = (_intervalCount >= INTERVAL_MIN_SAMPLES);
          uint16_t median = haveMedian ? intervalMedian() : 0;
          uint32_t threshold = haveMedian
            ? (static_cast<uint32_t>(median) * INTERVAL_RATIO_NUM) / INTERVAL_RATIO_DEN
            : 0U;
          if (haveMedian && threshold == 0U) {
            threshold = 1U;
          }
          
          if (haveMedian && deltaMs < threshold) {
            intervalRejected = true;
            if (_debugLog) {
              String dbg;
              dbg.reserve(40);
              dbg += F("[CLKDBG] XF d");
              dbg += deltaMs;
              dbg += F(" m");
              dbg += median;
              debugLog(dbg);
            }
          } else {
            // Cumulative Time Error Heuristic
            if (haveMedian) {
              int32_t diff = (int32_t)deltaMs - (int32_t)median;
              _timeError = (_timeError * TIME_ERROR_DECAY) + (float)diff;

              float limit = (float)median * 5.0f;
              if (_timeError > limit) _timeError = limit;
              if (_timeError < -limit) _timeError = -limit;

              float thresholdErr = -1.0f * (float)median * TIME_ERROR_THRESHOLD_RATIO;
              
              if (_timeError < thresholdErr) {
                intervalRejected = true;
                if (_debugLog) {
                  String dbg;
                  dbg.reserve(64);
                  dbg += F("[CLKDBG] ERR t");
                  dbg += (int)_timeError;
                  dbg += F(" m");
                  dbg += median;
                  dbg += F(" d");
                  dbg += deltaMs;
                  debugLog(dbg);
                }
              }
            }

            if (!intervalRejected) {
              recordIntervalSample(static_cast<uint16_t>(min<uint32_t>(deltaMs, 0xFFFF)));
            }
          }
        }
        if (intervalRejected) {
          continue;
        }
        _lastAcceptedMs = acceptMs;
      } else {
        _lastAcceptedMs = 0;
      }
#endif

      if (dir == MotionState::CLOSING) {
        _pos += 1;
        persistPos(false);
      } else if (dir == MotionState::OPENING) {
        _pos -= 1;
        persistPos(false);
      }

      if (_debugLog) {
        String dbg;
        dbg.reserve(28);
        dbg += F("[CLKDBG] R");
        char dirCode = 'I';
        if (dir == MotionState::CLOSING) dirCode = 'C';
        else if (dir == MotionState::OPENING) dirCode = 'O';
        dbg += dirCode;
        if (usedTailHold) {
          dbg += F("t");
        }
        dbg += F(" p");
        dbg += _pos;
        debugLog(dbg);
      }

      if (usedTailHold) anyTailHoldUsed = true;
    }
  }

  _sensorExpectedLow = _sensorLiveLow;
  _sensorPersisted = true;

  if (_simulate) {
    _simSensorLow = _sensorLiveLow;
  }

  if (_motion == MotionState::IDLE && anyTailHoldUsed) {
    _tailHoldUntil = millis() + TAIL_HOLD_MS;
  }
}

MotionState ClickCounter::computeEffectiveDirection(bool* tailHoldUsed) {
  if (tailHoldUsed) *tailHoldUsed = false;

  MotionState active = _motion;
  if (active != MotionState::IDLE) {
    _lastActiveDirection = active;
    return active;
  }

  if (_lastActiveDirection != MotionState::IDLE) {
    unsigned long nowMs = millis();
    long diff = static_cast<long>(_tailHoldUntil - nowMs);
    if (diff >= 0) {
      if (tailHoldUsed) *tailHoldUsed = true;
      return _lastActiveDirection;
    }
    _lastActiveDirection = MotionState::IDLE;
    _tailHoldUntil = 0;
  }

  return MotionState::IDLE;
}

void ClickCounter::attachHardwareIsr() {
  if (_isrAttached) return;

  gpio_config_t cfg = {};
  cfg.intr_type = GPIO_INTR_ANYEDGE;
  cfg.mode = GPIO_MODE_INPUT;
  cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.pin_bit_mask = (1ULL << _pin);
  gpio_config(&cfg);

  if (!s_isrServiceInstalled) {
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
      s_isrServiceInstalled = true;
    } else {
      if (Serial) {
        Serial.printf("[GPIO] Failed to install ISR service (err=%d)\n", static_cast<int>(err));
      }
    }
  }

  if (s_isrServiceInstalled) {
    esp_err_t add = gpio_isr_handler_add(static_cast<gpio_num_t>(_pin), ClickCounter::gpioIsrThunk, this);
    if (add == ESP_OK) {
      gpio_intr_enable(static_cast<gpio_num_t>(_pin));
      _isrAttached = true;
      _edgeCountIsr = 0;
      _lastIsrUs = 0;
    } else {
      if (Serial) {
        Serial.printf("[GPIO] Failed to add ISR handler (err=%d)\n", static_cast<int>(add));
      }
    }
  }
}

void ClickCounter::detachHardwareIsr() {
  if (!_isrAttached) return;
  gpio_intr_disable(static_cast<gpio_num_t>(_pin));
  gpio_isr_handler_remove(static_cast<gpio_num_t>(_pin));
  _isrAttached = false;
  _edgeCountIsr = 0;
  _lastIsrUs = 0;
}

void ClickCounter::persistEnd() {
  if (!_prefsOpen) return;
  if (_end < 0) _end = 0;
  _prefs.putInt(KEY_END, _end);
}

void ClickCounter::persistPos(bool force) {
  if (!_prefsOpen) return;
  if (!force && _pos == _lastPersistPos &&
      _sensorExpectedLow == _lastPersistLevelLow) {
    return;
  }

  PosRecV1 rec;
  rec.epoch = ++_epoch;
  rec.pos = _pos;
  rec.level = _sensorExpectedLow ? 1 : 0;
  memset(rec.reserved, 0, sizeof(rec.reserved));
  rec.crc32 = computeRecCrc(rec.epoch, rec.pos, rec.level);

  char key[12];
  snprintf(key, sizeof(key), "pos_%u", rec.epoch % POS_SLOTS);
  unsigned long started = millis();
  size_t stored = _prefs.putBytes(key, &rec, sizeof(rec));
  unsigned long duration = millis() - started;
  _lastPersistPos = _pos;
  _lastPersistLevelLow = _sensorExpectedLow;
  _lastPersistMs = millis();
  _sensorPersisted = true;
  if (stored != sizeof(rec)) {
    Serial.printf("[NVS] putBytes failed for %s (stored=%u, expected=%u, pos=%ld, epoch=%lu)\n",
                  key, static_cast<unsigned>(stored), static_cast<unsigned>(sizeof(rec)),
                  static_cast<long>(_pos), static_cast<unsigned long>(rec.epoch));
  } else if (duration > 25) {
    Serial.printf("[NVS] putBytes %s took %lums (pos=%ld)\n",
                  key, duration, static_cast<long>(_pos));
  }

  if (stored == sizeof(rec) && _debugLog) {
    String dbg;
    dbg.reserve(32);
    dbg += F("[CLKDBG] W#");
    dbg += (rec.epoch % POS_SLOTS);
    if (force) {
      dbg += F("F");
    }
    dbg += F(" p");
    dbg += _pos;
    debugLog(dbg);
  }
}

void ClickCounter::shiftCoordinateFrame(int32_t delta, bool adjustEnd) {
  if (delta == 0) return;

  _pos -= delta;
  if (adjustEnd) _end -= delta;
  _lastPersistPos = _pos;

  if (_calibrationActive) {
    _calibEntryPos -= delta;
    if (_calibOpenSet) _calibOpenRaw -= delta;
    if (_calibClosedSet) _calibClosedRaw -= delta;
  }
}

void ClickCounter::recomputeSpanFromMarks() {
  if (!_calibrationActive) return;

  int32_t candidate = INT32_MIN;

  if (_calibClosedSet) {
    candidate = _calibClosedRaw;
  } else if (_calibOpenSet && _calibEntryPos != 0) {
    candidate = _calibEntryPos;
  }

  if (candidate != INT32_MIN) {
    if (candidate < 0) candidate = -candidate;
    if (candidate == 0 && _calibEntryEnd > 0) candidate = _calibEntryEnd;
    if (candidate < 0) candidate = 0;
    if (candidate > SET_MAX_POS) candidate = SET_MAX_POS;
    _end = candidate;
  } else if (_calibEntryEnd > 0) {
    _end = _calibEntryEnd;
  }

  if (_calibOpenSet) {
    if (_pos < 0) _pos = 0;
    if (_pos > _end) _pos = _end;
  }

  if (_calibClosedSet && _pos > _end) {
    _pos = _end;
  }
}

void ClickCounter::clampCalibrationRange() {
  if (_pos < SET_MIN_POS) _pos = SET_MIN_POS;
  if (_pos > SET_MAX_POS) _pos = SET_MAX_POS;
  if (_end < 0) _end = 0;
  if (_end > SET_MAX_POS) _end = SET_MAX_POS;
}

void ClickCounter::prepareForMotion() {
  if (_simulate) {
    _sensorLiveLow = _sensorExpectedLow;
    _simSensorLow = _sensorExpectedLow;
    return;
  }

  bool hadPersisted = _sensorPersisted;
  bool previousExpected = _sensorExpectedLow;

  refreshLiveLevel();

  bool mismatch = hadPersisted && (_sensorLiveLow != previousExpected);
  if (mismatch) {
    String msg;
    msg.reserve(96);
    msg += F("[CLICK] Sensor baseline mismatch (stored=");
    msg += previousExpected ? F("LOW") : F("HIGH");
    msg += F(", actual=");
    msg += _sensorLiveLow ? F("LOW") : F("HIGH");
    msg += F("). Ignoring pending edges.");
    logMessage(msg);
  }

  clearPendingEdges();

  _sensorExpectedLow = _sensorLiveLow;
  _sensorPersisted = true;

  if (!hadPersisted || mismatch ||
      _sensorExpectedLow != _lastPersistLevelLow) {
    persistPos(true);
  }
}

void ClickCounter::refreshLiveLevel() {
  if (_simulate) {
    _sensorLiveLow = _simSensorLow;
    mirrorSensorLevel();
    return;
  }
  int level = digitalRead(_pin);
  _sensorLiveLow = (level == LOW);
  mirrorSensorLevel();
}

void ClickCounter::clearPendingEdges() {
  _edgePhase = 0;
  _sensorExpectedLow = _sensorLiveLow;
  if (_simulate) {
    return;
  }
  noInterrupts();
  _edgeCountIsr = 0;
  _lastIsrUs = 0;
  interrupts();
#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
  _lastAcceptedMs = 0;
  _timeError = 0.0f;
  seedIntervalWindow(intervalSeedValue());
#endif
}

#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
void ClickCounter::recordIntervalSample(uint16_t deltaMs) {
  _intervalWindow[_intervalIndex] = deltaMs;
  if (_intervalCount < INTERVAL_WINDOW) {
    ++_intervalCount;
  }
  _intervalIndex = (_intervalIndex + 1) % INTERVAL_WINDOW;
  updateIntervalStats(deltaMs);
}

uint16_t ClickCounter::intervalMedian() const {
  if (_intervalCount == 0) return 0;
  uint8_t n = _intervalCount;
  if (n > INTERVAL_WINDOW) n = INTERVAL_WINDOW;
  uint16_t temp[INTERVAL_WINDOW];
  for (uint8_t i = 0; i < n; ++i) {
    temp[i] = _intervalWindow[i];
  }
  for (uint8_t i = 0; i < n; ++i) {
    for (uint8_t j = i + 1; j < n; ++j) {
      if (temp[j] < temp[i]) {
        uint16_t swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  return temp[n / 2];
}

void ClickCounter::seedIntervalWindow(uint16_t seedMs) {
  _intervalIndex = 0;
  if (seedMs == 0) {
    memset(_intervalWindow, 0, sizeof(_intervalWindow));
    _intervalCount = 0;
    return;
  }
  for (uint8_t i = 0; i < INTERVAL_WINDOW; ++i) {
    _intervalWindow[i] = seedMs;
  }
  _intervalCount = INTERVAL_WINDOW;
}

uint16_t ClickCounter::intervalSeedValue() const {
  float mean = _intervalStats.meanMs;
  if (!std::isfinite(mean) || mean <= 0.0f) {
    mean = CLICK_COUNTER_INTERVAL_DEFAULT_MS;
  }
  float clamped = mean;
  if (clamped < static_cast<float>(INTERVAL_MIN_SEED_MS)) clamped = INTERVAL_MIN_SEED_MS;
  if (clamped > static_cast<float>(INTERVAL_MAX_SEED_MS)) clamped = INTERVAL_MAX_SEED_MS;
  return static_cast<uint16_t>(clamped + 0.5f);
}

void ClickCounter::applyIntervalDefaults() {
  _intervalStats.meanMs = CLICK_COUNTER_INTERVAL_DEFAULT_MS;
  float stdMs = CLICK_COUNTER_INTERVAL_DEFAULT_STD_MS;
  _intervalStats.varianceMs2 = stdMs * stdMs;
  _intervalStats.sampleCount = 0;
  _intervalStatsDirty = false;
}

void ClickCounter::loadIntervalStats() {
#if CLICK_COUNTER_INTERVAL_PERSIST_ENABLED
  if (!_prefsOpen || !_prefs.isKey(KEY_INTERVAL_STATS)) {
    applyIntervalDefaults();
    return;
  }
  IntervalStatsRec rec;
  size_t n = _prefs.getBytes(KEY_INTERVAL_STATS, &rec, sizeof(rec));
  if (n != sizeof(rec) || rec.magic != INTERVAL_STATS_MAGIC) {
    applyIntervalDefaults();
    return;
  }
  uint32_t crc = crc32(&rec, sizeof(rec) - sizeof(rec.crc32));
  if (crc != rec.crc32) {
    applyIntervalDefaults();
    return;
  }
  _intervalStats.meanMs = rec.meanMs;
  _intervalStats.varianceMs2 = rec.varianceMs2;
  _intervalStats.sampleCount = rec.sampleCount;
  if (!std::isfinite(_intervalStats.meanMs) || _intervalStats.meanMs <= 0.0f) {
    applyIntervalDefaults();
  } else {
    _intervalStatsDirty = false;
  }
#else
  applyIntervalDefaults();
#endif
}

void ClickCounter::persistIntervalStats() {
#if CLICK_COUNTER_INTERVAL_PERSIST_ENABLED
  if (!_prefsOpen || !_intervalStatsDirty || _intervalStats.sampleCount < INTERVAL_MIN_SAMPLES) {
    return;
  }
  IntervalStatsRec rec;
  rec.magic = INTERVAL_STATS_MAGIC;
  rec.meanMs = _intervalStats.meanMs;
  rec.varianceMs2 = _intervalStats.varianceMs2;
  rec.sampleCount = _intervalStats.sampleCount;
  rec.crc32 = 0;
  rec.crc32 = crc32(&rec, sizeof(rec) - sizeof(rec.crc32));
  _prefs.putBytes(KEY_INTERVAL_STATS, &rec, sizeof(rec));
  _intervalStatsDirty = false;
#endif
}

void ClickCounter::updateIntervalStats(uint16_t deltaMs) {
  float sample = static_cast<float>(deltaMs);
  if (!std::isfinite(sample) || sample <= 0.0f) {
    return;
  }
  if (_intervalStats.sampleCount == 0) {
    _intervalStats.meanMs = sample;
    _intervalStats.varianceMs2 = 0.0f;
    _intervalStats.sampleCount = 1;
    _intervalStatsDirty = true;
    return;
  }
  float prevMean = _intervalStats.meanMs;
  float alpha = INTERVAL_EWMA_ALPHA;
  if (alpha <= 0.0f || alpha >= 1.0f) {
    alpha = 0.12f;
  }
  float newMean = prevMean + alpha * (sample - prevMean);
  float diff = sample - prevMean;
  float newVar = (1.0f - alpha) * (_intervalStats.varianceMs2 + alpha * diff * diff);
  if (newVar < 0.0f) newVar = 0.0f;
  _intervalStats.meanMs = newMean;
  _intervalStats.varianceMs2 = newVar;
  if (_intervalStats.sampleCount < UINT32_MAX) {
    ++_intervalStats.sampleCount;
  }
  _intervalStatsDirty = true;
}
#endif

void ClickCounter::logMessage(const String& message) {
  if (!message.length()) return;
  if (_log) {
    _log(message);
  } else {
    Serial.println(message);
  }
}

void ClickCounter::debugLog(const String& message) {
  if (!message.length() || !_debugLog) return;
  _debugLog(message);
}

void ClickCounter::mirrorSensorLevel() {
  if (_statusLed) {
    _statusLed->onDriveLevel(!_sensorLiveLow);
  } else {
    digitalWrite(PIN_CLICK_DEBUG, _sensorLiveLow ? LOW : HIGH);
  }
}

void ClickCounter::loadFromNvs() {
  loadEnd();
  loadPos();
}

void ClickCounter::loadEnd() {
  if (_prefs.isKey(KEY_END)) {
    _end = _prefs.getInt(KEY_END, DEFAULT_END);
  } else {
    _end = DEFAULT_END;
    _prefs.putInt(KEY_END, _end);
  }
  if (_end < 0) _end = DEFAULT_END;
}

void ClickCounter::loadPos() {
  bool found = false;
  uint32_t bestEpoch = 0;
  int32_t bestPos = 0;
  bool bestLevelLow = false;
  bool bestHasLevel = false;
  uint8_t buffer[POS_REC_V1_SIZE];

  for (uint8_t i = 0; i < POS_SLOTS; ++i) {
    char key[12];
    snprintf(key, sizeof(key), "pos_%u", i);
    if (!_prefs.isKey(key)) continue;  // avoid noisy NOT_FOUND logs

    size_t n = _prefs.getBytes(key, buffer, sizeof(buffer));
    if (!n) continue;

    if (n == POS_REC_V1_SIZE) {
      PosRecV1 rec;
      memcpy(&rec, buffer, sizeof(rec));
      uint32_t crc = computeRecCrc(rec.epoch, rec.pos, rec.level);
      if (crc != rec.crc32) continue;
      if (!found || rec.epoch > bestEpoch) {
        bestEpoch = rec.epoch;
        bestPos = rec.pos;
        bestLevelLow = (rec.level != 0);
        bestHasLevel = true;
        found = true;
      }
    } else if (n == POS_REC_V0_SIZE) {
      PosRecV0 rec0;
      memcpy(&rec0, buffer, sizeof(rec0));
      uint32_t crc = computeRecCrcLegacy(rec0.epoch, rec0.pos);
      if (crc != rec0.crc32) continue;
      if (!found || rec0.epoch > bestEpoch) {
        bestEpoch = rec0.epoch;
        bestPos = rec0.pos;
        bestLevelLow = false;
        bestHasLevel = false;
        found = true;
      }
    }
  }

  if (found) {
    _epoch = bestEpoch;
    _pos = bestPos;
    _sensorExpectedLow = bestLevelLow;
    _sensorPersisted = bestHasLevel;
  } else {
    _epoch = 0;
    _pos = 0;
    _sensorExpectedLow = false;
    _sensorPersisted = false;
  }

  if (_pos < SET_MIN_POS) _pos = SET_MIN_POS;
  if (_pos > SET_MAX_POS) _pos = SET_MAX_POS;

  _sensorLiveLow = _sensorExpectedLow;
  _lastPersistLevelLow = _sensorExpectedLow;
}

uint32_t ClickCounter::crc32(const void* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  const uint8_t* p = static_cast<const uint8_t*>(data);
  while (len--) {
    crc ^= *p++;
    for (int i = 0; i < 8; ++i) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

uint32_t ClickCounter::computeRecCrc(uint32_t epoch, int32_t pos, uint8_t level) {
  uint8_t buf[sizeof(epoch) + sizeof(pos) + sizeof(level)];
  size_t off = 0;
  memcpy(buf + off, &epoch, sizeof(epoch));
  off += sizeof(epoch);
  memcpy(buf + off, &pos, sizeof(pos));
  off += sizeof(pos);
  buf[off] = level;
  return crc32(buf, sizeof(buf));
}

uint32_t ClickCounter::computeRecCrcLegacy(uint32_t epoch, int32_t pos) {
  uint8_t buf[sizeof(epoch) + sizeof(pos)];
  memcpy(buf, &epoch, sizeof(epoch));
  memcpy(buf + sizeof(epoch), &pos, sizeof(pos));
  return crc32(buf, sizeof(buf));
}

void IRAM_ATTR ClickCounter::gpioIsrThunk(void* arg) {
  if (!arg) return;
  static_cast<ClickCounter*>(arg)->onIsr();
}

void IRAM_ATTR ClickCounter::onIsr() {
  uint32_t now = micros();
  if ((uint32_t)(now - _lastIsrUs) < ISR_GATE_US) return;
  _lastIsrUs = now;
  _edgeCountIsr++;
}
