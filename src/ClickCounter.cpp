#include "ClickCounter.h"
#include "StatusLed.h"

#include <cstdio>
#include <cstring>
#include <climits>
#include <driver/gpio.h>
#include <esp_intr_alloc.h>
#include <esp_err.h>

bool ClickCounter::s_isrServiceInstalled = false;

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
  uint32_t edges = 0;
  noInterrupts();
  edges = _edgeCountIsr;
  _edgeCountIsr = 0;
  interrupts();

  if (!edges) {
    return;
  }

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

      if (dir == MotionState::CLOSING) {
        _pos += 1;
        persistPos(false);
      } else if (dir == MotionState::OPENING) {
        _pos -= 1;
        persistPos(false);
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
}

void ClickCounter::logMessage(const String& message) {
  if (!message.length()) return;
  if (_log) {
    _log(message);
  } else {
    Serial.println(message);
  }
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
