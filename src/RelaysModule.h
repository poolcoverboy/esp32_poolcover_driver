#pragma once
#include <Arduino.h>
#include "pins.h"
#include "StatusStore.h"
#include "AnalogController.h"   // MotionState

class RelaysModule {
public:
  using LogFn = void (*)(const String&);

  explicit RelaysModule(StatusStore& store, LogFn logger = nullptr)
    : _store(store), _log(logger) {}

  void begin(bool activeLow = true,
             uint32_t deadMs = 1000,
             uint32_t psuSpinMs = 1000) {
    _activeLow   = activeLow;
    _deadMs      = deadMs;
    _psuSpinMs   = psuSpinMs;

    uint8_t offLevel = _activeLow ? HIGH : LOW;
    auto primePin = [&](uint8_t pin) {
      digitalWrite(pin, offLevel);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, offLevel);
    };

    primePin(PIN_RELAY_FWD);
    primePin(PIN_RELAY_REV);
    primePin(PIN_RELAY_PSU);
    primePin(PIN_RELAY_EN);

    _cur = MotionState::IDLE;
    _want = MotionState::IDLE;
    _psuOn = false;
    _tChangeAllowed = 0;
    _tPsuReady = 0;
    _latchedDrive = MotionState::IDLE;
    _enableOn = false;
    _tEnableReady = 0;
    _psuHoldActive = false;
    _tPsuHoldOff = 0;
    driveEnable(false);

    if (_store.setStatus("Action", "Idle") && _log) {
      _log(String(F("[RELAYS] Action -> Idle")));
    }
  }

  // Desired motion (from buttons): OPENING / CLOSING / IDLE
  void request(MotionState want) {
    if (want != MotionState::IDLE) {
      _psuHoldActive = false;
    }
    // If switching direction while moving: enforce dead-time
    MotionState active = (_cur != MotionState::IDLE) ? _cur : _latchedDrive;
    if (want != MotionState::IDLE && active != MotionState::IDLE && want != active) {
      // stop, start dead-time window; PSU stays ON during this 1s
      allStop();
      _cur = MotionState::IDLE;
      _latchedDrive = MotionState::IDLE;
      _tChangeAllowed = millis() + _deadMs;
      if (_store.setStatus("Action", "Idle (dead-time)") && _log) {
        _log(String(F("[RELAYS] Action -> Idle (dead-time)")));
      }
    }
    _want = want;
  }

  void emergencyPanicOff(const char* reason = "panic") {
    allStop();
    drive(PIN_RELAY_PSU, false);
    _psuOn = false;
    _cur = MotionState::IDLE;
    _want = MotionState::IDLE;
    _psuHoldActive = false;
    if (_store.setStatus("Action", "ERROR Panic") && _log) {
      const char* why = (reason && reason[0]) ? reason : "panic";
      _log(String(F("[RELAYS] Action -> ERROR Panic (")) + why + F(")"));
    }
  }

  void update() {
    const unsigned long now = millis();

    // If the operator wants IDLE (Neutral): stop & PSU OFF immediately (per current spec)
    if (_want == MotionState::IDLE) {
      if (_cur != MotionState::IDLE) {
        allStop();
        _cur = MotionState::IDLE;
        _latchedDrive = MotionState::IDLE;
        _psuHoldActive = true;
        _tPsuHoldOff = now + _psuHoldMs;
      }
      if (_enableOn) driveEnable(false);
      if (_latchedDrive != MotionState::IDLE) {
        stopDirections();
        _latchedDrive = MotionState::IDLE;
      }

      if (_psuHoldActive && (long)(_tPsuHoldOff - now) <= 0) {
        _psuHoldActive = false;
      }

      if (_psuHoldActive) {
        if (!_psuOn) {
          drive(PIN_RELAY_PSU, true);
          _psuOn = true;
        }
      } else if (_psuOn) {
        drive(PIN_RELAY_PSU, false);
        _psuOn = false;
      }

      const char* label = _psuHoldActive ? "Idle (PSU hold)" : "Idle";
      if (_store.setStatus("Action", label) && _log) {
        _log(String(F("[RELAYS] Action -> ")) + label);
      }
      return;
    }

    // If still in dead-time after a direction change, just wait (PSU stays ON)
    if (now < _tChangeAllowed) {
      if (_enableOn) driveEnable(false);
      return;
    }

    // Need motion -> ensure PSU is ON and has spun up
    if (!_psuOn) {
      drive(PIN_RELAY_PSU, true);
      _psuOn = true;
      _tPsuReady = now + _psuSpinMs;
      if (_store.setStatus("Action", "PSU spin-up") && _log) {
        _log(String(F("[RELAYS] Action -> PSU spin-up")));
      }
      return;
    }
    if (now < _tPsuReady) {
      if (_enableOn) driveEnable(false);
      return;
    }

    MotionState target = _want;

    // Adjust direction relays if needed (keep enable open while switching)
    if (_latchedDrive != target) {
      if (_enableOn) driveEnable(false);
      stopDirections();
      if (target == MotionState::OPENING) {
        drive(PIN_RELAY_FWD, true);
      } else if (target == MotionState::CLOSING) {
        drive(PIN_RELAY_REV, true);
      }
      _latchedDrive = target;
      _tEnableReady = now + _enableDelayMs;
      if (_store.setStatus("Action", target == MotionState::OPENING ? "Opening (arming)"
                                                                    : "Closing (arming)") && _log) {
        _log(String(F("[RELAYS] Action -> ")) +
             (target == MotionState::OPENING ? F("Opening (arming)") : F("Closing (arming)")));
      }
      return;
    }

    // Engage enable relay once direction contacts are set and delay elapsed
    if (!_enableOn) {
      if (now < _tEnableReady) return;
      driveEnable(true);
      _cur = _latchedDrive;
      const char* label = (_cur == MotionState::OPENING) ? "Opening" : "Closing";
      if (_store.setStatus("Action", label) && _log) {
        _log(String(F("[RELAYS] Action -> ")) + label);
      }
      return;
    }

    // Maintain current status when running
    _cur = _latchedDrive;
  }

  MotionState current() const { return _cur; }

private:
  StatusStore& _store;
  LogFn _log = nullptr;

  bool _activeLow = true;
  bool _psuOn = false;
  bool _enableOn = false;

  MotionState _cur  = MotionState::IDLE;
  MotionState _want = MotionState::IDLE;
  MotionState _latchedDrive = MotionState::IDLE;

  unsigned long _tChangeAllowed = 0;
  unsigned long _tPsuReady = 0;
  unsigned long _tEnableReady = 0;

  uint32_t _deadMs = 1000;
  uint32_t _psuSpinMs = 1000;
  static constexpr uint32_t _enableDelayMs = 200;
  static constexpr uint32_t _psuHoldMs = 60000;
  bool _psuHoldActive = false;
  unsigned long _tPsuHoldOff = 0;

  inline void drive(uint8_t pin, bool on) {
    digitalWrite(pin, (_activeLow ? !on : on));
  }
  inline void allStop() {
    stopDirections();
    driveEnable(false);
    _latchedDrive = MotionState::IDLE;
    _tEnableReady = 0;
  }
  inline void stopDirections() {
    drive(PIN_RELAY_FWD, false);
    drive(PIN_RELAY_REV, false);
  }
  inline void driveEnable(bool on) {
    digitalWrite(PIN_RELAY_EN, (_activeLow ? !on : on));
    _enableOn = on;
    if (!on) _tEnableReady = 0;
  }
};
