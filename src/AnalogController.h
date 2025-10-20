#pragma once
#include <Arduino.h>
#include "StatusStore.h"
#include "pins.h"

// Minimal internal state for now
enum class MotionState { IDLE, OPENING, CLOSING };

// simple debounced button (active-low default)
class DebouncedBtn {
public:
  void begin(uint8_t pin, bool activeLow=true, uint16_t debounceMs=60) {
    _pin=pin; _activeLow=activeLow; _debounceMs=debounceMs;
    pinMode(_pin, INPUT_PULLUP);
    _stable = raw();
    _lastRead = _stable;
    _tChange = millis();
  }
  void update() {
    bool r = raw();
    if (r != _lastRead) { _lastRead=r; _tChange=millis(); }
    else if ((millis()-_tChange) > _debounceMs) { _stable=r; }
  }
  bool pressed() const { return _stable; }
private:
  uint8_t _pin=0; bool _activeLow=true; bool _stable=false, _lastRead=false;
  uint16_t _debounceMs=60; unsigned long _tChange=0;
  bool raw() const { bool v=digitalRead(_pin); return _activeLow ? !v : v; }
};

class AnalogController {
public:
  AnalogController(StatusStore &store, const char* rowLabel,
                   uint8_t pinUp=18, uint8_t pinDown=19, bool activeLow=true)
  : _store(store), _label(rowLabel), _pinUp(pinUp), _pinDown(pinDown), _activeLow(activeLow) {}

  void begin() {
    _btnUp.begin(_pinUp, _activeLow);
    _btnDown.begin(_pinDown, _activeLow);

    // WARNING: Boot must never translate a latched manual switch into motion.
    // Capture the current wiring state as the baseline so only future
    // transitions out of Neutral count as commands.
    hydrateInitialState();
  }

  // call frequently from loop()
  void update() {
    _btnUp.update();
    _btnDown.update();

    bool up = _btnUp.pressed();
    bool dn = _btnDown.pressed();

    MotionState newState = MotionState::IDLE;
    const char* status = "Neutral";

    if (up && !dn) { newState = MotionState::OPENING; status = "Open"; }
    else if (dn && !up) { newState = MotionState::CLOSING; status = "Close"; }
    else { newState = MotionState::IDLE; status = "Neutral"; }

    if (newState != _state) {
      _state = newState;
      // expose mapping to outside
      _mapped = _state;
      _store.setStatus(_label, status);
    }
  }

  MotionState state() const { return _state; }     // button-derived state (OPENING/CLOSING/IDLE)
  MotionState mapped() const { return _mapped; }   // same for now; reserved for future include HA/touch, etc.

private:
  StatusStore &_store;
  const char* _label;
  uint8_t _pinUp, _pinDown;
  bool _activeLow;
  DebouncedBtn _btnUp, _btnDown;
  MotionState _state = MotionState::IDLE;
  MotionState _mapped = MotionState::IDLE;

  void hydrateInitialState() {
    // WARNING: Keep this logic in sync with update(); removing it reintroduces
    // power-up motion commands when a wall switch is held during reboot.
    bool up = _btnUp.pressed();
    bool dn = _btnDown.pressed();

    if (up && !dn) {
      _state = MotionState::OPENING;
      _mapped = _state;
      _store.setStatus(_label, "Open");
    } else if (dn && !up) {
      _state = MotionState::CLOSING;
      _mapped = _state;
      _store.setStatus(_label, "Close");
    } else {
      _state = MotionState::IDLE;
      _mapped = _state;
      _store.setStatus(_label, "Neutral");
    }
  }
};
