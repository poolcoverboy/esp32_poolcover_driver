#include "StatusLed.h"

namespace {
  constexpr StatusLed::Frame BOOT_SEQ[] = {
    {200, true},
    {200, false}
  };

  constexpr StatusLed::Frame IDLE_SEQ[] = {
    {120, true},
    {1880, false}
  };

  constexpr StatusLed::Frame CONNECTIVITY_SEQ[] = {
    {120, true},
    {120, false},
    {120, true},
    {1500, false}
  };

  constexpr StatusLed::Frame PANIC_SEQ[] = {
    {80, true},
    {80, false}
  };

  constexpr StatusLed::Frame SET_MODE_SEQ[] = {
    {120, true},
    {120, false},
    {120, true},
    {120, false},
    {120, true},
    {1200, false}
  };
}

void StatusLed::begin(uint8_t pin, bool activeLow) {
  _pin = pin;
  _activeLow = activeLow;
  pinMode(_pin, OUTPUT);
  applyLevel(false);
  _initialized = true;
  _pattern = Pattern::BOOT;
  _frameIndex = 0;
  _nextFrameAt = 0;
}

void StatusLed::setPattern(Pattern pattern) {
  if (_pattern == pattern) return;
  _pattern = pattern;
  _frameIndex = 0;
  _nextFrameAt = 0;
}

void StatusLed::setDriveActive(bool active) {
  if (_driveActive == active) return;
  _driveActive = active;
  if (!_driveActive) {
    _frameIndex = 0;
    _nextFrameAt = 0;
  }
}

void StatusLed::onDriveLevel(bool levelHigh) {
  _driveLevel = levelHigh;
  if (_driveActive && _initialized) {
    applyLevel(levelHigh);
  }
}

void StatusLed::update() {
  if (!_initialized) return;

  if (_driveActive) {
    applyLevel(_driveLevel);
    return;
  }

  size_t frameCount = 0;
  const Frame* frames = framesForPattern(_pattern, &frameCount);
  if (!frames || frameCount == 0) {
    applyLevel(false);
    return;
  }

  unsigned long now = millis();
  if (_nextFrameAt == 0 || now >= _nextFrameAt) {
    const Frame& frame = frames[_frameIndex];
    applyLevel(frame.levelHigh);
    _nextFrameAt = now + frame.durationMs;
    _frameIndex = (_frameIndex + 1) % frameCount;
  }
}

void StatusLed::applyLevel(bool high) {
  bool actual = _activeLow ? !high : high;
  if (_ledState == high) {
    digitalWrite(_pin, actual);
    return;
  }
  _ledState = high;
  digitalWrite(_pin, actual);
}

const StatusLed::Frame* StatusLed::framesForPattern(Pattern pattern, size_t* count) const {
  switch (pattern) {
    case Pattern::BOOT:
      *count = sizeof(BOOT_SEQ) / sizeof(BOOT_SEQ[0]);
      return BOOT_SEQ;
    case Pattern::IDLE:
      *count = sizeof(IDLE_SEQ) / sizeof(IDLE_SEQ[0]);
      return IDLE_SEQ;
    case Pattern::CONNECTIVITY_LOSS:
      *count = sizeof(CONNECTIVITY_SEQ) / sizeof(CONNECTIVITY_SEQ[0]);
      return CONNECTIVITY_SEQ;
    case Pattern::PANIC:
      *count = sizeof(PANIC_SEQ) / sizeof(PANIC_SEQ[0]);
      return PANIC_SEQ;
    case Pattern::SET_MODE:
      *count = sizeof(SET_MODE_SEQ) / sizeof(SET_MODE_SEQ[0]);
      return SET_MODE_SEQ;
    default:
      *count = 0;
      return nullptr;
  }
}
