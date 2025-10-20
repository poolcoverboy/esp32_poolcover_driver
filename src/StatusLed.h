#pragma once
#include <Arduino.h>

class StatusLed {
public:
  enum class Pattern {
    BOOT,
    IDLE,
    CONNECTIVITY_LOSS,
    PANIC,
    SET_MODE
  };

  void begin(uint8_t pin, bool activeLow = false);
  void update();
  void setPattern(Pattern pattern);
  void setDriveActive(bool active);
  void onDriveLevel(bool levelHigh);

  struct Frame {
    uint16_t durationMs;
    bool levelHigh;
  };

private:
  void applyLevel(bool high);
  const Frame* framesForPattern(Pattern pattern, size_t* count) const;

  uint8_t _pin = 255;
  bool _activeLow = false;
  bool _initialized = false;

  Pattern _pattern = Pattern::BOOT;
  uint8_t _frameIndex = 0;
  unsigned long _nextFrameAt = 0;
  bool _ledState = false;

  bool _driveActive = false;
  bool _driveLevel = false;
};
