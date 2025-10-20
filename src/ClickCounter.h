#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "pins.h"
#include "AnalogController.h"  // MotionState


// Default to hardware click counting; simulation can be toggled at runtime.
#ifndef CLICK_COUNTER_USE_SIMULATION
#define CLICK_COUNTER_USE_SIMULATION 0
#endif

class StatusLed;

class ClickCounter {
public:
  using LogFn = void (*)(const String&);

  void begin(uint8_t pinClick = PIN_CLICK_IN,
             bool simulate = CLICK_COUNTER_USE_SIMULATION);
  void setLogger(LogFn logger);
  void setStatusLed(class StatusLed* led);
  void setMotion(MotionState s);
  void update(bool allowBeyondLimits = false);

  void beginCalibration();
  void setOpenHere();
  void setClosedHere();
  void finalizeCalibration();
  void clearPanic();

  void setSimulation(bool simulate);

  bool canOpen() const;
  bool canClose() const;
  bool panic() const;

  int32_t position() const;
  int32_t end() const;

  void forcePersist();

private:
  struct PosRecV0 {
    uint32_t epoch;
    int32_t  pos;
    uint32_t crc32;
  };

  struct PosRecV1 {
    uint32_t epoch;
    int32_t  pos;
    uint8_t  level;
    uint8_t  reserved[3];
    uint32_t crc32;
  };

  static constexpr size_t POS_REC_V0_SIZE = sizeof(PosRecV0);
  static constexpr size_t POS_REC_V1_SIZE = sizeof(PosRecV1);

  static constexpr const char* NAMESPACE      = "poolcover";
  static constexpr const char* KEY_END        = "end";
  static constexpr uint8_t     POS_SLOTS      = 8;
  static constexpr uint32_t    ISR_GATE_US    = 2250;
  static constexpr int32_t     SET_MIN_POS    = -512;
  static constexpr int32_t     SET_MAX_POS    = 8192;
  static constexpr int32_t     DEFAULT_END    = 256;
  static constexpr uint32_t    TAIL_HOLD_MS   = 100;

  void simulateTicks();
  void drainHardwareEdges();
  void persistEnd();
  void persistPos(bool force);
  void loadFromNvs();
  void loadEnd();
  void loadPos();
  static uint32_t crc32(const void* data, size_t len);
  static uint32_t computeRecCrc(uint32_t epoch, int32_t pos, uint8_t level);
  static uint32_t computeRecCrcLegacy(uint32_t epoch, int32_t pos);
  void shiftCoordinateFrame(int32_t delta, bool adjustEnd = true);
  void recomputeSpanFromMarks();
  void clampCalibrationRange();
  void prepareForMotion();
  void refreshLiveLevel();
  void clearPendingEdges();
  void logMessage(const String& message);
  void mirrorSensorLevel();
  void processEdgeBatch(uint32_t edges);
  MotionState computeEffectiveDirection(bool* tailHoldUsed);
  void attachHardwareIsr();
  void detachHardwareIsr();

  static void IRAM_ATTR gpioIsrThunk(void* arg);
  void IRAM_ATTR onIsr();
  static bool s_isrServiceInstalled;

  Preferences _prefs;
  uint8_t _pin = PIN_CLICK_IN;
  bool _simulate = true;

  LogFn _log = nullptr;
  StatusLed* _statusLed = nullptr;

  volatile uint32_t _edgeCountIsr = 0;
  volatile uint32_t _lastIsrUs = 0;
  bool _isrAttached = false;

  MotionState _motion = MotionState::IDLE;
  MotionState _lastMotion = MotionState::IDLE;

  int32_t _pos = 0;
  int32_t _end = DEFAULT_END;
  bool _panic = false;

  uint32_t _epoch = 0;

  unsigned long _lastSimTickMs = 0;
  unsigned long _lastPersistMs = 0;
  int32_t _lastPersistPos = 0;
  bool _lastPersistLevelLow = false;

  bool _prefsOpen = false;

  bool _calibrationActive = false;
  bool _calibOpenSet = false;
  bool _calibClosedSet = false;
  int32_t _calibEntryPos = 0;
  int32_t _calibEntryEnd = DEFAULT_END;
  int32_t _calibOpenRaw = 0;
  int32_t _calibClosedRaw = 0;

  bool _sensorExpectedLow = false;
  bool _sensorLiveLow = false;
  bool _sensorPersisted = false;
  bool _simSensorLow = false;
  bool _overshootLogged = false;
  MotionState _lastActiveDirection = MotionState::IDLE;
  unsigned long _tailHoldUntil = 0;
  uint8_t _edgePhase = 0;
};
