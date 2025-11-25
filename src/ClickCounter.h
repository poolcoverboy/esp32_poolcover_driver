#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "pins.h"
#include "AnalogController.h"  // MotionState


// Default to hardware click counting; simulation can be toggled at runtime.
#ifndef CLICK_COUNTER_USE_SIMULATION
#define CLICK_COUNTER_USE_SIMULATION 0
#endif

#ifndef CLICK_COUNTER_INTERVAL_FILTER_ENABLED
#define CLICK_COUNTER_INTERVAL_FILTER_ENABLED 1
#endif

#ifndef CLICK_COUNTER_INTERVAL_PERSIST_ENABLED
#define CLICK_COUNTER_INTERVAL_PERSIST_ENABLED 1
#endif

#ifndef CLICK_COUNTER_INTERVAL_DEFAULT_MS
#define CLICK_COUNTER_INTERVAL_DEFAULT_MS 880.0f
#endif

#ifndef CLICK_COUNTER_INTERVAL_DEFAULT_STD_MS
#define CLICK_COUNTER_INTERVAL_DEFAULT_STD_MS 80.0f
#endif

class StatusLed;

class ClickCounter {
public:
  using LogFn = void (*)(const String&);

  void begin(uint8_t pinClick = PIN_CLICK_IN,
             bool simulate = CLICK_COUNTER_USE_SIMULATION);
  void setLogger(LogFn logger);
  void setDebugLogger(LogFn logger);
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
  static constexpr const char* KEY_INTERVAL_STATS = "ivstats";
  static constexpr uint8_t     POS_SLOTS      = 8;
  static constexpr uint32_t    ISR_GATE_US    = 50000;  // 50 ms debounce between level changes
  static constexpr int32_t     SET_MIN_POS    = -512;
  static constexpr int32_t     SET_MAX_POS    = 8192;
  static constexpr int32_t     DEFAULT_END    = 256;
  static constexpr uint32_t    TAIL_HOLD_MS   = 100;
#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
  static constexpr uint8_t     INTERVAL_WINDOW        = 15;
  static constexpr uint8_t     INTERVAL_MIN_SAMPLES   = 4;
  static constexpr uint8_t     INTERVAL_RATIO_NUM     = 3;  // 60% of rolling median
  static constexpr uint8_t     INTERVAL_RATIO_DEN     = 5;
  static constexpr uint16_t    INTERVAL_MIN_SEED_MS   = 200;
  static constexpr uint16_t    INTERVAL_MAX_SEED_MS   = 5000;
  static constexpr float       INTERVAL_EWMA_ALPHA    = 0.12f;
  static constexpr uint32_t    INTERVAL_STATS_MAGIC   = 0x494E564C; // 'INVL'
  
  // Cumulative time error heuristic
  static constexpr float       TIME_ERROR_DECAY       = 0.90f;
  static constexpr float       TIME_ERROR_THRESHOLD_RATIO = 1.5f;
#endif

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
  void debugLog(const String& message);
  void mirrorSensorLevel();
  void processEdgeBatch(uint32_t edges);
  MotionState computeEffectiveDirection(bool* tailHoldUsed);
  void attachHardwareIsr();
  void detachHardwareIsr();
#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
  struct IntervalStatsState {
    float meanMs = CLICK_COUNTER_INTERVAL_DEFAULT_MS;
    float varianceMs2 = CLICK_COUNTER_INTERVAL_DEFAULT_STD_MS * CLICK_COUNTER_INTERVAL_DEFAULT_STD_MS;
    uint32_t sampleCount = 0;
  };

  void recordIntervalSample(uint16_t deltaMs);
  uint16_t intervalMedian() const;
  void seedIntervalWindow(uint16_t seedMs);
  uint16_t intervalSeedValue() const;
  void loadIntervalStats();
  void persistIntervalStats();
  void applyIntervalDefaults();
  void updateIntervalStats(uint16_t deltaMs);
#endif

  static void IRAM_ATTR gpioIsrThunk(void* arg);
  void IRAM_ATTR onIsr();
  static bool s_isrServiceInstalled;

  Preferences _prefs;
  uint8_t _pin = PIN_CLICK_IN;
  bool _simulate = true;

  LogFn _log = nullptr;
  LogFn _debugLog = nullptr;
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
#if CLICK_COUNTER_INTERVAL_FILTER_ENABLED
  unsigned long _lastAcceptedMs = 0;
  uint16_t _intervalWindow[INTERVAL_WINDOW] = {0};
  uint8_t _intervalCount = 0;
  uint8_t _intervalIndex = 0;
  IntervalStatsState _intervalStats;
  bool _intervalStatsDirty = false;
  float _timeError = 0.0f;
#endif
};
