#include <Arduino.h>
#include <Preferences.h>
#include "StatusStore.h"
#include "WifiModule.h"
#include "AnalogController.h"
#include "RelaysModule.h"
#include "MqttModule.h"
#include "ClickCounter.h"
#include "StatusLed.h"
#include "RingLogger.h"
#include "pins.h"

static constexpr size_t LOG_BUFFER_BYTES = 8 * 1024;
static constexpr unsigned long LOG_SNAPSHOT_INTERVAL_MS = 1500;
static constexpr unsigned long POS_STATUS_INTERVAL_MS = 500;

static StatusStore statusStore;
static RingLogger ringLog(LOG_BUFFER_BYTES);
static WifiModule* wifi = nullptr;
static AnalogController* analogCtl = nullptr;
static RelaysModule* relays = nullptr;
static MqttModule* mqtt = nullptr;
static ClickCounter clicks;
static StatusLed statusLed;

static MotionState lastAnalogRaw = MotionState::IDLE;
static MotionState lastAnalogEffective = MotionState::IDLE;
static MotionState lastRelay = MotionState::IDLE;
static MotionState commandedMotion = MotionState::IDLE;
static bool setModeActive = false;
static bool panicLatched = false;
static bool lastMqttConnected = false;
static unsigned long lastLogSnapshotMs = 0;
static unsigned long lastPosStatusMs = 0;
static const char* lastModeLabel = "LOCAL";
static bool clickSimulationEnabled = false;
static uint8_t mqttSetModeStreak = 0;
static constexpr uint8_t MQTT_TOGGLE_THRESHOLD = 3;
static bool loggedOpenLimit = false;
static bool loggedCloseLimit = false;
static bool analogEdgeArmed = false;

static Preferences configPrefs;
static bool configPrefsOpen = false;
static constexpr uint32_t DEFAULT_SAFETY_MAX_RUN_SECONDS = 300;
static constexpr unsigned long NO_CLICK_PANIC_WINDOW_MS = 5000UL;
static constexpr uint32_t NO_CLICK_PANIC_WINDOW_SECONDS = NO_CLICK_PANIC_WINDOW_MS / 1000UL;
static uint32_t safetyMaxRunSeconds = DEFAULT_SAFETY_MAX_RUN_SECONDS;

static bool driveActive = false;
static unsigned long driveLastUpdateMs = 0;
static uint32_t driveAccumMs = 0;

static bool noClickMonitorActive = false;
static unsigned long noClickStartMs = 0;
static int32_t noClickPosAtEnable = 0;

static bool panicRebootPending = false;
static unsigned long panicRebootAtMs = 0;
static bool manualResetArmed = false;

enum class CommandSource { NONE, WALL_SWITCH, HOME_ASSISTANT };

static MotionState analogLatched = MotionState::IDLE;
static MotionState haLatched = MotionState::IDLE;
static uint32_t commandSeqCounter = 0;
static uint32_t analogCommandSeq = 0;
static uint32_t haCommandSeq = 0;
static CommandSource activeCommandSource = CommandSource::NONE;
static bool haClearedLocally = false;
static MotionState lastHaDesired = MotionState::IDLE;

static void logLine(const String& message);
static void logLine(const __FlashStringHelper* message) {
  logLine(String(message));
}

static void triggerPanic(const char* reason, bool requestReboot);
static void schedulePanicReboot(unsigned long now);
static void resetSafetyRuntime(const char* reason);
static void onMqttSetMaxRuntime(uint32_t seconds);
static void loadSafetyConfig();
static void persistSafetyMaxRunSeconds(uint32_t seconds);

static const char* motionLabel(MotionState state) {
  switch (state) {
    case MotionState::OPENING: return "Opening";
    case MotionState::CLOSING: return "Closing";
    default: return "Idle";
  }
}

static const char* analogSwitchLabel(MotionState state) {
  switch (state) {
    case MotionState::OPENING: return "Open";
    case MotionState::CLOSING: return "Close";
    default: return "Neutral";
  }
}

static void resetMqttSetModeStreak() { mqttSetModeStreak = 0; }

static void applySimulationMode(bool enable, const char* origin) {
  if (clickSimulationEnabled == enable) return;
  clicks.setSimulation(enable);
  clickSimulationEnabled = enable;
  const __FlashStringHelper* modeLabel = enable
    ? F("[SIM] Click counter -> SIMULATION")
    : F("[SIM] Click counter -> HARDWARE");
  logLine(String(origin ? origin : "") + modeLabel);
}

static void toggleSimulationMode(const char* origin) {
  applySimulationMode(!clickSimulationEnabled, origin);
  resetMqttSetModeStreak();
}

static const char* computeModeLabel(CommandSource source,
                                    bool inSetMode) {
  if (inSetMode) return "SET";
  if (source == CommandSource::HOME_ASSISTANT) return "AUTO";
  return "LOCAL";
}

static void clearHaDesiredLocal() {
  if (!mqtt) return;
  mqtt->clearHaDesired();
  haClearedLocally = true;
}

static void updateSafetyRow() {
  const char* label = panicLatched ? "Panic"
                                   : (setModeActive ? "Set Mode" : "Nominal");
  statusStore.setStatus("Safety", label);
}

static void loadSafetyConfig() {
  configPrefsOpen = configPrefs.begin("poolcfg", false);
  if (!configPrefsOpen) {
    safetyMaxRunSeconds = DEFAULT_SAFETY_MAX_RUN_SECONDS;
    logLine(F("[SAFETY] Preferences unavailable, using defaults"));
    return;
  }

  uint32_t stored = configPrefs.getUInt("max_run_s", 0);
  if (stored == 0) {
    stored = DEFAULT_SAFETY_MAX_RUN_SECONDS;
    configPrefs.putUInt("max_run_s", stored);
  }

  safetyMaxRunSeconds = stored;
  logLine(String(F("[SAFETY] Max runtime limit = ")) + safetyMaxRunSeconds + F(" s"));
}

static void persistSafetyMaxRunSeconds(uint32_t seconds) {
  if (!configPrefsOpen) return;
  configPrefs.putUInt("max_run_s", seconds);
}

static void resetSafetyRuntime(const char* reason) {
  driveAccumMs = 0;
  if (driveActive) {
    driveLastUpdateMs = millis();
  } else {
    driveLastUpdateMs = 0;
  }
  if (reason && reason[0]) {
    logLine(String(F("[SAFETY] Runtime guard reset: ")) + reason);
  }
}

static void schedulePanicReboot(unsigned long now) {
  panicRebootPending = true;
  panicRebootAtMs = now + 500UL;
}

static void triggerPanic(const char* reason, bool requestReboot) {
  if (panicLatched) return;
  panicLatched = true;
  updateSafetyRow();
  const char* why = (reason && reason[0]) ? reason : "panic";
  logLine(String(F("[PANIC] Triggered: ")) + why);
  if (relays) relays->emergencyPanicOff(why);
  clearHaDesiredLocal();
  resetMqttSetModeStreak();
  analogLatched = MotionState::IDLE;
  analogCommandSeq = ++commandSeqCounter;
  driveActive = false;
  driveAccumMs = 0;
  driveLastUpdateMs = 0;
  noClickMonitorActive = false;
  if (requestReboot) {
    schedulePanicReboot(millis());
  }
}

static void onMqttSetMaxRuntime(uint32_t seconds) {
  uint32_t clamped = seconds;
  if (clamped < 30U) {
    clamped = 30U;
  } else if (clamped > 1800U) {
    clamped = 1800U;
  }

  if (clamped != seconds) {
    logLine(String(F("[SAFETY] Max runtime clamp applied (requested=")) + seconds +
            F(" s)"));
  }

  if (clamped == safetyMaxRunSeconds) {
    logLine(String(F("[SAFETY] Max runtime unchanged (")) + safetyMaxRunSeconds + F(" s)"));
    return;
  }

  safetyMaxRunSeconds = clamped;
  persistSafetyMaxRunSeconds(safetyMaxRunSeconds);
  logLine(String(F("[SAFETY] Max runtime updated -> ")) + safetyMaxRunSeconds + F(" s"));
  resetSafetyRuntime("config change");
}

static void enterSetMode(const char* origin) {
  if (setModeActive) return;
  setModeActive = true;
  relays->request(MotionState::IDLE);
  relays->update();
  clicks.clearPanic();
  clicks.beginCalibration();
  clicks.forcePersist();
  clearHaDesiredLocal();
  panicLatched = false;
  updateSafetyRow();
  logLine(String(origin) + F("Entering SET mode (limits relaxed)"));
  analogEdgeArmed = false;
  lastAnalogEffective = MotionState::IDLE;
  if (analogCtl) {
    lastAnalogRaw = analogCtl->state();
  } else {
    lastAnalogRaw = MotionState::IDLE;
  }
  analogLatched = MotionState::IDLE;
  analogCommandSeq = ++commandSeqCounter;
}

static void exitSetMode(const char* origin) {
  if (!setModeActive) return;
  setModeActive = false;
  relays->request(MotionState::IDLE);
  relays->update();
  clicks.finalizeCalibration();
  clearHaDesiredLocal();
  updateSafetyRow();
  logLine(String(origin) + F("Exiting SET mode (limits enforced)"));
  if (analogCtl) {
    lastAnalogRaw = analogCtl->state();
  } else {
    lastAnalogRaw = MotionState::IDLE;
  }
  analogEdgeArmed = (lastAnalogRaw == MotionState::IDLE);
  analogLatched = MotionState::IDLE;
  analogCommandSeq = ++commandSeqCounter;
}

static void processHaCommands() {
  if (!mqtt) return;
  while (mqtt->hasPendingCommand()) {
    String cmd = mqtt->popCommand();
    if (!cmd.length()) continue;
    if (cmd == "set_open_here") {
      resetMqttSetModeStreak();
      clicks.setOpenHere();
      logLine(F("[CMD] Marked current position as fully open"));
    } else if (cmd == "set_closed_here") {
      resetMqttSetModeStreak();
      clicks.setClosedHere();
      logLine(F("[CMD] Marked current position as fully closed"));
    } else if (cmd == "enter_set_mode") {
      if (mqttSetModeStreak < UINT8_MAX) {
        ++mqttSetModeStreak;
        if (mqttSetModeStreak >= MQTT_TOGGLE_THRESHOLD) {
          toggleSimulationMode("[CMD] ");
        }
      }
      enterSetMode("[CMD] ");
    } else if (cmd == "exit_set_mode") {
      resetMqttSetModeStreak();
      exitSetMode("[CMD] ");
    } else {
      resetMqttSetModeStreak();
    }
  }
}

static void logLine(const String& message) {
  if (!message.length()) return;
  if (Serial && Serial.availableForWrite() > message.length() + 2) {
    Serial.println(message);
  } else {
    Serial.println(message);
  }
  ringLog.append(message);
  if (mqtt) {
    mqtt->publishLogLine(message);
    unsigned long now = millis();
    if (lastLogSnapshotMs == 0 || now - lastLogSnapshotMs >= LOG_SNAPSHOT_INTERVAL_MS) {
      lastLogSnapshotMs = now;
      mqtt->publishLogSnapshot(ringLog.blob());
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();

  const char* rows[] = { "Wifi", "HASS", "Mode", "Action", "Analog", "Pos", "Safety" };
  statusStore.configure(rows, sizeof(rows) / sizeof(rows[0]));
  statusStore.setStatus("Wifi", "Connecting");
  statusStore.setStatus("HASS", "Waiting");
  statusStore.setStatus("Mode", lastModeLabel);
  statusStore.setStatus("Action", "Idle");
  statusStore.setStatus("Analog", "Neutral");
  statusStore.setStatus("Pos", "0 (0%)");
  updateSafetyRow();

  logLine(F("[BOOT] Pool cover controller (ESP32-32U headless)"));

  loadSafetyConfig();
  resetSafetyRuntime("boot");

  statusLed.begin(PIN_STATUS_LED, /*activeLow=*/false);
  statusLed.setPattern(StatusLed::Pattern::BOOT);

  relays = new RelaysModule(statusStore, logLine);
  relays->begin(RELAYS_ACTIVE_LOW != 0, 1000, 2000);
  relays->request(MotionState::IDLE);
  relays->update();

  wifi = new WifiModule(statusStore, logLine);
  wifi->begin();

  analogCtl = new AnalogController(statusStore, "Analog",
                                   PIN_BTN_UP, PIN_BTN_DOWN, true);
  analogCtl->begin();
  lastAnalogRaw = analogCtl->state();
  lastAnalogEffective = MotionState::IDLE;
  analogEdgeArmed = (lastAnalogRaw == MotionState::IDLE);

  mqtt = new MqttModule(statusStore, logLine);
  mqtt->begin();
  mqtt->setMaxRuntimeHandler(onMqttSetMaxRuntime);

  clicks.setStatusLed(&statusLed);
  clicks.begin(PIN_CLICK_IN, /*simulate=*/false);
  clicks.setLogger(logLine);
  clickSimulationEnabled = false;
  logLine(F("[BOOT] Click counter ready (hardware ISR)"));

  logLine(F("[INIT] Modules initialized. Waiting for Wi-Fi/MQTT..."));
}

void loop() {
  unsigned long now = millis();

  if (wifi) wifi->update();

  MotionState analogState = lastAnalogEffective;
  if (analogCtl) {
    analogCtl->update();
    MotionState raw = analogCtl->state();

    if (setModeActive) {
      analogEdgeArmed = false;
    }

    if (raw != lastAnalogRaw) {
      lastAnalogRaw = raw;
      MotionState newEffective = lastAnalogEffective;

      if (raw == MotionState::IDLE) {
        analogEdgeArmed = true;
        newEffective = MotionState::IDLE;
      } else if (analogEdgeArmed) {
        analogEdgeArmed = false;
        newEffective = raw;
      } else {
        newEffective = MotionState::IDLE;
      }

      if (newEffective != lastAnalogEffective) {
        if (newEffective != MotionState::IDLE) {
          clearHaDesiredLocal();
        }
        resetMqttSetModeStreak();
        logLine(String(F("[INPUT] Analog switch -> ")) + analogSwitchLabel(newEffective));
        lastAnalogEffective = newEffective;
        analogLatched = newEffective;
        analogCommandSeq = ++commandSeqCounter;
      }
    }

    analogState = lastAnalogEffective;
  } else {
    analogState = MotionState::IDLE;
  }

  processHaCommands();

  MotionState haDesired = mqtt ? mqtt->desiredFromHA() : MotionState::IDLE;
  if (haDesired != lastHaDesired) {
    lastHaDesired = haDesired;
    haLatched = haDesired;
    if (haClearedLocally && haDesired == MotionState::IDLE) {
      haClearedLocally = false;
    } else {
      haClearedLocally = false;
      haCommandSeq = ++commandSeqCounter;
    }
  }

  CommandSource selectedSource = CommandSource::NONE;
  uint32_t selectedSeq = 0;
  MotionState target = panicLatched ? MotionState::IDLE : commandedMotion;

  if (!panicLatched) {
    if (analogCommandSeq > selectedSeq) {
      selectedSeq = analogCommandSeq;
      target = analogLatched;
      selectedSource = CommandSource::WALL_SWITCH;
    }
    if (haCommandSeq > selectedSeq) {
      selectedSeq = haCommandSeq;
      target = haLatched;
      selectedSource = CommandSource::HOME_ASSISTANT;
    }
  }

  bool consumeAnalogCommand = (selectedSource == CommandSource::WALL_SWITCH);

  if (selectedSource == CommandSource::WALL_SWITCH) {
    if (!manualResetArmed && driveActive) {
      resetSafetyRuntime("manual interaction");
    }
    manualResetArmed = true;
  } else {
    manualResetArmed = false;
  }

  if (!setModeActive) {
    bool canOpen = clicks.canOpen();
    bool canClose = clicks.canClose();

    if (target == MotionState::OPENING && !canOpen) {
      if (!loggedOpenLimit) {
        logLine(F("[LIMIT] Open boundary reached, stopping motion"));
        loggedOpenLimit = true;
      }
      resetSafetyRuntime("open limit reached");
      target = MotionState::IDLE;
      if (selectedSource == CommandSource::HOME_ASSISTANT) {
        clearHaDesiredLocal();
      }
    } else if (canOpen && loggedOpenLimit) {
      loggedOpenLimit = false;
    }

    if (target == MotionState::CLOSING && !canClose) {
      if (!loggedCloseLimit) {
        logLine(F("[LIMIT] Close boundary reached, stopping motion"));
        loggedCloseLimit = true;
      }
      resetSafetyRuntime("close limit reached");
      target = MotionState::IDLE;
      if (selectedSource == CommandSource::HOME_ASSISTANT) {
        clearHaDesiredLocal();
      }
    } else if (canClose && loggedCloseLimit) {
      loggedCloseLimit = false;
    }
  } else {
    loggedOpenLimit = false;
    loggedCloseLimit = false;
  }

  if (consumeAnalogCommand) {
    // WARNING: Analog switch acts as a momentary trigger only. Never let its
    // latched position block Home Assistant or subsequent commands.
    if (lastAnalogEffective != MotionState::IDLE) {
      analogEdgeArmed = false;
    }
    analogLatched = MotionState::IDLE;
    analogCommandSeq = 0;
    haLatched = MotionState::IDLE;
    haCommandSeq = 0;
    lastHaDesired = MotionState::IDLE;
  }

  activeCommandSource = selectedSource;

  const char* modeLabel = computeModeLabel(activeCommandSource, setModeActive);
  if (modeLabel != lastModeLabel) {
    lastModeLabel = modeLabel;
    statusStore.setStatus("Mode", lastModeLabel);
    logLine(String(F("[MODE] -> ")) + lastModeLabel);
  }

  if (target != commandedMotion) {
    commandedMotion = target;
    logLine(String(F("[CTRL] Commanded motion -> ")) + motionLabel(commandedMotion));
  }

  if (relays) {
    relays->request(commandedMotion);
    relays->update();
    MotionState relayState = relays->current();

    if (relayState != MotionState::IDLE) {
      if (!driveActive || lastRelay == MotionState::IDLE) {
        driveActive = true;
        driveAccumMs = 0;
        driveLastUpdateMs = now;
        noClickMonitorActive = true;
        noClickStartMs = now;
        noClickPosAtEnable = clicks.position();
      } else {
        if (driveLastUpdateMs != 0) {
          unsigned long delta = now - driveLastUpdateMs;
          driveAccumMs += static_cast<uint32_t>(delta);
        }
        driveLastUpdateMs = now;
      }
    } else {
      if (driveActive) {
        driveActive = false;
        driveAccumMs = 0;
        driveLastUpdateMs = 0;
      }
      noClickMonitorActive = false;
    }
    if (relayState != lastRelay) {
      lastRelay = relayState;
      logLine(String(F("[CTRL] Relay state -> ")) + motionLabel(relayState));
    }

    if (driveActive && safetyMaxRunSeconds > 0) {
      uint64_t limitMs = static_cast<uint64_t>(safetyMaxRunSeconds) * 1000ULL;
      if (static_cast<uint64_t>(driveAccumMs) >= limitMs) {
        triggerPanic("max-runtime-exceeded", true);
      }
    }

    clicks.setMotion(relayState);
  }

  statusLed.setDriveActive(driveActive);

  clicks.update(setModeActive);

  if (noClickMonitorActive) {
    int32_t currentPos = clicks.position();
    if (currentPos != noClickPosAtEnable) {
      noClickMonitorActive = false;
    } else if ((long)(now - noClickStartMs) >= (long)NO_CLICK_PANIC_WINDOW_MS) {
      triggerPanic("no-click-after-enable", true);
      noClickMonitorActive = false;
    }
  }

  if (!setModeActive && clicks.panic() && !panicLatched) {
    triggerPanic("click-out-of-range", false);
  }

  int32_t pos = clicks.position();
  int32_t end = clicks.end();
  if (end < 1) end = 1;
  int32_t pct = (pos <= 0) ? 0 : (int32_t)((static_cast<int64_t>(pos) * 100) / end);
  if (pct > 100) pct = 100;

  StatusLed::Pattern ledPattern = StatusLed::Pattern::IDLE;
  bool wifiConnected = wifi && wifi->isConnected();
  bool mqttConnected = mqtt && mqtt->isConnected();
  if (panicLatched || panicRebootPending) {
    ledPattern = StatusLed::Pattern::PANIC;
  } else if (setModeActive) {
    ledPattern = StatusLed::Pattern::SET_MODE;
  } else if (!wifiConnected || !mqttConnected) {
    ledPattern = StatusLed::Pattern::CONNECTIVITY_LOSS;
  } else if (!driveActive) {
    ledPattern = StatusLed::Pattern::IDLE;
  }
  statusLed.setPattern(ledPattern);

  if (now - lastPosStatusMs >= POS_STATUS_INTERVAL_MS || commandedMotion == MotionState::IDLE) {
    lastPosStatusMs = now;
    String posStatus;
    posStatus.reserve(24);
    posStatus += pos;
    posStatus += F(" (");
    posStatus += pct;
    posStatus += F("%)");
    statusStore.setStatus("Pos", posStatus);
  }

  updateSafetyRow();

  if (mqtt) {
    uint32_t runtimeElapsedSec = driveActive ? (driveAccumMs / 1000UL) : 0;
    mqtt->update(lastModeLabel,
                 relays ? relays->current() : MotionState::IDLE,
                 analogState,
                 analogSwitchLabel(analogState),
                 setModeActive,
                 panicLatched,
                 clicks,
                 safetyMaxRunSeconds,
                 runtimeElapsedSec,
                 driveActive,
                 NO_CLICK_PANIC_WINDOW_SECONDS);

    bool connected = mqtt->isConnected();
    if (connected && !lastMqttConnected) {
      mqtt->publishLogSnapshot(ringLog.blob());
    }
    lastMqttConnected = connected;
  }

  if (panicRebootPending) {
    unsigned long rebootNow = millis();
    if ((long)(rebootNow - panicRebootAtMs) >= 0) {
      panicRebootPending = false;
      logLine(F("[PANIC] Forcing reboot"));
      delay(50);
      ESP.restart();
    }
  }

  statusLed.update();

  delay(5);
}
