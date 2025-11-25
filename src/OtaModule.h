#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include "ota_config.h"

class OtaModule {
public:
  using LogFn = void (*)(const String&);

  explicit OtaModule(LogFn logger = nullptr)
    : _log(logger) {}

  // Optional hostname override (defaults to OTA_HOSTNAME if nullptr)
  void begin(const char* hostname = nullptr) {
    if (hostname && hostname[0]) {
      _hostname = hostname;
    } else {
#ifdef OTA_HOSTNAME
      _hostname = OTA_HOSTNAME;
#else
      _hostname = nullptr;
#endif
    }
  }

  void update(bool wifiConnected) {
    if (!wifiConnected) return;
    if (!_initialized) {
      start();
    }
    ArduinoOTA.handle();
  }

private:
  LogFn _log = nullptr;
  const char* _hostname = nullptr;
  bool _initialized = false;
  uint8_t _lastProgress = 255;

  void start() {
    if (_initialized) return;
    if (_hostname && _hostname[0]) {
      ArduinoOTA.setHostname(_hostname);
    }
#ifdef OTA_PORT
    ArduinoOTA.setPort(OTA_PORT);
#endif
#ifdef OTA_PASSWORD
    ArduinoOTA.setPassword(OTA_PASSWORD);
#endif

    ArduinoOTA.onStart([this]() {
      const char* cmd = commandLabel();
      if (_log) {
        _log(String(F("[OTA] Update starting (")) + cmd + F(")"));
      }
    });

    ArduinoOTA.onEnd([this]() {
      if (_log) {
        _log(F("[OTA] Update completed"));
      }
      _lastProgress = 255;
    });

    ArduinoOTA.onProgress([this](unsigned int progress, unsigned int total) {
      if (total == 0) return;
      uint8_t pct = static_cast<uint8_t>((progress * 100U) / total);
      if (pct == _lastProgress) return;
      _lastProgress = pct;
      if ((pct % 10U) == 0U && _log) {
        _log(String(F("[OTA] Progress ")) + pct + F("%"));
      }
    });

    ArduinoOTA.onError([this](ota_error_t error) {
      if (!_log) return;
      const char* reason = "unknown";
      switch (error) {
        case OTA_AUTH_ERROR: reason = "auth"; break;
        case OTA_BEGIN_ERROR: reason = "begin"; break;
        case OTA_CONNECT_ERROR: reason = "connect"; break;
        case OTA_RECEIVE_ERROR: reason = "receive"; break;
        case OTA_END_ERROR: reason = "end"; break;
        default: break;
      }
      _log(String(F("[OTA] Error: ")) + reason);
    });

    ArduinoOTA.begin();
    _initialized = true;
    if (_log) {
      _log(String(F("[OTA] Ready on port ")) + OTA_PORT);
    }
  }

  static const char* commandLabel() {
    const int cmd = ArduinoOTA.getCommand();
    switch (cmd) {
      case U_FLASH: return "flash";
      case U_SPIFFS: return "spiffs";
#ifdef U_FS
      case U_FS: return "fs";
#endif
      default: return "unknown";
    }
  }
};
