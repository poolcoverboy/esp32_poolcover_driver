#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "StatusStore.h"
#include "RingLogger.h"

class HttpConsole {
public:
  using StatusCallback = void (*)(JsonDocument& doc);

  HttpConsole(StatusStore& store, RingLogger& logger);

  void begin();
  void update();

  void setStatusCallback(StatusCallback cb) { _statusCallback = cb; }

private:
  WebServer _server;
  StatusStore& _store;
  RingLogger& _logger;
  StatusCallback _statusCallback = nullptr;

  void handleRoot();
  void handleStatus();
  void handleLogs();
  void handleNotFound();
};
