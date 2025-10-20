#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <vector>
#include "mqtt_config.h"
#include "StatusStore.h"
#include "ClickCounter.h"
#include "AnalogController.h"

class MqttModule {
public:
  using LogFn = void (*)(const String&);
  using MaxRuntimeHandler = void (*)(uint32_t seconds);

  explicit MqttModule(StatusStore& store, LogFn logger = nullptr)
    : _store(store), _wifiClient(), _mqtt(_wifiClient), _log(logger) {}

  void begin() {
    _mqtt.setServer(MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    _mqtt.setCallback([this](char* topic, byte* payload, unsigned int len) {
      this->onMessage(topic, payload, len);
    });
    _mqtt.setKeepAlive(20);        // seconds
    _mqtt.setSocketTimeout(5);     // seconds
    _mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE);
    _lastConnTry = 0;
    _lastHeartbeat = 0;
    _lastStatePub = 0;
    _haLastSeen = 0;
    _haConnected = false;
    _cmdQueue.clear();
  }

  void setMaxRuntimeHandler(MaxRuntimeHandler handler) {
    _maxRuntimeHandler = handler;
  }

  void update(const char* modeStr,
              MotionState action,
              MotionState analogState,
              const char* analogLabel,
              bool setModeActive,
              bool panicActive,
              const ClickCounter& clicks,
              uint32_t safetyMaxRunSeconds,
              uint32_t safetyElapsedSeconds,
              bool safetyActive,
              uint32_t noClickGuardSeconds) {
    ensureConnected();

    const unsigned long now = millis();

    if (_mqtt.connected() && (now - _lastHeartbeat > HEARTBEAT_SEC * 1000UL)) {
      _lastHeartbeat = now;
      StaticJsonDocument<128> doc;
      doc["alive"] = true;
      doc["uptime"] = (uint32_t)(millis() / 1000UL);
      publishJson(TOPIC_HEARTBEAT, doc, /*retain=*/false);
    }

    if (_mqtt.connected() && (now - _lastStatePub > 1000UL)) {
      _lastStatePub = now;
      publishState(now,
                   modeStr,
                   action,
                   analogState,
                   analogLabel,
                   setModeActive,
                   panicActive,
                   clicks,
                   safetyMaxRunSeconds,
                   safetyElapsedSeconds,
                   safetyActive,
                   noClickGuardSeconds);
    }

    updateHaRow(now);

    if (_mqtt.connected()) {
      _mqtt.loop();
    }
  }

  MotionState desiredFromHA() const { return _haDesired; }
  void clearHaDesired() { _haDesired = MotionState::IDLE; }

  bool haConnected() const { return _haConnected; }
  bool isConnected() { return _mqtt.connected(); }

  bool hasPendingCommand() const { return !_cmdQueue.empty(); }
  String popCommand() {
    if (_cmdQueue.empty()) return String();
    String cmd = _cmdQueue.front();
    _cmdQueue.erase(_cmdQueue.begin());
    return cmd;
  }

  void publishLogLine(const String& line) {
    if (!_mqtt.connected()) return;
    _mqtt.publish(TOPIC_LOG_STREAM, line.c_str(), /*retain=*/false);
    _mqtt.publish(TOPIC_LOG_LAST, line.c_str(), /*retain=*/true);
  }

  void publishLogSnapshot(const String& blob) {
    if (!_mqtt.connected()) return;
    _mqtt.publish(TOPIC_LOG_BLOB, blob.c_str(), /*retain=*/true);
  }

private:
  static constexpr unsigned long HA_STALE_MS = 300000UL;  // 5 minutes

  StatusStore& _store;
  WiFiClient _wifiClient;
  PubSubClient _mqtt;
  unsigned long _lastConnTry{0};
  unsigned long _lastHeartbeat{0};
  unsigned long _lastStatePub{0};
  unsigned long _haLastSeen{0};
  bool _haConnected{false};
  MotionState _haDesired{MotionState::IDLE};
  std::vector<String> _cmdQueue;
  LogFn _log{nullptr};
  unsigned long _lastHaStaleLog{0};
  MaxRuntimeHandler _maxRuntimeHandler{nullptr};

  void ensureConnected() {
    if (_mqtt.connected()) return;

    const unsigned long now = millis();
    if (now - _lastConnTry < 2000UL) return;
    _lastConnTry = now;

    String clientId = String(DEVICE_NAME) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    String target = String(MQTT_BROKER_HOST) + ":" + String(MQTT_BROKER_PORT);
    if (_log) _log(String(F("[MQTT] Connecting to ")) + target);

    bool ok = _mqtt.connect(clientId.c_str(),
                            MQTT_USERNAME[0] ? MQTT_USERNAME : nullptr,
                            MQTT_PASSWORD[0] ? MQTT_PASSWORD : nullptr,
                            TOPIC_AVAIL,
                            1,
                            true,
                            "offline");

    if (!ok) {
      if (_log) _log(String(F("[MQTT] Connect failed, state=")) + _mqtt.state());
      _store.setStatus("HASS", "Waiting");
      return;
    }

    _mqtt.publish(TOPIC_AVAIL, "online", true);
    _mqtt.subscribe(TOPIC_CMD, 1);
    _mqtt.subscribe(TOPIC_HA_STATUS, 0);

    _haConnected = true;
    _haLastSeen = millis();
    _lastHaStaleLog = 0;
    _store.setStatus("HASS", "OK");

    if (_log) _log(String(F("[MQTT] Connected to ")) + target);
  }

  void onMessage(char* topic, byte* payload, unsigned int len) {
    String t(topic);
    String body;
    body.reserve(len + 1);
    for (unsigned int i = 0; i < len; ++i) body += static_cast<char>(payload[i]);

    const unsigned long now = millis();

    if (t == TOPIC_HA_STATUS) {
      String s = body;
      s.trim();
      s.toLowerCase();
      if (s == "online") {
        _haLastSeen = now;
        _haConnected = true;
        _lastHaStaleLog = 0;
        _store.setStatus("HASS", "OK");
        if (_log) _log(String(F("[MQTT] HA status -> online")));
      } else if (s == "offline") {
        _haConnected = false;
        _lastHaStaleLog = 0;
        _store.setStatus("HASS", "Waiting");
        if (_log) _log(String(F("[MQTT] HA status -> offline")));
      }
      return;
    }

    if (t == TOPIC_CMD) {
      StaticJsonDocument<256> doc;
      DeserializationError e = deserializeJson(doc, body);
      if (e) return;

      const char* cmd = doc["cmd"] | "";
      if (!cmd || !cmd[0]) return;

      String scmd(cmd);
      scmd.toLowerCase();

      if (_log) _log(String(F("[MQTT] Command received: ")) + scmd);

      if      (scmd == "open_auto"   || scmd == "open_manually") setHaDesired(MotionState::OPENING);
      else if (scmd == "close_auto"  || scmd == "close_manually") setHaDesired(MotionState::CLOSING);
      else if (scmd == "stop")                                      setHaDesired(MotionState::IDLE);
      else if (scmd == "set_open_here")    _cmdQueue.push_back("set_open_here");
      else if (scmd == "set_closed_here")  _cmdQueue.push_back("set_closed_here");
      else if (scmd == "enter_set_mode")   _cmdQueue.push_back("enter_set_mode");
      else if (scmd == "exit_set_mode")    _cmdQueue.push_back("exit_set_mode");
      else if (scmd == "set_max_runtime") {
        uint32_t seconds = doc["seconds"] | 0U;
        if (seconds == 0U) seconds = doc["value"] | 0U;
        if (seconds == 0U) seconds = doc["seconds_s"] | 0U;
        if (seconds > 0U) {
          if (_log) _log(String(F("[MQTT] Set max runtime -> ")) + seconds + F(" s"));
          if (_maxRuntimeHandler) {
            _maxRuntimeHandler(seconds);
          }
        } else {
          if (_log) _log(String(F("[MQTT] Invalid max runtime payload")));
        }
      }
      else if (scmd == "ping") {
        StaticJsonDocument<128> pong;
        pong["ok"] = true;
        publishJson(TOPIC_PONG, pong, /*retain=*/false);
      }

      _haLastSeen = now;
      _haConnected = true;
      _lastHaStaleLog = 0;
      _store.setStatus("HASS", "OK");
    }
  }

  void setHaDesired(MotionState s) {
    if (_haDesired == s) return;
    _haDesired = s;
    if (_log) _log(String(F("[MQTT] HA desired -> ")) + motionToStr(s));
  }

  void updateHaRow(unsigned long now) {
    if (!_mqtt.connected()) {
      if (_haConnected) {
        _haConnected = false;
        if (_log) _log(String(F("[MQTT] Broker disconnected")));
      }
      _store.setStatus("HASS", "Waiting");
      return;
    }

    bool stale = (now - _haLastSeen) > HA_STALE_MS;
    if (stale) {
      if (_haConnected) {
        _haConnected = false;
        if (_log) _log(String(F("[MQTT] HA heartbeat stale")));
        _lastHaStaleLog = now;
      } else {
        if (_lastHaStaleLog == 0 || now - _lastHaStaleLog >= 60000UL) {
          _lastHaStaleLog = now;
          if (_log) _log(String(F("[MQTT] HA heartbeat still stale")));
        }
      }
      _store.setStatus("HASS", "Stale");
      return;
    }

    if (!_haConnected) {
      if (_log) _log(String(F("[MQTT] HA heartbeat restored")));
    }
    _haConnected = true;
    _lastHaStaleLog = 0;
    _store.setStatus("HASS", "OK");
  }

  void publishState(unsigned long now,
                    const char* modeStr,
                    MotionState action,
                    MotionState analogState,
                    const char* analogLabel,
                    bool setModeActive,
                    bool panicActive,
                    const ClickCounter& clicks,
                    uint32_t safetyMaxRunSeconds,
                    uint32_t safetyElapsedSeconds,
                    bool safetyActive,
                    uint32_t noClickGuardSeconds) {
    StaticJsonDocument<512> doc;
    doc["mode"] = modeStr;
    doc["action"] = motionToStr(action);
    JsonObject analog = doc.createNestedObject("analog");
    analog["switch"] = analogLabel ? analogLabel : "Neutral";
    analog["motion"] = motionToStr(analogState);
    doc["set_mode_active"] = setModeActive;
    doc["panic"] = panicActive;
    doc["pos"] = clicks.position();
    doc["end"] = clicks.end();
    JsonObject wifi = doc.createNestedObject("wifi");
    wifi["ip"] = WiFi.localIP().toString();
    wifi["rssi"] = (int)WiFi.RSSI();
    bool brokerConnected = _mqtt.connected();
    doc["ha_connected"] = brokerConnected;
    doc["uptime"] = (uint32_t)(millis() / 1000UL);

    JsonObject safety = doc.createNestedObject("safety");
    safety["max_run_s"] = safetyMaxRunSeconds;
    safety["run_elapsed_s"] = safetyElapsedSeconds;
    safety["active"] = safetyActive;
    safety["no_click_guard_s"] = noClickGuardSeconds;

    _haConnected = brokerConnected;
    if (_haConnected) {
      _haLastSeen = now;
      _lastHaStaleLog = 0;
      _store.setStatus("HASS", "OK");
    }

    publishJson(TOPIC_STATE, doc, /*retain=*/true);
  }

  template<typename TJsonDoc>
  void publishJson(const char* topic, const TJsonDoc& doc, bool retain) {
    char buf[768];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n > 0) {
      _mqtt.publish(topic, buf, retain);
    }
  }

  static const char* motionToStr(MotionState s) {
    switch (s) {
      case MotionState::OPENING: return "OPENING";
      case MotionState::CLOSING: return "CLOSING";
      default: return "IDLE";
    }
  }
};
