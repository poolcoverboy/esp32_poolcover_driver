#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include "wifi_config.h"
#include "StatusStore.h"

class WifiModule {
public:
  using LogFn = void (*)(const String&);

  explicit WifiModule(StatusStore &store, LogFn logger = nullptr)
    : _store(store), _log(logger) {}

  void begin() {
    WiFi.persistent(true);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.setHostname("esp32-32u-poolcover");
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

#ifdef WIFI_STATIC_IP
    IPAddress primaryDns;
#ifdef WIFI_STATIC_PRIMARY_DNS
    primaryDns = WIFI_STATIC_PRIMARY_DNS;
#endif
    IPAddress secondaryDns;
#ifdef WIFI_STATIC_SECONDARY_DNS
    secondaryDns = WIFI_STATIC_SECONDARY_DNS;
#endif
    bool ok = WiFi.config(WIFI_STATIC_IP,
                          WIFI_STATIC_GATEWAY,
                          WIFI_STATIC_SUBNET,
                          primaryDns,
                          secondaryDns);
    if (!ok && _log) {
      _log(String(F("[WIFI] Failed to apply static IP config")));
    }
#endif

    _store.setStatus("Wifi", "Connecting");
    if (_log) _log(String(F("[WIFI] Starting connection")));
    _connected = false;
    _backoffMs = 1000;
    _lastAttempt = 0;
    _lastInfoPush = 0;

    // Optional hints
    _channel = (uint8_t)WIFI_CHANNEL_HINT;
#ifdef WIFI_BSSID_HINT
    {
      const uint8_t hint[6] = WIFI_BSSID_HINT;
      memcpy(_bssid, hint, 6);
      _haveBssid = true;
    }
#else
    memset(_bssid, 0, sizeof(_bssid));
    _haveBssid = false;
#endif

    startConnect(true);
  }

  void update() {
    const uint32_t now = millis();
    wl_status_t s = WiFi.status();

    if (s == WL_CONNECTED) {
      if (!_connected) {
        _connected = true;
        _backoffMs = 2000;
        _store.setStatus("Wifi", connectedStatus());
        if (_log) _log(String(F("[WIFI] Connected: ")) + connectedStatus());
        // Learn channel & BSSID on first success if not fixed
        if (_channel == 0 || !_haveBssid) {
          _channel = WiFi.channel();
          const uint8_t* cur = WiFi.BSSID();
          if (cur) { memcpy(_bssid, cur, 6); _haveBssid = true; }
        }
      }
      // periodic refresh (IP may change)
      if (now - _lastInfoPush > 30000UL) {
        _lastInfoPush = now;
        _store.setStatus("Wifi", connectedStatus());
      }
      return;
    }

    // not connected
    if (_connected) {
      _connected = false;
      if (_store.setStatus("Wifi", "Disconnected") && _log) {
        _log(String(F("[WIFI] Disconnected")));
      }
      _backoffMs = 1000;
      _lastAttempt = 0;
    }

    if (now - _lastAttempt >= _backoffMs) {
      _lastAttempt = now;
      startConnect(false);
      _backoffMs = min<uint32_t>(_backoffMs * 2, 60000UL);
    }
  }

  bool isConnected() const { return _connected; }

private:
  StatusStore &_store;
  LogFn _log = nullptr;
  bool _connected = false;

  uint8_t _channel = 0;
  uint8_t _bssid[6] = {0};
  bool _haveBssid = false;

  uint32_t _lastAttempt = 0;
  uint32_t _lastInfoPush = 0;
  uint32_t _backoffMs = 1000;

  void startConnect(bool /*first*/) {
#if defined(WIFI_SSID) && defined(WIFI_PASS)
    // If we don't yet know channel/BSSID, scan once to pick the best AP
    if (_channel == 0 || !_haveBssid) {
      int n = WiFi.scanNetworks(false, true);
      int bestIdx = -1; int bestRSSI = -127;
      uint8_t bestCh = 0; uint8_t bestBssid[6] = {0};
      for (int i = 0; i < n; ++i) {
        if (WiFi.SSID(i) == String(WIFI_SSID)) {
          int rssi = WiFi.RSSI(i);
          if (rssi > bestRSSI) {
            bestRSSI = rssi; bestIdx = i;
            bestCh = (uint8_t)WiFi.channel(i);
            const uint8_t *b = WiFi.BSSID(i);
            memcpy(bestBssid, b, 6);
          }
        }
      }
      if (bestIdx >= 0) {
        _channel = bestCh;
        memcpy(_bssid, bestBssid, 6);
        _haveBssid = true;
      }
    }

    if (_log) {
      _log(String(F("[WIFI] Attempting connection")));
    }
    _store.setStatus("Wifi", "Connecting");
    if (_haveBssid && _channel > 0) {
      WiFi.begin(WIFI_SSID, WIFI_PASS, _channel, _bssid, true);
    } else if (_channel > 0) {
      WiFi.begin(WIFI_SSID, WIFI_PASS, _channel);
    } else {
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
#else
# error "Please define WIFI_SSID and WIFI_PASS in include/wifi_config.h"
#endif // defined(WIFI_SSID) && defined(WIFI_PASS)
  }

  String connectedStatus() const {
    String ip = WiFi.localIP().toString();
    int32_t rssi = WiFi.RSSI();
    return String("OK (") + ip + ", " + rssi + " dBm)";
  }
};
