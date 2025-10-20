#pragma once

// ---- MQTT broker (Home Assistant typically runs Mosquitto add-on) ----
#define MQTT_BROKER_HOST   "YOUR_MQTT_BROKER_IP"
#define MQTT_BROKER_PORT   1883
#define MQTT_USERNAME      "YOUR_MQTT_USERNAME"
#define MQTT_PASSWORD      "YOUR_MQTT_PASSWORD"

// ---- Device identity & topics ----
#define DEVICE_NAME        "esp32-32u-poolcover"
#define BASE_TOPIC         "poolcover"      // poolcover/...

// Telemetry topics (published by device)
#define TOPIC_AVAIL        BASE_TOPIC "/tele/availability"  // "online"/"offline" (retained)
#define TOPIC_STATE        BASE_TOPIC "/tele/state"         // JSON retained
#define TOPIC_HEARTBEAT    BASE_TOPIC "/tele/heartbeat"     // JSON not-retained
#define TOPIC_PONG         BASE_TOPIC "/tele/pong"          // JSON reply to ping
#define TOPIC_LOG_STREAM   BASE_TOPIC "/tele/log_stream"    // log stream (non-retained)
#define TOPIC_LOG_LAST     BASE_TOPIC "/tele/log_last"      // single last line (retained)
#define TOPIC_LOG_BLOB     BASE_TOPIC "/tele/log_blob"      // multi-line buffer (retained)


// Commands (subscribed by device)
#define TOPIC_CMD          BASE_TOPIC "/cmnd"               // JSON { "cmd": "...", ... }

// We also subscribe to HA status to infer reachability
#define TOPIC_HA_STATUS    "homeassistant/status"           // "online"/"offline"

// Heartbeat / ping cadence
#define HEARTBEAT_SEC      15
