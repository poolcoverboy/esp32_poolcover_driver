#pragma once

// OTA credentials & identity.
// Update OTA_PASSWORD to a strong passphrase before flashing.
#define OTA_PASSWORD "YOUR_OTA_PASSWORD"

// Hostname advertised to the network/browser.
// Leave as-is to reuse the Wi-Fi hostname.
#define OTA_HOSTNAME "esp32-32u-poolcover"

// OTA service port (default for ArduinoOTA = 3232).
#define OTA_PORT 3232
