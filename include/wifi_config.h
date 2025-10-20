#pragma once

// ***** REQUIRED *****
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

// ***** OPTIONAL HINTS *****
// If you know the exact AP channel, set 1..13; else leave 0 to auto-learn.
#define WIFI_CHANNEL_HINT 0

// Static networking (comment out to use DHCP)
#define WIFI_STATIC_IP        IPAddress(192, 168, 1, 175)
#define WIFI_STATIC_GATEWAY   IPAddress(192, 168, 1, 1)
#define WIFI_STATIC_SUBNET    IPAddress(255, 255, 255, 0)
#define WIFI_STATIC_PRIMARY_DNS  IPAddress(192, 168, 1, 1)
#define WIFI_STATIC_SECONDARY_DNS IPAddress(8, 8, 8, 8)

// If you want to pin to one AP BSSID (MAC), uncomment and fill it in.
// Example: {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF}
// #define WIFI_BSSID_HINT {0xDE,0xAD,0xBE,0xEF,0x01,0x23}
