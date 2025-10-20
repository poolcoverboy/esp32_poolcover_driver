// include/pins.h
#pragma once
#define PIN_BTN_UP        16   // Yellow
#define PIN_BTN_DOWN      17   // White
#define PIN_RELAY_PSU     26   // Green
#define PIN_RELAY_FWD     25   // Blue
#define PIN_RELAY_REV     27   // Brown
#define PIN_RELAY_EN      23   // Orange (series enable)
#define PIN_CLICK_IN      32   // Gray
#define PIN_CLICK_DEBUG   19   // Legacy debug LED (mirrored click input)
#define PIN_STATUS_LED    PIN_CLICK_DEBUG  // Shared status LED output

#ifndef RELAYS_ACTIVE_LOW
#define RELAYS_ACTIVE_LOW 1
#endif
