# Repository Guidelines

## Project Structure & Modules
- `src/` hosts runtime modules such as `StatusStore`, `WifiModule`, `AnalogController`, `RelaysModule`, `ClickCounter`, and `MqttModule`; keep new features isolated and wire them through `main.cpp`.
- `include/` centralizes shared headers and secrets; treat `pins.h` as the wiring source of truth and clone `wifi_config.h` from its template before flashing.
- `test/` mirrors module names for PlatformIO/Unity tests; place HA assets (`lablace.yaml`, `configuration_appedix.yaml`) at the repo root and update them whenever backend topics or payloads change.
- Hardware notes, architecture plans, and SOW details live in `README.md`; reference that doc before large changes.

## Build, Test & Development Commands
- `pio run` builds the `esp32c6` environment; run before every commit.
- `pio run --target upload` flashes firmware; confirm the correct USB device in PlatformIO.
- `pio device monitor --baud 115200` tails serial logs (mirrors MQTT `tele/log_*`).
- `pio test --environment esp32c6` executes automated tests; add `--filter test_<module>` to focus.
- `pio run -t clean` clears cached artifacts when switching branches or toolchains.

## Coding & Safety Conventions
- Follow the existing two-space indentation, brace placement from `main.cpp`, `CamelCase` classes, `camelCase` functions, and `SCREAMING_SNAKE_CASE` constants.
- Maintain module boundaries: UI, Wi-Fi, MQTT, relays, click counting, and persistence should stay independently testable.
- Enforce hard invariants in code reviews and tests: never energize both direction relays, always respect `[0, end]` limits, apply ≥1000 ms dead-time before reversing, and ensure PSU spin-up ≥1000 ms before motion.
- Implement `pos`/`end` persistence with NVS wear-leveling (rotating slots, CRC) per the SOW; flush on motion cadence and on stop.

## Testing Expectations
- Cover interlock, dead-time, limit clamps, panic behavior, MQTT connectivity, and persistence replay under power loss using PlatformIO tests or scripted simulations.
- For hardware-only checks (PC817 ISR, relay polarity, long-cable debounce), document manual steps and measured timings in PR descriptions.
- Validate HA integration by exercising MQTT commands, confirming retained state/log topics, and synchronizing the YAML dashboard cards with backend field names.

## Commit & PR Guidelines
- Use short, present-tense commit subjects that describe the main change (`Persist pos in NVS`).
- Include code, docs, and HA YAML updates in the same change when they must stay aligned.
- PRs must outline problem, approach, safety considerations, and proof of testing (`pio run`, `pio test`, hardware logs/screenshots).
- Link related issues or TODOs and call out follow-up work (e.g., moving click counting from simulation to ISR).
