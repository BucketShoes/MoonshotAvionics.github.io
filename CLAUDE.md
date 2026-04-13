# Moonshot Avionics — Project Context for Claude

## What this is

A model rocket avionics and telemetry system built around ESP32-C3 (rocket) and ESP32 (base station). Communicates via SX1262 LoRa radio (AU915 channel plan). A web dashboard (JavaScript/HTML) runs on the base station and in-browser via BLE. This is a personal/hobby project, currently unflown but bench-tested before each change.

## Safety context

Most of this codebase is cosmetic or tracking-only (telemetry display, logging, voice callouts). However:

- **Pyro channels exist** (parachute ejection charges) and are triggered automatically by flight phase logic. Bugs here have real consequences.
- **The pyro decision path is**: `sensors.h/cpp` → `flight.h/cpp` (phase state machine) → pyro GPIO. Flight phase data also feeds into `telemetry.h/cpp` (logged and transmitted).
- **Flight warnings are noted in `flight.h` and `flight.cpp`** but the sensors feeding those decisions are upstream.
- **Arming is required** before any automatic pyro action. Ground test firing is command-only (FIRE PYRO command, while armed).
- **Future direction**: ESP32 will eventually be a telemetry/logging coprocessor alongside a dedicated STM32 primary flight computer. The ESP32 retains pyro capability as backup even then. Active control surfaces (loop timing constraints) are planned but not yet implemented.
- **When touching flight logic or sensor data used in flight decisions**, apply extra care and flag any tradeoffs or edge cases explicitly.

## Non-blocking rule

All functions called from `loop()` while armed **must** be non-blocking (naming convention: `nonblocking` prefix). `executeLogDownload()` is the only intentionally blocking call and is refused while armed. Sector erases in `nonblockingLogging()` block ~30–50ms (infrequent). Do not introduce blocking calls into the main loop.

## System overview

```
rocket_avionics/         ESP32-C3 firmware (primary focus)
base_station/            ESP32 base station firmware
docs/                    Web dashboard (JS/HTML/CSS)
design documents/        Specifications — see index below
```

### Rocket firmware modules (`rocket_avionics/`)

| File | Owns |
|---|---|
| `config.h` | All compile-time constants, pin numbers, protocol codes |
| `globals.h` | Cross-module extern declarations |
| `sensors.h/.cpp` | IMU drivers: ADXL345, ITG3200, VCM5883L, BMP280 |
| `flight.h/.cpp` | Flight phase state machine, arming, pyro decisions |
| `telemetry.h/.cpp` | Packet builders (0x01–0x0D pages), flash log writer |
| `commands.h/.cpp` | HMAC auth, nonce replay protection, command dispatch |
| `radio.h/.cpp` | SX1262 TX/RX state machine, CSMA, scheduling |
| `gps.h/.cpp` | NMEA parsing, GPS state |
| `log_store.h` | Flash ring buffer, index partition |
| `rocket_avionics.ino` | `setup()`, `loop()`, init state machine, globals |

Dependency order (no cycles): `config.h` → `globals.h` → sensor/gps modules → `flight.h` → `telemetry.h`, `commands.h`, `radio.h`

## Design documents index

Pull the relevant doc into the conversation when working in that area.

| Doc | When to consult |
|---|---|
| `Code architecture - Avionics.md` | Adding/moving files, understanding module boundaries, dependency questions |
| `flight phases.txt` | Any change to arming, launch detect, apogee, pyro triggers, landing detection |
| `Packet, log, and data formats.txt` | Wire protocol, data page formats, BLE GATT services, log record layout |
| `Lora command listing.txt` | Adding/modifying commands, HMAC/nonce handling, download protocol |
| `Ring buffer layout and log storage format.txt` | Flash storage, index partition, boot recovery, log erase |
| `Channel mapping.txt` | LoRa frequencies, future hopping system, GPS time sync |
| `avionics structure.txt` | Simple vs fancy flight model distinction (pyro uses simple model only) |
| `JS notes.txt` | Web dashboard: session rebuilding, log history loading, packet decoding |
| `voice callouts.txt` | Voice announcement logic and abbreviated number formatting |

## Key design decisions to preserve

- **Simple model only for flight decisions**: barometric EMA + accelerometer threshold. The Kalman/fusion data is for post-flight analysis and telemetry display only. Never route fancy-model data into pyro or phase decisions.
- **Two baro EMAs**: slow (30s period, arming/calibration), fast (1s period, flight decisions and telemetry header). Both always running.
- **Disarm semantics**: clears armed flag and sets phase to IDLE only. Does NOT reset EMAs, ground level, peaks, orientation, or launch time. Re-arming resets everything.
- **Arming stability**: 10-second continuous window. One bad reading resets that sensor's timer. Any sensor error in the last 10s blocks arming (except gyro/mag/GPS which are tracked but not required).
- **B+C launch detect window**: 5 seconds (not 10 — the spec doc has an uncorrected old value).
- **ARM command**: 0 bytes = force arm with defaults; 9 bytes = `uint16 boostAccelMg, uint16 boostAltM, uint16 coastAltM, uint16 mainDeployAltM, uint8 flags` (bit 0 = force).
- **State flags [9:11]**: `baro_ok`, `accel_ok`, `arm_ready` (not reserved — the packet format doc has an older version).
- **Fusion altitude in telemetry header**: 1-second baro EMA in metres MSL, not raw baro or GPS. `-32768` = invalid.
- **Log protection point**: set on ARM at 60 seconds before arm time. Ring buffer stops logging when full (does not overwrite protected region).
- **executeLogDownload() refused while armed** — it is fully blocking and bandwidth-heavy.
- **Relays**: All base stations are also relays (not yet implemented). Reference to a base station is the one the user is connected to. References to relays are other identical base stations placed far away acting in their role as a relay, sending traffic on backhaul.
- **Timed window mode (Phase 1 implemented)**: `CMD_SET_HOPPING` (0x13) enables 420ms windows alternating TX (odd index) and RX (even index). Base station auto-bootstraps at 2s boot and every 60s if no rocket packet heard. `FLASH_SYNC_TIMING=true` shows sync visually via LED. No channel changes yet — Phase 2 adds Fisher-Yates channel permutation per window. Window timing: `hopSyncOffsetUs` (rocket, micros) and `bsSyncOffsetMs` (base, millis) set at command TxDone/rxDone respectively. No drift correction yet — re-bootstrap if lost. SX1262 returns to standby automatically after single TX and after timed RX (timeout or rxDone); no explicit standby calls needed.

## Known TODOs in the codebase

- Rocket BLE char 0002 result reporting is noted as wrong in the packet format doc (TODO `@@@@`).
- Rocket BLE char 0005 log fetch is missing timestamp and SNR per record (TODO `@@@`).
- SET RELAY RADIO command (0x30) BW param is unresolved (TODO in command doc).
- Timed window Phase 2: add frequency hopping per window (Fisher-Yates, 53 BW125 channels, seed 0, `CMD_ENABLE_HOPPING`).

## What's not yet documented (gaps)

- **Base station firmware** has no architecture doc equivalent to `Code architecture - Avionics.md`.
- **BLE implementation** (`ble.h/.cpp`) has only `rocket ble.md` — no equivalent base station BLE doc.
- **Hardware**: no doc covering ESP32-C3 memory limits, I2C bus layout, pin assignments (partially in `config.h`), or SEN0140 v2 IMU board specifics.
- **Test/validation approach**: no doc on how bench testing is done, what a pre-flight check looks like, or what regression testing exists.
- **Relay devices**: mentioned in commands and backhaul packet format but no architecture doc.

## Keep updated
- When changing functionality, consider if it should also require an update to the design docs but do not make frivolous changes.
- If this CLAUDE.md file falls out of sync with a designs, update it.