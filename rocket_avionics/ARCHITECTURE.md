# Rocket Avionics — Architecture

## Guiding principle

Each file owns one concern. Most changes should only require reading and editing one file.

---

## Files in this sketch

### `config.h`
All compile-time constants: pin numbers, timing thresholds, LoRa default settings, AU915 channel plan, TX scheduling constants, HMAC constants, packet type codes, command IDs, result codes, GPS encoding status codes, log constants. Also contains `txRateToIntervalUs()` as a static inline since it is a pure function of config values. No variables, no state.

**Edit this when:** changing hardware pins, adjusting timing constants, adding a new command ID, changing protocol constants.

---

### `globals.h`
`extern` declarations for system-level state defined in the `.ino` that multiple modules need to read or write: `InitState` enum, `nvs`, `logStore`/`logStoreOk`, `batteryMv`, `peaks` (`PeakValues` struct), `armRecordCounter`, TX rate/enable state, `gpsUartStarted`, `runningMaxLoopUs`.

**Edit this when:** adding a new cross-cutting global that doesn't belong to a specific module.

---

### `gps.h` + `gps.cpp`
GPS serial port (`HardwareSerial gpsSerial`), parsed GPS state (`GPSState gps`), UTC time (`utcTimeMs`), and all NMEA parsing (GGA position, RMC speed/course/time, GSA dilution). `nonblockingGPS()` drains the UART and calls the parsers.

**Edit this when:** adding support for new NMEA sentences, changing GPS state fields, fixing GPS parsing bugs.

---

### `radio.h` + `radio.cpp`
SX1262 radio hardware object, active channel/SF/power config (loaded from NVS), the TX/RX state machine (`nonblockingRadio()`), RSSI exponential moving average for channel-busy detection, fixed-schedule TX with CSMA deferral and a 1000ms hard ceiling, post-TX keepout jitter, DIO1 interrupt service routine, and helpers (`radioStartRx`, `radioStartTx`, `radioSetDownloadConfig`, `radioRestoreNormalConfig`, `updateActiveFreqBw`, `channelToFreqMHz`).

**Edit this when:** changing TX scheduling logic, CSMA behaviour, RSSI EMA parameters, adding a new radio mode, or changing channel plan.

---

### `commands.h` + `commands.cpp`
HMAC-SHA256 command authentication (truncated to 10 bytes), nonce replay protection, command dispatch (`executeCommand`), received-packet parsing (`processReceivedPacket`), and the blocking log download protocol (`executeLogDownload`, `sendLogChunk`, `listenForDownloadRequest`). Owns `hmacKey`, `hmacKeyValid`, `highestNonce`, and `lastAck`.

**Edit this when:** adding a new command, changing the log download protocol, modifying HMAC/nonce handling.

---

### `telemetry.h` + `telemetry.cpp`
All telemetry wire format code: `buildTelemetryPacket()`, individual data page builders (0x01–0x0D), the page rotation cycle, binary serialisation helpers (`writeU8`/`U16`/`S16`/`S32`/`U32`), `buildStateFlags()`, `encodeGpsFrac()`. Also owns flash log writing (`logDataPage`, `logPage`, `logReceivedCommand`) and the non-blocking log scheduler (`nonblockingLogging()`).

**Edit this when:** changing the telemetry packet format, adding or modifying a data page, changing log intervals or the page cycle order.

---

### `rocket_avionics.ino`
Top-level entry point. Defines all globals declared in `globals.h`. Contains `setup()`, `loop()`, and the subsystem functions that don't belong to a specific module: `nonblockingInit()` (hardware sequencing state machine), `nonblockingButton()`, `nonblockingBattery()`, `nonblockingPeakTracking()`, `nonblockingLoopStats()`. Also holds `DEFAULT_HMAC_KEY` (only used in `INIT_NVS`).

**Edit this when:** changing startup/init sequencing, boot defaults, loop order, or the slow-loop blame-tracking slots.

---

### `sensors.h` (header-only)
Direct-register SEN0140 v2 10DOF IMU driver: ADXL345 accelerometer (milli-g), ITG3200 gyroscope (0.1 deg/s), VCM5883L / HMC5883L magnetometer (milligauss), BMP280 barometer (Pa + cm MSL altitude + filtered vertical velocity). Everything is implemented as `static` / `inline` functions in the header. Sensor data exposed via `extern` globals `accelData`, `gyroData`, `magData`, `baroData`. `sensorsInit()` does blocking I2C setup at boot; `nonblockingSensors()` runs the per-sensor state machines each loop iteration.

---

### `sensors.cpp`
Defines the `AccelData`, `GyroData`, `MagData`, `BaroData`, and `SensorState` global instances. No other code — all implementation is in `sensors.h`.

---

### `flight.h` + `flight.cpp`
Flight phase state machine: IDLE → ARMED → BOOST → COAST → DROGUE → MAIN_DEPLOY → DESCENT → LANDED. Handles arming stability checks (10-second window, baro/accel/gyro thresholds), launch detection (sustained high-accel or altitude triggers), apogee detection (baro fast EMA reversal), main chute altitude threshold, and landing detection. Maintains two barometric EMAs (slow 30s for calibration, fast 1s for flight decisions). Exposes `flightState` (the full state struct), `flightTryArm()`, `flightDisarm()`, `flightGetPhase()`, `flightIsArmReady()`, `flightBaroOk()`, `flightAccelOk()`, and the `isArmed` convenience macro.

---

### `log_store.h`
Flash ring-buffer log store. Writes variable-length records (up to 254 bytes payload + 6-byte header: timestamp, SNR, length) to a dedicated flash partition, with an index partition for fast seeking. Records are written sequentially; old records are overwritten when the ring wraps. Supports a "protection point" (set on arm) to prevent overwriting flight-critical data during download. Exposes `begin()`, `writeRecord()`, `readRecordRaw()`, `getRecordCounter()`, `getVirtualPos()`, `setProtectionPoint()`, `clearProtectionPoint()`, `eraseLogs()`.

---

## Dependency graph (headers only — no cycles)

```
config.h
  └── (no deps)

globals.h
  ├── config.h
  ├── log_store.h
  └── Preferences.h

gps.h
  └── Arduino.h

sensors.h
  ├── Arduino.h
  └── Wire.h

flight.h
  ├── Arduino.h
  └── sensors.h

telemetry.h
  ├── config.h
  └── log_store.h

commands.h
  ├── Arduino.h
  └── config.h

radio.h
  ├── RadioLib.h
  └── config.h
```

Implementation files include the headers they need without creating cycles.

---

## Safety notes

- `nonblockingRadio()` calls `buildTelemetryPacket()` and `processReceivedPacket()` — keep both fast.
- `nonblockingLogging()` may block for ~30–50ms on sector erase crossings (infrequent; one per ~4KB of log data).
- `executeLogDownload()` is fully blocking — it is refused while armed.
- All functions called from `loop()` while armed must have `nonblocking` in their name.







notes on the split out of large single ino file:
The ~2278-line rocket_avionics.ino is now split into 10 files (7 new, 3 modified):

New files
FileWhat changed here
config.h - All #define constants — pins, timing, protocol codes, HMAC params, TX scheduling. Also txRateToIntervalUs() as a static inline.
globals.h - extern declarations for cross-cutting state owned by main: InitState enum, nvs, logStore, batteryMv, peaks, TX config, etc.
gps.h + gps.cpp - GPSState, gpsSerial, utcTimeMs, all NMEA parsing, nonblockingGPS().
radio.h + radio.cpp - SX1262 hardware, active channel/SF/power config, TX/RX state machine, RSSI EMA, TX scheduling, DIO1 ISR.
commands.h + commands.cpp - HMAC auth, nonce replay protection, command dispatch, blocking log download protocol.
telemetry.h + telemetry.cpp - All data page builders (0x01–0x0D), buildTelemetryPacket(), flash log writing, nonblockingLogging().
ARCHITECTURE.md - Description of every file including pre-existing ones (sensors.h/cpp, flight.h/cpp, log_store.h), the dependency graph, and safety notes.
Modified files
rocket_avionics.ino — now only contains setup(), loop(), the init state machine, battery/button/peaks/loop-stats functions, global definitions, and the default HMAC key.
flight.h — added #define isArmed (flightState.armed) so commands.cpp and telemetry.cpp can use it without importing the macro from main.