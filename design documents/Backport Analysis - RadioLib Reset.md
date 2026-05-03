# Backport Analysis: Raw Driver → RadioLib Reset

**From:** 417c353 (raw sx1262 driver, abandoned)  
**To:** 0ca2d1d (RadioLib, currently checked out)  
**Purpose:** Identify non-radio features to backport before reimplementing hopping on RadioLib.

---

## Summary Recommendation

**Backport to the RadioLib version.** The raw driver branch accumulated severe architectural issues, sync reliability problems, and feature regressions during debugging. The non-radio features added are significant and worth preserving. The hopping plan (`Radiolib Hopping.md`) already covers the radio rewrite; the backport below is everything *else*.

**Backport order (critical path):** Pyro → Flight → Config/Defaults → Telemetry → BLE → Board abstraction

---

## A) Non-Radio Changes to Backport

### 1. Pyro System (NEW — `pyro.cpp`, `pyro.h`)
**Effort: Easy | Priority: CRITICAL**

Complete hardware-backed pyro channel control using ESP32 RMT peripheral:
- RMT-timed pulses (1µs resolution, loop_count repeats in hardware)
- Three GPIO outputs with latching (prevents re-fire across loop passes)
- Continuity sensing via digitalRead on sense pins with pulldowns
- High-voltage sense via ADC (10k:100k divider, 2750mV threshold)
- `PyroState` struct: fires, active channel, continuity, HV, duration

**Why it matters:** Pyro timing is now hardware-guaranteed, not loop-dependent. Decoupled from CPU load, watchdog resets, logging stalls.

**To backport:** Self-contained. Wire into `main.cpp` setup/loop. Add includes to `flight.cpp`, `commands.cpp`, `telemetry.cpp`.

---

### 2. Flight Phase Machine (`flight.cpp`, `flight.h`)
**Effort: Easy-Medium | Priority: HIGH**

Changes:
- **PHASE_APOGEE now used:** Enters for one loop (for logging), fires drogue, advances to PHASE_DROGUE
- **PHASE_GROUND_TEST:** ARM with flag bit[1]=1 → no flight transitions while armed (bench testing)
- **Main deploy latching:** `mainDeployFired` flag prevents re-fire on subsequent loops
- **Thrust coast snapshot:** On PHASE_COAST entry, freezes thrust ring buffer, sets `thrustLoraForce`/`thrustBleForce`
- **Extended ARM params:** 13-byte format adds `drogueFireMs` (u16) + `mainFireMs` (u16) to existing 9-byte format
- **Pyro channel routing:** `drogueChannel` (1–2), `mainChannel` (2–3) in `FlightConfig`
- **Refuse arm** if coast thrust snapshot pending LoRa send

**New fields needed in FlightState:** `mainDeployFired`, `thrustCoastEntryUs`  
**New fields needed in FlightConfig:** `drogueFireMs`, `mainFireMs`, `drogueChannel`, `mainChannel`

---

### 3. Thrust Curve Capture (NEW)
**Effort: Medium | Priority: MEDIUM**

200 Hz accelerometer ring buffer captured during boost, frozen on coast entry:
- `thrustBuf[2048]` int16_t ring, `THRUST_SAMPLE_RATE_HZ=200` (5ms period)
- `nonblockingThrust()` samples accel.x, wraps in ring
- Coast entry: freezes ring, sets `thrustLoraForce` and `thrustBleForce` flags
- **Page 0x0E:** Variable-length (header 6 bytes + encoded samples)
  - Two-pass encode: find min/max, then 8-bit `((val − min) * 255) / (max − min)`
  - SF-dependent sample count: SF≤7→200, SF8→100, SF9+→50 samples in LoRa packet
- New globals: `thrustBuf`, `thrustBufHead`, `thrustBufActive`, `thrustLoraForce`, `thrustBleForce`

**Why it matters:** Real boost acceleration curve for post-flight analysis. Coast detection confirmation. Diagnostic for misfire detection.

---

### 4. Board Abstraction (NEW — `board_config.h`)
**Effort: Easy | Priority: MEDIUM**

All board-specific GPIO in one file, selected via `-DBOARD_WIRELESS_TRACKER` or `-DBOARD_LORA32_V4`:
- **Wireless Tracker (V1.1, 8MB):** GPS RX=33/TX=34/RST=35 @ 115200, SDA=4/SCL=5, Pyro CH1=45/CH2=46/CH3=42, sense=39/40/41, HV_ADC=6
- **LoRa32 V4 (16MB+PSRAM):** GPS RX=38/TX=39/RST=42 @ 9600 (L76K), VGNSS_CTRL=34, SDA=17/SCL=18, Pyro CH1=45/CH2=47/CH3=48, sense=4/5/6, HV_ADC=3, FEM EN=2/CTL=7/PA=46
- New `[env:rocket_v4]` in `platformio.ini`

**To backport:** Create `board_config.h` with Tracker section, update `config.h`/`sensors.h` to `#include "board_config.h"`, set `-DBOARD_WIRELESS_TRACKER` in base env.

---

### 5. BLE Transport Enhancements (`ble.cpp`, `ble.h`)
**Effort: Medium | Priority: MEDIUM**

- **Log fetch flow control:** Hold last PDU in `fetchPendingBuf[514]` if `notify()` fails; retry next loop instead of dropping records
- **End-of-fetch marker:** Send `0xFF` (1 byte) instead of empty packet
- **TX power:** `NimBLEDevice::setPower(ESP_PWR_LVL_P9)` (+9 dBm max)
- **Thrust forced send:** `thrustBleForce` → coast entry forces 0x0E page to BLE once, even if unsubscribed
- **Diagnostics:** `rktFetchNotifyOk`, `rktFetchNotifyDrop`, `rktFetchBytesSent` counters per fetch
- **`volatile bool fetchActive`** (written by NimBLE task, read by main loop — already correct, formalised)
- **Page mask mirroring:** `bleSubPageMask`, `bleTxPhy` globals for use outside `ble.cpp`

**Why it matters:** BLE log fetch reliability — no gaps when mobile host can't keep up.

---

### 6. Command System Updates (`commands.cpp`, `commands.h`)
**Effort: Easy-Medium | Priority: CRITICAL**

- **CMD_FIRE_PYRO implemented** (was CMD_ERR_UNKNOWN):
  - 3-byte format: `ch` + `dur_ms` (u16) or 4-byte with `flags` (bit[0] = stay in current phase)
  - Calls `pyroFire(ch, dur)`, optionally forces PHASE_GROUND_TEST
- **CMD_ARM extended:** Now accepts 0, 9, or 13 bytes (13-byte adds `drogueFireMs` u16 + `mainFireMs` u16)
- **`lastValidCmdUs`:** Track when last valid command received — used for sync timeout / base watchdog
- **`rxPosInSlot`:** Position within slot at command RX (2ms units, 0–255), added to page 0x0A response — diagnostic for timing jitter

---

### 7. Telemetry Data Pages (`telemetry.cpp`, `telemetry.h`)
**Effort: Medium | Priority: HIGH**

- **Page 0x0A (cmd ack):** 9→10 bytes, added `rxPosInSlot`
- **Page 0x0B (flight status):** 8→6 bytes; pyro flags properly populated from `pyroState` (was hardcoded zeros)
  - Flags: [0–2]=ch1/2/3_fired, [4–6]=ch1/2/3_cont, [8]=hv_present
- **Page 0x0C (radio health):** 5→6 bytes; added `syncFlags` byte ([0]=in_sync, [6:1]=seqIdx)
- **Page 0x0E (thrust curve) NEW:** Variable-length, two-pass encoded (see section 3)
- **Page 0x0F (pyro status) NEW:** 6 bytes — continuity, HV, fired flags, active channel, duration, HV millivolts
- **LOGI enum reordered:** `LOGI_HEADER=0` first, added `LOGI_THRUST_CURVE`, `LOGI_PYRO_STATUS`
- **PAGE_CYCLE simplified:** Removed baro-heavy cycle; faster/more useful rotation
- **`buildStateFlags()`:** Now sets bits 5–7 from `pyroState` (was TODO)
- **Forced telem send:** `thrustLoraForce` triggers 0x0E on next WIN_TELEM regardless of cycle position

---

### 8. Serial, Boot, Power Changes (`main.cpp`, `config.h`)
**Effort: Easy | Priority: LOW**

**Rocket:**
- `Serial.setTxTimeoutMs(0)` — non-blocking serial (no USB host when flying)
- CPU frequency: 240 MHz → 80 MHz (power save; adequate for loop budget)
- WiFi deinit: `esp_wifi_stop()` + `esp_wifi_deinit()` on boot (RF block shared with BLE)
- Loop slot timing: added SLOT_THRUST, SLOT_PYRO, SLOT_ESKF entries

**Base Station:**
- Serial baud: 115200 → 2,000,000 (faster SD log fetch)
- PING command: `bsBuildPingCmdPacket()` sent every 60s keepalive
- BUSY-wait: `BlockingReadyWait()` spins on LORA_BUSY_PIN up to 2ms before TX (was assumed instant)
- BLE fetch diagnostics: `BLE_FETCH_VERBOSE` define

---

### 9. Config & Defaults (`config.h`, `platformio.ini`)
**Effort: Medium | Priority: MEDIUM**

Board-agnostic constants to adopt:
- `LORA_PREAMBLE`: 6 → **8** symbols (better CAD sensitivity)
- `LORA_LR_SF = 11`, `LORA_LR_CR` (long-range mode constants)
- Default channel: 65 → **3** (125kHz vs 500kHz)
- Default SF: 5 → **9** (better range for testing)
- `ROCKET_NO_BASE_HEARD_THRESHOLD_US = 130s` (>2 ping intervals, widens RX window if base silent)
- `PYRO_HV_PRESENT_MV = 2750`, `PYRO_HV_DIVIDER_RATIO = 11`
- `THRUST_BUF_SIZE = 2048`, `THRUST_SAMPLE_RATE_HZ = 200`
- `PAGE_THRUST_CURVE = 0x0E`, `PAGE_PYRO_STATUS = 0x0F`
- `BLE_LOGFETCH_MAX_PDU`: 502 → **514** (use full 517 MTU − 3 ATT header)
- Build flags: `-O2` (was `-Os`), `-std=gnu++17`
- Monitor DTR/RTS = 0 (avoid auto-reset on serial open)

**SKIP from config:** 15-slot sequence, 420ms slot duration — these are tied to the raw driver's slot machine and conflict with the RadioLib hopping plan's approach. Keep the 2-slot structure in the RadioLib version until the hopping rewrite.

---

## B) Features Not in the RadioLib Hopping Plan

These exist in the newer commit but aren't addressed in `Radiolib Hopping.md` or `Hopping radio slot structure.md`:

1. **Thrust curve (page 0x0E):** Orthogonal to hopping — works with any slot structure. Needs adding to packet format doc once backported.

2. **WIN_LR slot (every ~6s in 15-slot sequence):** Hopping doc mentions LR as "future" but proposes "every 63rd packet via hopping index" — a different mechanism. Not conflicting, but the two approaches need reconciling.

3. **PHASE_GROUND_TEST:** Not mentioned in hopping doc. Orthogonal — any window can receive ARM command.

4. **Configurable pyro channel routing per ARM:** Hopping doc assumes fixed assignments. New feature allows per-ARM config.

5. **Command RX slot position (`rxPosInSlot`):** Useful timing diagnostic for hopping. Not in hopping plan but should be — add to plan.

6. **`lastValidCmdUs` timeout:** Used for base-side watchdog / RX window widening. The hopping plan covers RX window widening but doesn't specify the trigger mechanism.

7. **BLE log fetch flow control:** Transport reliability, orthogonal to hopping.

---

## C) Radio-Specific Changes to Skip

These are in the newer commit but handled by RadioLib or the hopping plan — do not backport:

- Raw `sx1262_driver` vs RadioLib — this is the whole point of the reset
- DIO1 ISR / `syncAnchorUs` slot machine — RadioLib hopping plan covers this
- `WIN_BACKHAUL`, `WIN_MULTIPURPOSE`, `WIN_RDF`, `WIN_GFSK` window modes — future, unused
- 15-slot sequence / 420ms `SLOT_DURATION_US` — tied to raw driver's timing model
- `CMD_SET_SYNC` using `dio1CaptureVal` — defer until DIO1 infrastructure exists in RadioLib version
- Radio health page sync flags (0x0C byte 5) — defer until slot machine stable on RadioLib

---

## Backport Priority Table

| Feature | Effort | Priority | Action |
|---------|--------|----------|--------|
| Pyro system (`pyro.cpp/h`) | Easy | CRITICAL | Backport |
| CMD_FIRE_PYRO implementation | Easy-Med | CRITICAL | Backport |
| Flight phase machine (apogee, latching, test mode) | Easy-Med | HIGH | Backport |
| Extended ARM params (13-byte, fire durations) | Easy | HIGH | Backport |
| Telemetry pages (0x0A/0B/0C updates, 0x0F new) | Medium | HIGH | Backport |
| Thrust curve (ring buffer, page 0x0E) | Medium | MEDIUM | Backport |
| Board abstraction (`board_config.h`) | Easy | MEDIUM | Backport |
| BLE flow control + fetch reliability | Medium | MEDIUM | Backport |
| Config defaults (SF9, preamble 8, HV/thrust constants) | Medium | MEDIUM | Backport |
| Serial/boot/power changes (80MHz, WiFi deinit) | Easy | LOW | Backport |
| Base station PING + BUSY-wait | Easy | LOW | Backport |
| 15-slot sequence / 420ms slots | Hard | — | Skip |
| CMD_SET_SYNC DIO1 timing | Easy | — | Defer |
| Sync flags in page 0x0C | Easy | — | Defer |
| WIN_BACKHAUL / WIN_RDF etc. | — | — | Skip |
