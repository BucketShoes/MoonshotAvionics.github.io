# Radio Architecture — RadioLib Hopping Rewrite

## Overview

Rewrite radio handling for both rocket and base station using RadioLib, replacing the current low-level SPI implementation. The primary goals are reliability, hopping support from day one, and an architecture that can accommodate relays, multiple modulations, and power management without later structural changes.

This document covers the target architecture. Implementation is phased (see bottom). It supersedes the previous `Radiolib Hopping.md` and `Hopping radio slot structure.md`.

---

## Core Timing Model

**Slot-based, drift-followed, no GPS discipline.**

- Slot duration: 200ms (configurable constant in shared `radio_config.h`)
- Slot index: monotonically incrementing counter on each device
- Rocket sets its own clock. Base station follows drift via EMA.
- No handshake for initial sync — base station acquires by listening (see Cold Start Acquisition).
- GPS time is intentionally **not** used for slot discipline. The system must work without GPS lock, indoors, after a hard landing and reboot, and with GPS powered down. GPS timing may be added later as an optional enhancement but must not be assumed by this architecture — doing so would bake in bad assumptions about GPS availability, prevent sleep modes, and create additional failure modes.

**Rocket timing rule:**  
At the start of each new slot, act if and only if the current time is within ±10ms of the intended slot boundary. If not within that window, skip and wait for the next slot. Never send late — a late transmission corrupts drift calculations on the base station and reaches nobody.

**Base station timing rule:**  
Start listening 20ms early (before the expected slot start, on the next hop channel, next modulation). Start sending a command 5ms after the slot start. When a packet is received, compute `rxdone_time − calculated_airtime = estimated_tx_start`. Feed this into a slow EMA to track the rocket's clock. Apply separately per packet type (different airtimes for different modulations).

**Shared constants (`radio_config.h`):**  
`SLOT_DURATION_US`, `RX_LISTEN_TIMEOUT_MS` (150ms), `DRIFT_LEAD_US` (20ms), `CMD_SEND_DELAY_US` (5ms), `SLOT_OVERRUN_MAX` (2), `TIMING_GATE_MS` (10ms), relay random re-send interval bounds, and all channel/modulation parameters.

---

## Frequency Hopping

- Default: 23 channels, selected from AU915 upper band (channels 64–86, ~902–914 MHz, 200kHz spacing)
- Hop sequence: Fisher-Yates shuffle of channel indices, hardcoded seed, deterministic pure function of slot index
- All rocket TX hops. **Command channel is fixed** — see Command Path.
- Hop function: `channel = HOP_SEQUENCE[slot_index % NUM_CHANNELS]`
- `NUM_CHANNELS` (23) and the full hop sequence are defined in `radio_config.h`
- FCC extension: increase `NUM_CHANNELS` to 50+ with a longer Fisher-Yates seed; same function, longer period

The hop sequence length and slot sequence length must be **coprime** so all (slot_type, channel) combinations are eventually visited.

---

## Slot Sequence

A fixed repeating sequence of slot types, defined in `radio_config.h`. Length must be coprime with `NUM_CHANNELS`.

**Slot types:**
- `WIN_TELEM` — rocket TX telemetry, base RX. Hopped channel, telemetry modulation.
- `WIN_CMD` — base TX command OR relay TX, rocket RX. Fixed command channel, command modulation.
- `WIN_LR` — rocket TX long-range location. Hopped channel, LR modulation. Always overruns into the following WIN_CONTINUE.
- `WIN_CONTINUE` — explicit no-op. Allocated immediately after any slot that always overruns (e.g. WIN_LR). The radio is left in whatever state the previous slot put it in; no slot action is taken and the overrun counter is not incremented.
- `WIN_OFF` — radio in standby. Used in low-power cycle variants.

`WIN_CONTINUE` is the mechanism for handling always-overrunning slots. Variable-length slot arithmetic (`if slot 3 is 2x long, what slot are we in at time T?`) is avoided entirely — slot index is always `floor(time / SLOT_DURATION) % SLOT_SEQUENCE_LEN`.

**Current sequence (subject to tuning):**
```
TELEM, TELEM, TELEM, CMD, TELEM, TELEM, TELEM, CMD, TELEM, LR, CONTINUE, CMD
```
Length 12 (coprime with 23 channels → full coverage every 276 slots ≈ 55 seconds).

**Slot index derivation:**  
`slot_type = SLOT_SEQUENCE[slot_index % SLOT_SEQUENCE_LEN]`  
`hop_channel = HOP_SEQUENCE[slot_index % NUM_CHANNELS]`  
Both are pure functions of `slot_index`. Nothing else is needed.

**Overrun tracking:**  
Record what operation was started in the current slot. When an operation finishes (RxDone IRQ, TxDone IRQ, or timeout), compute airtime from that record. Do not assume the radio is idle at a slot boundary — check IRQ flags.

---

## Slot Overrun Handling

Some operations legitimately take longer than one slot (e.g. a 400ms command receive in a 200ms slot, or WIN_LR always overruns into WIN_CONTINUE).

Rules:
1. At each slot boundary, check if radio is still active: TxDone not fired, or `PreambleDetected`/`HeaderValid` set without `RxDone`.
2. If active, skip the slot action. Increment an overrun counter.
3. After `SLOT_OVERRUN_MAX` (default 2) consecutive unexpected overruns, call `setStandby()` and resume normal slot actions. This is the safety cutoff for a stuck radio.
4. `WIN_CONTINUE` is an explicit no-op — the overrun counter is **not** incremented for it. It is expected to be mid-packet.
5. Commands usually produce dead air (no command pending → RX with 150ms timeout). If `PreambleDetected` fires before timeout expiry, the radio keeps listening past 150ms. Let it. If it overruns into the next slot, apply rule 2 for that next slot.

**IRQ usage:**  
SX1262 exposes `PreambleDetected` and `HeaderValid` as first-class IRQs on DIO1. Use `setDio1Action()` with a combined IRQ mask including these plus `RxDone`, `TxDone`, `Timeout`, and `CrcErr`. Poll `getIrqFlags()` at slot boundaries — non-blocking, ~10µs.

```cpp
// Setup
radio.setDio1Action(onDio1);
radio.setDioIrqParams(RADIOLIB_SX126X_IRQ_ALL, RADIOLIB_SX126X_IRQ_ALL);

// At each slot boundary (non-blocking)
uint16_t flags = radio.getIrqFlags();
bool preamble  = flags & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED;
bool header    = flags & RADIOLIB_SX126X_IRQ_HEADER_VALID;
bool rxDone    = flags & RADIOLIB_SX126X_IRQ_RX_DONE;
bool txDone    = flags & RADIOLIB_SX126X_IRQ_TX_DONE;
bool timeout_  = flags & RADIOLIB_SX126X_IRQ_TIMEOUT;
```

The DIO1 ISR only sets a volatile flag. All logic runs in `loop()`.

---

## Modulations

| Slot type | Modulation | Channel |
|---|---|---|
| WIN_TELEM | Configurable SF (default SF9), BW 125kHz | Hopped |
| WIN_CMD | LoRa SF9, BW 125kHz | Fixed |
| WIN_LR | LoRa SF12, BW 125kHz | Hopped |
| WIN_CONTINUE | — | — |
| WIN_OFF | standby | — |

GFSK is a future option for WIN_TELEM. Narrowband GFSK may outperform LoRa at range; wideband GFSK may allow multi-page telemetry per slot. Neither is defined yet — bench testing needed. The architecture must not assume telemetry will always be LoRa.

Exact modulation parameters are **configurable via `SET_RADIO_PARAMS` command** and should not be hardcoded in logic. Use named constant structs from `radio_config.h`. Each slot type has its own modulation config; they may differ and will change.

---

## Telemetry Header — Slot Sequence Byte

Add 1 byte to the 0xAF telemetry header: `uint8_t slot_seq_index` = `slot_index % SLOT_SEQUENCE_LEN` at time of transmission.

This allows the base station to:
- Determine which slot in the sequence the rocket believes it's in
- Derive which hop channel it should be on (cross-check against received channel)
- Anchor slot tracking after cold-start acquisition

The received channel is known from context (the frequency we were listening on); it does not need to be encoded in the packet.

---

## Long-Range Packets

WIN_LR packets are extremely payload-limited. At SF12/BW125, airtime for a few bytes exceeds 400ms — always overruns the 200ms slot, hence always followed by WIN_CONTINUE.

- Payload: ~3 bytes — packed GPS coordinates + flags. Exact format TBD.
- Only 1 LR slot in the sequence. The receiver knows it's an LR packet by its modulation; the channel is confirmed by hop function + slot index.
- Acquisition via LR packets is architecturally possible but not implemented in phase 2. Telemetry-based acquisition is the primary path.

---

## Command Path

**Fixed channel, not hopped.** The command channel is the same for all base stations and relays. This is the only time the rocket is guaranteed not to be transmitting.

**Rocket during WIN_CMD:**  
Switch to fixed command channel and command modulation. Start RX with 150ms timeout. If `PreambleDetected` fires: keep listening, do not enforce slot boundary until packet completes or safety cutout triggers. If `RxDone` fires: process command, resume slot tracking. If timeout fires with no preamble: resume next slot normally.

**Base station during WIN_CMD:**  
- If command queued: wait 5ms from slot start, TX. No CSMA — timing must be exact. Accept that collisions with relay traffic are possible; rely on retry.
- If no command queued and not in relay-send window: start RX with 150ms timeout (same overrun logic). Log any received relay packet; do not re-relay.
- If in relay-send window: TX relay packet (see Relay section).

**Command queue:**  
Replace the current boolean flag with a proper FIFO queue. Multiple commands can be enqueued. Each is sent in the next available CMD slot. Commands include nonce for deduplication (re-sending the same nonce means the rocket processes only the first copy it hears).

---

## Relay Architecture (Phase 3)

All base stations hear the rocket. Relay allows a base station that heard the rocket to share that data with others that didn't.

**Relay mechanism:**  
Each base station tracks "last heard telemetry." After a random delay of 120–300 seconds (re-randomised after each send), transmit that record in the next WIN_CMD slot.

Receiving bases: log relay packet for client serving. Do not re-relay. Do not feed into drift EMA.

**Relay packet format (not 0xAF):**  
Distinct packet type prefix. Contains:
- Original telemetry payload
- RSSI/SNR as heard by the relaying base station
- Timestamp of original reception
- Relay base station ID
- Slot index and channel it was originally heard on

**Collision handling:**  
No CSMA — we can't wait without missing the timing. Send blind. Random intervals reduce collision probability. This is eventual-delivery, not time-critical.

**Rocket ignores relay packets:**  
Relay packets have a distinct magic byte / packet type. Rocket's command handler only acts on packets addressed to it.

---

## Cold Start Acquisition

Base station starts with no timing information.

1. Pick a random channel from the hop sequence (any of the 23).
2. Set telemetry modulation.
3. Start RX with very long timeout (SX1262 supports ~262 seconds via max timer value; alternatively, loop 150ms listens without changing channel).
4. Wait for a telemetry packet.
5. On receipt: read `slot_seq_index` from header. Derive expected hop channel for that slot index and confirm it matches the received channel. If consistent: lock to this timing. Feed `rxdone_time − airtime` into drift EMA as first sample.
6. Begin normal slotted operation with lead/follow timing.

**Expected acquisition time:**  
(SLOT_SEQUENCE_LEN / num_telem_slots) × NUM_CHANNELS × SLOT_DURATION  
≈ (12 / 8) × 23 × 200ms ≈ 7 seconds typical.

Because the hop sequence and slot sequence lengths are coprime, listening on any single channel will eventually catch a telemetry slot. No dedicated beacon or lighthouse channel needed.

---

## Shared Configuration File

`radio_config.h` at the repository root, included by both firmwares as `../radio_config.h`.

Contains:
- All timing constants
- Hop sequence array and `NUM_CHANNELS`
- Slot sequence array and `SLOT_SEQUENCE_LEN`
- Channel frequency table (AU915 upper band)
- Modulation parameter structs for TELEM, CMD, LR
- Magic bytes / packet type identifiers
- Relay timing bounds
- Any other constant shared between firmwares

Per-device configuration (TX power, relay enable, etc.) stays in the respective `config.h`.

---

## Blocking Budget

Complies with the non-blocking rules in CLAUDE.md. Phase 1 relaxes the main loop limit to 10ms worst case; radio operations should consume ≤1ms of that.

- SPI transactions (RadioLib calls): ≤1ms typical, ≤3ms worst case. Acceptable.
- IRQ flag reads (`getIrqFlags()`): ~10µs. Always acceptable.
- `setStandby()`: ≤500µs. Acceptable.
- Starting TX/RX (`startTransmit()`, `startReceive()`): ≤1ms. Acceptable.
- **Never call `*_BLOCKING` or `waitBusy` from `loop()`**, even from a non-armed path. These remain init-only in `setup()` paths.
- Safety cutout `setStandby()` at 2-slot overrun: bounded, acceptable.

---

## Implementation Phases

### Phase 1 — RadioLib port, shared config, command queue
Port rocket and base station radio to RadioLib. Introduce `radio_config.h`. Replace boolean command flag with queue. Retain existing single fixed modulation, no hopping. Validate bench timing and IRQ handling via `PreambleDetected`/`HeaderValid`. Reduce code duplication between rocket and base station.

### Phase 2 — Hopping, passive sync, multiple modulations
Add hop function. All rocket TX hops. Add `slot_seq_index` byte to 0xAF header. Add WIN_LR + WIN_CONTINUE to slot sequence. Add cold-start acquisition on base station. Add drift EMA on base station. Establish fixed command channel. Define LR modulation parameters.

### Phase 3 — Relays, HMAC, power saving
Relay packet format and base station relay TX/RX. Replace per-packet CRC with HMAC-32 (commands already have full HMAC + nonce). Sleep/low-power cycle variants (alternate slot sequences with WIN_OFF). "Quiet" command for post-landing battery saving.

### Phase 4 — Multiple rockets, FCC band, GPS timing option
Slot-phase offset per rocket. Extend to 50 channels for FCC (same hop function, longer period). GPS-disciplined timing as optional enhancement (does not replace drift following; adds a bit flag "I have GPS time" to header; receivers use it as a hint only).

---

## Open Questions

- **GFSK vs LoRa for telemetry.** Narrowband GFSK may outperform wideband LoRa at range. Needs bench testing with SX1262 at various BW settings. Architecture supports swapping without structural change.
- **LR packet payload.** 3-byte packed GPS format not yet defined.
- **Multi-page GFSK telem.** If wideband GFSK fits multiple pages per slot, the one-page-per-slot assumption needs revisiting. Defer to Phase 2 evaluation.
- **WIN_CONTINUE and overrun counter.** WIN_CONTINUE slots do not increment the overrun counter; only unexpected overruns do. Confirm the safety cutout does not count WIN_CONTINUE as an overrun.
- **`Hopping radio slot structure.md`** — superseded by this document. Can be deleted or archived.
