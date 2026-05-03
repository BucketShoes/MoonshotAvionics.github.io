# RadioLib + Hopping — Radio Rewrite Design

This document supersedes the current `radio.h`/`radio.cpp` approach on both
rocket and base station. The current low-level SX1262 driver has proven
unreliable and architecturally unable to grow into the planned features
(hopping, multiple modulations, relays, long-range scan packets). The plan is
to roll back to RadioLib and rebuild forward with hopping in scope from day
one.

This document is the source of truth. Do not mirror the structure of the
current `radio.*` files. Do not import their assumptions. Mine the old
RadioLib-era code only as a reference for non-radio features that have been
added since.

---

## Goals and non-goals

**Goals**
- Reliable TX in TX windows, reliable RX in RX windows. This is the bar the
  current code does not clear.
- Frequency hopping per slot from the very first implementation.
- Architecture that supports multiple modulations, long-range packets,
  relays, sleep/power modes, multi-rocket, and HMAC payloads as later
  additions without restructuring.
- Shared configuration between rocket and base station via a single header.

**Non-goals (deferred, but design must accommodate)**
- GPS-disciplined slot timing. Explicitly rejected for now (see
  "GPS timing — rejected" below).
- HMAC over every packet. CRC + 0xAF packet type/rocket id magic stays for v1. Per-command HMAC
  with nonce is unchanged from current.
- Multi-rocket support. Single rocket only in v1, but hop sequence and slot
  schedule must be pure functions so multi-rocket is just a phase offset
  later.
- FCC band extension. AU only (23 hop channels). Same algorithm extends to
  larger N later.

---

## Phased plan

### Phase 1 — RadioLib rollback + hopping
- Roll back the radio layer to the old RadioLib-based implementation as a
  starting point.
- Move shared constants into `radio_config.h` consumed by both projects via
  `#include "../radio_config.h"` (or equivalent).
- Add hopping for all rocket TX. Command channel stays fixed.
- Implement slot types: TELEM, CMD, LR, CONTINUE, OFF.
- Implement passive drift following on the base station.
- Add 1 byte to the 0xAF telemetry header: slot index (0–255). Channel is
  inferred from where it was heard.
- Loop budget on rocket loosened from 1ms to 10ms worst case during this
  phase, leaving radio operations ~1ms headroom for SPI/contention. The
  pyro-safety constraint still applies — long blocking calls remain
  forbidden in the armed loop.

### Phase 2 — Bring missing features forward
- Audit features added between the old RadioLib code and current code that
  are unrelated to the radio layer (logging, BLE, voice callouts, packet
  pages, command set, etc.) and port them onto the new radio foundation.
- Done as a separate session — not part of the radio rewrite itself.

### Phase 3 — Relays, HMAC, power
- Relay re-broadcast of last-known telem during CMD slots.
- Relay-heard packet format with metadata (when heard, which relay,
  rocket→relay rssi/snr).
- Optional swap of CRC for HMAC on application payloads.
- Sleep / quiet-on-ground power modes; configurable telem rate fallback.

### Phase 4 — Multi-rocket
- Slot-phase offset per rocket ID. Same hop sequence, same slot schedule.

Phases 2–4 are out of scope for the immediate work. This document focuses
on phase 1 with the design choices needed to keep phases 2–4 implementable
without restructuring.

---

## Slot model

### Slot types
- `WIN_TELEM` — rocket TX telemetry, base RX. Hopping channel, telem
  modulation.
- `WIN_CMD` — base TX commands (or relay re-broadcast); rocket RX; other
  bases RX. Fixed command channel, command modulation.
- `WIN_LR` — rocket TX long-range location packet. Hopping channel, LR
  modulation. Always overruns the slot.
- `WIN_CONTINUE` — explicit no-op. Allocated immediately after any slot
  whose airtime is *always* longer than `SLOT_DURATION_US`. Nothing is
  scheduled here because the prior slot is still on air.
- `WIN_OFF` — radio in standby. Used to lower telem rate without changing
  slot schedule shape (`telem,telem,telem,cmd` becomes
  `telem,off,off,cmd`).

### Slot duration
- `SLOT_DURATION_US = 200_000`.
- Slot index = `(local_time_us / SLOT_DURATION_US) mod SLOT_SEQUENCE_LEN`.
- Pure modular arithmetic — no variable-length slots. Overruns are handled
  via `WIN_CONTINUE` placeholders, not by varying slot duration. A
  variable-length scheme would make "which slot are we in at time T?"
  ambiguous; the CONTINUE approach keeps the time→slot map a clean modulo.

### Slot sequence (initial v1 proposal)
Mostly TELEM with occasional CMD and one LR. Final exact sequence to be
tuned, but on the order of 20–30 entries. Example shape:

```
TELEM, TELEM, TELEM, CMD,
TELEM, TELEM, TELEM, CMD,
TELEM, TELEM, TELEM, LR, CONTINUE,
TELEM, TELEM, TELEM, CMD,
TELEM, TELEM, TELEM, CMD,
```

`SLOT_SEQUENCE_LEN` must be coprime with `NUM_HOP_CHANNELS` so that every
(slot_type, channel) pair eventually occurs. With `NUM_HOP_CHANNELS = 23`
(prime), any non-multiple-of-23 length works. CONTINUE counts as a slot for
length purposes.

A FINDME slot type was considered and dropped: a base station camping on
one channel in telem modulation acquires within ~6 s from telem alone (see
"Cold acquisition"), which is comparable to what FINDME would buy.

---

## Hopping

- `NUM_HOP_CHANNELS = 23`, drawn from a pool of 64 candidate frequencies
  via Fisher–Yates with a hardcoded seed.
- Hop function is pure: `channel(slot_index) = HOP_TABLE[slot_index mod 23]`
  (or equivalent — slot index is a global counter, not slot-within-sequence).
- Command channel is **fixed**, not hopped. It is a separate frequency
  unrelated to the hop set, configured at runtime via existing radio-config
  command.
- All rocket TX is on the hop channel. Only `WIN_CMD` (whoever transmits)
  uses the fixed command channel.

---

## Modulations

For phase 1, three named configs. Exact parameters to be locked in during
implementation, but the assignments below are the defaults.

- **Telem modulation** — GFSK. Bandwidth and bitrate configurable. This is
  what scanners camp on for cold acquisition, so it must be the most-used
  modulation.
- **Command modulation** — LoRa SF9 / BW125. Used for `WIN_CMD` (both
  base→rocket commands and relay re-broadcasts).
- **Long-range modulation** — LoRa SF12 / BW125. Used for `WIN_LR` only.
  Payload is extremely tight (~3 bytes for compressed GPS + flags); airtime
  exceeds one slot, hence CONTINUE follows it.

Modulation is selected by slot type via a lookup table. Do not bake
"modulation == telem" assumptions outside that lookup. The set of
modulations and their assignment to slot types is expected to grow
(narrowband/wideband telem, GFSK variants, etc.) — phase 1 should treat
this as a `slot_type → modulation_config` map, not hardcoded if-chains.

The configurable `SET RADIO PARAMS` command in v1 adjusts:
- TELEM modulation SF (hop channel, BW125 hardcoded)
- COMMAND modulation SF9/BW125 (fixed channel, hardcoded for now)
- TX power (configured)
- Backhaul/relay settings ignored in v1.

---

## Timing model

### Rocket timing — strict, simple
- Rocket has no drift correction and no smart waiting.
- At the start of every loop, check current slot index against last
  serviced slot. If it advanced, the new slot's action runs **only if the
  loop is within 10 ms of the slot start**. If later than 10 ms, skip the
  slot entirely.
- Sending at the wrong time corrupts the base station's drift EMA. **Late
  is worse than missed.** This is critical: do not "catch up" or delay an
  action to fit. If you missed it, you missed it.
- This may surprise an implementer expecting graceful-degradation or
  "always finish what you started" behaviour. It's deliberate. The base
  station relies on rocket TX being either on time or absent.

### Base station timing — passive drift following
- Base RX starts ~20 ms early on the upcoming slot's channel/modulation,
  *if* the radio is not still busy from the previous slot's action.
- Base TX (commands, relay re-broadcast) starts 5 ms after the slot start
  according to the base's own clock.
- On every successful RX, compute `inferred_tx_start = rxdone_time -
  airtime(packet_type, payload_len)` and feed it into a very slow EMA that
  represents "rocket time vs base time" offset. The EMA drives the 20 ms
  early-start bias for future slots.
- Different packet types have different airtime; airtime calc must use the
  actual packet's modulation and length, not assume a fixed value.

### Recording last operation
- The radio is not always idle at slot boundary — overruns are normal. The
  scheduler must record what the last operation was (TX vs RX, packet type,
  start time) so that when it completes, airtime can be computed and the
  drift EMA can be updated correctly. You can't infer this from "what
  should be on air right now" because of overruns.

---

## Overrun handling

- `WIN_CMD` is usually dead air, sometimes 300+ ms when a command is sent.
- `WIN_LR` is always 500+ ms.
- A single SX1262 RX with timeout naturally handles incoming long packets:
  the timeout governs how long it waits *for a packet to start*; once
  PreambleDetected/HeaderValid fires, the radio keeps listening through
  RxDone regardless of the timer. Use this — set RX timeout ~150 ms (less
  than slot duration), and don't force standby just because the timer has
  expired. Check `getIrqStatus()` for `PreambleDetected` / `HeaderValid` /
  `RxDone` before any standby decision.
- IRQ wiring: `setDio1Action()` with a mask covering RxDone, PreambleDetected,
  HeaderValid (and TxDone). Poll `getIrqStatus()` at slot boundary and when
  servicing the radio. RadioLib does expose these — earlier difficulties
  were not a RadioLib limitation.
- **Overrun budget: 2 slots.** If the radio is still active at the end of
  slot N+2, force standby and resume normal scheduling. This is a safety
  cutoff for "stuck busy" failures and should not be hit in normal
  operation.
- After a known-long slot (`WIN_LR`), the next slot is `WIN_CONTINUE` and
  is an explicit no-op. The radio is expected to still be active; the
  scheduler does nothing. CONTINUE only follows slots whose airtime is
  *always* longer than slot duration (LR). CMD overruns are handled by the
  generic 2-slot budget because most CMD slots are dead air and don't need
  a CONTINUE wasted on them.

### Why CONTINUE is a slot, not a runtime decision
The earlier instinct was "let the radio stay busy if a packet is coming
in." That works for opportunistic long packets (CMD) but conflates two
ideas: (a) a packet is in progress so don't interrupt, and (b) this slot
was always going to be too long, so don't schedule anything. Making (b)
explicit in the schedule simplifies reasoning — at any time T, slot index
is `(T / SLOT_DURATION_US) mod SLOT_SEQUENCE_LEN` with no ambiguity, and
you can pre-compute "is the next slot meaningful or is it a CONTINUE."

---

## Cold acquisition

- Base station picks a random hop channel and listens **continuously** in
  telem modulation. Implementation: single-shot RX with a very long
  timeout (`0xFFFFFF` ticks ≈ 262 s, then re-arm) — not slot-by-slot RX.
  We want the radio in RX, not bouncing in and out of standby.
- When a telem packet is received: read the slot index from the header
  byte → know which slot of `SLOT_SEQUENCE`. Combined with the channel
  it was heard on, this fully determines the (slot_index, hop_index)
  position. From that point on, follow drift normally.
- Telem density is high enough (~70%+ of slots) that with 23 channels and
  a coprime sequence length, a base camping on one channel hits a telem
  TX within roughly `slot_duration × num_channels / telem_fraction` —
  on the order of 6 s expected.
- Long-range packets *could* in principle be used for acquisition (they
  carry enough info), but in v1 we acquire from telem only. LR is for
  range, not sync.

---

## Commands and relays

### Command sending
- Command lives in the base station's queue.
- At the start of the next `WIN_CMD` slot, base TX begins 5 ms after slot
  start. No CSMA — there is no time to wait for a clear channel without
  losing slot timing. Send blind; if two senders collide, both lose, retry
  next CMD slot.
- Rocket is in RX on the command channel/modulation throughout every
  `WIN_CMD` slot. If a command is heard, process it; otherwise the slot
  ends.
- Same nonce on retry — rocket dedup is unchanged. If a command is sent
  multiple times because base wasn't sure it was heard, rocket only acts
  once.

### Relay re-broadcasts
- Every base station occasionally re-broadcasts its last-known-good
  telemetry on the command channel during a `WIN_CMD` slot. Random delay
  60–120 s between re-broadcasts (per base).
- Other base stations RX on the command channel during `WIN_CMD` slots
  unless they are themselves transmitting (a queued command, or their own
  re-broadcast turn). They do not need a "this is a relay packet" header
  bit — *whatever shows up in a CMD slot that isn't a command for the
  rocket is, by definition, either a command from this base or a relay
  re-broadcast from another.*
- The rocket also receives in `WIN_CMD` slots. Rocket dispatcher already
  ignores anything not addressed to it / not a valid command (first byte
  not the command magic, HMAC fails, etc.) — so relay re-broadcasts are
  naturally filtered out at the rocket. No additional flag needed.
- Relay packet **format** is distinct from a 0xAF telem packet. It carries:
  - The original telem payload (or a compressed subset),
  - Timestamp it was heard,
  - Relay base station ID,
  - Rocket→relay RSSI/SNR (the original reception quality, not the
    base→base reception quality).
  - Other relay-specific fields TBD.
- A relay-heard packet is **not** fed into the drift EMA — it's on command
  modulation, and the timing relationship to rocket TX is not what the
  drift calc expects.
- A relay-heard packet is logged at the receiving base station for later
  serving via the existing log/fetch path. It is not re-broadcast onward
  as a 0xAF telem packet.

### Why no header bit, no CSMA
The earlier suggestion of a "this is base→rocket vs base→base" header bit
and CSMA on relay TX was wrong:
- There is no shared header. Commands are commands; relay packets are a
  different format. The receiver disambiguates by parsing.
- CSMA requires waiting, which breaks slot timing. Send blind, accept
  occasional collisions, rely on the 60–120 s random delay to make
  collisions rare.

---

## GPS timing — rejected for v1

GPS-disciplined TDMA was considered and rejected. Reasons:
- Requires GPS lock before any radio comms — bench testing indoors
  becomes painful, and the rocket can't talk before lock.
- Forces GPS always-on, which conflicts with "let the rocket sleep on the
  ground after landing to save battery."
- A brief power interruption at landing (rocket nose-down in a ditch)
  could prevent re-acquisition; we then never hear from it again.
- Cold-start GPS lock can exceed the ~6 s passive-acquisition time we get
  from telem-camping anyway.
- It bakes in assumptions that make drift-following / sleep / quiet-mode
  features harder to add later. Easier to add GPS timing on top of a
  drift-following architecture than to retrofit drift-following onto a
  GPS-locked one.

The two ideas (GPS timing vs telem-based drift) are also less compatible
than they look — once GPS is "the clock," there's no scan to maintain, and
implementations tend to assume both ends always have GPS time. Avoid the
trap.

If GPS timing is added later, treat it as an optimisation that tightens
RX windows by a few ms and possibly shortens initial acquisition. Not a
replacement for the passive-sync mechanism.

---

## Header changes (phase 1)

Add 1 byte to the 0xAF telemetry packet header: **slot index** (0–255,
i.e. the slot's position in `SLOT_SEQUENCE`, *not* a global counter).
This wraps every `SLOT_SEQUENCE_LEN` slots, which is fine — it's only used
for acquisition, and the receiver knows the schedule.

Channel is **not** in the header — it's inferred from the frequency the
packet was heard on.

Existing fields (fusion altitude, state flags, etc.) are unchanged.

---

## Shared configuration

A single `radio_config.h` (or similarly named) lives at a path included
by both `rocket_avionics/` and `base_station/` (e.g. one level up, or a
shared `common/` dir). Both `#include "../radio_config.h"` or equivalent.

Contents:
- `NUM_HOP_CHANNELS` (23)
- `HOP_POOL_SIZE` (64)
- `HOP_SEED` (hardcoded)
- `SLOT_DURATION_US` (200_000)
- `SLOT_SEQUENCE[]` and `SLOT_SEQUENCE_LEN`
- RX timeout default (~150 ms)
- Base RX early-start margin (~20 ms)
- Base TX in-slot offset (~5 ms)
- Rocket on-time tolerance (10 ms)
- Overrun safety cutoff (2 slots)
- Cold-acquisition RX timeout
- Modulation parameter structs (telem, command, LR)
- Command channel frequency
- Default TX power

The Fisher–Yates hop table is generated at boot from `HOP_SEED` so both
sides produce the same table; no need to ship the table itself.

---

## What an implementer must not do

These are mistakes the conversation that produced this doc identified
explicitly. Reviewers should reject any code doing them:

1. **Do not delay an action to "send it late."** Rocket actions either
   happen within 10 ms of slot start or are skipped. Late TX poisons the
   drift EMA on the base. This may feel wrong if the implementer is
   pattern-matching on common "graceful degradation" idioms — it isn't
   graceful here, it's harmful.
2. **Do not use CSMA for command or relay TX.** No time to wait. Send
   blind on slot timing, accept collisions.
3. **Do not assume only one modulation exists.** Slot type → modulation
   is a lookup, not a hardcoded constant.
4. **Do not infer slot length from "what should be on air."** Track the
   actual last operation; let it complete; compute airtime from there.
5. **Do not use blocking radio init calls (`*_BLOCKING`) outside boot.**
   Existing project rule, unchanged. Now that we're on RadioLib, equivalent
   constraints apply to whatever RadioLib calls block — keep them in setup
   only.
6. **Do not bake in "we have GPS time" assumptions.** Even when GPS time
   is later added, the system must still work without it.
7. **Do not add a header bit for "this is a relay packet."** Relay packets
   are a distinct format; receivers parse to disambiguate.
8. **Do not run a CONTINUE slot's logic.** CONTINUE is an explicit no-op.
   It exists so that `(T / SLOT_DURATION) mod SLOT_SEQUENCE_LEN` stays
   well-defined when the prior slot is always too long.

---

## Open items / TODO during implementation

- Lock down exact GFSK telem parameters (bandwidth, bitrate, deviation,
  preamble length).
- Lock down exact LR LoRa parameters (SF, BW, CR, preamble; budget for
  3-byte payload airtime well under 2× slot duration).
- Confirm `0xFFFFFF`-tick continuous RX is the cleanest way to do
  cold-acquisition camping in RadioLib, vs a re-arm loop. Both work; pick
  one.
- Decide exact `SLOT_SEQUENCE` contents and length. Must be coprime with
  23 (any non-multiple of 23 is fine). Must contain exactly one LR + one
  CONTINUE. Several CMD slots distributed evenly. Rest TELEM.
- Confirm RadioLib's airtime API matches the modulation we configure
  (LoRa explicit-header airtime, GFSK airtime). We've been bitten by this
  before.
- Define the relay-heard packet binary format and add to the packet
  formats doc when implemented (phase 3).

---

## Flight-safety notes

- This rewrite touches the path that delivers flight-phase telemetry to
  the ground but does **not** alter the pyro decision path itself
  (`sensors → flight → pyro` is independent of the radio layer).
- The 10 ms loosened loop budget on the rocket applies during phase 1
  bring-up. The original 1 ms ceiling for armed flight must be restored
  before any armed bench test or flight; or the new radio code must be
  shown to fit within 1 ms per loop on the armed path. Treat 10 ms as
  development scaffolding, not a permanent change.
- Bench test the new radio with the rocket **unarmed** first.
- Verify `executeLogDownload()` is still refused while armed after the
  rewrite — it's the canonical "must stay disabled in flight" path and
  is worth re-checking any time radio code changes.
- Verify per-slot config switching still completes within budget — the
  RadioLib equivalent of `applyCfgIfNeeded()` should be measured and
  added to the timing budget list in `main.cpp`.
