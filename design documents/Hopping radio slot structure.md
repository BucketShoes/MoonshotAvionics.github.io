# Radio timing setup

## 1. Goal and assumptions

The radios need tight timing for two main reasons:
- **Short listen windows** reduce battery usage. The rocket sits out in a field running off its
  own battery — it must spend as little time as possible in RX while still being reachable.
- **Frequency hopping** (future). Once per-packet hopping is added, both sides must know the
  channel and modulation the next packet will use. The command channel will not hop; the
  telemetry channels will. Modulation already changes per slot (SF/header/CR/preamble), so
  both sides already need slot-level agreement on radio params.

Assumption: crystals are good enough that drift between sync events is not a functional
concern for the intended mission duration. Explicit drift tracking is a future feature (see
§9); for now we rely on good crystals and manual base reboot when drift exceeds tolerance.

## 2. Slot model

A slot is a fixed time window during which exactly one transmission (or listen) is
scheduled. All slots are currently the same length (`SLOT_DURATION_US`, 420 ms) but
individual slot durations may vary in future — don't assume uniformity in new code.

The slot sequence is a repeating pattern indexed by slot number since the anchor. Current
pattern is a debug simplification `[WIN_TELEM, WIN_CMD]`; the production pattern will be a
longer interleaved mix of WIN_TELEM / WIN_CMD / WIN_LR / possibly WIN_OFF etc. For low-duty
operation, the unsynced rocket may eventually switch to a different sequence with a
higher proportion of WIN_CMD slots to aid resync.

Slot types in use:
- **WIN_TELEM** — rocket TX telemetry / base RX.
- **WIN_CMD** — base TX commands / rocket RX.
- **WIN_LR** — future: long-range low-rate, implicit header, no CRC, SF=11 code,
  LI (long-interleave) coding rate planned.
- **WIN_OFF / WIN_FINDME** — future.

Slot transitions may change modulation. The slot machine must assume different slots can
have different mod/pkt params; don't rely on the current coincidence that adjacent slots
share config.

## 3. Sync model

On boot both sides are unsynced.

- **Rocket**: wide listen in every WIN_CMD slot (`ROCKET_LONG_RX_TIMEOUT_US`, ≈400 ms of
  the 420 ms slot). Because the current sequence alternates WIN_TELEM / WIN_CMD, roughly
  half the slots are WIN_CMD — so for any single base-sync attempt landing at a uniform
  phase, there is ≈50% chance it overlaps with the rocket's listen window. (This is what
  "≈50% airtime" means in the sync context — it is *not* a modulation-duty-cycle figure.)
- **Base**: `BS_SYNC_BOOT_DELAY_MS` (2 s) after boot, sends a CMD_SET_SYNC on the command
  channel with `waitMs = 0` — it fires as soon as the radio is in standby. There is no
  shared slot clock yet, so no slot-alignment is attempted on this packet.
- **On TxDone** of the sync packet, the base sets its anchor.
  **On RxDone**, the rocket sets its anchor.
  These moments are simultaneous within propagation time (<< 1 µs at field ranges).
  Both sides set `syncSlotIndex = 1`, meaning the sync packet itself is treated as
  **out-of-sequence** — it is not part of the slot grid. The first real slot begins at
  the sync packet's end, and the very next loop iteration on each side acts on slot 0 of
  the new sequence.
- Sending CMD_SET_SYNC **always breaks any previous sync**. It must only be sent when
  either (a) we know we're unsynced this session, or (b) the user has explicitly
  commanded it from the UI.

Implementation detail: `radioSetSynced()` on the rocket resets `lastHandledSlotNum` to
0xFFFFFFFF so the first loop iteration after sync does act on slot 0 (not slot 1 or 2).
The exact mapping from slot index to slot type shifts with the sequence; nothing
load-bearing should assume "slot N is always type X".

Conceptually, the sync choice is - sending any sync (whether it works or not) ruins timing of any existing anchors; so don't send it unless you know you have no chance. However, once you've sent one, go for it until you get it working; then dont touch. The time limit is for FHSS rules on how often a single channel can be used (commands dont hop, only rocket packets), but ideally it would be nearly constant until we hear something back.

## 4. Sending sync — triggers

- **Automatic, internal** (base only, while `!bsSynced`):
  - First attempt fires `BS_SYNC_BOOT_DELAY_MS` after boot.
  - If still unsynced, retry every `BS_SYNC_RETRY_WALK_MS` (340 ms).
    This is deliberately **less than** `SLOT_DURATION_US` so successive attempts land at
    different phases of the rocket's slot cycle — each attempt sweeps offset by 340 ms
    from the previous, hitting a fresh phase of the rocket's listen window.
  - Cap at `BS_SYNC_TIGHT_RETRIES` (6) attempts in this tight mode.
  - After the tight-mode cap is exhausted, fall back to one attempt per `BS_SYNC_BACKOFF_MS`
    (120 s) indefinitely. This avoids hammering NVS with nonce writes.
  - All of this stops immediately if `bsSynced` becomes true.
- **User-initiated via BLE/WS**: queued through `queueCommandTx()` with whatever parameters
  the browser supplied (do not override `waitMs`). Same packet bytes, same re-anchor on
  TxDone — see §5 invariant.
- **There is NO silence-based auto-resync.** Long telemetry silence is treated as a
  signal-quality problem (distance, terrain, orientation) — not as evidence of lost sync.
  A prior "resync after 20/40 min of silence" feature was harmful (it broke still-valid
  syncs) and has been deleted. Do not reintroduce it.

## 5. Base-side timing discipline

- **Commands**: TX slightly *after* slot start — `BS_CMD_TX_OFFSET_US` (5 ms) into a
  WIN_CMD slot — to tolerate small clock offsets and ensure the rocket's listen window
  has opened.
- **Receive-type slots (WIN_TELEM, WIN_LR)**: start listening slightly *before* slot start —
  `BS_RX_EARLY_US` (10 ms) early — so small clock offsets and base-loop jitter don't cause
  the preamble to be missed. **This early RX must use the *upcoming* slot's modulation,
  not the current slot's.** The slot machine applies the upcoming slot's config just
  before firing the early RX.
- **Per-slot config switching**: `bsApplyCfgIfNeeded()` defers if the radio is not in
  STANDBY (an in-progress RX/TX must not be interrupted by mod/pkt-param SPI writes).
  Every slot conceptually sets its config; don't rely on the current-pattern coincidence
  that adjacent slots share modulation.
- **Safety cutoff**: if RX has been active for more than `RX_STUCK_MAX_SLOTS` (2) slot
  durations, force standby. Indicates a missed DIO1 IRQ or stuck DIO1 line.

**Re-anchor invariant** (load-bearing): the base re-anchors on TxDone of any CMD_SET_SYNC
packet, regardless of source. Detection is by byte inspection at dispatch —
`pktLen == 17 && pkt[2] == CMD_SET_SYNC (0x41)` sets `bsSyncTxInFlight`, which causes
`bsSetSyncedFromTx()` to run on TxDone. This must remain true for both internally
generated syncs and browser-forwarded syncs; do not narrow the check to a source flag.

## 6. Rocket-side timing discipline

- Rocket uses "simple timing": the first loop iteration after a slot boundary decides
  what to do this slot. No early-listen, no late-send on the rocket — it trusts the anchor.
- On sync, `radioSetSynced()` resets `lastHandledSlotNum` so the first loop iteration
  after sync does act on slot 0.
- **Never force standby mid-packet at a slot boundary.** If an RX is in progress
  (preamble detected or packet arriving), let it complete — the SX1262 returns to STANDBY
  on its own via RxDone/Timeout/Error IRQ. Forcing standby mid-packet loses the frame.
  The RX timeout parameter is only to catch "no preamble detected" — once a preamble is
  detected the chip stays in RX until the packet ends, even if that crosses the next
  slot boundary. This is OK; do not interrupt it. Long packets that spill into the next
  slot cost us that next slot's action, but that's better than corrupted frames. (Note:
  FCC dwell limits don't apply in AU, so multi-slot airtime is legally fine.)
- Same safety cutoff as base: `RX_STUCK_MAX_SLOTS` (2) full slots max.
- **Slot-duration RX timeouts on rocket**:
  - Synced and heard a valid command from base within `ROCKET_NO_BASE_HEARD_THRESHOLD_US`:
    short window `ROCKET_RX_TIMEOUT_US` (100 ms).
  - Unsynced, OR synced but no valid command for ≥ `ROCKET_NO_BASE_HEARD_THRESHOLD_US`
    (124 s — chosen to be >2 ping intervals so a single missed ping can't trip it):
    long window `ROCKET_LONG_RX_TIMEOUT_US` (400 ms).
- **The long-window fallback is CRITICAL for lost-rocket recovery.** The radio's primary
  mission is to locate a lost rocket. If the rocket ever drifts far enough that commands
  stop landing in its short window, it must re-open to a long window so a fresh
  CMD_SET_SYNC from the base can land. This is intentional battery cost. Do not remove,
  disable, or gate this fallback.
- Widening the window does **not** clear `radioSynced`, does **not** touch the anchor, and
  does **not** itself trigger any TX. It simply stretches the RX timeout until a valid
  command is heard again.
- The rocket does **not** clear `radioSynced` on its own. A lost sync is only "replaced"
  by a fresh CMD_SET_SYNC — never self-invalidated. Rocket reboot resets `radioSynced`;
  otherwise it persists until replaced.

## 7. Failure modes and recovery (explicit — these are NOT automatic)

- **Rocket reboots alone**: telemetry stops. The base does **not** auto-resync. The user
  decides whether to reboot the base (which issues a fresh sync 2 s later) or wait for
  signal to recover (it might have been a glitch). This is a deliberate UX choice.
- **Base reboots alone**: rocket continues thinking it's synced on the old anchor. When
  the base comes up, its auto-boot sync (2 s later) replaces the rocket's anchor.
- **Drift past tolerance**: manual base reboot. Automatic drift tracking is a future
  feature (see §9); no drift flags are recorded today.
- **Channel noise / walked-out-of-range**: telem drops but sync persists. User walks back
  or gets line-of-sight. No action needed.
- **Rocket far out of range for >124 s**: rocket widens its RX window to
  `ROCKET_LONG_RX_TIMEOUT_US`. A fresh CMD_SET_SYNC from the base will now land even if
  the old short-window alignment has drifted. Sync is not lost on the rocket until
  replaced; the wide window just makes the replacement possible.

## 8. Per-slot config deferral and early-listen interaction

When the base starts early RX for an upcoming receive-type slot, it must first queue and
apply the **upcoming** slot's config. Applying the current slot's config right before the
boundary is wrong if the upcoming slot has different modulation.

`bsApplyCfgIfNeeded()` is a no-op if the radio is not in STANDBY. The base's slot machine
checks STANDBY before firing the early RX path; when the path fires, the config switch
and RX start happen together on the same loop iteration. If the radio is still busy with
the previous slot when the early-listen window opens, the on-boundary fallback RX path
handles the next slot (losing only the 10 ms early-listen cushion, not the slot itself).

## 9. Future work (noted, not built)

- HeaderValid-timestamp-based drift compensation with explicit drift flags in telemetry.
- Variable slot durations (command slots potentially much shorter than telemetry slots).
- Frequency hopping per packet — telemetry channels hop; command channel stays fixed.
- Long-interleave coding rate for WIN_LR. Currently leaves the radio in a stuck-BUSY
  state in testing; cause unresolved, deferred (see `base_station/radio.h` `LORA_LR_CR`).
- Richer slot sequences (longer interleaved pattern) with a possible separate
  unsynced-mode sequence biased toward more WIN_CMD slots to aid resync.

## 10. Constants (current values — subject to change)

| Constant | Value | Role |
|---|---|---|
| `SLOT_DURATION_US` | 420 ms | Current slot length (uniform for now) |
| `SLOT_SEQUENCE` | `[WIN_TELEM, WIN_CMD]` | Debug-simplification 2-pattern |
| `BS_RX_EARLY_US` | 10 ms | Base starts receive-slot RX this early |
| `BS_CMD_TX_OFFSET_US` | 5 ms | Base TX'es this much after WIN_CMD start |
| `BS_RX_TIMEOUT_US` | 40 ms | Base synced RX window |
| `BS_LONG_RX_TIMEOUT_US` | 370 ms | Base pre-sync RX window |
| `BS_SYNC_BOOT_DELAY_MS` | 2000 ms | Base sends first sync this long after boot |
| `BS_SYNC_RETRY_WALK_MS` | 340 ms | Offset between tight-retry attempts |
| `BS_SYNC_TIGHT_RETRIES` | 6 | Max retries in tight mode before backing off |
| `BS_SYNC_BACKOFF_MS` | 120000 ms | Retry interval after tight mode exhausted |
| `BS_PING_INTERVAL_MS` | 60000 ms | Ping cadence |
| `ROCKET_RX_TIMEOUT_US` | 100 ms | Rocket WIN_CMD RX while synced + heard base recently |
| `ROCKET_LONG_RX_TIMEOUT_US` | 400 ms | Rocket WIN_CMD RX while unsynced or long-silent |
| `ROCKET_NO_BASE_HEARD_THRESHOLD_US` | 124 s | Rocket widens window after this long without a valid command |
| `RX_STUCK_MAX_SLOTS` | 2 | Safety cutoff for stuck RX (both sides) |

## Summary

Rocket listens wide while unsynced and during the long-RX fallback. Base sends sync; both
anchor on TxDone/RxDone of that packet. Rocket acts at slot boundaries (simple timing);
base starts RX early and TX late to absorb clock offsets. RX timeouts catch
no-preamble; once a preamble is detected, let the packet complete — do not force standby
on slot boundaries. No automatic silence-based resync — long silence is signal quality,
not lost sync. Manual base reboot is the escalation path for drift past tolerance.
