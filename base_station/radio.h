// base_station/radio.h — LoRa radio, slot clock, and sync for base station.
// Extracted from main.cpp. See radio.cpp for implementation.

#ifndef BS_RADIO_H
#define BS_RADIO_H

#include <Arduino.h>
#include <SPI.h>
#include "radio_hal.h"    // sx126x_hal_context_t, dio1Fired, dio1TimestampUs()
#include "sx126x.h"        // sx126x_driver API

// ===================== PIN / PROTOCOL CONSTANTS =====================

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

#define LED_PIN       18

// LED_MODE: 0 = logic-driven (default, follows radio state machine decisions).
// 1 = BUSY-pin-driven (directly mirrors SX1262 BUSY pin state for direct radio observation).
#define LED_MODE_LOGIC 0
#define LED_MODE_BUSY  1
#define LED_MODE       LED_MODE_LOGIC

#define LORA_CR       5      // 4/5 coding rate
#define LORA_PREAMBLE 8      // preamble symbols
// Sync word 0x12 (private) written as register pair 0x14, 0x24

#define PKT_LONGRANGE  0xBB
// WIN_LR slot config. BW follows activeChannel (same as normal slots).
// SF is separate — reduce for bench testing to shorten air time.
// WIN_LR spreading factor.
#define LORA_LR_SF    11
// WIN_LR coding rate. 0x05 = 4/5 long-interleave (LI). Not in sx126x_driver enum; cast at use site.
// TODO: @@@ confirm whether LI is causing stuck-BUSY seen in testing, or find real cause.
// If LI is the problem, replace with SX126X_LORA_CR_4_5 (standard 4/5 = 0x01).
#define LORA_LR_CR    SX126X_LORA_CR_4_5 // SX126X_LORA_CR_4_5 for normal 4/5 or for li4/5 use 0x05

#define FAVORITE_ROCKET_DEVICE_ID  0x92   // target device ID for commands to rocket (for bootstrap setup, we only have one rocket for debug. will need to make this a config later)

// ===================== SLOT TIMING =====================

enum WindowMode : uint8_t {
  WIN_TELEM  = 0,  // rocket TX telemetry / base RX - using hopping channel with user-configurable telem modulation
  WIN_CMD     = 1, // base TX commands / rocket RX - using the command channel, with command modulation params (typically same modulation as telem)
  WIN_OFF    = 2,  // radio off — neither side active, power save
  WIN_LR     = 3,  // long-range low-rate TX at high sf, implicit headers, etc
  WIN_FINDME = 4,  //  long-preamble beacon for passive scan without bootstrap, on specific modulation regardless of radio settings
  WIN_BACKHAUL = 5, //timeslot reserved for relay backhaul beween multiple bae stations - rocket stays quiet, but listens similar to a WIN_CMD but on hop channel with backhaul modulation. not valid for sending normal commands, but can help sync
  WIN_MULTIPURPOSE = 6,//long slow cycle between various functions. full anchor timing and state machine required to know which modulation to use.
  WIN_RDF = 7, //for radio distance/direction finding. send multiple short packets at variable signal strength and changing SF/BW for improved distance estimation
  WIN_GFSK = 8,//todo need different bandwidths
  WIN_CONTINUE=9,// extend previous slot. dont reset radio into standby at start of slot, etc.
  WIN_NARROWBAND_TELEM=10, //longer range version, but not as limited as LR
  WIN_WIDEBAND_TELEM=11, //extra bandwidth, gives all telemetry data pages in a single packet
};



static const WindowMode SLOT_SEQUENCE[] = { WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_LR, WIN_CMD, WIN_TELEM, WIN_TELEM,WIN_CMD,};

#define SLOT_SEQUENCE_LEN   (sizeof(SLOT_SEQUENCE) / sizeof(SLOT_SEQUENCE[0]))
#define SLOT_DURATION_US    420'000UL //how long between the timing points where messages are sent/listened for. note that this may change in futue, and some comments incorrectly assume itll always be this long.

// Base station RX window parameters (converted to RTC steps via /15.625 at use site).
#define BS_RX_TIMEOUT_US           80'000UL                   // synced telemetry RX window
#define BS_LONG_RX_TIMEOUT_US      (SLOT_DURATION_US - 120'000UL)  // pre-sync: nearly full slot - dont long for active sync, we only care if we got one at the right time, the rest are non-synced random noise

// Base station TX timing.
#define BS_RX_EARLY_US             40'000UL    // start RX this many µs before a receive-type slot boundary
#define BS_CMD_TX_OFFSET_US        5'000UL    // fire command this many µs into WIN_CMD (safety margin for early-starting rocket)

// Sync timing. See "Hopping radio slot structure.md" for the full model.
// Sync is only ever sent automatically while still never-synced this session (plus the user
// can force one via the UI). There is NO silence-based resync — long telem silence is treated
// as signal quality, not lost sync. A prior auto-resync-on-silence was harmful and has been removed.
#define BS_SYNC_BOOT_DELAY_MS      2'000UL     // first attempt fires this long after boot

// How often retries are sent while still unsynced. BS_SYNC_RETRY_INTERVAL_MS is the base
// period; BS_SYNC_RETRY_WALK_MS is added on top so each attempt's TX phase drifts by WALK
// relative to the previous. Over successive attempts this sweeps through all phases of
// the rocket's slot cycle.
//   actual_tight_period = BS_SYNC_RETRY_INTERVAL_MS + BS_SYNC_RETRY_WALK_MS
//   actual_backoff_period = BS_SYNC_BACKOFF_MS + BS_SYNC_RETRY_WALK_MS
// The command channel is not hopped and is not bandwidth-sensitive, so a modest retry
// cadence is fine — we just need to not hammer NVS with nonce writes.
#define BS_SYNC_RETRY_INTERVAL_MS  10'000UL     // nominal period between tight-mode retries - to leave time to see if we hear any telem
#define BS_SYNC_RETRY_WALK_MS      340UL       // phase offset added to each retry
                                               // (<SLOT_DURATION so retries walk through slot phases)
#define BS_SYNC_TIGHT_RETRIES      200UL         // how many retries to make at the tight interval
#define BS_SYNC_BACKOFF_MS         240'000UL   // after tight-mode retries exhaust, retry once per this
                                               // (still with WALK added so backoff retries also sweep phases)
#define BS_PING_INTERVAL_MS        60'000UL    // send ping if no command sent in this long (so the rocket knows it can reduce listen time safely)
#define BS_PING_SILENCE_MAX_MS     90'000UL    // stop pinging if no telem heard in this long. The point of
                                               // ping is to keep the rocket in narrow mode while sync is good.
                                               // If we've lost telem we WANT the rocket to fall back to wide
                                               // listen (after ROCKET_NO_BASE_HEARD_THRESHOLD_US) so the next
                                               // resync attempt has a chance to land.

// Drift calibration tuning (auto-correction of clock drift via posInSlot tracking).
// Per-packet limits are always applied. Per-minute limit tracks NET drift (signed), allowing
// bidirectional jitter correction within the budget (oscillation doesn't consume budget).
// Per-session limit scales with uptime to allow recovery from bad initial sync but prevent
// long-term runaway.
//
// Timing phases:
//   1. Fast window (first BS_DRIFT_FAST_WINDOW_MS after sync): aggressive per-minute limit
//      to recover from ±50ms initial sync jitter
//   2. Ramp phase: gradually relax from fast to conservative over BS_DRIFT_RAMP_DURATION_MS
//   3. Conservative phase: hold tight (crystal accuracy rarely changes)
#define BS_DRIFT_DEADBAND_US            500UL       // only correct if |EMA drift| > this (µs)
#define BS_DRIFT_CORRECTION_FACTOR      0.10f        // apply this fraction of EMA per packet
#define BS_DRIFT_EMA_RATIO              0.01f        // how fast the ema moves
#define BS_DRIFT_MAX_PER_PACKET_US      100UL       // max correction per single packet (µs) — low to filter jitter
#define BS_DRIFT_MAX_PER_MINUTE_US      2'000UL     // conservative per-minute limit after ramp (µs, NET drift)
#define BS_DRIFT_MAX_PER_MINUTE_FAST_US 20'000UL    // aggressive per-minute limit in fast window (µs, NET drift)
#define BS_DRIFT_FAST_WINDOW_MS         120'000UL   // duration of fast correction after sync (ms, ~2 minutes)
#define BS_DRIFT_RAMP_DURATION_MS       300'000UL   // ramp from fast to conservative over this (ms, ~5 minutes)
// Per-session cap scales with uptime: base + per_minute * uptime_minutes.
// At 5min: 50ms + 1ms*5 = 55ms. At 1hr: 50ms + 1ms*60 = 110ms. At 8hr: 50ms + 1ms*480 = 530ms.
// Allows recovery from bad initial sync without long-term drift over days.
#define BS_DRIFT_MAX_ACCUMULATED_BASE_US 50'000UL   // base accumulated drift safety cap (µs)
#define BS_DRIFT_MAX_ACCUMULATED_PER_MIN 1'000UL    // additional cap per minute of uptime (µs)

// Safety cutoff: force standby if RX has been active for more than this many slot durations.
// Applies to both base and rocket. Indicates a missed DIO1 IRQ (or stuck DIO1 line).
#define RX_STUCK_MAX_SLOTS         20UL

#define LOG_RX_START false
#define LOG_RX_DONE true
#define LOG_RX_TIMEOUT false
#define LOG_TX_START false
#define LOG_TX_TIMEOUT false
#define LOG_TX_DONE false
#define LOG_APPLYCFG false  // per-slot config apply (spammy)

#define LORA_RX_BOOSTED true //+3db rx gain, +3ma power usage


// ===================== RADIO STATE =====================

enum BsRadioState { BS_RADIO_STANDBY, BS_RADIO_RX_ACTIVE, BS_RADIO_TX_ACTIVE };

extern SPIClass             bsLoraSPI;
extern sx126x_hal_context_t bsRadioCtx;
extern BsRadioState         bsRadioState;
extern bool                 bsLoraReady;

// ===================== ACTIVE RADIO CONFIG =====================

#define DEFAULT_CHANNEL  3
#define DEFAULT_SF       9
#define DEFAULT_POWER    -9

extern uint8_t activeChannel;
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

#define DEFAULT_BH_CHANNEL 67
#define DEFAULT_BH_SF      5
#define DEFAULT_BH_POWER   -9

extern uint8_t bhChannel;
extern uint8_t bhSF;
extern int8_t  bhPower;

// ===================== SLOT CLOCK STATE =====================

extern bool          bsSynced;
extern unsigned long bsSyncAnchorUs;
extern unsigned long bsSyncAnchorOriginalUs;  // original anchor at sync time
extern int32_t       bsAnchorDriftUs;         // cumulative drift correction
extern uint32_t      bsSyncSlotIndex;
extern uint32_t      bsLastHandledSlot;
extern unsigned long bsMissedTelemSlots;

// HeaderValid timestamp (held until RX_DONE of a valid rocket telemetry packet)
extern uint64_t      bsPendingHeaderValidUs;

// Background RSSI EMA — noise floor / channel activity estimate.
// Sampled every loop while RX_ACTIVE (after the first loop of each RX window).
// Excludes received-packet RSSI so it reflects channel noise between our packets.
// Reported in telemetry page 12 bgRssi field. -128.0 = no samples yet.
extern float bsBgRssiEma;

// Track when last rocket telemetry was received (millis).
extern unsigned long bsLastTelemRxMs;

// Track when last command was sent (millis).
extern unsigned long bsLastCmdSentMs;

// ===================== COORDINATION FLAGS (main.cpp ↔ radio.cpp) =====================

// Set by bsHandleSyncSend() when a sync or ping packet needs to be built and queued.
// main.cpp calls bsBuildSyncCmdPacket() or bsBuildPingCmdPacket(), loads cmdTx, then clears this flag.
extern bool bsSyncNeedsQueue;

// Set by bsHandleSyncSend() when a ping packet needs to be built and queued.
// main.cpp calls bsBuildPingCmdPacket(), loads cmdTx, then clears this flag.
extern bool bsPingNeedsQueue;

// Set by bsHandleRadio() at BS_CMD_TX_OFFSET_US into WIN_CMD.
// main.cpp calls dispatchCmdTx(), then clears this flag.
extern bool bsWinCmdReady;

// Set by main.cpp before calling bsRadioStartTx() for a CMD_SET_SYNC packet.
// Cleared by bsHandleRadio() after TxDone — triggers bsSetSyncedFromTx().
extern bool bsSyncTxInFlight;

// ===================== PUBLIC API =====================

// Convert channel index to frequency MHz
static inline float bsChannelToFreqMHz(uint8_t ch) {
  if (ch < 64) return 915.2f + ch * 0.2f;
  if (ch < 72) return 915.9f + (ch - 64) * 1.6f;
  return 0.0f;
}

void bsUpdateActiveFreqBw();

// Per-slot radio config enum. NORMAL = standard operating params; LR = WIN_LR SF12 implicit.
enum RadioSlotConfig : uint8_t {
  RADIO_CFG_NORMAL = 0, //TODO: we shouldnt have normal/abnormal - the expectation should be that mnost types are all different. we also cant assume the old normal is the same as the new normal, if settings have changed since we applied it
  RADIO_CFG_LR     = 1,
};

extern RadioSlotConfig bsTargetCfg;
void bsApplyCfgIfNeeded();

// Initialise radio hardware. Call once in setup() after SPI is started.
bool bsRadioInit();

// Apply current activeChannel/activeSF/activePower to hardware. Radio must be in standby. WARNING: THIS BLOCKS BRIEFLY. DO NOT CALL WHILE ARMED
// BLOCKING — init only. Contains DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING calls. Only call from bsRadioInit().
void bsRadioApplyConfig_BLOCKING();

void bsRadioStartRx();
void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps);
bool bsRadioStartTx(const uint8_t* pkt, size_t len);
void bsRadioStandby();

// Set sync anchor from TxDone of our own CMD_SET_SYNC or other WIN_CMD TX.
// anchorUs = hardware-latched TxDone timestamp; slotIdx = WIN_CMD index.
void bsSetSyncedFromTx(uint64_t anchorUs);

// Build CMD_SET_SYNC packet. Updates nonce and writes to NVS.
// Returns packet length (always 17).
size_t bsBuildSyncCmdPacket(uint8_t *buf);

// Build CMD_PING packet. Updates nonce and writes to NVS.
// Returns packet length (always 17).
size_t bsBuildPingCmdPacket(uint8_t *buf);

// Main radio update. Call every loop iteration.
void bsHandleRadio();

// Auto-sync & ping management: send sync at boot (2s) or after 20min silence; send ping every 1min.
// Sets bsSyncNeedsQueue or bsPingNeedsQueue when it is time. main.cpp handles the actual queuing.
void bsHandleSyncSend();

// Callback invoked from bsHandleRadio() when a good LoRa packet is received.
// Implemented in main.cpp (handles transport push, log write, telem store).
void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF,
                        int32_t signedPosInSlot, uint32_t slotNum, uint8_t seqIdx,
                        uint8_t win, uint32_t timeOnAirMs, float driftEmaUs, uint32_t timeSinceSyncMs);

#endif // BS_RADIO_H
