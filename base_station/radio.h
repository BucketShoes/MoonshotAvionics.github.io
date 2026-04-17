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

#define LORA_CR       5      // 4/5 coding rate
#define LORA_PREAMBLE 6      // preamble symbols
// Sync word 0x12 (private) written as register pair 0x14, 0x24

#define ROCKET_DEVICE_ID  0x92   // target device ID for commands to rocket (for bootstrap setup, we only have one rocket for debug. will need to make this a config later)

// ===================== SLOT TIMING =====================

enum WindowMode : uint8_t {
  WIN_TELEM  = 0,  // rocket TX telemetry / base RX
  WIN_CMD     = 1,  // base TX commands / rocket RX
  WIN_OFF    = 2,  // radio off — neither side active
  WIN_LR     = 3,  // future: long-range low-rate TX
  WIN_FINDME = 4,  // future: long-preamble beacon for passive scan without bootstrap
};

static const WindowMode SLOT_SEQUENCE[] = { WIN_TELEM, WIN_CMD };
#define SLOT_SEQUENCE_LEN   2
#define SLOT_DURATION_US    2'000'000UL
// Pre-sync: RX window sized to nearly a full slot so we don't miss anything.
// SLOT_DURATION_US - 20ms margin, converted to RTC steps (15.625µs each).
#define BS_PRESYNC_RX_TIMEOUT_US   (SLOT_DURATION_US - 50'000UL)
#define BS_PRESYNC_RX_TIMEOUT_RAW  ((uint32_t)(BS_PRESYNC_RX_TIMEOUT_US / 15.625f))

// Base station RX window parameters
#define BS_RX_EARLY_US        10'000UL    // start RX this many µs before WIN_TELEM
#define BS_RX_TIMEOUT_US      40'000UL    // RX window duration
#define BS_RX_TIMEOUT_RAW     ((uint32_t)(BS_RX_TIMEOUT_US / 15.625f))
#define BS_CMD_TX_OFFSET_US   5'000UL     // fire command this many µs into WIN_CMD

#define BS_SYNC_BOOT_DELAY_MS  2'000      // send first sync 2s after boot
#define BS_SYNC_LOSS_SLOTS     500          // resync if this many WIN_TELEM slots missed



// ===================== RADIO STATE =====================

enum BsRadioState { BS_RADIO_STANDBY, BS_RADIO_RX_ACTIVE, BS_RADIO_TX_ACTIVE };

extern SPIClass             bsLoraSPI;
extern sx126x_hal_context_t bsRadioCtx;
extern BsRadioState         bsRadioState;
extern bool                 bsLoraReady;

// ===================== ACTIVE RADIO CONFIG =====================

#define DEFAULT_CHANNEL  65
#define DEFAULT_SF       5
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
extern uint32_t      bsSyncSlotIndex;
extern uint32_t      bsLastHandledSlot;
extern unsigned long bsMissedTelemSlots;

// HeaderValid timestamp (held until RX_DONE of a valid rocket telemetry packet)
extern uint64_t      bsPendingHeaderValidUs;

// Background RSSI EMA — updated on RX timeout (no packet received).
// Excludes packet RX windows so it reflects channel noise, not signal.
// int8 range: reported as-is in page 12 bgRssi field.
extern float         bsBgRssiEma;

// ===================== COORDINATION FLAGS (main.cpp ↔ radio.cpp) =====================

// Set by bsHandleSyncSend() when a sync packet needs to be built and queued.
// main.cpp calls bsBuildSyncCmdPacket(), loads cmdTx, then clears this flag.
extern bool bsSyncNeedsQueue;

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

// Initialise radio hardware. Call once in setup() after SPI is started.
bool bsRadioInit();

// Apply current activeChannel/activeSF/activePower to hardware. Radio must be in standby. WARNING: THIS BLOCKS BRIEFLY. DO NOT CALL WHILE ARMED
void bsRadioApplyConfig();

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

// Main radio update. Call every loop iteration.
void bsHandleRadio();

// Auto-sync management: send 2s after boot, retry every 60s.
// Sets bsSyncNeedsQueue when it is time to send. main.cpp handles the actual queuing.
void bsHandleSyncSend();

// Callback invoked from bsHandleRadio() when a good LoRa packet is received.
// Implemented in main.cpp (handles transport push, log write, telem store).
void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF);

#endif // BS_RADIO_H
