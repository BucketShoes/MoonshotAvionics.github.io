// radio.h — LoRa radio hardware, TX/RX state machine, and active channel config.
// Owns the SX1262 via sx126x_driver, all radio state, and slot-clock sync.
// Changes to radio protocol or channel plan only touch radio.h + radio.cpp.

#ifndef RADIO_H
#define RADIO_H

#include <SPI.h>
#include "radio_hal.h"    // sx126x_hal_context_t, dio1Fired, dio1TimestampUs()
#include "sx126x.h"        // sx126x_driver API
#include "config.h"

// ===================== RADIO STATE =====================

enum RadioState {
  RADIO_OFF,         // not yet initialised
  RADIO_STANDBY,     // sx126x in STDBY_RC, BUSY low, ready for commands
  RADIO_RX_ACTIVE,   // sx126x_set_rx called, waiting for DIO1
  RADIO_TX_ACTIVE,   // sx126x_set_tx called, waiting for DIO1
};

// ===================== HARDWARE OBJECTS =====================

extern SPIClass             loraSPI;
extern sx126x_hal_context_t radioCtx;

// ===================== RADIO STATE =====================

extern bool       loraReady;
extern RadioState radioState;

// ===================== ACTIVE RADIO CONFIG =====================
// Loaded from NVS at boot, updated by CMD_SET_RADIO.
// BW is derived from channel index (ch<64 = 125 kHz, ch>=64 = 500 kHz).

extern uint8_t activeChannel;
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

// ===================== SLOT CLOCK STATE =====================

extern bool          radioSynced;
extern unsigned long syncAnchorUs;
extern uint32_t      syncSlotIndex;
extern uint32_t      lastHandledSlotNum;

// ===================== RSSI EMA (stub) =====================

extern double rssiEma;

// ===================== STATS =====================

extern uint16_t delayedTxCount;
extern uint16_t invalidRxCount;

// ===================== CHANNEL TABLE =====================
// Channel 0-63:  BW125, 915.2 + ch*0.2 MHz
// Channel 64-71: BW500, 915.9 + (ch-64)*1.6 MHz

static inline float channelToFreqMHz(uint8_t ch) {
  if (ch < 64) return 915.2f + ch * 0.2f;
  if (ch < 72) return 915.9f + (ch - 64) * 1.6f;
  return 0.0f;
}

// ===================== PUBLIC API =====================

// Derive activeFreqMHz and activeBwKHz from activeChannel.
void updateActiveFreqBw();

// Initialise radio hardware (SX126x reset, standby, configure modulation/packet
// params, DIO2 RF switch, IRQ mask, MCPWM capture on DIO1).
// Called from the INIT_LORA state in nonblockingInit().
// Returns true on success.
bool radioInit();

// *** BLOCKING — INIT ONLY ***
// Apply radio parameters (frequency, SF, BW, power) from NVS or CMD_SET_RADIO.
// Radio must be in RADIO_STANDBY. Contains DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING calls (up to 100ms each).
// ONLY call from radioInit_BLOCKING() or CMD_SET_RADIO while disarmed.
void radioApplyConfig_BLOCKING();

// Per-slot radio config enum. Updated at each slot boundary; applyCfgIfNeeded()
// issues two SPI commands with up to 100µs BUSY spin between them (typical <20µs).
// Deferred if radio is not in STANDBY (retries next loop).
enum RadioSlotConfig : uint8_t {
  RADIO_CFG_NORMAL = 0,
  RADIO_CFG_LR     = 1,
};

// Start RX with the slot-appropriate timeout (short if synced + recently heard base;
// long if pre-sync or lost-rocket fallback). See radioStartRx() in radio.cpp.
void radioStartRx();

// Start windowed RX with a timeout in raw RTC steps (15.625 µs per tick).
void radioStartRxTimeout(uint32_t timeoutRtcSteps);

// Start async TX. Returns true if TX started.
bool radioStartTx(const uint8_t* pkt, size_t len);

// Put radio in standby.
void radioStandby();

// Set the slot clock sync point. Called from CMD_SET_SYNC handler.
// anchorUs  = micros() at the sync event
// slotIdx   = slot index that is current at that anchor
void radioSetSynced(unsigned long anchorUs, uint8_t slotIdx);

// Build the 3-byte on-air core for the 0xBB long-range packet (dithered lat/lon + low-batt).
// In WIN_LR slots only the core is transmitted (implicit header). Returns 3.
size_t buildLRPacketCore(uint8_t* buf);

// Main non-blocking radio update: slot-based TX/RX scheduling or bootstrap RX.
void nonblockingRadio();

#endif // RADIO_H
