// radio.h — LoRa radio hardware, TX/RX state machine, and active channel config.
// Owns the SX1262 object, all radio state, RSSI EMA, TX scheduling, and active
// channel/SF/power settings. Changes to radio protocol, CSMA, or channel plan
// only touch radio.h + radio.cpp.

#ifndef RADIO_H
#define RADIO_H

#include <RadioLib.h>
#include "config.h"

// ===================== RADIO STATE =====================

enum RadioState {
  RADIO_OFF,        // radio not yet initialised
  RADIO_IDLE,       // standby (WIN_OFF or between operations)
  RADIO_RX_ACTIVE,  // startReceive() called, waiting for RxDone or timeout via DIO1
  RADIO_TX_ACTIVE,  // startTransmit() called, waiting for TxDone via DIO1
};

// ===================== HARDWARE OBJECTS =====================

extern SX1262 radio;
extern SPIClass loraSPI;

// ===================== RADIO STATE =====================

extern bool loraReady;
extern RadioState radioState;
extern volatile bool dio1Fired;

// ===================== ACTIVE RADIO CONFIG =====================
// Loaded from NVS at boot, updated by CMD_SET_RADIO.
// BW is derived from channel index (ch<64 = 125kHz, ch>=64 = 500kHz).

extern uint8_t activeChannel;
extern uint8_t activeSF;
extern int8_t  activePower;
extern float   activeFreqMHz;
extern float   activeBwKHz;

// ===================== SLOT CLOCK STATE =====================
// Managed by radioSetSynced(). Both rocket and base anchor their slot clock
// to RxDone/TxDone of the CMD_SET_SYNC packet.

extern bool          radioSynced;         // false = bootstrap (continuous RX)
extern unsigned long syncAnchorUs;        // micros() at the sync event
extern uint32_t      syncSlotIndex;       // absolute slot index at anchor (= 1 after sync)
extern uint32_t      lastHandledSlotNum;  // last slot number acted on (prevents re-entry)

// ===================== RSSI EMA (stub) =====================
// No longer sampled continuously; kept so telemetry page 0x0C still compiles.
// Will read as 0 (no estimate available) until re-implemented.

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
  return 0.0f;  // invalid channel
}

// ===================== PUBLIC API =====================

// DIO1 interrupt handler (attach via radio.setDio1Action).
void IRAM_ATTR dio1ISR();

// Recompute activeFreqMHz and activeBwKHz from activeChannel.
void updateActiveFreqBw();

// Enter continuous RX mode (used during bootstrap and log download).
void radioStartRx();

// Start async TX. Returns true if TX started; falls back to idle on failure.
bool radioStartTx(const uint8_t* pkt, size_t len);

// Set the slot clock sync point. Called from CMD_SET_SYNC handler at RxDone.
// anchorUs = micros() at the sync event, slotIdx = slot index that starts now.
void radioSetSynced(unsigned long anchorUs, uint8_t slotIdx);

// Reconfigure radio hardware to the download settings specified.
bool radioSetDownloadConfig(float freqMHz, uint8_t sf, float bwKHz, int8_t power);

// Restore radio hardware to normal operating config (activeChannel/SF/power).
void radioRestoreNormalConfig();

// Main non-blocking radio update: slot-based TX/RX scheduling (or bootstrap continuous RX).
void nonblockingRadio();

#endif // RADIO_H
