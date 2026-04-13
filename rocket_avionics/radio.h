// radio.h — LoRa radio hardware, TX/RX state machine, and active channel config.
// Owns the SX1262 object, all radio state, RSSI EMA, TX scheduling, and active
// channel/SF/power settings. Changes to radio protocol, CSMA, or channel plan
// only touch radio.h + radio.cpp.

#ifndef RADIO_H
#define RADIO_H

#include <RadioLib.h>
#include "config.h"

// ===================== SX1262 IRQ FLAGS =====================

#define IRQ_PREAMBLE_DETECTED  RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED
#define IRQ_HEADER_VALID       RADIOLIB_SX126X_IRQ_HEADER_VALID
#define IRQ_HEADER_ERR         RADIOLIB_SX126X_IRQ_HEADER_ERR
#define IRQ_RX_DONE            RADIOLIB_SX126X_IRQ_RX_DONE

// ===================== RADIO STATE =====================

enum RadioState {
  RADIO_OFF,           // radio not yet initialised
  RADIO_RX_LISTENING,  // continuous RX, waiting for packets or TX time
  RADIO_TX_ACTIVE,     // async TX in progress, waiting for DIO1 TX-done
  RADIO_RX_RECEIVING   // reserved for future explicit preamble state
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

// ===================== TX SCHEDULING =====================

extern unsigned long nextTxDueUs;

// ===================== TIMED WINDOWS / HOPPING =====================
// When hopEnabled, the radio uses fixed 420ms windows instead of CSMA scheduling.
// Even window index = RX for commands (40ms timeout). Odd = TX telemetry.
// hopSyncOffsetUs is the micros() value at which window 0 occurred.

extern bool     hopEnabled;
extern uint64_t hopSyncOffsetUs;

enum WindowType { WIN_TELEM, WIN_LONG_RANGE, WIN_SCANNER, WIN_RX, WIN_OFF };

// Returns the current window index based on micros() and hopSyncOffsetUs.
uint32_t hopCurrentWindowIndex();

// Returns the window type for the current window (Phase 1: WIN_TELEM or WIN_RX).
WindowType hopCurrentWindowType();

// ===================== RSSI EMA =====================

extern double rssiEma;  // background noise floor estimate

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

// Enter continuous RX mode.
void radioStartRx();

// Start async TX. Returns true if TX started; falls back to RX on failure.
bool radioStartTx(const uint8_t* pkt, size_t len);

// Reconfigure radio hardware to the download settings specified.
bool radioSetDownloadConfig(float freqMHz, uint8_t sf, float bwKHz, int8_t power);

// Restore radio hardware to normal operating config (activeChannel/SF/power).
void radioRestoreNormalConfig();

// Main non-blocking radio update: RSSI EMA, preamble detection, TX scheduling, RX handling.
void nonblockingRadio();

#endif // RADIO_H
