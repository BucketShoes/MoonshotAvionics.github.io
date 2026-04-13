// radio.cpp — LoRa radio hardware, TX/RX state machine, RSSI EMA, TX scheduling.
// See radio.h for the public API.

#include <Arduino.h>
#include <esp_random.h>
#include "radio.h"
#include "telemetry.h"
#include "commands.h"
#include "globals.h"

// ===================== HARDWARE OBJECTS =====================

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN, LORA_BUSY_PIN, loraSPI);

// ===================== ACTIVE RADIO CONFIG =====================
// Loaded from NVS in INIT_NVS; updateActiveFreqBw() derives freq/BW from channel.

uint8_t activeChannel = DEFAULT_CHANNEL;
uint8_t activeSF      = DEFAULT_SF;
int8_t  activePower   = DEFAULT_POWER;
float   activeFreqMHz = 917.5f;
float   activeBwKHz   = 500.0f;

// ===================== RADIO STATE =====================

bool        loraReady  = false;
RadioState  radioState = RADIO_OFF;
volatile bool dio1Fired = false;

// ===================== TX SCHEDULING =====================

unsigned long nextTxDueUs    = 0;
unsigned long keepoutUntilUs = 0;
bool          preambleActive = false;
bool          txDeferredThisSlot = false;

// ===================== TIMED WINDOWS / HOPPING =====================

bool     hopEnabled      = false;
uint64_t hopSyncOffsetUs = 0;

// ===================== RSSI EMA =====================

double        rssiEma           = -120.0;
bool          rssiEmaInitialised = false;
unsigned long lastRssiSampleUs  = 0;
unsigned long lastRssiHighUs    = 0;

// ===================== STATS =====================

uint16_t delayedTxCount = 0;
uint16_t invalidRxCount = 0;

// ===================== DIO1 ISR =====================

void IRAM_ATTR dio1ISR() {
  dio1Fired = true;
}

// ===================== HELPERS =====================

static unsigned long randomKeepout() {
  return TX_KEEPOUT_MIN_US + (esp_random() % TX_KEEPOUT_RANGE_US);
}

void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz = (activeChannel < 64) ? 125.0f : 500.0f;
}

uint32_t hopCurrentWindowIndex() {
  uint64_t elapsed = (uint64_t)micros() - hopSyncOffsetUs;
  return (uint32_t)(elapsed / HOP_WINDOW_US);
}

WindowType hopCurrentWindowType() {
  uint32_t idx = hopCurrentWindowIndex();
  return (idx & 1) ? WIN_TELEM : WIN_RX;
}

// ===================== RADIO CONFIG =====================

bool radioSetDownloadConfig(float freqMHz, uint8_t sf, float bwKHz, int8_t power) {
  radio.standby();
  if (radio.setFrequency(freqMHz)       != RADIOLIB_ERR_NONE) return false;
  if (radio.setSpreadingFactor(sf)      != RADIOLIB_ERR_NONE) return false;
  if (radio.setBandwidth(bwKHz)         != RADIOLIB_ERR_NONE) return false;
  if (radio.setOutputPower(power)       != RADIOLIB_ERR_NONE) return false;
  return true;
}

void radioRestoreNormalConfig() {
  radio.standby();
  radio.setFrequency(activeFreqMHz);
  radio.setSpreadingFactor(activeSF);
  radio.setBandwidth(activeBwKHz);
  radio.setOutputPower(activePower);
}

// ===================== RX / TX =====================

void radioStartRx() {
  int state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    radioState = RADIO_RX_LISTENING;
  } else {
    Serial.print("RX start fail: ");
    Serial.println(state);
    radioState = RADIO_OFF;
  }
  dio1Fired = false;
}

bool radioStartTx(const uint8_t* pkt, size_t len) {
  int state = radio.startTransmit(pkt, len);
  if (state == RADIOLIB_ERR_NONE) {
    radioState = RADIO_TX_ACTIVE;
    dio1Fired = false;
    return true;
  } else {
    Serial.print("TX start fail: ");
    Serial.println(state);
    radioStartRx();
    return false;
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================
//
// Two modes:
//
// CSMA mode (hopEnabled == false, default):
//   Continuous RX between TX slots. TX uses a fixed schedule.
//   Deferred TX (channel busy) does not drift the schedule.
//   Hard ceiling: TX anyway after TX_MAX_DEFER_US regardless of blockers.
//
// Timed window mode (hopEnabled == true):
//   420ms windows. Even index = WIN_RX (40ms single-shot RX). Odd index = WIN_TELEM (TX).
//   Everything uses single-shot RX (timed startReceive); radio returns to standby on its own
//   after rxDone, rxTimeout, or txDone — no explicit standby needed for normal flow.
//   LED (FLASH_SYNC_TIMING): on for the duration of TX only (WIN_TELEM).
//   The base station opens a single-shot RX window at every boundary, so both sides are
//   active for a brief aligned window each 420ms.

static void hopLedSet(bool on) {
#if FLASH_SYNC_TIMING
  ledcWrite(LED_PIN, on ? 255 : 0);
#endif
  (void)on;
}

// Read IRQ flags and log which events fired. Returns the raw IRQ word.
static uint16_t hopLogIrq(const char* context) {
  uint16_t irq = (uint16_t)radio.getIrqFlags();
  Serial.print("  IRQ("); Serial.print(context); Serial.print(")=0x");
  Serial.print(irq, HEX);
  if (irq & RADIOLIB_SX126X_IRQ_TX_DONE)        Serial.print(" TX_DONE");
  if (irq & RADIOLIB_SX126X_IRQ_RX_DONE)        Serial.print(" RX_DONE");
  if (irq & RADIOLIB_SX126X_IRQ_TIMEOUT)        Serial.print(" TIMEOUT");
  if (irq & RADIOLIB_SX126X_IRQ_CRC_ERR)        Serial.print(" CRC_ERR");
  if (irq & RADIOLIB_SX126X_IRQ_HEADER_ERR)     Serial.print(" HDR_ERR");
  if (irq & RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED) Serial.print(" PREAMBLE");
  Serial.println();
  return irq;
}

void nonblockingRadio() {
  if (!loraReady) return;

  unsigned long now = micros();

  // ===================== TIMED WINDOW MODE =====================
  if (hopEnabled) {
    static uint32_t lastWindowIndex = 0xFFFFFFFF;
    uint32_t winIdx = hopCurrentWindowIndex();

    // --- Handle DIO1 events while a window is active ---
    if (dio1Fired) {
      dio1Fired = false;
      uint16_t irq = hopLogIrq(radioState == RADIO_TX_ACTIVE ? "TX" : "RX");

      if (radioState == RADIO_TX_ACTIVE) {
        // TX_DONE: SX1262 already returned to standby.
        hopLedSet(false);
        radioState = RADIO_OFF;
        // Fall through to window boundary check — next window may already be due.

      } else if (radioState == RADIO_RX_LISTENING) {
        // RX_DONE or TIMEOUT: SX1262 already returned to standby.
        if (irq & RADIOLIB_SX126X_IRQ_RX_DONE) {
          uint8_t rxBuf[64];
          int state = radio.readData(rxBuf, sizeof(rxBuf));
          if (state == RADIOLIB_ERR_NONE) {
            size_t rxLen = radio.getPacketLength();
            float pktRssi = radio.getRSSI(true);
            float pktSnr  = radio.getSNR();
            processReceivedPacket(rxBuf, rxLen, (int8_t)pktRssi, (int8_t)pktSnr);
          } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            Serial.println("RX: CRC fail");
            invalidRxCount++;
          } else {
            Serial.print("RX err: "); Serial.println(state);
            invalidRxCount++;
          }
        }
        // TIMEOUT: no packet this window — radio already in standby, nothing to read.
        preambleActive = false;
        radioState = RADIO_OFF;
        // Fall through to window boundary check.

      } else {
        // Unexpected DIO1 while RADIO_OFF — stale interrupt. Ignore.
        Serial.print("HOP: unexpected DIO1 in state "); Serial.println(radioState);
      }
    } else if (radioState != RADIO_OFF) {
      // Radio is mid-TX or mid-RX and DIO1 hasn't fired yet — wait for it.
      return;
    }

    // --- Window boundary check — only reached when radio is idle ---
    if (winIdx != lastWindowIndex) {
      lastWindowIndex = winIdx;
      WindowType wt = hopCurrentWindowType();

      // startReceiveCommon calls standby() internally. startTransmit (stageMode TX) does NOT,
      // but after a timed RX the chip auto-returns to standby; by the next 420ms boundary
      // BUSY will be settled, so calling startTransmit directly is safe.
      // Do NOT call radio.standby() here — it races with the chip's own BUSY settling
      // after a timed RX timeout and causes -705.

      if (wt == WIN_TELEM) {
        Serial.print("WIN "); Serial.print(winIdx);
        Serial.print(": TELEM ch="); Serial.println(activeChannel);

        uint8_t pkt[32];
        size_t len = buildTelemetryPacket(pkt);
        hopLedSet(true);
        int txState = radio.startTransmit(pkt, len);
        if (txState == RADIOLIB_ERR_NONE) {
          radioState = RADIO_TX_ACTIVE;
          dio1Fired = false;
        } else {
          Serial.print("  TX start fail: "); Serial.println(txState);
          hopLedSet(false);
          radioState = RADIO_OFF;
        }

      } else {  // WIN_RX
        Serial.print("WIN "); Serial.print(winIdx);
        Serial.print(": RX ch="); Serial.println(activeChannel);

        int rxState = radio.startReceive(radio.calculateRxTimeout(HOP_RX_TIMEOUT_MS * 1000UL));
        if (rxState == RADIOLIB_ERR_NONE) {
          radioState = RADIO_RX_LISTENING;
          dio1Fired = false;
        } else {
          Serial.print("  RX start fail: "); Serial.println(rxState);
          radioState = RADIO_OFF;
        }
      }
    }
    return;
  }

  // ===================== CSMA MODE (pre-bootstrap) =====================

  switch (radioState) {

    case RADIO_OFF:
      radioStartRx();
      break;

    case RADIO_TX_ACTIVE:
      if (dio1Fired) {
        dio1Fired = false;
        keepoutUntilUs = now + randomKeepout();
        radioStartRx();
      }
      break;

    case RADIO_RX_LISTENING: {

      // --- Poll IRQ for preamble detection ---
      if (!preambleActive) {
        uint16_t irq = radio.getIrqFlags();
        if (irq & IRQ_PREAMBLE_DETECTED) {
          preambleActive = true;
        }
      }

      // --- Check for header error (preamble without valid packet) ---
      if (preambleActive && !dio1Fired) {
        uint16_t irq = radio.getIrqFlags();
        if (irq & IRQ_HEADER_ERR) {
          preambleActive = false;
          lastRssiSampleUs = now;
          invalidRxCount++;
          Serial.println("RX: header error (preamble without valid packet)");
          radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
          if (!hopEnabled) radioStartRx();
          break;
        }
      }

      // --- Handle DIO1: full packet received ---
      if (dio1Fired) {
        dio1Fired = false;

        uint8_t rxBuf[64];
        int state = radio.readData(rxBuf, sizeof(rxBuf));

        if (state == RADIOLIB_ERR_NONE) {
          size_t rxLen = radio.getPacketLength();
          float pktRssi = radio.getRSSI(true);
          float pktSnr  = radio.getSNR();

          processReceivedPacket(rxBuf, rxLen, (int8_t)pktRssi, (int8_t)pktSnr);
        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
          Serial.println("RX: CRC fail");
          invalidRxCount++;
        } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
          Serial.print("RX err: ");
          Serial.println(state);
          invalidRxCount++;
        }

        preambleActive = false;
        lastRssiSampleUs = now;
        // If CMD_SET_HOPPING just enabled timed windows, go to standby and let
        // the window boundary logic take over. Do not restart continuous RX.
        if (hopEnabled) {
          radio.standby();
          radioState = RADIO_OFF;
        } else {
          radioStartRx();
        }
        break;
      }

      // --- RSSI sampling and EMA update (excluding preamble windows) ---
      if (!preambleActive && (now - lastRssiSampleUs) >= RSSI_SAMPLE_INTERVAL_US) {
        double instantRssi = (double)radio.getRSSI(false);

        if (!rssiEmaInitialised) {
          rssiEma = instantRssi;
          rssiEmaInitialised = true;
        } else {
          unsigned long dtUs = now - lastRssiSampleUs;
          double alpha = (double)dtUs / (double)RSSI_EMA_TAU_US;
          if (alpha > 1.0) alpha = 1.0;
          rssiEma += alpha * (instantRssi - rssiEma);
        }
        lastRssiSampleUs = now;
        logPages[LOGI_RADIO_HEALTH].freshMask |= 0xFF;

        double busyThreshold = rssiEma + RSSI_BUSY_THRESHOLD_DB;
        if (instantRssi > busyThreshold) {
          lastRssiHighUs = now;
        }
      }

      // --- TX scheduling (fixed schedule) ---
      if (!txSendingEnabled || txIntervalUs == 0) {
        // Keep schedule pointer moving so we don't burst-send when re-enabled
        if (txIntervalUs > 0 && (long)(now - nextTxDueUs) >= 0) {
          do {
            nextTxDueUs += txIntervalUs;
          } while ((long)(now - nextTxDueUs) >= 0);
          txDeferredThisSlot = false;
        }
        break;
      }

      if ((long)(now - nextTxDueUs) >= 0) {
        unsigned long overdueUs = now - nextTxDueUs;

        bool blocked = false;
        if (preambleActive)                                blocked = true;
        if ((long)(keepoutUntilUs - now) > 0)              blocked = true;
        if ((now - lastRssiHighUs) < RSSI_HIGH_HOLDOFF_US) blocked = true;

        bool forceTx = (overdueUs >= TX_MAX_DEFER_US);

        if (blocked && !forceTx) {
          if (!txDeferredThisSlot && overdueUs >= TX_LATE_THRESHOLD_US) {
            delayedTxCount++;
            txDeferredThisSlot = true;
          }
          break;
        }

        // Advance schedule to next slot
        do {
          nextTxDueUs += txIntervalUs;
        } while ((long)(now - nextTxDueUs) >= 0);
        txDeferredThisSlot = false;

        uint8_t pkt[32];
        size_t len = buildTelemetryPacket(pkt);

        Serial.print("TX: ");
        printPacketHex(pkt, len);
        Serial.print(" (");
        Serial.print(len);
        Serial.print("B pg=0x");
        Serial.print(pkt[10], HEX);
        if (forceTx) Serial.print(" FORCED");
        Serial.println(")");

        radioStartTx(pkt, len);
      }
      break;
    }

    case RADIO_RX_RECEIVING:
      break;
  }
}
