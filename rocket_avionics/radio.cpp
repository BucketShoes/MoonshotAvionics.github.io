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
//   TX uses a fixed schedule (t=0, t=interval, t=2*interval, ...).
//   Deferred TX (channel busy) does not drift the schedule.
//   Hard ceiling: TX anyway after TX_MAX_DEFER_US regardless of blockers.
//   Deferral reasons:
//     1. Preamble detected (matching modulation — no processing gain)
//     2. Post-TX keepout active
//     3. High RSSI within last RSSI_HIGH_HOLDOFF_US (non-matching signal — polite defer)
//   RSSI EMA: samples ALL non-preamble time, including high-RSSI from non-matching signals.
//
// Timed window mode (hopEnabled == true):
//   420ms windows. Even index = WIN_RX (40ms timeout). Odd index = WIN_TELEM.
//   Window transitions are detected by comparing hopCurrentWindowIndex() to lastWindowIndex.
//   On WIN_TELEM: build and TX packet immediately. LED on if FLASH_SYNC_TIMING.
//   On WIN_RX: open 40ms RX window. LED on if FLASH_SYNC_TIMING.
//   DIO1 in WIN_RX: packet received, clear LED, restart standby (next window picks up).
//   DIO1 in WIN_TELEM: TX complete, clear LED, go to standby (not back to RX).

static void hopLedSet(bool on) {
#if FLASH_SYNC_TIMING
  ledcWrite(LED_PIN, on ? 255 : 0);
#endif
  (void)on;
}

void nonblockingRadio() {
  if (!loraReady) return;

  unsigned long now = micros();

  // ===================== TIMED WINDOW MODE =====================
  //
  // Single-TX and single-RX modes: the radio goes to standby automatically
  // after TxDone or after the RX timeout/packet. DIO1 fires in both cases.
  //
  // Window boundaries only switch mode when the radio is idle (not mid-TX or
  // mid-RX). A packet arriving late may straddle a window boundary — that's fine,
  // we let it finish and pick up the correct mode on the next boundary check.

  if (hopEnabled) {
    static uint32_t lastWindowIndex = 0xFFFFFFFF;
    uint32_t winIdx = hopCurrentWindowIndex();

    // Handle DIO1 events from previous TX or RX.
    if (dio1Fired) {
      dio1Fired = false;
      hopLedSet(false);

      if (radioState == RADIO_TX_ACTIVE) {
        // TxDone: radio goes to standby automatically in single-TX mode.
        radioState = RADIO_OFF;

      } else if (radioState == RADIO_RX_LISTENING) {
        // RxDone or RX timeout: radio goes to standby automatically.
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
          Serial.print("RX err: "); Serial.println(state);
          invalidRxCount++;
        }
        preambleActive = false;
        radioState = RADIO_OFF;
      }
    }

    // Window boundary check — only act if radio is idle (not mid-TX or mid-RX).
    if (winIdx != lastWindowIndex && radioState == RADIO_OFF) {
      lastWindowIndex = winIdx;
      WindowType wt = hopCurrentWindowType();

      if (wt == WIN_TELEM) {
        Serial.print("WIN "); Serial.print(winIdx);
        Serial.print(": TELEM ch="); Serial.println(activeChannel);

        uint8_t pkt[32];
        size_t len = buildTelemetryPacket(pkt);
        hopLedSet(true);
        radioStartTx(pkt, len);

      } else {  // WIN_RX
        Serial.print("WIN "); Serial.print(winIdx);
        Serial.print(": RX ch="); Serial.println(activeChannel);

        hopLedSet(true);
        int rxState = radio.startReceive(HOP_RX_TIMEOUT_MS);
        if (rxState == RADIOLIB_ERR_NONE) {
          radioState = RADIO_RX_LISTENING;
        } else {
          Serial.print("RX win start fail: "); Serial.println(rxState);
          radioState = RADIO_OFF;
        }
        dio1Fired = false;
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
          radioStartRx();
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
        radioStartRx();
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
