// radio.cpp — LoRa radio hardware, TX/RX state machine, slot-based scheduling.
// See radio.h for the public API.

#include <Arduino.h>
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

// ===================== SLOT CLOCK STATE =====================

bool          radioSynced        = false;
unsigned long syncAnchorUs       = 0;
uint32_t      syncSlotIndex      = 0;
uint32_t      lastHandledSlotNum = 0xFFFFFFFF;  // invalid sentinel

// ===================== RSSI EMA (stub) =====================

double rssiEma = 0.0;  // not sampled in slot-sync mode; kept for telemetry page 0x0C

// ===================== STATS =====================

uint16_t delayedTxCount = 0;
uint16_t invalidRxCount = 0;

// ===================== DIO1 ISR =====================

void IRAM_ATTR dio1ISR() {
  dio1Fired = true;
}

// ===================== HELPERS =====================

void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz = (activeChannel < 64) ? 125.0f : 500.0f;
}

// LED is configured as PWM via ledcAttach in setup(). Use ledcWrite, not digitalWrite.
// 255 = 100% duty = full brightness (short flashes, no time for PWM dimming to matter).
static void ledOn()  { ledcWrite(LED_PIN, 255); }
static void ledOff() { ledcWrite(LED_PIN, 0);   }

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

// ===================== SYNC =====================

void radioSetSynced(unsigned long anchorUs, uint8_t slotIdx) {
  syncAnchorUs       = anchorUs;
  syncSlotIndex      = slotIdx;
  lastHandledSlotNum = 0xFFFFFFFF;  // force re-entry on next slot
  radioSynced        = true;
  Serial.print("SYNC: anchor="); Serial.print(anchorUs);
  Serial.print("us slot="); Serial.println(slotIdx);
}

// ===================== RX / TX =====================

void radioStartRx() {
  int state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    radioState = RADIO_RX_ACTIVE;
  } else {
    Serial.print("RX start fail: ");
    Serial.println(state);
    radioState = RADIO_IDLE;
  }
  dio1Fired = false;
}

bool radioStartTx(const uint8_t* pkt, size_t len) {
  int state = radio.startTransmit(pkt, len);
  if (state == RADIOLIB_ERR_NONE) {
    radioState = RADIO_TX_ACTIVE;
    dio1Fired  = false;
    return true;
  } else {
    Serial.print("TX start fail: ");
    Serial.println(state);
    radio.standby();
    radioState = RADIO_IDLE;
    return false;
  }
}

// ===================== RX PACKET HANDLER =====================

static void handleRxDone() {
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
  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // normal timeout — no log spam
  } else {
    Serial.print("RX err: ");
    Serial.println(state);
    invalidRxCount++;
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================
//
// Bootstrap (radioSynced == false):
//   Continuous RX — waiting for CMD_SET_SYNC.
//   Any received packet is dispatched via processReceivedPacket().
//
// Synced (radioSynced == true):
//   Slot clock drives behaviour. One action per slot.
//   WIN_TELEM: rocket TX telemetry, LED on during TX.
//   WIN_RX:    rocket RX commands, LED on during RX window.
//   WIN_OFF:   radio standby, LED off.

void nonblockingRadio() {
  if (!loraReady) return;

  unsigned long now = micros();

  // ---- Bootstrap mode: continuous RX ----
  if (!radioSynced) {
    switch (radioState) {
      case RADIO_OFF:
        radioStartRx();
        break;

      case RADIO_IDLE:
        radioStartRx();
        break;

      case RADIO_TX_ACTIVE:
        if (dio1Fired) {
          dio1Fired = false;
          radioStartRx();
        }
        break;

      case RADIO_RX_ACTIVE:
        if (dio1Fired) {
          dio1Fired = false;
          handleRxDone();
          // After CMD_SET_SYNC, radioSynced may now be true.
          // If still not synced, keep listening.
          if (!radioSynced) {
            radioStartRx();
          }
        }
        break;
    }
    return;
  }

  // ---- Synced mode: slot-based scheduling ----

  unsigned long elapsed    = now - syncAnchorUs;
  uint32_t      slotNum    = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot  = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx     = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win        = SLOT_SEQUENCE[seqIdx];

  // Handle DIO1 completion for any in-progress TX/RX first,
  // regardless of which slot we're currently in.
  if (dio1Fired) {
    dio1Fired = false;
    if (radioState == RADIO_TX_ACTIVE) {
      ledOff();
      radio.standby();
      radioState = RADIO_IDLE;
      Serial.print("SLOT TxDone posInSlot="); Serial.println(posInSlot);
    } else if (radioState == RADIO_RX_ACTIVE) {
      ledOff();
      handleRxDone();
      // handleRxDone may have set radioSynced=true again if re-sync happened
      radio.standby();
      radioState = RADIO_IDLE;
      Serial.print("SLOT RxDone posInSlot="); Serial.println(posInSlot);
    }
    return;
  }

  // Only act once per slot transition.
  if (slotNum == lastHandledSlotNum) return;

  // Don't preempt an in-progress operation from a previous slot.
  if (radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE) return;

  // New slot — act.
  lastHandledSlotNum = slotNum;

  switch (win) {

    case WIN_TELEM: {
      if (!txSendingEnabled) {
        // TX suppressed by user (button toggle). Skip this slot.
        Serial.println("SLOT WIN_TELEM skipped (TX disabled)");
        break;
      }
      uint8_t pkt[32];
      size_t len = buildTelemetryPacket(pkt);
      Serial.print("SLOT WIN_TELEM TX: ");
      printPacketHex(pkt, len);
      Serial.print(" ("); Serial.print(len); Serial.print("B pg=0x");
      Serial.print(pkt[10], HEX); Serial.println(")");
      ledOn();
      if (!radioStartTx(pkt, len)) {
        ledOff();
      }
      break;
    }

    case WIN_RX: {
      Serial.println("SLOT WIN_RX: starting receive");
      ledOn();
      // startReceive with timeout (ms). DIO1 fires on RxDone or RxTimeout.
      int st = radio.startReceive((uint32_t)BS_RX_TIMEOUT_MS);
      if (st == RADIOLIB_ERR_NONE) {
        radioState = RADIO_RX_ACTIVE;
        dio1Fired  = false;
      } else {
        ledOff();
        Serial.print("WIN_RX startReceive fail: "); Serial.println(st);
        radioState = RADIO_IDLE;
      }
      break;
    }

    case WIN_OFF:
    default:
      if (radioState != RADIO_IDLE) {
        radio.standby();
        radioState = RADIO_IDLE;
      }
      ledOff();
      break;
  }
}
