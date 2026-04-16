// radio.cpp — LoRa radio hardware, TX/RX state machine, slot-based scheduling.
// See radio.h for the public API.

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"  // sx126x_hal_reset/wakeup declarations, sx126x_hal_status_t
#include "telemetry.h"
#include "commands.h"
#include "globals.h"

// ===================== HARDWARE OBJECTS =====================

SPIClass loraSPI(FSPI);

sx126x_hal_context_t radioCtx = {
  .spi      = &loraSPI,
  .nss      = LORA_NSS_PIN,
  .busy     = LORA_BUSY_PIN,
  .rst      = LORA_RST_PIN,
  .initMode = true,  // cleared to false at end of radioInit()
};

// ===================== ACTIVE RADIO CONFIG =====================

uint8_t activeChannel = DEFAULT_CHANNEL;
uint8_t activeSF      = DEFAULT_SF;
int8_t  activePower   = DEFAULT_POWER;
float   activeFreqMHz = 917.5f;
float   activeBwKHz   = 500.0f;

// ===================== RADIO STATE =====================

bool        loraReady  = false;
RadioState  radioState = RADIO_OFF;

// ===================== SLOT CLOCK STATE =====================

bool          radioSynced        = false;
unsigned long syncAnchorUs       = 0;
uint32_t      syncSlotIndex      = 0;
uint32_t      lastHandledSlotNum = 0xFFFFFFFF;

// ===================== RSSI EMA =====================
// Updated at WIN_CMD timeout — channel noise sampled when no command arrives.

double rssiEma = 0.0;
static bool rssiEmaInit = false;
#define RSSI_EMA_ALPHA 0.1

// ===================== STATS =====================

uint16_t delayedTxCount = 0;
uint16_t invalidRxCount = 0;

// ===================== HELPERS =====================

void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz   = (activeChannel < 64) ? 125.0f : 500.0f;
}

static sx126x_lora_bw_t bwKHzToEnum(float bwKHz) {
  if (bwKHz >= 490.0f) return SX126X_LORA_BW_500;
  if (bwKHz >= 240.0f) return SX126X_LORA_BW_250;
  return SX126X_LORA_BW_125;
}

static sx126x_lora_sf_t sfToEnum(uint8_t sf) {
  return (sx126x_lora_sf_t)sf;
}

static void ledOn()  { ledcWrite(LED_PIN, 255); }
static void ledOff() { ledcWrite(LED_PIN, 0);   }

// ===================== INIT =====================

bool radioInit() {
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  // HAL is in initMode so it spins on BUSY between each subsequent command automatically.
  sx126x_hal_reset(&radioCtx);
  if (!radioWaitBusy(&radioCtx, 100)) {
    Serial.println("LoRa init: BUSY stuck after reset");
    return false;
  }

  if (sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_standby failed");
    return false;
  }

  if (sx126x_set_pkt_type(&radioCtx, SX126X_PKT_TYPE_LORA) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_pkt_type failed");
    return false;
  }

  // DIO2 as RF switch control
  sx126x_set_dio2_as_rf_sw_ctrl(&radioCtx, true);

  // Sync word 0x12 (private network) — register pair 0x0740/0x0741 = 0x14, 0x24
  {
    const uint8_t syncWord[2] = { 0x14, 0x24 };
    sx126x_write_register(&radioCtx, 0x0740, syncWord, 2);
  }

  radioApplyConfig();

  // IRQ mask: TX_DONE | RX_DONE | TIMEOUT — all on DIO1
  sx126x_set_dio_irq_params(&radioCtx,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
    SX126X_IRQ_NONE,
    SX126X_IRQ_NONE);

  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);

  // Attach GPIO interrupt on DIO1
  radioMcpwmInit(LORA_DIO1_PIN);

  // Init complete — switch HAL back to drop-on-BUSY (non-blocking, safe for armed loop)
  radioCtx.initMode = false;

  radioState = RADIO_STANDBY;
  return true;
}

void radioApplyConfig() {
  sx126x_mod_params_lora_t modParams = {};
  modParams.sf = sfToEnum(activeSF);
  modParams.bw = bwKHzToEnum(activeBwKHz);
  modParams.cr = SX126X_LORA_CR_4_5;
  modParams.ldro = 0;
  sx126x_set_lora_mod_params(&radioCtx, &modParams);
  radioWaitBusy(&radioCtx);

  sx126x_pkt_params_lora_t pktParams = {};
  pktParams.preamble_len_in_symb = LORA_PREAMBLE;
  pktParams.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pktParams.pld_len_in_bytes     = 255;
  pktParams.crc_is_on            = true;
  pktParams.invert_iq_is_on      = false;
  sx126x_set_lora_pkt_params(&radioCtx, &pktParams);
  radioWaitBusy(&radioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&radioCtx, freqHz);
  radioWaitBusy(&radioCtx);

  sx126x_set_tx_params(&radioCtx, activePower, SX126X_RAMP_200_US);
  radioWaitBusy(&radioCtx);

  // PA config for SX1262 (device_sel=0, hp_max=7, pa_lut=1 per datasheet §13.1.14)
  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  sx126x_set_pa_cfg(&radioCtx, &paCfg);
  radioWaitBusy(&radioCtx);
}

// ===================== SYNC =====================

void radioSetSynced(unsigned long anchorUs, uint8_t slotIdx) {
  radioStandby();
  dio1Fired              = false;
  radioState             = RADIO_STANDBY;
  syncAnchorUs           = anchorUs;
  syncSlotIndex          = slotIdx;
  lastHandledSlotNum     = 0xFFFFFFFF;
  radioSynced            = true;
  Serial.print("SYNC: anchor="); Serial.print(anchorUs);
  Serial.print("us slot="); Serial.println(slotIdx);
}

// ===================== RX / TX =====================

void radioStartRx() {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("RX: BUSY — skip");
    return;
  }
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  if (sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, SX126X_RX_CONTINUOUS) == SX126X_STATUS_OK) {
    radioState = RADIO_RX_ACTIVE;
  } else {
    Serial.println("RX start fail");
    radioState = RADIO_STANDBY;
  }
}

void radioStartRxTimeout(uint32_t timeoutRtcSteps) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("RX(timeout): BUSY — skip");
    return;
  }
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  if (sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, timeoutRtcSteps) == SX126X_STATUS_OK) {
    radioState = RADIO_RX_ACTIVE;
  } else {
    Serial.println("RX(timeout) start fail");
    radioState = RADIO_STANDBY;
  }
}

bool radioStartTx(const uint8_t* pkt, size_t len) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("TX: BUSY — skip");
    return false;
  }
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  if (sx126x_write_buffer(&radioCtx, 0, pkt, (uint8_t)len) != SX126X_STATUS_OK) {
    Serial.println("TX: write_buffer fail");
    return false;
  }
  if (sx126x_set_tx(&radioCtx, 0) == SX126X_STATUS_OK) {
    radioState = RADIO_TX_ACTIVE;
    return true;
  }
  Serial.println("TX start fail");
  radioState = RADIO_STANDBY;
  return false;
}

void radioStandby() {
  sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  radioState = RADIO_STANDBY;
}

// ===================== IRQ / RX PACKET HANDLER =====================

static void handleRxDone() {
  sx126x_rx_buffer_status_t bufStatus = {};
  if (sx126x_get_rx_buffer_status(&radioCtx, &bufStatus) != SX126X_STATUS_OK) {
    Serial.println("RX: get_rx_buffer_status fail");
    invalidRxCount++;
    return;
  }

  uint8_t rxBuf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(rxBuf)) {
    Serial.print("RX: bad length "); Serial.println(rxLen);
    invalidRxCount++;
    return;
  }

  if (sx126x_read_buffer(&radioCtx, bufStatus.buffer_start_pointer, rxBuf, rxLen) != SX126X_STATUS_OK) {
    Serial.println("RX: read_buffer fail");
    invalidRxCount++;
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&radioCtx, &pktStatus);
  int8_t pktRssi = pktStatus.rssi_pkt_in_dbm;
  int8_t pktSnr  = pktStatus.snr_pkt_in_db;

  bool structOk = (rxLen >= 2) && (rxBuf[0] == PKT_TELEMETRY || rxBuf[0] == PKT_COMMAND ||
                                    rxBuf[0] == PKT_BACKHAUL);
  if (!structOk) {
    Serial.print("RX: unknown type 0x"); Serial.println(rxBuf[0], HEX);
    invalidRxCount++;
    return;
  }

  processReceivedPacket(rxBuf, rxLen, pktRssi, pktSnr);
}

static void radioHandleIrq() {
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_get_and_clear_irq_status(&radioCtx, &irqFlags);

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    ledOff();
    radioState = RADIO_STANDBY;
    Serial.println("TxDone");
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    ledOn();  // brief flash: LED on, packet processed, then off
    handleRxDone();
    ledOff();
    radioState = RADIO_STANDBY;
    Serial.println("RxDone");
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    ledOff();
    // Sample background RSSI on CMD window timeout (noise floor estimate)
    int16_t rssiDbm = 0;
    if (sx126x_get_rssi_inst(&radioCtx, &rssiDbm) == SX126X_STATUS_OK) {
      if (!rssiEmaInit) { rssiEma = rssiDbm; rssiEmaInit = true; }
      else rssiEma += RSSI_EMA_ALPHA * (rssiDbm - rssiEma);
    }
    radioState = RADIO_STANDBY;
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    radioState = RADIO_STANDBY;
    invalidRxCount++;
    Serial.println("RX: CRC/header error");
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================
//
// Behaviour:
//   Always TX telem periodically (txIntervalUs before sync; WIN_TELEM slots after sync).
//   Always RX whenever not transmitting — continuous RX, TX preempts when it's time.
//   LED flashes briefly on each received packet.

static unsigned long lastTelemTxUs = 0;

void nonblockingRadio() {
  if (!loraReady) return;

  // Always process DIO1 first
  if (dio1Fired) {
    radioHandleIrq();
  }

  // If radio is in standby, start RX — we want to always be listening
  if (radioState == RADIO_STANDBY) {
    radioStartRx();
    // Fall through — check if we need to TX right now (will preempt the just-started RX)
  }

  unsigned long now = micros();

  if (!radioSynced) {
    // ---- Bootstrap: TX telem at txIntervalUs rate, RX between ----
    if (txSendingEnabled && (now - lastTelemTxUs) >= txIntervalUs) {
      if (radioState == RADIO_RX_ACTIVE) radioStandby();
      if (radioState == RADIO_STANDBY) {
        uint8_t pkt[32];
        size_t len = buildTelemetryPacket(pkt);
        Serial.print("TELEM TX (no sync) len="); Serial.println(len);
        lastTelemTxUs = now;
        ledOn();
        if (!radioStartTx(pkt, len)) {
          ledOff();
        }
      }
    }
    return;
  }

  // ---- Synced mode: slot-based TX ----
  if (radioState == RADIO_TX_ACTIVE) return;  // TX in progress, wait for TxDone IRQ

  unsigned long elapsed   = now - syncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];

  if (win == WIN_TELEM && slotNum != lastHandledSlotNum) {
    lastHandledSlotNum = slotNum;
    if (!txSendingEnabled) {
      Serial.println("SLOT WIN_TELEM skipped (TX disabled)");
      return;
    }
    // Preempt RX and TX telem
    if (radioState == RADIO_RX_ACTIVE) radioStandby();
    uint8_t pkt[32];
    size_t len = buildTelemetryPacket(pkt);
    Serial.print("SLOT WIN_TELEM TX len="); Serial.println(len);
    lastTelemTxUs = now;
    ledOn();
    if (!radioStartTx(pkt, len)) {
      ledOff();
    }
  }

  if (win == WIN_CMD && slotNum != lastHandledSlotNum) {
    lastHandledSlotNum = slotNum;
    Serial.print("SLOT WIN_CMD pos="); Serial.print(posInSlot); Serial.println("us (listening)");
    // RX is already running continuously; nothing extra needed
  }
}
