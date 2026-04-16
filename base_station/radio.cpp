// base_station/radio.cpp — LoRa radio, slot machine, and sync logic for base station.

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"

// ===================== HARDWARE OBJECTS =====================

SPIClass bsLoraSPI(FSPI);

sx126x_hal_context_t bsRadioCtx = {
  .spi      = &bsLoraSPI,
  .nss      = LORA_NSS_PIN,
  .busy     = LORA_BUSY_PIN,
  .rst      = LORA_RST_PIN,
  .initMode = true,  // cleared to false at end of bsRadioInit()
};

// ===================== ACTIVE RADIO CONFIG =====================

uint8_t activeChannel = DEFAULT_CHANNEL;
uint8_t activeSF      = DEFAULT_SF;
int8_t  activePower   = DEFAULT_POWER;
float   activeFreqMHz = 917.5f;
float   activeBwKHz   = 500.0f;

uint8_t bhChannel = DEFAULT_BH_CHANNEL;
uint8_t bhSF      = DEFAULT_BH_SF;
int8_t  bhPower   = DEFAULT_BH_POWER;

// ===================== RADIO STATE =====================

BsRadioState bsRadioState = BS_RADIO_STANDBY;
bool         bsLoraReady  = false;

// ===================== SLOT CLOCK STATE =====================

bool          bsSynced           = false;
unsigned long bsSyncAnchorUs     = 0;
uint32_t      bsSyncSlotIndex    = 0;
uint32_t      bsLastHandledSlot  = 0xFFFFFFFF;
unsigned long bsMissedTelemSlots = 0;

uint64_t      bsPendingHeaderValidUs = 0;

float         bsBgRssiEma = 0.0f;
static bool   bsBgRssiInit = false;
#define BG_RSSI_ALPHA  0.1f

// ===================== SYNC BOOKKEEPING =====================

static unsigned long bsSyncBootMs     = 0;
static unsigned long bsLastSyncSendMs = 0;
#define BS_SYNC_RETRY_MS  60000UL

// ===================== HELPERS =====================

void bsUpdateActiveFreqBw() {
  activeFreqMHz = bsChannelToFreqMHz(activeChannel);
  activeBwKHz   = (activeChannel < 64) ? 125.0f : 500.0f;
}

static sx126x_lora_bw_t bwKHzToEnum(float bwKHz) {
  if (bwKHz >= 490.0f) return SX126X_LORA_BW_500;
  if (bwKHz >= 240.0f) return SX126X_LORA_BW_250;
  return SX126X_LORA_BW_125;
}

static void bsLedOn()  { ledcWrite(LED_PIN, 255); }
static void bsLedOff() { ledcWrite(LED_PIN, 0);   }

// Wait for BUSY to deassert. INIT ONLY — blocks. Max ~5ms; BUSY clears in <1ms after standby.
// Only called from dispatchCmdTx after bsRadioStandby(), never from the main loop directly.
static bool bsWaitBusyShort() {
  unsigned long t0 = millis();
  while (digitalRead(LORA_BUSY_PIN)) {
    if (millis() - t0 > 5) {
      Serial.println("BS TX: BUSY stuck after standby");
      return false;
    }
  }
  return true;
}

// ===================== INIT =====================

bool bsRadioInit() {
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  sx126x_hal_reset(&bsRadioCtx);
  if (!radioWaitBusy(&bsRadioCtx, 100)) {
    Serial.println("LoRa init: BUSY stuck after reset");
    return false;
  }

  if (sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_standby failed");
    return false;
  }

  if (sx126x_set_pkt_type(&bsRadioCtx, SX126X_PKT_TYPE_LORA) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_pkt_type failed");
    return false;
  }

  sx126x_set_dio2_as_rf_sw_ctrl(&bsRadioCtx, true);

  const uint8_t syncWord[2] = { 0x14, 0x24 };
  sx126x_write_register(&bsRadioCtx, 0x0740, syncWord, 2);

  bsRadioApplyConfig();

  sx126x_set_dio_irq_params(&bsRadioCtx,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
    SX126X_IRQ_NONE,
    SX126X_IRQ_NONE);

  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);

  radioMcpwmInit(LORA_DIO1_PIN);
  Serial.print("DIO1 interrupt attached on GPIO"); Serial.println(LORA_DIO1_PIN);

  bsRadioCtx.initMode = false;

  bsSyncBootMs     = millis();
  bsLastSyncSendMs = 0;
  bsRadioState     = BS_RADIO_STANDBY;
  return true;
}

void bsRadioApplyConfig() {
  sx126x_mod_params_lora_t modParams = {};
  modParams.sf   = (sx126x_lora_sf_t)activeSF;
  modParams.bw   = bwKHzToEnum(activeBwKHz);
  modParams.cr   = SX126X_LORA_CR_4_5;
  modParams.ldro = 0;
  sx126x_set_lora_mod_params(&bsRadioCtx, &modParams);
  radioWaitBusy(&bsRadioCtx);

  sx126x_pkt_params_lora_t pktParams = {};
  pktParams.preamble_len_in_symb = LORA_PREAMBLE;
  pktParams.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pktParams.pld_len_in_bytes     = 255;
  pktParams.crc_is_on            = true;
  pktParams.invert_iq_is_on      = false;
  sx126x_set_lora_pkt_params(&bsRadioCtx, &pktParams);
  radioWaitBusy(&bsRadioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&bsRadioCtx, freqHz);
  radioWaitBusy(&bsRadioCtx);

  sx126x_set_tx_params(&bsRadioCtx, activePower, SX126X_RAMP_200_US);
  radioWaitBusy(&bsRadioCtx);

  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  sx126x_set_pa_cfg(&bsRadioCtx, &paCfg);
  radioWaitBusy(&bsRadioCtx);
}

// ===================== RX / TX =====================

void bsRadioStartRx() {
  if (digitalRead(LORA_BUSY_PIN)) { Serial.println("BS RX: BUSY — skip"); return; }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  if (sx126x_set_rx_with_timeout_in_rtc_step(&bsRadioCtx, SX126X_RX_CONTINUOUS) == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_RX_ACTIVE;
    Serial.println("BS RX started");
  } else {
    Serial.println("BS RX start fail");
    bsRadioState = BS_RADIO_STANDBY;
  }
}

void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps) {
  if (digitalRead(LORA_BUSY_PIN)) { Serial.println("BS RX(timeout): BUSY — skip"); return; }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  if (sx126x_set_rx_with_timeout_in_rtc_step(&bsRadioCtx, timeoutRtcSteps) == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_RX_ACTIVE;
  } else {
    Serial.println("BS RX(timeout) start fail");
    bsRadioState = BS_RADIO_STANDBY;
  }
}

bool bsRadioStartTx(const uint8_t* pkt, size_t len) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.print("BS TX: BUSY=1 before write_buffer — drop");
    return false;
  }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  Serial.print("BS TX: writing "); Serial.print(len); Serial.println("B to FIFO...");
  if (sx126x_write_buffer(&bsRadioCtx, 0, pkt, (uint8_t)len) != SX126X_STATUS_OK) {
    Serial.println("BS TX: write_buffer fail"); return false;
  }
  Serial.println("BS TX: starting TX...");
  if (sx126x_set_tx(&bsRadioCtx, 0) == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_TX_ACTIVE;
    Serial.println("BS TX: active (waiting for TxDone on DIO1)");
    return true;
  }
  Serial.println("BS TX start fail");
  bsRadioState = BS_RADIO_STANDBY;
  return false;
}

void bsRadioStandby() {
  sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC);
  bsRadioState = BS_RADIO_STANDBY;
}

// ===================== SYNC =====================

void bsSetSyncedFromTx(uint64_t anchorUs) {
  bsSyncAnchorUs     = (unsigned long)anchorUs;
  bsSyncSlotIndex    = 1;
  bsLastHandledSlot  = 0xFFFFFFFF;
  bsSynced           = true;
  bsMissedTelemSlots = 0;
  Serial.print("BS SYNCED anchor="); Serial.print(bsSyncAnchorUs); Serial.println("us");
}

// ===================== RX PACKET HANDLER =====================

static void bsHandleRxDone() {
  sx126x_rx_buffer_status_t bufStatus = {};
  if (sx126x_get_rx_buffer_status(&bsRadioCtx, &bufStatus) != SX126X_STATUS_OK) {
    Serial.println("BS RX: get_rx_buffer_status fail");
    return;
  }

  uint8_t buf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(buf)) {
    Serial.print("BS RX: bad length "); Serial.println(rxLen);
    return;
  }

  if (sx126x_read_buffer(&bsRadioCtx, bufStatus.buffer_start_pointer, buf, rxLen) != SX126X_STATUS_OK) {
    Serial.println("BS RX: read_buffer fail");
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&bsRadioCtx, &pktStatus);
  float snrF  = (float)pktStatus.snr_pkt_in_db;
  float rssiF = (float)pktStatus.rssi_pkt_in_dbm;

  bool isTelemetry = (rxLen >= 10 && buf[0] == 0xAF && buf[1] == ROCKET_DEVICE_ID);
  if (isTelemetry) bsMissedTelemSlots = 0;

  // Print received packet bytes
  Serial.print("BS RX "); Serial.print(rxLen); Serial.print("B snr:");
  Serial.print(snrF, 1); Serial.print(" rssi:"); Serial.print(rssiF, 0);
  Serial.print(" [");
  for (uint8_t i = 0; i < rxLen && i < 8; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX); Serial.print(" ");
  }
  if (rxLen > 8) Serial.print("...");
  Serial.println("]");

  bsOnPacketReceived(buf, rxLen, snrF, rssiF);
}

// ===================== IRQ HANDLER =====================

static void bsRadioHandleIrq() {
  uint64_t eventUs = dio1TimestampUs();
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  if (sx126x_get_and_clear_irq_status(&bsRadioCtx, &irqFlags) != SX126X_STATUS_OK) {
    Serial.println("BS IRQ: get_and_clear failed");
    bsRadioState = BS_RADIO_STANDBY;
    return;
  }

  Serial.print("BS IRQ flags=0x"); Serial.println(irqFlags, HEX);

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    bsLedOff();
    bsRadioState = BS_RADIO_STANDBY;
    Serial.print("BS TxDone ts="); Serial.println((unsigned long)eventUs);
    if (bsSyncTxInFlight) {
      bsSyncTxInFlight = false;
      bsSetSyncedFromTx(eventUs);
    }
    // Restart RX immediately after TX completes
    bsRadioStartRx();
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    bsLedOn();
    bsRadioState = BS_RADIO_STANDBY;
    bsHandleRxDone();
    bsLedOff();
    // Restart RX
    bsRadioStartRx();
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    bsLedOff();
    int16_t rssiDbm = 0;
    if (sx126x_get_rssi_inst(&bsRadioCtx, &rssiDbm) == SX126X_STATUS_OK) {
      if (!bsBgRssiInit) { bsBgRssiEma = (float)rssiDbm; bsBgRssiInit = true; }
      else bsBgRssiEma += BG_RSSI_ALPHA * ((float)rssiDbm - bsBgRssiEma);
    }
    bsRadioState = BS_RADIO_STANDBY;
    bsRadioStartRx();
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    Serial.println("BS RX: CRC/header error");
    bsRadioState = BS_RADIO_STANDBY;
    bsRadioStartRx();
  }
}

// ===================== SYNC MANAGEMENT =====================

bool bsSyncNeedsQueue  = false;
bool bsSyncTxInFlight  = false;
bool bsWinCmdReady     = false;

void bsHandleSyncSend() {
  unsigned long now = millis();

  bool timeForSync = (bsLastSyncSendMs == 0) && ((now - bsSyncBootMs) >= BS_SYNC_BOOT_DELAY_MS);
  if (bsLastSyncSendMs != 0 && (now - bsLastSyncSendMs) >= BS_SYNC_RETRY_MS) {
    timeForSync = true;
  }

  if (!timeForSync) return;

  bsLastSyncSendMs = now;  // set immediately so it doesn't retrigger next loop
  bsSyncNeedsQueue = true;
  Serial.print("BS SYNC: time to send (synced="); Serial.print(bsSynced); Serial.println(")");
}

// ===================== MAIN RADIO UPDATE =====================
// This function only handles IRQs and slot-clock bookkeeping.
// TX dispatch and RX management is done in main.cpp / IRQ handler.

static bool bsCmdSentThisSlot = false;

void bsHandleRadio() {
  if (dio1Fired) {
    bsRadioHandleIrq();
  }

  // Slot-clock bookkeeping (only when synced; RX/TX driven by main.cpp + IRQ handler)
  if (!bsSynced) return;

  unsigned long now       = micros();
  unsigned long elapsed   = now - bsSyncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((bsSyncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];

  if (slotNum != bsLastHandledSlot) {
    bsCmdSentThisSlot = false;
    bsLastHandledSlot = slotNum;
    if (win == WIN_TELEM) {
      bsMissedTelemSlots++;
      Serial.print("SLOT WIN_TELEM #"); Serial.print(slotNum);
      Serial.print(" pos="); Serial.print(posInSlot); Serial.println("us");
    } else if (win == WIN_CMD) {
      Serial.print("SLOT WIN_CMD #"); Serial.print(slotNum);
      Serial.print(" pos="); Serial.print(posInSlot); Serial.println("us");
    }
  }

  if (win == WIN_CMD && !bsCmdSentThisSlot && posInSlot >= BS_CMD_TX_OFFSET_US) {
    bsCmdSentThisSlot = true;
    bsWinCmdReady     = true;
  }
}
