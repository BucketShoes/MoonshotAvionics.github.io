// base_station/radio.cpp — LoRa radio, slot machine, and sync logic for base station.
// See radio.h for the public API.

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"  // sx126x_hal_reset/wakeup declarations, sx126x_hal_status_t

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

// Background RSSI EMA — updated at RX timeout, not on received packets.
float         bsBgRssiEma = 0.0f;
static bool   bsBgRssiInit = false;
#define BG_RSSI_ALPHA  0.1f

// ===================== SYNC BOOKKEEPING =====================

static unsigned long bsSyncBootMs     = 0;   // set when init completes
static unsigned long bsLastSyncSendMs = 0;   // millis() of last sync TX attempt
#define BS_SYNC_RETRY_MS  60000UL            // resend sync every 60 s

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

// ===================== INIT =====================

bool bsRadioInit() {
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  // HAL is in initMode so it spins on BUSY between each command automatically.
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

  // Sync word 0x12 (private network) → register pair 0x0740/0x0741 = 0x14, 0x24
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

  // Init complete — switch HAL back to drop-on-BUSY (non-blocking, safe for armed loop)
  bsRadioCtx.initMode = false;

  bsSyncBootMs  = millis();
  bsLastSyncSendMs = 0;
  bsRadioState  = BS_RADIO_STANDBY;
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
  if (digitalRead(LORA_BUSY_PIN)) { Serial.println("BS TX: BUSY — skip"); return false; }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  if (sx126x_write_buffer(&bsRadioCtx, 0, pkt, (uint8_t)len) != SX126X_STATUS_OK) {
    Serial.println("BS TX: write_buffer fail"); return false;
  }
  if (sx126x_set_tx(&bsRadioCtx, 0) == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_TX_ACTIVE;
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
  bsSyncSlotIndex    = 1;           // WIN_CMD = slot index 1; we just transmitted in WIN_CMD
  bsLastHandledSlot  = 0xFFFFFFFF;
  bsSynced           = true;
  bsMissedTelemSlots = 0;
  Serial.print("BS SYNC anchor="); Serial.print(bsSyncAnchorUs); Serial.println("us");
}

// ===================== RX PACKET HANDLER =====================

static void bsHandleRxDone() {
  sx126x_rx_buffer_status_t bufStatus = {};
  if (sx126x_get_rx_buffer_status(&bsRadioCtx, &bufStatus) != SX126X_STATUS_OK) {
    Serial.println("BS RX: get_rx_buffer_status fail");
    bsPendingHeaderValidUs = 0;
    return;
  }

  uint8_t buf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(buf)) {
    Serial.print("BS RX: bad length "); Serial.println(rxLen);
    bsPendingHeaderValidUs = 0;
    return;
  }

  if (sx126x_read_buffer(&bsRadioCtx, bufStatus.buffer_start_pointer, buf, rxLen) != SX126X_STATUS_OK) {
    Serial.println("BS RX: read_buffer fail");
    bsPendingHeaderValidUs = 0;
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&bsRadioCtx, &pktStatus);
  float snrF  = (float)pktStatus.snr_pkt_in_db;
  float rssiF = (float)pktStatus.rssi_pkt_in_dbm;

  bool isTelemetry = (rxLen >= 10 && buf[0] == 0xAF && buf[1] == ROCKET_DEVICE_ID);
  if (isTelemetry) {
    bsMissedTelemSlots = 0;
  }
  bsPendingHeaderValidUs = 0;

  bsOnPacketReceived(buf, rxLen, snrF, rssiF);
}

// ===================== IRQ HANDLER =====================

static void bsRadioHandleIrq() {
  uint64_t eventUs = dio1TimestampUs();
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_get_and_clear_irq_status(&bsRadioCtx, &irqFlags);

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    bsLedOff();
    bsRadioState = BS_RADIO_STANDBY;
    Serial.print("BS TxDone ts="); Serial.println((unsigned long)eventUs);
    // If this was a sync TX, anchor the slot clock
    if (bsSyncTxInFlight) {
      bsSyncTxInFlight = false;
      bsSetSyncedFromTx(eventUs);
    }
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    bsLedOn();
    bsHandleRxDone();
    bsLedOff();
    bsRadioState = BS_RADIO_STANDBY;
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    bsLedOff();
    bsPendingHeaderValidUs = 0;
    int16_t rssiDbm = 0;
    if (sx126x_get_rssi_inst(&bsRadioCtx, &rssiDbm) == SX126X_STATUS_OK) {
      if (!bsBgRssiInit) { bsBgRssiEma = (float)rssiDbm; bsBgRssiInit = true; }
      else bsBgRssiEma += BG_RSSI_ALPHA * ((float)rssiDbm - bsBgRssiEma);
    }
    bsRadioState = BS_RADIO_STANDBY;
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    bsPendingHeaderValidUs = 0;
    bsRadioState = BS_RADIO_STANDBY;
    Serial.println("BS RX: CRC/header error");
  }
}

// ===================== SYNC MANAGEMENT =====================

// Flag read by main.cpp to enqueue the sync packet
bool bsSyncNeedsQueue = false;

// Set by main.cpp before bsRadioStartTx for a sync packet; cleared on TxDone
bool bsSyncTxInFlight = false;

// Flag: main.cpp should attempt TX now (set in WIN_CMD at offset)
bool bsWinCmdReady = false;

void bsHandleSyncSend() {
  unsigned long now = millis();

  // First sync: send 2 s after boot
  bool timeForSync = (bsLastSyncSendMs == 0) && ((now - bsSyncBootMs) >= BS_SYNC_BOOT_DELAY_MS);

  // Periodic resync every 60 s (also handles sync-loss recovery)
  if (bsLastSyncSendMs != 0 && (now - bsLastSyncSendMs) >= BS_SYNC_RETRY_MS) {
    timeForSync = true;
  }

  if (!timeForSync) return;

  // Check if we can send right now
  if (bsSynced) {
    // We're synced — queue via normal WIN_CMD slot machinery (bsWinCmdReady path)
    Serial.println("BS SYNC: queuing via WIN_CMD slot");
  } else {
    // Not yet synced — send immediately (out-of-turn, no slot clock yet)
    Serial.println("BS SYNC: queuing immediately (not synced)");
  }

  bsLastSyncSendMs = now;
  bsSyncNeedsQueue  = true;
}

// ===================== MAIN RADIO UPDATE =====================

static bool bsCmdSentThisSlot = false;

void bsHandleRadio() {
  // Always handle DIO1 first
  if (dio1Fired) {
    bsRadioHandleIrq();
  }

  // Keep radio listening whenever it returns to standby
  if (bsRadioState == BS_RADIO_STANDBY) {
    bsRadioStartRx();
    return;
  }

  if (!bsSynced) return;  // not synced: just keep RX running, nothing else to schedule

  // ---- Synced mode: slot-based TX scheduling ----
  // RX is always running (continuous); we only need to preempt it to transmit
  // in WIN_CMD slots.

  unsigned long now       = micros();
  unsigned long elapsed   = now - bsSyncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((bsSyncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];

  // Reset per-slot guard on slot transition
  if (slotNum != bsLastHandledSlot) {
    bsCmdSentThisSlot = false;
    if (win == WIN_TELEM) {
      bsMissedTelemSlots++;
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        Serial.print("SLOT WIN_TELEM pos="); Serial.print(posInSlot); Serial.println("us");
      }
    } else if (win == WIN_CMD) {
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        Serial.print("SLOT WIN_CMD pos="); Serial.print(posInSlot); Serial.println("us");
      }
    }
    bsLastHandledSlot = slotNum;
  }

  // In WIN_CMD: raise bsWinCmdReady at the configured TX offset
  if (win == WIN_CMD && !bsCmdSentThisSlot && posInSlot >= BS_CMD_TX_OFFSET_US) {
    bsCmdSentThisSlot = true;
    bsWinCmdReady     = true;
  }
}
