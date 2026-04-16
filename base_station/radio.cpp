// base_station/radio.cpp — LoRa radio, slot machine, and sync logic for base station.
// See radio.h for the public API.

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"  // sx126x_hal_reset/wakeup declarations, sx126x_hal_status_t

// ===================== HARDWARE OBJECTS =====================

SPIClass bsLoraSPI(FSPI);

sx126x_hal_context_t bsRadioCtx = {
  .spi  = &bsLoraSPI,
  .nss  = LORA_NSS_PIN,
  .busy = LORA_BUSY_PIN,
  .rst  = LORA_RST_PIN,
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
static bool   bsBgRssiInit = false;  // true after first sample
#define BG_RSSI_ALPHA  0.1f           // EMA smoothing factor (~10 timeouts)

// ===================== SYNC BOOKKEEPING =====================

static bool          bsSyncPending  = false;
static bool          bsSyncSent     = false;
static unsigned long bsSyncBootMs   = 0;

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

  sx126x_hal_reset(&bsRadioCtx);
  if (!radioWaitBusy(&bsRadioCtx, 100)) {
    Serial.println("LoRa init: BUSY stuck after reset");
    return false;
  }

  if (sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_standby failed");
    return false;
  }
  radioWaitBusy(&bsRadioCtx);

  if (sx126x_set_pkt_type(&bsRadioCtx, SX126X_PKT_TYPE_LORA) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_pkt_type failed");
    return false;
  }
  radioWaitBusy(&bsRadioCtx);

  sx126x_set_dio2_as_rf_sw_ctrl(&bsRadioCtx, true);
  radioWaitBusy(&bsRadioCtx);

  // Sync word 0x12 (private network) → register pair 0x0740/0x0741 = 0x14, 0x24
  const uint8_t syncWord[2] = { 0x14, 0x24 };
  sx126x_write_register(&bsRadioCtx, 0x0740, syncWord, 2);
  radioWaitBusy(&bsRadioCtx);

  bsRadioApplyConfig();
  radioWaitBusy(&bsRadioCtx);

  sx126x_set_dio_irq_params(&bsRadioCtx,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_VALID,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_VALID,
    SX126X_IRQ_NONE,
    SX126X_IRQ_NONE);
  radioWaitBusy(&bsRadioCtx);

  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  radioWaitBusy(&bsRadioCtx);

  radioMcpwmInit(LORA_DIO1_PIN);

  bsSyncBootMs = millis();
  bsRadioState = BS_RADIO_STANDBY;
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
  bsSyncAnchorUs    = (unsigned long)anchorUs;
  bsSyncSlotIndex   = 1;           // WIN_CMD = 1; we just transmitted in WIN_CMD
  bsLastHandledSlot = 0xFFFFFFFF;
  bsSynced          = true;
  bsSyncSent        = true;
  bsMissedTelemSlots = 0;
  Serial.print("BS SYNC anchor="); Serial.print(bsSyncAnchorUs); Serial.println("us (from TxDone)");
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

  // Check for rocket telemetry (0xAF) — update missed-telem counter and drift log
  bool isTelemetry = (rxLen >= 10 && buf[0] == 0xAF && buf[1] == ROCKET_DEVICE_ID);
  if (isTelemetry) {
    bsMissedTelemSlots = 0;

    // Compute drift: back-calculate preamble start from HeaderValid timestamp.
    // symbolTimeUs = 2^SF / BW_Hz * 1e6
    // preambleStartUs = headerValidUs - (preambleSymbols + 4.25) * symbolTimeUs
    // expectedPreambleStartUs = bsSyncAnchorUs + slotNum * SLOT_DURATION_US
    // drift = preambleStartUs - expectedPreambleStartUs
    if (bsPendingHeaderValidUs != 0 && bsSynced) {
      unsigned long elapsed  = (unsigned long)bsPendingHeaderValidUs - bsSyncAnchorUs;
      uint32_t slotNum       = (uint32_t)(elapsed / SLOT_DURATION_US);
      float    bwHz          = activeBwKHz * 1000.0f;
      float    symbolTimeUs  = (float)(1UL << activeSF) / bwHz * 1e6f;
      float    preambleOffUs = ((float)LORA_PREAMBLE + 4.25f) * symbolTimeUs;
      uint64_t preambleStartUs = bsPendingHeaderValidUs - (uint64_t)preambleOffUs;
      uint64_t expectedUs    = (uint64_t)bsSyncAnchorUs + (uint64_t)slotNum * SLOT_DURATION_US;
      int32_t  drift         = (int32_t)(preambleStartUs - expectedUs);
      Serial.print("DRIFT: preambleStart="); Serial.print((unsigned long)preambleStartUs);
      Serial.print(" expected="); Serial.print((unsigned long)expectedUs);
      Serial.print(" drift="); Serial.print(drift); Serial.println("us");
      // anchorCorrection intentionally not applied yet — phase 2
    }
  }
  bsPendingHeaderValidUs = 0;

  // Pass to main.cpp for transport dispatch
  bsOnPacketReceived(buf, rxLen, snrF, rssiF);
}

// ===================== IRQ HANDLER =====================

static void bsRadioHandleIrq() {
  uint64_t eventUs = dio1TimestampUs();
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_get_and_clear_irq_status(&bsRadioCtx, &irqFlags);

  if (irqFlags & SX126X_IRQ_HEADER_VALID) {
    bsPendingHeaderValidUs = eventUs;
  }

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    bsLedOff();
    // Anchor slot clock to this TxDone if this was a sync command.
    // (bsHandleRadio sets bsSynced when it queues sync TX — checked after IRQ.)
    bsRadioState = BS_RADIO_STANDBY;
    Serial.print("BS TxDone ts="); Serial.println((unsigned long)eventUs);
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    bsLedOff();
    bsHandleRxDone();
    bsRadioState = BS_RADIO_STANDBY;
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    bsLedOff();
    bsPendingHeaderValidUs = 0;
    // Sample instantaneous RSSI for background noise EMA.
    // Radio is still in RX at this point (timeout just fired), so the reading
    // reflects channel noise at a random moment in the listen window.
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

// ===================== SYNC PACKET =====================

// cmdTx is owned by main.cpp; bsBuildSyncCmdPacket only builds the packet bytes.
// Callers must include <mbedtls/md.h> and provide HMAC_KEY / HMAC_KEY_LEN / HMAC_TRUNC_LEN.

// Forward declaration — implemented in main.cpp (has access to secrets.h and highestNonce)
extern size_t bsBuildSyncCmdPacket(uint8_t* buf);

// ===================== SYNC SEND =====================

void bsTriggerSyncSend() {
  bsSyncPending = true;
}

// Pending TX slot flag: set when sync was queued, cleared after TxDone anchors the clock.
static bool bsSyncTxPending = false;
static uint64_t bsSyncTxDoneUs = 0;  // captured by IRQ path, processed here

void bsHandleSyncSend() {
  unsigned long now = millis();

  if (!bsSyncSent && (now - bsSyncBootMs) >= BS_SYNC_BOOT_DELAY_MS) {
    bsSyncPending = true;
  }

  if (bsSynced && bsMissedTelemSlots >= BS_SYNC_LOSS_SLOTS) {
    Serial.println("BS SYNC: too many missed telem slots, resyncing");
    bsSyncPending = true;
    bsMissedTelemSlots = 0;
  }

  // Nothing to do
  if (!bsSyncPending) return;

  // Sync is sent as a normal command via queueCommandTx() from main.cpp.
  // Raise the flag; main.cpp calls bsBuildSyncCmdPacket and queues it.
  bsSyncPending = false;
  bsSyncNeedsQueue = true;
}

// Flag read by main.cpp to enqueue the sync packet
bool bsSyncNeedsQueue = false;

// ===================== MAIN RADIO UPDATE =====================

// Per-slot guards (replace bsEarlyRxStarted / bsCmdSentThisSlot)
static bool bsRxStartedThisSlot  = false;
static bool bsCmdSentThisSlot    = false;
// Track last TX's sync intent — set by main.cpp before calling bsRadioStartTx for sync
bool bsSyncTxInFlight = false;

void bsHandleRadio() {
  // Always handle DIO1 first
  if (dio1Fired) {
    uint64_t txDoneUs = dio1TimestampUs();  // read before clearing dio1Fired
    bsRadioHandleIrq();

    // If we just completed a sync TX, anchor the slot clock
    if (bsSyncTxInFlight && bsRadioState == BS_RADIO_STANDBY) {
      bsSyncTxInFlight = false;
      bsSetSyncedFromTx(txDoneUs);
    }
  }

  unsigned long now = micros();

  // ---- Bootstrap mode ----
  if (!bsSynced) {
    if (bsRadioState == BS_RADIO_STANDBY) {
      static unsigned long lastBootstrapPrintMs = 0;
      unsigned long nowMs = millis();
      if (nowMs - lastBootstrapPrintMs >= 5000) {
        lastBootstrapPrintMs = nowMs;
        Serial.print("BS bootstrap RX (no sync) uptime="); Serial.print(nowMs); Serial.println("ms");
      }
      bsRadioStartRx();
    }
    return;
  }

  // ---- Synced mode ----

  unsigned long elapsed   = now - bsSyncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((bsSyncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];
  uint32_t      timeToNext = SLOT_DURATION_US - posInSlot;

  // Reset per-slot guards on slot transition
  if (slotNum != bsLastHandledSlot) {
    bsRxStartedThisSlot = false;
    bsCmdSentThisSlot   = false;
  }

  // Don't preempt in-progress operations — let them finish
  if (bsRadioState == BS_RADIO_TX_ACTIVE || bsRadioState == BS_RADIO_RX_ACTIVE) return;

  switch (win) {

    case WIN_TELEM: {
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        bsMissedTelemSlots++;
        Serial.print("SLOT WIN_TELEM pos="); Serial.print(posInSlot); Serial.println("us");
      }
      // Start listening once per slot if not already started
      if (!bsRxStartedThisSlot) {
        bsRxStartedThisSlot = true;
        bsLedOn();
        bsRadioStartRxTimeout(BS_RX_TIMEOUT_RAW);
      }
      break;
    }

    case WIN_CMD: {
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        Serial.print("SLOT WIN_CMD pos="); Serial.print(posInSlot); Serial.println("us");
      }

      // Fire queued command once per slot at BS_CMD_TX_OFFSET_US.
      // Raise the flag once; main.cpp checks it, fires bsRadioStartTx, and clears it.
      if (!bsCmdSentThisSlot && posInSlot >= BS_CMD_TX_OFFSET_US) {
        bsCmdSentThisSlot = true;
        bsWinCmdReady = true;
      }

      // Start early-listen when close to next WIN_TELEM
      if (!bsRxStartedThisSlot && timeToNext <= BS_RX_EARLY_US) {
        bsRxStartedThisSlot = true;
        bsLedOn();
        bsRadioStartRxTimeout(BS_RX_TIMEOUT_RAW);
      }
      break;
    }

    case WIN_OFF:
    default:
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        Serial.println("SLOT WIN_OFF");
      }
      bsRadioStandby();
      bsLedOff();
      break;
  }
}

// Flag: main.cpp should attempt TX now (set in WIN_CMD at offset)
bool bsWinCmdReady = false;
