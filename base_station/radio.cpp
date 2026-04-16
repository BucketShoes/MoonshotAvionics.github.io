// base_station/radio.cpp — LoRa radio, slot machine, and sync logic for base station.
//
// TEST MODE: uncomment to replace the slot state machine with a bare TX every 5s + continuous RX.
// Use this to verify radio hardware works before debugging the state machine.
// When test mode is active, bsHandleRadio() just does: TX a fixed packet every 5s, RX otherwise.
#define BS_RADIO_TEST_MODE

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
// Prime number — avoids re-aligning to the same slot boundary every retry.
#define BS_SYNC_RETRY_MS  31337UL

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
#ifdef BS_RADIO_TEST_MODE
  Serial.println("*** BS RADIO TEST MODE ACTIVE — slot machine disabled ***");
#endif
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  Serial.println("BS LoRa: resetting...");
  sx126x_hal_reset(&bsRadioCtx);
  if (!radioWaitBusy(&bsRadioCtx, 100)) {
    Serial.println("BS LoRa init FAIL: BUSY stuck after reset");
    return false;
  }
  Serial.println("BS LoRa: BUSY cleared after reset");

  sx126x_status_t st;
  st = sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC);
  Serial.print("BS LoRa: set_standby -> "); Serial.println(st);
  if (st != SX126X_STATUS_OK) return false;

  st = sx126x_set_pkt_type(&bsRadioCtx, SX126X_PKT_TYPE_LORA);
  Serial.print("BS LoRa: set_pkt_type LORA -> "); Serial.println(st);
  if (st != SX126X_STATUS_OK) return false;

  st = sx126x_set_dio2_as_rf_sw_ctrl(&bsRadioCtx, true);
  Serial.print("BS LoRa: set_dio2_rf_sw -> "); Serial.println(st);

  const uint8_t syncWord[2] = { 0x14, 0x24 };
  st = sx126x_write_register(&bsRadioCtx, 0x0740, syncWord, 2);
  Serial.print("BS LoRa: syncword 0x14,0x24 (private 0x12) -> "); Serial.println(st);

  bsRadioApplyConfig();

  sx126x_irq_mask_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                               SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR;
  st = sx126x_set_dio_irq_params(&bsRadioCtx, irqMask, irqMask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  Serial.print("BS LoRa: set_irq_params (mask=0x"); Serial.print(irqMask, HEX);
  Serial.print(") -> "); Serial.println(st);

  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  Serial.println("BS LoRa: IRQ cleared");

  radioMcpwmInit(LORA_DIO1_PIN);
  Serial.print("BS LoRa: DIO1 interrupt attached on GPIO"); Serial.println(LORA_DIO1_PIN);

  bsRadioCtx.initMode = false;

  bsSyncBootMs     = millis();
  bsLastSyncSendMs = 0;
  bsRadioState     = BS_RADIO_STANDBY;

  Serial.print("BS LoRa init OK: ch="); Serial.print(activeChannel);
  Serial.print(" freq="); Serial.print(activeFreqMHz, 1); Serial.print("MHz");
  Serial.print(" SF="); Serial.print(activeSF);
  Serial.print(" BW="); Serial.print((int)activeBwKHz); Serial.print("kHz");
  Serial.print(" pwr="); Serial.print(activePower); Serial.print("dBm");
  Serial.print(" preamble="); Serial.print(LORA_PREAMBLE); Serial.println("sym");
  return true;
}


void bsRadioApplyConfig() {
  // TODO: @@@ LONG BLOCKING - radioWaitBusy calls in bsRadioApplyConfig block up to 100ms each.
  // Safe only in init or when radio is known idle. Never call while armed or from loop at runtime.
  sx126x_mod_params_lora_t modParams = {};
  modParams.sf   = (sx126x_lora_sf_t)activeSF;
  modParams.bw   = bwKHzToEnum(activeBwKHz);
  modParams.cr   = SX126X_LORA_CR_4_5;
  modParams.ldro = 0;
  sx126x_status_t st = sx126x_set_lora_mod_params(&bsRadioCtx, &modParams);
  Serial.print("BS applyConfig: mod_params SF="); Serial.print(activeSF);
  Serial.print(" BW="); Serial.print((int)activeBwKHz);
  Serial.print(" CR=4/5 -> "); Serial.println(st);
  radioWaitBusy(&bsRadioCtx);

  sx126x_pkt_params_lora_t pktParams = {};
  pktParams.preamble_len_in_symb = LORA_PREAMBLE;
  pktParams.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pktParams.pld_len_in_bytes     = 255;
  pktParams.crc_is_on            = true;
  pktParams.invert_iq_is_on      = false;
  st = sx126x_set_lora_pkt_params(&bsRadioCtx, &pktParams);
  Serial.print("BS applyConfig: pkt_params preamble="); Serial.print(LORA_PREAMBLE);
  Serial.print(" explicit_hdr crc_on iq_normal -> "); Serial.println(st);
  radioWaitBusy(&bsRadioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  st = sx126x_set_rf_freq(&bsRadioCtx, freqHz);
  Serial.print("BS applyConfig: freq="); Serial.print(freqHz); Serial.print("Hz -> "); Serial.println(st);
  radioWaitBusy(&bsRadioCtx);

  // PA config must come BEFORE SetTxParams per SX1262 datasheet §13.1.14.
  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  st = sx126x_set_pa_cfg(&bsRadioCtx, &paCfg);
  Serial.print("BS applyConfig: pa_cfg duty=0x04 hp_max=0x07 device_sel=0 -> "); Serial.println(st);
  radioWaitBusy(&bsRadioCtx);

  st = sx126x_set_tx_params(&bsRadioCtx, activePower, SX126X_RAMP_200_US);
  Serial.print("BS applyConfig: tx_params pwr="); Serial.print(activePower); Serial.print("dBm ramp=200us -> "); Serial.println(st);
  radioWaitBusy(&bsRadioCtx);
}

// ===================== RX / TX =====================

// Pre-sync: RX window sized to nearly a full slot so we don't miss anything.
// SLOT_DURATION_US - 20ms margin, converted to RTC steps (15.625µs each).
#define BS_PRESYNC_RX_TIMEOUT_US   (SLOT_DURATION_US - 20000UL)
#define BS_PRESYNC_RX_TIMEOUT_RAW  ((uint32_t)(BS_PRESYNC_RX_TIMEOUT_US / 15.625f))

// Synced RX window: listen early (BS_RX_EARLY_US before slot) for duration BS_RX_TIMEOUT_US.
// Both defined in radio.h.

void bsRadioStartRx() {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("BS RX: BUSY — skip");
    return;
  }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  uint32_t timeoutRaw = bsSynced ? BS_RX_TIMEOUT_RAW : BS_PRESYNC_RX_TIMEOUT_RAW;
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&bsRadioCtx, timeoutRaw);
  if (st == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_RX_ACTIVE;
    bsLedOn();
    Serial.print("BS RX: started timeout=");
    Serial.print(bsSynced ? BS_RX_TIMEOUT_US : BS_PRESYNC_RX_TIMEOUT_US);
    Serial.println("us");
  } else {
    Serial.print("BS RX: start fail st="); Serial.println(st);
    bsRadioState = BS_RADIO_STANDBY;
  }
}

void bsRadioStartRxTimeout(uint32_t timeoutRtcSteps) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.print("BS RX(timeout): BUSY — skip");
    return;
  }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&bsRadioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_RX_ACTIVE;
    bsLedOn();
    Serial.print("BS RX(timeout): started raw="); Serial.println(timeoutRtcSteps);
  } else {
    Serial.print("BS RX(timeout): start fail st="); Serial.println(st);
    bsRadioState = BS_RADIO_STANDBY;
  }
}

bool bsRadioStartTx(const uint8_t* pkt, size_t len) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("BS TX: BUSY before write_buffer — drop");
    return false;
  }
  sx126x_clear_irq_status(&bsRadioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;

  Serial.print("BS TX: writing "); Serial.print(len); Serial.print("B [");
  for (size_t i = 0; i < len && i < 8; i++) {
    if (pkt[i] < 0x10) Serial.print("0");
    Serial.print(pkt[i], HEX); Serial.print(" ");
  }
  if (len > 8) Serial.print("...");
  Serial.println("]");

  sx126x_status_t st = sx126x_write_buffer(&bsRadioCtx, 0, pkt, (uint8_t)len);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS TX: write_buffer fail st="); Serial.println(st);
    return false;
  }

  // TODO: @@@ Blocking - brief BUSY spin after write_buffer before set_tx; should be <200µs
  // The SX1262 reasserts BUSY briefly after write_buffer while it copies data internally.
  // If BUSY is still high when set_tx is called, the HAL drops it (returns error).
  // Spin up to 500µs here to ensure set_tx always lands.
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN)) {
      if (micros() - t0 > 500) {
        Serial.println("BS TX: BUSY stuck after write_buffer (>500µs) — abort");
        return false;
      }
    }
  }

  st = sx126x_set_tx(&bsRadioCtx, 0);  // timeout=0 means no TX timeout (fires TxDone when done)
  if (st == SX126X_STATUS_OK) {
    bsRadioState = BS_RADIO_TX_ACTIVE;
    bsLedOn();
    Serial.print("BS TX: started "); Serial.print(len); Serial.println("B — waiting TxDone");
    return true;
  }
  Serial.print("BS TX: set_tx fail st="); Serial.println(st);
  bsRadioState = BS_RADIO_STANDBY;
  return false;
}

void bsRadioStandby() {
  sx126x_status_t st = sx126x_set_standby(&bsRadioCtx, SX126X_STANDBY_CFG_RC);
  bsRadioState = BS_RADIO_STANDBY;
  bsLedOff();
  Serial.print("BS STANDBY st="); Serial.println(st);
}

// ===================== SYNC =====================

void bsSetSyncedFromTx(uint64_t anchorUs) {
  bsSyncAnchorUs     = (unsigned long)anchorUs;
  // The sync packet is sent during WIN_CMD (slot index 1 in the WIN_TELEM/WIN_CMD sequence).
  // Both sides anchor at this event: base at TxDone, rocket at RxDone.
  // slotIndex=1 means: at the anchor time, slot 1 (WIN_CMD) is current.
  // Next slot (slotNum=1) will be seqIdx=(1+1)%2=0=WIN_TELEM — rocket TX, base RX. Correct.
  bsSyncSlotIndex    = 1;
  bsLastHandledSlot  = 0xFFFFFFFF;
  bsSynced           = true;
  bsMissedTelemSlots = 0;
  Serial.print("BS SYNCED: anchor="); Serial.print(bsSyncAnchorUs);
  Serial.print("us slotIdx="); Serial.println(bsSyncSlotIndex);
}

// ===================== RX PACKET HANDLER =====================

static void bsHandleRxDone() {
  sx126x_rx_buffer_status_t bufStatus = {};
  sx126x_status_t st = sx126x_get_rx_buffer_status(&bsRadioCtx, &bufStatus);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS RX: get_rx_buffer_status fail st="); Serial.println(st);
    return;
  }

  uint8_t buf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(buf)) {
    Serial.print("BS RX: bad length "); Serial.println(rxLen);
    return;
  }

  st = sx126x_read_buffer(&bsRadioCtx, bufStatus.buffer_start_pointer, buf, rxLen);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS RX: read_buffer fail st="); Serial.println(st);
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&bsRadioCtx, &pktStatus);
  float snrF  = (float)pktStatus.snr_pkt_in_db;
  float rssiF = (float)pktStatus.rssi_pkt_in_dbm;

  bool isTelemetry = (rxLen >= 10 && buf[0] == 0xAF && buf[1] == ROCKET_DEVICE_ID);
  if (isTelemetry) bsMissedTelemSlots = 0;

  Serial.print("BS RX: "); Serial.print(rxLen); Serial.print("B snr=");
  Serial.print(snrF, 1); Serial.print(" rssi="); Serial.print(rssiF, 0);
  Serial.print(" telem="); Serial.print(isTelemetry ? "YES" : "NO");
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
  sx126x_status_t st = sx126x_get_and_clear_irq_status(&bsRadioCtx, &irqFlags);
  if (st != SX126X_STATUS_OK) {
    Serial.print("BS IRQ: get_and_clear fail st="); Serial.println(st);
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    return;
  }

  Serial.print("BS IRQ: flags=0x"); Serial.print(irqFlags, HEX);
  Serial.print(" ts="); Serial.print((unsigned long)eventUs); Serial.println("us");

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    Serial.print("BS TxDone at "); Serial.print((unsigned long)eventUs); Serial.println("us");
    if (bsSyncTxInFlight) {
      bsSyncTxInFlight = false;
      bsSetSyncedFromTx(eventUs);
    }
    // Restart RX after TX completes
    bsRadioStartRx();
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    bsRadioState = BS_RADIO_STANDBY;
    // LED stays on (already on from RX start) while we read the packet
    Serial.print("BS RxDone at "); Serial.print((unsigned long)eventUs); Serial.println("us");
    bsHandleRxDone();
    bsLedOff();
    bsRadioStartRx();
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    int16_t rssiDbm = 0;
    if (sx126x_get_rssi_inst(&bsRadioCtx, &rssiDbm) == SX126X_STATUS_OK) {
      if (!bsBgRssiInit) { bsBgRssiEma = (float)rssiDbm; bsBgRssiInit = true; }
      else bsBgRssiEma += BG_RSSI_ALPHA * ((float)rssiDbm - bsBgRssiEma);
    }
    Serial.print("BS RX timeout at "); Serial.print((unsigned long)eventUs);
    Serial.print("us bgRssi="); Serial.println((int)bsBgRssiEma);
    bsRadioStartRx();
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    bsRadioState = BS_RADIO_STANDBY;
    bsLedOff();
    Serial.print("BS RX: ");
    if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("CRC_ERROR ");
    if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("HEADER_ERROR ");
    Serial.print("at "); Serial.print((unsigned long)eventUs); Serial.println("us");
    bsRadioStartRx();
  }

  if (irqFlags == 0) {
    Serial.println("BS IRQ: flags=0 (spurious or ISR race)");
    // Don't change state — radio is still doing whatever it was doing.
  }
}

// ===================== SYNC MANAGEMENT =====================

bool bsSyncNeedsQueue  = false;
bool bsSyncTxInFlight  = false;
bool bsWinCmdReady     = false;

void bsHandleSyncSend() {
  unsigned long now = millis();

  bool timeForSync = (bsLastSyncSendMs == 0) && ((now - bsSyncBootMs) >= BS_SYNC_BOOT_DELAY_MS);
  if (!timeForSync && bsLastSyncSendMs != 0 && (now - bsLastSyncSendMs) >= BS_SYNC_RETRY_MS) {
    timeForSync = true;
  }

  if (!timeForSync) return;

  bsLastSyncSendMs = now;
  bsSyncNeedsQueue = true;
  Serial.print("BS SYNC: queuing sync (synced="); Serial.print(bsSynced);
  Serial.print(" retry="); Serial.print(BS_SYNC_RETRY_MS); Serial.println("ms)");
}

// ===================== SIMPLE TEST MODE =====================
// When BS_RADIO_TEST_MODE is defined: TX a hardcoded packet every 5s, RX otherwise.
// Bypasses all slot/sync logic. Use to verify DIO1 ISR, PA, and basic SPI.

#ifdef BS_RADIO_TEST_MODE

// Minimal valid-looking command packet (wrong HMAC — rocket will reject it, but will
// log "HMAC fail" which proves the packet arrived and was decoded).
static const uint8_t bsTestPkt[] = {
  0x9A,       // PKT_COMMAND
  0x92,       // ROCKET_DEVICE_ID
  0x40,       // CMD_PING
  0x01, 0x00, 0x00, 0x00,  // nonce=1
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // fake HMAC (10 bytes)
};
static unsigned long bsTestLastTxMs = 0;
static bool bsTestRxStarted = false;

void bsHandleRadio() {
  uint32_t isrNow = dio1IsrCount;
  static uint32_t bsLastIsrCountT = 0;
  if (isrNow != bsLastIsrCountT) {
    Serial.print("TEST DIO1 ISR! count="); Serial.println(isrNow);
    bsLastIsrCountT = isrNow;
  }

  // Fallback 1: DIO1 pin polling
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (bsRadioState == BS_RADIO_TX_ACTIVE || bsRadioState == BS_RADIO_RX_ACTIVE)) {
    Serial.println("TEST DIO1 pin HIGH (poll fallback)");
    dio1CaptureVal = (uint32_t)micros();
    dio1Fired = true;
  }

  // Fallback 2: poll IRQ register directly every ~500ms when stuck in TX/RX active.
  // DIO1 may not pulse (wrong wiring or pin conflict) but the register still updates.
  static unsigned long bsLastIrqPollMs = 0;
  unsigned long nowPollMs = millis();
  if ((bsRadioState == BS_RADIO_TX_ACTIVE || bsRadioState == BS_RADIO_RX_ACTIVE) &&
      !dio1Fired && (nowPollMs - bsLastIrqPollMs) >= 100) {
    bsLastIrqPollMs = nowPollMs;
    sx126x_irq_mask_t irqFlags = 0;
    sx126x_get_irq_status(&bsRadioCtx, &irqFlags);
    if (irqFlags != 0) {
      Serial.print("TEST IRQ poll: flags=0x"); Serial.print(irqFlags, HEX);
      Serial.println(" (DIO1 never pulsed but register has flags — wiring issue)");
      // Synthesise a DIO1 event so the IRQ handler runs
      dio1CaptureVal = (uint32_t)micros();
      dio1Fired = true;
    } else {
      Serial.print("TEST IRQ poll: flags=0 state="); Serial.print(bsRadioState);
      Serial.print(" busy="); Serial.println(digitalRead(LORA_BUSY_PIN));
    }
  }

  if (dio1Fired) {
    bsRadioHandleIrq();  // handles TxDone/RxDone/Timeout, restarts RX after each
  }
  unsigned long nowMs = millis();
  if (bsRadioState == BS_RADIO_STANDBY && !bsTestRxStarted) {
    bsRadioStartRx();
    bsTestRxStarted = true;
  }
  if (bsRadioState == BS_RADIO_STANDBY || (bsRadioState == BS_RADIO_RX_ACTIVE)) {
    if (nowMs - bsTestLastTxMs >= 5000) {
      bsTestLastTxMs = nowMs;
      bsTestRxStarted = false;
      Serial.println("TEST TX: sending test packet");
      if (bsRadioState == BS_RADIO_RX_ACTIVE) {
        bsRadioStandby();
        unsigned long t0 = micros();
        while (digitalRead(LORA_BUSY_PIN) && micros() - t0 < 2000) {}
      }
      bsRadioStartTx(bsTestPkt, sizeof(bsTestPkt));
    }
  }
}

#else  // normal mode below

// ===================== MAIN RADIO UPDATE =====================

static bool bsCmdSentThisSlot = false;

// Track ISR count to detect if ISR is ever firing
static uint32_t bsLastIsrCount = 0;

void bsHandleRadio() {
  // Check ISR count — log if new fires detected (ISR itself can't Serial.print)
  uint32_t isrNow = dio1IsrCount;
  if (isrNow != bsLastIsrCount) {
    Serial.print("BS DIO1 ISR fired! count="); Serial.print(isrNow);
    Serial.print(" dio1Pin="); Serial.println(digitalRead(LORA_DIO1_PIN));
    bsLastIsrCount = isrNow;
  }

  // Fallback 1: DIO1 pin polling
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (bsRadioState == BS_RADIO_TX_ACTIVE || bsRadioState == BS_RADIO_RX_ACTIVE)) {
    Serial.println("BS DIO1 pin HIGH (poll fallback)");
    dio1CaptureVal = (uint32_t)micros();
    dio1Fired = true;
  }

  // Fallback 2: poll IRQ register directly every 100ms when stuck in TX/RX
  static unsigned long bsLastIrqPollMs2 = 0;
  {
    unsigned long npMs = millis();
    if ((bsRadioState == BS_RADIO_TX_ACTIVE || bsRadioState == BS_RADIO_RX_ACTIVE) &&
        !dio1Fired && (npMs - bsLastIrqPollMs2) >= 100) {
      bsLastIrqPollMs2 = npMs;
      sx126x_irq_mask_t irqFlags = 0;
      sx126x_get_irq_status(&bsRadioCtx, &irqFlags);
      if (irqFlags != 0) {
        Serial.print("BS IRQ poll hit: flags=0x"); Serial.println(irqFlags, HEX);
        dio1CaptureVal = (uint32_t)micros();
        dio1Fired = true;
      } else {
        Serial.print("BS IRQ poll: flags=0 state="); Serial.print(bsRadioState);
        Serial.print(" busy="); Serial.println(digitalRead(LORA_BUSY_PIN));
      }
    }
  }

  if (dio1Fired) {
    bsRadioHandleIrq();
  }

  // Slot clock always runs (anchor=0 pre-sync, which is fine — slots just cycle from boot).
  unsigned long now       = micros();
  unsigned long elapsed   = now - bsSyncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((bsSyncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];

  if (slotNum != bsLastHandledSlot) {
    bsCmdSentThisSlot = false;
    bsLastHandledSlot = slotNum;
    Serial.print("BS SLOT #"); Serial.print(slotNum);
    Serial.print(bsSynced ? " [SYNC] " : " [NOSYNC] ");
    Serial.print(win == WIN_TELEM ? "WIN_TELEM" : win == WIN_CMD ? "WIN_CMD" : "WIN_OTHER");
    Serial.print(" pos="); Serial.print(posInSlot); Serial.println("us");

    if (win == WIN_TELEM) {
      bsMissedTelemSlots++;
      Serial.print("BS missed telem slots="); Serial.println(bsMissedTelemSlots);
    }
  }

  // Signal WIN_CMD ready for TX dispatch (both synced and unsynced paths use this).
  // Pre-sync: slot machine still runs from anchor=0, so WIN_CMD slots cycle normally.
  // The wait=0 on sync packets means the expired-wait path fires immediately; the
  // slot signal here also fires for any other queued commands.
  if (win == WIN_CMD && !bsCmdSentThisSlot && posInSlot >= BS_CMD_TX_OFFSET_US) {
    bsCmdSentThisSlot = true;
    bsWinCmdReady     = true;
  }
}

#endif  // BS_RADIO_TEST_MODE
