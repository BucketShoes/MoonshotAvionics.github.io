// radio.cpp — LoRa radio hardware, TX/RX state machine, slot-based scheduling.
// See radio.h for the public API.
//
// TEST MODE: uncomment to replace the slot state machine with bare continuous RX.
// Rocket just listens and logs everything it hears. No telem TX, no sync.
// Use this to verify DIO1 ISR and basic SPI before debugging the state machine.
#define ROCKET_RADIO_TEST_MODE

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
#ifdef ROCKET_RADIO_TEST_MODE
  Serial.println("*** ROCKET RADIO TEST MODE ACTIVE — RX only, no telem TX ***");
#endif
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  Serial.println("LoRa: resetting...");
  sx126x_hal_reset(&radioCtx);
  if (!radioWaitBusy(&radioCtx, 100)) {
    Serial.println("LoRa init FAIL: BUSY stuck after reset");
    return false;
  }
  Serial.println("LoRa: BUSY cleared after reset");

  sx126x_status_t st;
  st = sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  Serial.print("LoRa: set_standby -> "); Serial.println(st);
  if (st != SX126X_STATUS_OK) return false;

  // Heltec Wireless Tracker V1.1: TCXO is powered via DIO3 at 1.8V.
  // Without this call the oscillator never starts and the radio accepts SPI commands
  // but never completes any RF operation (TxDone/RxDone/Timeout never fire).
  // Timeout = 5ms (320 × 15.625µs steps = 5000µs) for TCXO startup.
  // Must be called BEFORE set_pkt_type and any modulation config.
  st = sx126x_set_dio3_as_tcxo_ctrl(&radioCtx, SX126X_TCXO_CTRL_1_8V, 320);
  Serial.print("LoRa: set_dio3_tcxo 1.8V 5ms -> "); Serial.println(st);
  // After TCXO command, chip re-calibrates — wait for BUSY to clear.
  if (!radioWaitBusy(&radioCtx, 100)) {
    Serial.println("LoRa init FAIL: BUSY stuck after TCXO setup");
    return false;
  }

  // DC-DC regulator (Heltec board uses DCDC, not LDO).
  st = sx126x_set_reg_mode(&radioCtx, SX126X_REG_MODE_DCDC);
  Serial.print("LoRa: set_reg_mode DCDC -> "); Serial.println(st);

  st = sx126x_set_pkt_type(&radioCtx, SX126X_PKT_TYPE_LORA);
  Serial.print("LoRa: set_pkt_type LORA -> "); Serial.println(st);
  if (st != SX126X_STATUS_OK) return false;

  st = sx126x_set_dio2_as_rf_sw_ctrl(&radioCtx, true);
  Serial.print("LoRa: set_dio2_rf_sw -> "); Serial.println(st);

  const uint8_t syncWord[2] = { 0x14, 0x24 };
  st = sx126x_write_register(&radioCtx, 0x0740, syncWord, 2);
  Serial.print("LoRa: syncword 0x14,0x24 (private 0x12) -> "); Serial.println(st);

  radioApplyConfig();

  // IRQ mask: TX_DONE | RX_DONE | TIMEOUT | CRC_ERROR | HEADER_ERROR — all on DIO1
  sx126x_irq_mask_t irqMask = SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT |
                               SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR;
  st = sx126x_set_dio_irq_params(&radioCtx, irqMask, irqMask, SX126X_IRQ_NONE, SX126X_IRQ_NONE);
  Serial.print("LoRa: set_irq_params (mask=0x"); Serial.print(irqMask, HEX);
  Serial.print(") -> "); Serial.println(st);

  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  Serial.println("LoRa: IRQ cleared");

  radioMcpwmInit(LORA_DIO1_PIN);
  Serial.print("LoRa: DIO1 interrupt attached on GPIO"); Serial.println(LORA_DIO1_PIN);

  radioCtx.initMode = false;

  radioState = RADIO_STANDBY;

  Serial.print("LoRa init OK: ch="); Serial.print(activeChannel);
  Serial.print(" freq="); Serial.print(activeFreqMHz, 1); Serial.print("MHz");
  Serial.print(" SF="); Serial.print(activeSF);
  Serial.print(" BW="); Serial.print((int)activeBwKHz); Serial.print("kHz");
  Serial.print(" pwr="); Serial.print(activePower); Serial.print("dBm");
  Serial.print(" preamble="); Serial.print(LORA_PREAMBLE); Serial.println("sym");
  return true;
}

void radioApplyConfig() {
  // TODO: @@@ LONG BLOCKING - radioWaitBusy calls block up to 100ms each.
  // Safe in init path. If called from CMD_SET_RADIO at runtime, that command
  // must be refused while armed, and the block must be bounded.
  sx126x_mod_params_lora_t modParams = {};
  modParams.sf = sfToEnum(activeSF);
  modParams.bw = bwKHzToEnum(activeBwKHz);
  modParams.cr = SX126X_LORA_CR_4_5;
  modParams.ldro = 0;
  sx126x_status_t st = sx126x_set_lora_mod_params(&radioCtx, &modParams);
  Serial.print("LoRa applyConfig: mod_params SF="); Serial.print(activeSF);
  Serial.print(" BW="); Serial.print((int)activeBwKHz);
  Serial.print(" CR=4/5 -> "); Serial.println(st);
  radioWaitBusy(&radioCtx);

  sx126x_pkt_params_lora_t pktParams = {};
  pktParams.preamble_len_in_symb = LORA_PREAMBLE;
  pktParams.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pktParams.pld_len_in_bytes     = 255;
  pktParams.crc_is_on            = true;
  pktParams.invert_iq_is_on      = false;
  st = sx126x_set_lora_pkt_params(&radioCtx, &pktParams);
  Serial.print("LoRa applyConfig: pkt_params preamble="); Serial.print(LORA_PREAMBLE);
  Serial.print(" explicit_hdr crc_on iq_normal -> "); Serial.println(st);
  radioWaitBusy(&radioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  st = sx126x_set_rf_freq(&radioCtx, freqHz);
  Serial.print("LoRa applyConfig: freq="); Serial.print(freqHz); Serial.print("Hz -> "); Serial.println(st);
  radioWaitBusy(&radioCtx);

  // PA config must come BEFORE SetTxParams per SX1262 datasheet §13.1.14.
  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  st = sx126x_set_pa_cfg(&radioCtx, &paCfg);
  Serial.print("LoRa applyConfig: pa_cfg duty=0x04 hp_max=0x07 device_sel=0 -> "); Serial.println(st);
  radioWaitBusy(&radioCtx);

  st = sx126x_set_tx_params(&radioCtx, activePower, SX126X_RAMP_200_US);
  Serial.print("LoRa applyConfig: tx_params pwr="); Serial.print(activePower); Serial.print("dBm ramp=200us -> "); Serial.println(st);
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
  Serial.print("us slotIdx="); Serial.println(slotIdx);
}

// ===================== RX / TX =====================

// Pre-sync RX window: nearly a full slot period.
// We want to be listening most of the time to catch any sync from the base.
// 20ms margin before the next slot so we re-arm cleanly before the next window.
#define PRESYNC_RX_TIMEOUT_US   (SLOT_DURATION_US - 20000UL)
#define PRESYNC_RX_TIMEOUT_RAW  ((uint32_t)(PRESYNC_RX_TIMEOUT_US / 15.625f))

void radioStartRx() {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("RX: BUSY — skip");
    return;
  }
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  // Always use a timeout — continuous RX means the radio never returns to standby
  // between slots, blocking any TX that needs to preempt it.
  uint32_t timeoutRaw = radioSynced ? ROCKET_RX_TIMEOUT_RAW : PRESYNC_RX_TIMEOUT_RAW;
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, timeoutRaw);
  if (st == SX126X_STATUS_OK) {
    radioState = RADIO_RX_ACTIVE;
    ledOn();
    Serial.print("RX: started timeout=");
    Serial.print(radioSynced ? (uint32_t)ROCKET_RX_TIMEOUT_US : (uint32_t)PRESYNC_RX_TIMEOUT_US);
    Serial.println("us");
  } else {
    Serial.print("RX: start fail st="); Serial.println(st);
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
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    radioState = RADIO_RX_ACTIVE;
    ledOn();
    Serial.print("RX(timeout): started raw="); Serial.println(timeoutRtcSteps);
  } else {
    Serial.print("RX(timeout): start fail st="); Serial.println(st);
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

  Serial.print("TX: writing "); Serial.print(len); Serial.print("B [");
  for (size_t i = 0; i < len && i < 8; i++) {
    if (pkt[i] < 0x10) Serial.print("0");
    Serial.print(pkt[i], HEX); Serial.print(" ");
  }
  if (len > 8) Serial.print("...");
  Serial.println("]");

  sx126x_status_t st = sx126x_write_buffer(&radioCtx, 0, pkt, (uint8_t)len);
  if (st != SX126X_STATUS_OK) {
    Serial.print("TX: write_buffer fail st="); Serial.println(st);
    return false;
  }

  // TODO: @@@ Blocking - brief BUSY spin after write_buffer, expected <200µs, bounded at 500µs
  {
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN)) {
      if (micros() - t0 > 500) {
        Serial.println("TX: BUSY stuck after write_buffer (>500µs) — abort");
        return false;
      }
    }
  }

  st = sx126x_set_tx(&radioCtx, 0);  // timeout=0 = no TX timeout (fires TxDone when done)
  if (st == SX126X_STATUS_OK) {
    radioState = RADIO_TX_ACTIVE;
    ledOn();
    Serial.print("TX: started "); Serial.print(len); Serial.println("B — waiting TxDone");
    return true;
  }
  Serial.print("TX: set_tx fail st="); Serial.println(st);
  radioState = RADIO_STANDBY;
  return false;
}

void radioStandby() {
  sx126x_status_t st = sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  radioState = RADIO_STANDBY;
  ledOff();
  Serial.print("STANDBY st="); Serial.println(st);
}

// ===================== IRQ / RX PACKET HANDLER =====================

static void handleRxDone() {
  sx126x_rx_buffer_status_t bufStatus = {};
  sx126x_status_t st = sx126x_get_rx_buffer_status(&radioCtx, &bufStatus);
  if (st != SX126X_STATUS_OK) {
    Serial.print("RX: get_rx_buffer_status fail st="); Serial.println(st);
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

  st = sx126x_read_buffer(&radioCtx, bufStatus.buffer_start_pointer, rxBuf, rxLen);
  if (st != SX126X_STATUS_OK) {
    Serial.print("RX: read_buffer fail st="); Serial.println(st);
    invalidRxCount++;
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&radioCtx, &pktStatus);
  int8_t pktRssi = pktStatus.rssi_pkt_in_dbm;
  int8_t pktSnr  = pktStatus.snr_pkt_in_db;

  Serial.print("RX: "); Serial.print(rxLen);
  Serial.print("B rssi="); Serial.print(pktRssi);
  Serial.print(" snr="); Serial.print(pktSnr);
  Serial.print(" [");
  for (uint8_t i = 0; i < rxLen && i < 8; i++) {
    if (rxBuf[i] < 0x10) Serial.print("0");
    Serial.print(rxBuf[i], HEX); Serial.print(" ");
  }
  if (rxLen > 8) Serial.print("...");
  Serial.println("]");

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
  uint64_t eventUs = dio1TimestampUs();
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_status_t st = sx126x_get_and_clear_irq_status(&radioCtx, &irqFlags);
  if (st != SX126X_STATUS_OK) {
    Serial.print("IRQ: get_and_clear fail st="); Serial.println(st);
    radioState = RADIO_STANDBY;
    ledOff();
    return;
  }

  Serial.print("IRQ: flags=0x"); Serial.print(irqFlags, HEX);
  Serial.print(" ts="); Serial.print((unsigned long)eventUs); Serial.println("us");

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    radioState = RADIO_STANDBY;
    ledOff();
    Serial.print("TxDone at "); Serial.print((unsigned long)eventUs); Serial.println("us");
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    radioState = RADIO_STANDBY;
    // LED stays on (set in radioStartRx) while we read the packet, then off
    Serial.print("RxDone at "); Serial.print((unsigned long)eventUs); Serial.println("us");
    handleRxDone();
    ledOff();
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    radioState = RADIO_STANDBY;
    ledOff();
    int16_t rssiDbm = 0;
    if (sx126x_get_rssi_inst(&radioCtx, &rssiDbm) == SX126X_STATUS_OK) {
      if (!rssiEmaInit) { rssiEma = rssiDbm; rssiEmaInit = true; }
      else rssiEma += RSSI_EMA_ALPHA * (rssiDbm - rssiEma);
    }
    Serial.print("RX timeout at "); Serial.print((unsigned long)eventUs);
    Serial.print("us bgRssi="); Serial.println((int)rssiEma);
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    radioState = RADIO_STANDBY;
    ledOff();
    invalidRxCount++;
    Serial.print("RX: ");
    if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("CRC_ERROR ");
    if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("HEADER_ERROR ");
    Serial.print("at "); Serial.print((unsigned long)eventUs); Serial.println("us");
  }

  if (irqFlags == 0) {
    Serial.println("IRQ: flags=0 (spurious or ISR race)");
    // Don't change state — radio is still doing whatever it was doing.
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================
//
// Pre-sync: TX telem at txIntervalUs rate. RX for nearly a full slot between TX.
//           Slot clock runs from anchor=0, so WIN_CMD slots are defined but have
//           no special pre-sync behaviour beyond cycling.
// Synced:   WIN_TELEM — stop RX, TX telem.
//           WIN_CMD   — RX with ROCKET_RX_TIMEOUT_RAW to listen for a command.
// LED: on whenever radio is active (RX or TX), off when standby.

static unsigned long lastTelemTxUs = 0;
static uint32_t lastIsrCount = 0;

#ifdef ROCKET_RADIO_TEST_MODE
// In test mode: just RX continuously, log everything received. No TX, no sync.
void nonblockingRadio() {
  if (!loraReady) return;
  uint32_t isrNow = dio1IsrCount;
  if (isrNow != lastIsrCount) {
    Serial.print("TEST DIO1 ISR! count="); Serial.println(isrNow);
    lastIsrCount = isrNow;
  }
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE)) {
    Serial.println("TEST DIO1 pin HIGH (poll fallback)");
    dio1CaptureVal = (uint32_t)micros();
    dio1Fired = true;
  }
  static unsigned long tLastIrqPollMs = 0;
  {
    unsigned long npMs = millis();
    if ((radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE) &&
        !dio1Fired && (npMs - tLastIrqPollMs) >= 100) {
      tLastIrqPollMs = npMs;
      sx126x_irq_mask_t irqFlags = 0;
      sx126x_get_irq_status(&radioCtx, &irqFlags);
      if (irqFlags != 0) {
        Serial.print("TEST IRQ poll hit: flags=0x"); Serial.println(irqFlags, HEX);
        dio1CaptureVal = (uint32_t)micros();
        dio1Fired = true;
      } else {
        Serial.print("TEST IRQ poll: 0 busy="); Serial.println(digitalRead(LORA_BUSY_PIN));
      }
    }
  }
  if (dio1Fired) radioHandleIrq();
  if (radioState == RADIO_STANDBY) radioStartRx();
}
#else

void nonblockingRadio() {
  if (!loraReady) return;

  // Check ISR count — log if new fires detected
  uint32_t isrNow = dio1IsrCount;
  if (isrNow != lastIsrCount) {
    Serial.print("DIO1 ISR fired! count="); Serial.print(isrNow);
    Serial.print(" dio1Pin="); Serial.println(digitalRead(LORA_DIO1_PIN));
    lastIsrCount = isrNow;
  }

  // Fallback 1: DIO1 pin polling
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE)) {
    Serial.println("DIO1 pin HIGH (poll fallback)");
    dio1CaptureVal = (uint32_t)micros();
    dio1Fired = true;
  }

  // Fallback 2: poll IRQ register directly every 100ms when stuck in TX/RX
  static unsigned long lastIrqPollMs = 0;
  {
    unsigned long npMs = millis();
    if ((radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE) &&
        !dio1Fired && (npMs - lastIrqPollMs) >= 100) {
      lastIrqPollMs = npMs;
      sx126x_irq_mask_t irqFlags = 0;
      sx126x_get_irq_status(&radioCtx, &irqFlags);
      if (irqFlags != 0) {
        Serial.print("IRQ poll hit: flags=0x"); Serial.println(irqFlags, HEX);
        dio1CaptureVal = (uint32_t)micros();
        dio1Fired = true;
      } else {
        Serial.print("IRQ poll: flags=0 state="); Serial.print(radioState);
        Serial.print(" busy="); Serial.println(digitalRead(LORA_BUSY_PIN));
      }
    }
  }

  // Always process DIO1 first
  if (dio1Fired) {
    radioHandleIrq();
  }

  unsigned long now = micros();

  if (!radioSynced) {
    // ---- Bootstrap mode: TX telem at txIntervalUs, RX between ----
    // After IRQ (TxDone/timeout/RxDone), radioState==STANDBY — restart RX immediately.
    if (radioState == RADIO_STANDBY) {
      if (txSendingEnabled && (now - lastTelemTxUs) >= txIntervalUs) {
        // Time to TX — don't restart RX, go straight to TX
        uint8_t pkt[32];
        size_t len = buildTelemetryPacket(pkt);
        lastTelemTxUs = now;
        radioStartTx(pkt, len);
      } else {
        radioStartRx();
      }
    } else if (radioState == RADIO_RX_ACTIVE && txSendingEnabled && (now - lastTelemTxUs) >= txIntervalUs) {
      // Time to TX — preempt ongoing RX
      radioStandby();
      // TODO: @@@ Blocking - brief BUSY spin after standby before TX, bounded at 500µs
      unsigned long t0 = micros();
      while (digitalRead(LORA_BUSY_PIN)) {
        if (micros() - t0 > 500) {
          Serial.println("TELEM TX: BUSY stuck after standby — skip");
          return;
        }
      }
      uint8_t pkt[32];
      size_t len = buildTelemetryPacket(pkt);
      lastTelemTxUs = now;
      radioStartTx(pkt, len);
    }
    return;
  }

  // ---- Synced mode: slot-based TX/RX ----
  if (radioState == RADIO_TX_ACTIVE) return;  // TX in progress — wait for TxDone IRQ

  // After any event (TxDone, RxDone, timeout), restart RX for the current slot.
  // WIN_TELEM will preempt it immediately if it's time to TX.
  if (radioState == RADIO_STANDBY) {
    radioStartRx();
  }

  unsigned long elapsed   = now - syncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];

  if (slotNum != lastHandledSlotNum) {
    lastHandledSlotNum = slotNum;
    Serial.print("SLOT #"); Serial.print(slotNum);
    Serial.print(win == WIN_TELEM ? " WIN_TELEM" : win == WIN_CMD ? " WIN_CMD" : " WIN_OTHER");
    Serial.print(" pos="); Serial.print(posInSlot); Serial.println("us");

    if (win == WIN_TELEM && txSendingEnabled) {
      // Preempt RX (which was just started above) to TX telem.
      if (radioState == RADIO_RX_ACTIVE) {
        radioStandby();
        // TODO: @@@ Blocking - brief BUSY spin after standby before TX, bounded at 500µs
        unsigned long t0 = micros();
        while (digitalRead(LORA_BUSY_PIN)) {
          if (micros() - t0 > 500) {
            Serial.println("SLOT WIN_TELEM: BUSY stuck after standby — skip slot");
            return;
          }
        }
      }
      if (radioState == RADIO_STANDBY) {
        uint8_t pkt[32];
        size_t len = buildTelemetryPacket(pkt);
        lastTelemTxUs = now;
        radioStartTx(pkt, len);
        // TxDone IRQ will fire when done; radio returns to standby, then RX restarts next loop.
      }
    }
    // WIN_CMD: RX is already running from the radioState==STANDBY block above.
    // ROCKET_RX_TIMEOUT_RAW ensures it times out before the next slot.
    // No extra action needed — just let it run.
  }
}

#endif  // ROCKET_RADIO_TEST_MODE
