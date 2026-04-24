// radio.cpp — LoRa radio hardware, TX/RX state machine, slot-based scheduling.
// See radio.h for the public API.
//
// TEST MODE: uncomment to replace the slot state machine with bare continuous RX.
// Rocket just listens and logs everything it hears. No telem TX, no sync.
// Use this to verify DIO1 ISR and basic SPI before debugging the state machine.
//#define ROCKET_RADIO_TEST_MODE

#include <Arduino.h>
#include "radio.h"
#include "sx126x.h"
#include "sx126x_hal.h"  // sx126x_hal_reset/wakeup declarations, sx126x_hal_status_t
#include "telemetry.h"
#include "commands.h"
#include "globals.h"
#include "gps.h"

// ===================== HARDWARE OBJECTS =====================

SPIClass loraSPI(FSPI);

sx126x_hal_context_t radioCtx = {
  .spi           = &loraSPI,
  .nss           = LORA_NSS_PIN,
  .busy          = LORA_BUSY_PIN,
  .rst           = LORA_RST_PIN,
  .initMode      = true,   // cleared to false at end of radioInit()
  .allowBusyRead = false,
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

// ===================== PER-SLOT CONFIG STATE (forward-declared for radioStartTx) =====================

static RadioSlotConfig appliedCfg = RADIO_CFG_NORMAL;
static RadioSlotConfig targetCfg  = RADIO_CFG_NORMAL;

// ===================== SLOT CLOCK STATE =====================

bool          radioSynced        = false;
unsigned long syncAnchorUs       = 0;
uint32_t      syncSlotIndex      = 0;
uint32_t      lastHandledSlotNum = 0xFFFFFFFF;

// ===================== RSSI EMA =====================
// Updated at WIN_CMD timeout — channel noise sampled when no command arrives.

double rssiEma = 0.0;//average rssi sampled when there is NO packet for us - i.e. background only - do not include samples if an actual packet is recieved
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

//radio on. show the brighter flash for timing sync (later, might make this separate tx vs rx).
static void ledOnTX()  { ledcWrite(LED_PIN, 64); }
static void ledOnRX()  { ledcWrite(LED_PIN, 5); }
static void ledOff() {
  // Idle LED: show whether we're in good sync (heard command recently).
  // Brightness indicates quality: bright = good sync, dim = lost sync.
  bool inGoodSync = (lastValidCmdUs != 0 && (micros() - lastValidCmdUs) < ROCKET_NO_BASE_HEARD_THRESHOLD_US);
  ledcWrite(LED_PIN, inGoodSync ? 0 : 1);
}

// ===================== INIT =====================

bool radioInit() {
#ifdef ROCKET_RADIO_TEST_MODE
  Serial.println("*** ROCKET RADIO TEST MODE ACTIVE — RX only, no telem TX ***");
#endif
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  // V4 FEM (front-end module) init: enable amplifier and antenna control, PA off initially.
  // Tracker has no external FEM (DIO2 RF switch handled internally by SX1262).
#ifdef LORA_FEM_EN_PIN
  pinMode(LORA_FEM_EN_PIN,  OUTPUT); digitalWrite(LORA_FEM_EN_PIN,  HIGH);
  pinMode(LORA_FEM_CTL_PIN, OUTPUT); digitalWrite(LORA_FEM_CTL_PIN, HIGH);
  pinMode(LORA_FEM_PA_PIN,  OUTPUT); digitalWrite(LORA_FEM_PA_PIN,  LOW);
  Serial.println("LoRa: FEM init (EN=HIGH, CTL=HIGH, PA=LOW)");
#endif

  Serial.println("LoRa: resetting...");
  sx126x_hal_reset(&radioCtx);
  if (!DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx, 100)) {
    Serial.println("LoRa init FAIL: BUSY stuck after reset");
    return false;
  }
  Serial.println("LoRa: BUSY cleared after reset");

  sx126x_status_t st;
  st = sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  Serial.print("LoRa: set_standby -> "); Serial.println(st);
  if (st != SX126X_STATUS_OK) return false;

  // TCXO power via DIO3 (voltage set in board_config.h: LORA_TCXO_VOLTAGE).
  // Without this call the oscillator never starts and the radio accepts SPI commands
  // but never completes any RF operation (TxDone/RxDone/Timeout never fire).
  // Timeout = 5ms (320 × 15.625µs steps = 5000µs) for TCXO startup.
  // Must be called BEFORE set_pkt_type and any modulation config.
  st = sx126x_set_dio3_as_tcxo_ctrl(&radioCtx, LORA_TCXO_VOLTAGE, 320);
  Serial.print("LoRa: set_dio3_tcxo 1.8V 5ms -> "); Serial.println(st);
  // After TCXO command, chip re-calibrates — wait for BUSY to clear.
  if (!DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx, 100)) {
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

  radioApplyConfig_BLOCKING();

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

void radioApplyConfig_BLOCKING() {
  // BLOCKING — init path only. Contains DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING
  // calls (up to 100ms each). If called from CMD_SET_RADIO, that command must be
  // refused while armed.
  sx126x_mod_params_lora_t modParams = {};
  modParams.sf = sfToEnum(activeSF);
  modParams.bw = bwKHzToEnum(activeBwKHz);
  modParams.cr = SX126X_LORA_CR_4_5;
  modParams.ldro = 0;
  sx126x_status_t st = sx126x_set_lora_mod_params(&radioCtx, &modParams);
  Serial.print("LoRa applyConfig: mod_params SF="); Serial.print(activeSF);
  Serial.print(" BW="); Serial.print((int)activeBwKHz);
  Serial.print(" CR=4/5 -> "); Serial.println(st);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  sx126x_pkt_params_lora_t pktParams = {};
  pktParams.preamble_len_in_symb = LORA_PREAMBLE;
  pktParams.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pktParams.pld_len_in_bytes     = 255;
  pktParams.crc_is_on            = true;
  pktParams.invert_iq_is_on      = false;
  st = sx126x_set_lora_pkt_params(&radioCtx, &pktParams);
  Serial.print("LoRa applyConfig: pkt_params preamble="); Serial.print(LORA_PREAMBLE);
  Serial.print(" explicit_hdr crc_on iq_normal -> "); Serial.println(st);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  st = sx126x_set_rf_freq(&radioCtx, freqHz);
  Serial.print("LoRa applyConfig: freq="); Serial.print(freqHz); Serial.print("Hz -> "); Serial.println(st);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  // PA config must come BEFORE SetTxParams per SX1262 datasheet §13.1.14.
  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  st = sx126x_set_pa_cfg(&radioCtx, &paCfg);
  Serial.print("LoRa applyConfig: pa_cfg duty=0x04 hp_max=0x07 device_sel=0 -> "); Serial.println(st);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);

  st = sx126x_set_tx_params(&radioCtx, activePower, SX126X_RAMP_200_US);
  Serial.print("LoRa applyConfig: tx_params pwr="); Serial.print(activePower); Serial.print("dBm ramp=200us -> "); Serial.println(st);
  DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(&radioCtx);
}

// ===================== WIN_LR PACKET BUILDER =====================

// Builds the 3-byte on-air core for the 0xBB long-range packet.
// Encodes dithered lat/lon fractions and low-battery flag.
// Returns 3. buf must be at least 3 bytes.
size_t buildLRPacketCore(uint8_t* buf) {
  uint16_t latFrac = 0x7FF;  // >=2000 = error sentinel in 11 bits
  uint16_t lonFrac = 0x7FF;
  if (gps.valid) {
    double latF = fmod(fabs(gps.lat), 1.0) * 2000.0;
    double lonF = fmod(fabs(gps.lon), 1.0) * 2000.0;
    // Probabilistic rounding: fractional part = probability of rounding up.
    latFrac = (uint16_t)latF + ((rand() / (float)RAND_MAX) < (latF - (int)latF) ? 1 : 0);
    lonFrac = (uint16_t)lonF + ((rand() / (float)RAND_MAX) < (lonF - (int)lonF) ? 1 : 0);
    if (latFrac > 1999) latFrac = 1999;
    if (lonFrac > 1999) lonFrac = 1999;
  }
  // Low battery: 100% true at <=3400mV, 0% true at >=3700mV, probabilistic between.
  bool lowBatt = false;
  if (batteryMv <= 3400) {
    lowBatt = true;
  } else if (batteryMv < 3700) {
    lowBatt = ((rand() / (float)RAND_MAX) < (3700.0f - (float)batteryMv) / 300.0f);
  }
  // Bit layout: [0..10]=latFrac, [11..21]=lonFrac, [22]=lowBatt, [23]=reserved
  uint32_t word = ((uint32_t)(latFrac & 0x7FF))
                | ((uint32_t)(lonFrac & 0x7FF) << 11)
                | ((uint32_t)lowBatt            << 22);
  buf[0] = (uint8_t)(word);
  buf[1] = (uint8_t)(word >> 8);
  buf[2] = (uint8_t)(word >> 16);
  return 3;
}

// ===================== SYNC =====================

void radioSetSynced(unsigned long anchorUs, uint8_t slotIdx) {
  radioStandby();
  dio1Fired              = false;
  radioState             = RADIO_STANDBY;
  syncAnchorUs           = anchorUs;
  syncSlotIndex          = slotIdx;
  lastHandledSlotNum     = 0xFFFFFFFF;
  lastValidCmdUs         = (unsigned long)micros();  // Sync packet is a command — updates our sync state
  Serial.print("SYNC: anchor="); Serial.print(anchorUs);
  Serial.print("us slotIdx="); Serial.println(slotIdx);
}

// ===================== RX / TX =====================

void radioStartRxTimeout(uint32_t timeoutRtcSteps) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.print("RX: BUSY — skip timeout "); Serial.print(timeoutRtcSteps);
    Serial.println(" RTC");
    return;
  }
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;
  sx126x_status_t st = sx126x_set_rx_with_timeout_in_rtc_step(&radioCtx, timeoutRtcSteps);
  if (st == SX126X_STATUS_OK) {
    radioState = RADIO_RX_ACTIVE;
    ledOnRX();
    if (LOG_RX_START) {
      uint64_t nowUs     = (uint64_t)micros();
      uint64_t elapsed   = nowUs - (uint64_t)syncAnchorUs;
      uint32_t slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
      uint32_t posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
      uint8_t  seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
      uint32_t timeoutUs = (uint32_t)(timeoutRtcSteps * 15.625f);
      Serial.print("RxStart: posInSlot="); Serial.print(posInSlot);
      Serial.print("us slot="); Serial.print(slotNum);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.print((int)SLOT_SEQUENCE[seqIdx]);
      Serial.print(" timeout="); Serial.print(timeoutUs); Serial.println("us");
    }
  } else {
    Serial.print("RX: set_rx fail st="); Serial.print(st);
    Serial.print(" rtc="); Serial.println(timeoutRtcSteps);
    radioState = RADIO_STANDBY;
  }
}

void radioStartRx() {
  // Always use a timeout — continuous RX means the radio never returns to standby
  // between slots, blocking any TX that needs to preempt it.
  // Short RX window when we've heard a valid command recently (within ROCKET_NO_BASE_HEARD_THRESHOLD_US).
  // Long RX window otherwise (lost-rocket recovery) to catch a fresh command after timing drift.
  bool hasRecentCommand = (lastValidCmdUs != 0 &&
                           (micros() - lastValidCmdUs) < ROCKET_NO_BASE_HEARD_THRESHOLD_US);
  uint32_t timeoutUs = hasRecentCommand ? ROCKET_RX_TIMEOUT_US : ROCKET_LONG_RX_TIMEOUT_US;
  radioStartRxTimeout((uint32_t)(timeoutUs / 15.625f));
}

bool radioStartTx(const uint8_t* pkt, size_t len) {
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("TX: BUSY — skip");
    return false;
  }
  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);
  dio1Fired = false;

  // Set pkt params to match current slot config. WIN_LR uses implicit header + no CRC;
  // all other slots (currently) use explicit header + CRC. Must be set before each TX.
  sx126x_pkt_params_lora_t pp = {};
  if (appliedCfg == RADIO_CFG_LR) {
    pp.preamble_len_in_symb = 5;
    pp.header_type          = SX126X_LORA_PKT_IMPLICIT;
    pp.pld_len_in_bytes     = (uint8_t)len;
    pp.crc_is_on            = false;
  } else {
    pp.preamble_len_in_symb = LORA_PREAMBLE;
    pp.header_type          = SX126X_LORA_PKT_EXPLICIT;
    pp.pld_len_in_bytes     = (uint8_t)len;
    pp.crc_is_on            = true;
  }
  pp.invert_iq_is_on = false;
  sx126x_set_lora_pkt_params(&radioCtx, &pp);

  sx126x_status_t st = sx126x_write_buffer(&radioCtx, 0, pkt, (uint8_t)len);
  if (st != SX126X_STATUS_OK) {
    Serial.print("TX: write_buffer fail st="); Serial.println(st);
    return false;
  }

  // Non-blocking BUSY check after write_buffer. If still BUSY, abort TX for this slot.
  // The slot machine will attempt TX again next slot boundary — no spin.
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("TX: BUSY after write_buffer — abort");
    return false;
  }

  // V4 FEM PA: enable amplifier for TX (Tracker has no external FEM).
#ifdef LORA_FEM_PA_PIN
  digitalWrite(LORA_FEM_PA_PIN, HIGH);
#endif

  st = sx126x_set_tx(&radioCtx, 0);  // timeout=0 = no TX timeout (fires TxDone when done)
  if (st == SX126X_STATUS_OK) {
    radioState = RADIO_TX_ACTIVE;
    ledOnTX();
    if (LOG_TX_START) {
      uint64_t nowUs     = (uint64_t)micros();
      uint64_t elapsed   = nowUs - (uint64_t)syncAnchorUs;
      uint32_t slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
      uint32_t posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
      uint8_t  seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
      Serial.print("TxStart: posInSlot="); Serial.print(posInSlot);
      Serial.print("us slot="); Serial.print(slotNum);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.print((int)SLOT_SEQUENCE[seqIdx]);
      Serial.print(" len="); Serial.println((unsigned)len);
    }
    return true;
  }
  Serial.print("TX: set_tx fail st="); Serial.println(st);
  radioState = RADIO_STANDBY;
  return false;
}

void radioStandby() {
  // V4 FEM PA: disable amplifier (Tracker has no external FEM).
#ifdef LORA_FEM_PA_PIN
  digitalWrite(LORA_FEM_PA_PIN, LOW);
#endif
  sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
  radioState = RADIO_STANDBY;
  ledOff();
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

  bool structOk = (rxLen >= 2) && (rxBuf[0] == PKT_TELEMETRY || rxBuf[0] == PKT_COMMAND ||
                                    rxBuf[0] == PKT_BACKHAUL);
  if (!structOk) {
    Serial.print("RX: unknown type 0x"); Serial.println(rxBuf[0], HEX);
    invalidRxCount++;
    return;
  }
//TODO: WTF: no the bg rssi ema is for the non-packet times, when there iisnt any signal.. it should EXCLUDE packet rssis. the packet rssi goes into the cmd ack, but not into the bgrssi
  // Update noise floor EMA from actual received packet RSSI.
  //if (!rssiEmaInit) { rssiEma = pktRssi; rssiEmaInit = true; }
  //else rssiEma += RSSI_EMA_ALPHA * (pktRssi - rssiEma);

  // Log one summary line per received packet, no faster than 1/sec.
  static unsigned long lastRxLogMs = 0;
  unsigned long nowMs = millis();
  if (nowMs - lastRxLogMs >= 1000) {
    lastRxLogMs = nowMs;
    const char* typeStr = (rxBuf[0] == PKT_COMMAND) ? "CMD" :
                          (rxBuf[0] == PKT_TELEMETRY) ? "TELEM" : "BH";
    Serial.print("RX "); Serial.print(typeStr);
    Serial.print(" "); Serial.print(rxLen); Serial.print("B rssi=");
    Serial.print(pktRssi); Serial.print(" snr="); Serial.println(pktSnr);
  }

  processReceivedPacket(rxBuf, rxLen, pktRssi, pktSnr);
}

static void radioHandleIrq() {
  //TODO: wtf: is this actually the irq handler? if so, this is way too big for an isr - but it looks like its just called if polling sees the flag true - so this needs to be renamed something more appropriate
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

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    // V4 FEM PA: disable amplifier at end of TX (Tracker has no external FEM).
#ifdef LORA_FEM_PA_PIN
    digitalWrite(LORA_FEM_PA_PIN, LOW);
#endif
    radioState = RADIO_STANDBY;
    if (LOG_TX_DONE)
    {
      uint64_t elapsed   = eventUs - (uint64_t)syncAnchorUs;
      uint32_t slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
      uint32_t posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
      uint8_t  seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
      Serial.print("TxDone: posInSlot="); Serial.print(posInSlot);
      Serial.print("us slot="); Serial.print(slotNum);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.println((int)SLOT_SEQUENCE[seqIdx]);
    }
    ledOff();
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    radioState = RADIO_STANDBY;
    // Force standby after RX_DONE. Some RX modes (implicit header, high SF like WIN_LR)
    // leave the radio in a transitional state where it won't idle naturally.
    // Issue explicit standby command to force the transition.
    sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC);
    // Wait for BUSY to clear. Typical: <20µs, but cap at 1ms to avoid infinite spin.
    unsigned long t0 = micros();
    while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 1000) {}
    // Log RxDone timing relative to current slot — diagnostic for sync drift. Pre-sync
    // the anchor is 0 so posInSlot is (micros() % SLOT_DURATION); that's still useful
    // because CMD_SET_SYNC RxDone is the event that SETS the anchor.
    if (LOG_RX_DONE) {
      uint64_t elapsed   = eventUs - (uint64_t)syncAnchorUs;
      uint32_t slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
      uint32_t posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
      uint8_t  seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
      Serial.print("RxDone: posInSlot="); Serial.print(posInSlot);
      Serial.print("us slot="); Serial.print(slotNum);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.println((int)SLOT_SEQUENCE[seqIdx]);
    }
    handleRxDone();
    ledOff();
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    radioState = RADIO_STANDBY;
    ledOff();
    if (LOG_RX_TIMEOUT) {
      uint64_t elapsed   = eventUs - (uint64_t)syncAnchorUs;
      uint32_t slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
      uint32_t posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
      uint8_t  seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
      Serial.print("RxTimeout: posInSlot="); Serial.print(posInSlot);
      Serial.print("us slot="); Serial.print(slotNum);
      Serial.print(" seqIdx="); Serial.print(seqIdx);
      Serial.print(" win="); Serial.println((int)SLOT_SEQUENCE[seqIdx]);
    }
  }

  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    radioState = RADIO_STANDBY;
    ledOff();
    invalidRxCount++;
    // Rate-limit header/CRC error logs — they can be frequent on a busy channel.
    static unsigned long lastRxErrLogMs = 0;
    unsigned long nowMs = millis();
    if (nowMs - lastRxErrLogMs >= 5000) {
      lastRxErrLogMs = nowMs;
      Serial.print("RX: ");
      if (irqFlags & SX126X_IRQ_CRC_ERROR)    Serial.print("CRC_ERROR ");
      if (irqFlags & SX126X_IRQ_HEADER_ERROR) Serial.print("HEADER_ERROR ");
      Serial.println();
    }
  }

  if (irqFlags == 0) {
    Serial.println("IRQ: flags=0 (spurious or ISR race)");
    // Don't change state — radio is still doing whatever it was doing.
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================
//
// Single slot-based path — anchor=0 pre-sync, anchor=RxDone time when synced.
// WIN_TELEM: TX telem at slot boundary (or RX if txSendingEnabled=false).
// WIN_CMD:   RX for the slot. radioStartRx() picks short vs long timeout:
//            - short (ROCKET_RX_TIMEOUT_US) when synced and heard a valid cmd recently
//            - long (ROCKET_LONG_RX_TIMEOUT_US) when never-synced this session OR when
//              synced but no valid cmd in ROCKET_NO_BASE_HEARD_THRESHOLD_US (lost-rocket
//              recovery — lets a fresh CMD_SET_SYNC land after drift).
// LED: on whenever radio is active (RX or TX), off when standby.

static unsigned long lastTelemTxUs = 0;

#ifdef ROCKET_RADIO_TEST_MODE
// In test mode: just RX continuously, log everything received. No TX, no sync.
void nonblockingRadio() {
  if (!loraReady) return;
  static uint32_t lastIsrCount = 0;
  uint32_t isrNow = dio1IsrCount;
  if (isrNow != lastIsrCount) {
    Serial.print("TEST DIO1 ISR! count="); Serial.println(isrNow);
    lastIsrCount = isrNow;
  }
  if (!dio1Fired && digitalRead(LORA_DIO1_PIN) &&
      (radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE)) {
    Serial.println("TEST DIO1 pin HIGH (poll fallback)");
    dio1CaptureVal = micros();
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
        dio1CaptureVal = micros();
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

// ===================== PER-SLOT CONFIG SWITCH =====================
// Applies slot radio params when targetCfg differs from appliedCfg.
// Two SPI commands back-to-back; BUSY between them spins up to 100µs (typical: <20µs).
// Called once per slot boundary — not every loop — so 100µs is well within budget.

static bool applyCfgIfNeeded() {
  uint64_t nowUs = (uint64_t)micros();
  uint64_t elapsed = nowUs - (uint64_t)syncAnchorUs;
  uint32_t slotNum = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t seqIdx = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  if (radioState != RADIO_STANDBY) {
    Serial.print("applyCfg FAILED: radioState="); Serial.print(radioState);
    Serial.print(" (not STANDBY) at posInSlot="); Serial.print(posInSlot);
    Serial.print("us slot="); Serial.print(slotNum);
    Serial.print(" seqIdx="); Serial.println(seqIdx);
    return false;
  }

  sx126x_mod_params_lora_t mp = {};
  if (targetCfg == RADIO_CFG_LR) {
    mp.sf   = (sx126x_lora_sf_t)LORA_LR_SF;
    mp.bw   = bwKHzToEnum(activeBwKHz);
    mp.cr   = (sx126x_lora_cr_t)LORA_LR_CR;
    mp.ldro = 1;
    if (LOG_APPLYCFG) {
      Serial.print("applyCfg: LR SF"); Serial.print(LORA_LR_SF);
      Serial.print(" BW"); Serial.print((int)activeBwKHz); Serial.println(" CR-LI LDRO");
    }
  } else {
    mp.sf   = sfToEnum(activeSF);
    mp.bw   = bwKHzToEnum(activeBwKHz);
    mp.cr   = SX126X_LORA_CR_4_5;
    mp.ldro = 0;
    if (LOG_APPLYCFG) {
      Serial.print("applyCfg: NORMAL @ pis=");
      Serial.println(posInSlot);
    }
  }
  sx126x_set_lora_mod_params(&radioCtx, &mp);

  // Wait for BUSY to clear between the two SPI commands. Typical: <20µs. Hard cap: 100µs.
  unsigned long t0 = micros();
  while (digitalRead(LORA_BUSY_PIN) && (micros() - t0) < 100) {}

  sx126x_pkt_params_lora_t pp = {};
  if (targetCfg == RADIO_CFG_LR) {
    pp.preamble_len_in_symb = 5;
    pp.header_type          = SX126X_LORA_PKT_IMPLICIT;
    pp.pld_len_in_bytes     = 3;
    pp.crc_is_on            = false;
  } else {
    pp.preamble_len_in_symb = LORA_PREAMBLE;
    pp.header_type          = SX126X_LORA_PKT_EXPLICIT;
    pp.pld_len_in_bytes     = 255;
    pp.crc_is_on            = true;
  }
  pp.invert_iq_is_on = false;
  sx126x_set_lora_pkt_params(&radioCtx, &pp);

  appliedCfg = targetCfg;
  return true;
}

void nonblockingRadio() {
  if (!loraReady) return;

  if (dio1Fired) {
    radioHandleIrq();
  }

  if (radioState == RADIO_TX_ACTIVE) return;  // TX in progress — wait for TxDone IRQ

  // Single slot-based path for both pre-sync and synced operation.
  // Pre-sync: anchor=0, slots cycle from boot. WIN_CMD RX uses the long window.
  // Synced: anchor from CMD_SET_SYNC RxDone. WIN_CMD RX short by default, long if we
  //   haven't heard from base in ROCKET_NO_BASE_HEARD_THRESHOLD_US (see radioStartRx).
  unsigned long now      = micros();
  unsigned long elapsed  = now - syncAnchorUs;
  uint32_t      slotNum  = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint8_t       seqIdx   = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win      = SLOT_SEQUENCE[seqIdx];

  static bool slotActionDone = false;  // true after TX fired or RX started this slot

  // On new slot boundary: never force standby on an active RX. If radioState is still
  // RADIO_RX_ACTIVE, the SX1262 is either still in the preamble detection window or actively
  // receiving a packet. The hardware timeout (no preamble) or packet-end IRQ will return it
  // to STANDBY on its own. Forcing standby mid-packet loses the frame.
  // applyCfgIfNeeded() is a no-op while RX_ACTIVE and will retry next slot boundary.
  // Safety cutoff: if RX has been active for more than RX_STUCK_MAX_SLOTS full slots,
  // something is stuck (missed DIO1 or IRQ failure) — force standby to recover.
  static unsigned long rxActiveStartUs = 0;
  if (radioState == RADIO_RX_ACTIVE && rxActiveStartUs == 0) rxActiveStartUs = now;
  if (radioState != RADIO_RX_ACTIVE) rxActiveStartUs = 0;
  if (radioState == RADIO_RX_ACTIVE &&
      (now - rxActiveStartUs) > RX_STUCK_MAX_SLOTS * SLOT_DURATION_US) {
    Serial.print("RADIO: RX stuck >"); Serial.print(RX_STUCK_MAX_SLOTS);
    Serial.println(" slots — forcing standby");
    radioStandby();
    rxActiveStartUs = 0;
  }

  if (slotNum != lastHandledSlotNum) {
    lastHandledSlotNum = slotNum;
    slotActionDone = false;

    RadioSlotConfig newTarget = (win == WIN_LR) ? RADIO_CFG_LR : RADIO_CFG_NORMAL;
    if (newTarget != targetCfg) targetCfg = newTarget;

    if (!applyCfgIfNeeded()) {
      // Config failed to apply — skip TX/RX for this slot.
      slotActionDone = true;
      return;
    }
  }

  if (radioState != RADIO_STANDBY) return;
  if (slotActionDone) return;  // TX complete or RX started — wait for next slot boundary

  slotActionDone = true;
  if (win == WIN_TELEM) {
    if (txSendingEnabled) {
      uint8_t pkt[255];  // 255 to accommodate thrust curve page (up to 217 bytes at SF≤7)
      size_t len = buildTelemetryPacket(pkt);
      radioStartTx(pkt, len);
    } else {
      // TX disabled — listen for the slot (e.g. ground test, download mode).
      radioStartRx();
    }
  } else if (win == WIN_LR) {
    if (txSendingEnabled) {
      uint8_t pkt[3];
      size_t len = buildLRPacketCore(pkt);
      radioStartTx(pkt, len);
    }
    // If TX disabled, stay standby for WIN_LR (nothing to RX from ourselves).
  } else {
    // WIN_CMD (and any future slot types): listen for a command.
    radioStartRx();
  }
}

#endif  // ROCKET_RADIO_TEST_MODE
