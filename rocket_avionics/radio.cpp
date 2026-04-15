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
  .spi  = &loraSPI,
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
// Excludes packet reception so it reflects background noise, not signal strength.

double rssiEma = 0.0;
static bool rssiEmaInit = false;
#define RSSI_EMA_ALPHA 0.1

// ===================== STATS =====================

uint16_t delayedTxCount = 0;
uint16_t invalidRxCount = 0;

// ===================== HeaderValid capture (base-station drift logging) =====================
// Captured on HEADER_VALID IRQ, consumed/discarded on RX_DONE or TIMEOUT.

static uint64_t pendingHeaderValidUs = 0;

// ===================== HELPERS =====================

void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz   = (activeChannel < 64) ? 125.0f : 500.0f;
}

static sx126x_lora_bw_t bwKHzToEnum(float bwKHz) {
  if (bwKHz >= 490.0f) return SX126X_LORA_BW_500;
  if (bwKHz >= 240.0f) return SX126X_LORA_BW_250;
  return SX126X_LORA_BW_125;  // default; narrower BW not used in current channel plan
}

static sx126x_lora_sf_t sfToEnum(uint8_t sf) {
  // SX126X_LORA_SF5..SF12 enum values equal 0x05..0x0C which matches the SF integer
  return (sx126x_lora_sf_t)sf;
}

static void ledOn()  { ledcWrite(LED_PIN, 255); }
static void ledOff() { ledcWrite(LED_PIN, 0);   }

// ===================== INIT =====================

bool radioInit() {
  // SPI must already be started (loraSPI.begin) before calling this.
  // Pin modes for NSS and RST are set here; BUSY is input-only.
  pinMode(LORA_NSS_PIN,  OUTPUT);
  digitalWrite(LORA_NSS_PIN, HIGH);
  pinMode(LORA_BUSY_PIN, INPUT);

  // Reset: HAL holds RST low 1 ms then releases. Chip needs ~3 ms to be ready.
  // After reset we poll BUSY in a short spin — this is init only, acceptable.
  sx126x_hal_reset(&radioCtx);
  unsigned long t0 = millis();
  while (digitalRead(LORA_BUSY_PIN) && (millis() - t0) < 100) {}
  if (digitalRead(LORA_BUSY_PIN)) {
    Serial.println("LoRa init: BUSY stuck after reset");
    return false;
  }

  // Standby (RC oscillator)
  if (sx126x_set_standby(&radioCtx, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_standby failed");
    return false;
  }

  // Packet type: LoRa
  if (sx126x_set_pkt_type(&radioCtx, SX126X_PKT_TYPE_LORA) != SX126X_STATUS_OK) {
    Serial.println("LoRa init: set_pkt_type failed");
    return false;
  }

  // DIO2 as RF switch control (replaces radio.setDio2AsRfSwitch(true))
  sx126x_set_dio2_as_rf_sw_ctrl(&radioCtx, true);

  // Sync word (0x12 = private network). Write directly to register.
  // SX126x LoRa sync word register: 0x0740 (MSB), 0x0741 (LSB).
  // Private = 0x1424 per datasheet §4.2.1.
  {
    const uint8_t syncWord[2] = { 0x14, 0x24 };
    sx126x_write_register(&radioCtx, 0x0740, syncWord, 2);
  }

  // Modulation and packet parameters are applied by radioApplyConfig() below.
  radioApplyConfig();

  // IRQ mask: TX_DONE | RX_DONE | TIMEOUT | HEADER_VALID — all on DIO1, none on DIO2/DIO3.
  sx126x_set_dio_irq_params(&radioCtx,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_VALID,
    SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT | SX126X_IRQ_HEADER_VALID,
    SX126X_IRQ_NONE,
    SX126X_IRQ_NONE);

  sx126x_clear_irq_status(&radioCtx, SX126X_IRQ_ALL);

  // MCPWM hardware capture on DIO1 — replaces attachInterrupt / setDio1Action.
  radioMcpwmInit(LORA_DIO1_PIN);

  radioState = RADIO_STANDBY;
  return true;
}

void radioApplyConfig() {
  // Call from RADIO_STANDBY only. No explicit standby call needed — we're already there.
  sx126x_mod_params_lora_t modParams = {};
  modParams.sf = sfToEnum(activeSF);
  modParams.bw = bwKHzToEnum(activeBwKHz);
  modParams.cr = SX126X_LORA_CR_4_5;
  modParams.ldro = 0;  // low data rate optimise: set to 1 for SF11/SF12 + BW125; off otherwise
  sx126x_set_lora_mod_params(&radioCtx, &modParams);

  sx126x_pkt_params_lora_t pktParams = {};
  pktParams.preamble_len_in_symb = LORA_PREAMBLE;
  pktParams.header_type          = SX126X_LORA_PKT_EXPLICIT;
  pktParams.pld_len_in_bytes     = 255;  // max for variable-length explicit-header packets
  pktParams.crc_is_on            = true;
  pktParams.invert_iq_is_on      = false;
  sx126x_set_lora_pkt_params(&radioCtx, &pktParams);

  // Frequency (Hz)
  uint32_t freqHz = (uint32_t)(activeFreqMHz * 1e6f + 0.5f);
  sx126x_set_rf_freq(&radioCtx, freqHz);

  // TX power and ramp time
  sx126x_set_tx_params(&radioCtx, activePower, SX126X_RAMP_200_US);

  // PA config for SX1262 (device_sel=0 = SX1262, hp_max=7, pa_lut=1 per datasheet §13.1.14)
  sx126x_pa_cfg_params_t paCfg = { .pa_duty_cycle = 0x04, .hp_max = 0x07, .device_sel = 0x00, .pa_lut = 0x01 };
  sx126x_set_pa_cfg(&radioCtx, &paCfg);
}

// ===================== SYNC =====================

void radioSetSynced(unsigned long anchorUs, uint8_t slotIdx) {
  radioStandby();
  dio1Fired          = false;
  pendingHeaderValidUs = 0;
  radioState         = RADIO_STANDBY;
  syncAnchorUs       = anchorUs;
  syncSlotIndex      = slotIdx;
  lastHandledSlotNum = 0xFFFFFFFF;
  radioSynced        = true;
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
  // Continuous RX (SX126X_RX_CONTINUOUS = no timeout)
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
  // Load FIFO then start TX (0 ms timeout = single packet, no timeout)
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
    pendingHeaderValidUs = 0;
    return;
  }

  uint8_t rxBuf[255];
  uint8_t rxLen = bufStatus.pld_len_in_bytes;
  if (rxLen == 0 || rxLen > sizeof(rxBuf)) {
    Serial.print("RX: bad length "); Serial.println(rxLen);
    invalidRxCount++;
    pendingHeaderValidUs = 0;
    return;
  }

  if (sx126x_read_buffer(&radioCtx, bufStatus.buffer_start_pointer, rxBuf, rxLen) != SX126X_STATUS_OK) {
    Serial.println("RX: read_buffer fail");
    invalidRxCount++;
    pendingHeaderValidUs = 0;
    return;
  }

  sx126x_pkt_status_lora_t pktStatus = {};
  sx126x_get_lora_pkt_status(&radioCtx, &pktStatus);
  int8_t pktRssi = pktStatus.rssi_pkt_in_dbm;
  int8_t pktSnr  = pktStatus.snr_pkt_in_db;

  // Basic structural validation before using the HeaderValid timestamp.
  // Telemetry has no HMAC — check minimum length and known packet type byte.
  bool structOk = (rxLen >= 2) && (rxBuf[0] == PKT_TELEMETRY || rxBuf[0] == PKT_COMMAND ||
                                    rxBuf[0] == PKT_BACKHAUL);
  if (!structOk) {
    Serial.print("RX: unknown type 0x"); Serial.println(rxBuf[0], HEX);
    invalidRxCount++;
    pendingHeaderValidUs = 0;
    return;
  }

  // Log HeaderValid position if captured.
  if (pendingHeaderValidUs != 0) {
    if (radioSynced) {
      unsigned long posInSlot = (unsigned long)(pendingHeaderValidUs - syncAnchorUs) % SLOT_DURATION_US;
      Serial.print("RX: HeaderValid posInSlot="); Serial.print(posInSlot); Serial.println("us");
    }
    pendingHeaderValidUs = 0;
  }

  processReceivedPacket(rxBuf, rxLen, pktRssi, pktSnr);
}

// Called from nonblockingRadio() when dio1Fired is set.
static void radioHandleIrq() {
  uint64_t eventUs = dio1TimestampUs();  // hardware-latched capture time
  dio1Fired = false;

  sx126x_irq_mask_t irqFlags = 0;
  sx126x_get_and_clear_irq_status(&radioCtx, &irqFlags);

  if (irqFlags & SX126X_IRQ_HEADER_VALID) {
    // Hold until RX_DONE confirms a valid packet; discard on TIMEOUT or bad packet.
    pendingHeaderValidUs = eventUs;
  }

  if (irqFlags & SX126X_IRQ_TX_DONE) {
    ledOff();
    radioState = RADIO_STANDBY;
    unsigned long posInSlot = radioSynced ?
      (unsigned long)((micros() - syncAnchorUs) % SLOT_DURATION_US) : 0;
    Serial.print("SLOT TxDone posInSlot="); Serial.println(posInSlot);
  }

  if (irqFlags & SX126X_IRQ_RX_DONE) {
    ledOff();
    handleRxDone();
    radioState = RADIO_STANDBY;
    unsigned long posInSlot = radioSynced ?
      (unsigned long)((micros() - syncAnchorUs) % SLOT_DURATION_US) : 0;
    Serial.print("SLOT RxDone posInSlot="); Serial.println(posInSlot);
  }

  if (irqFlags & SX126X_IRQ_TIMEOUT) {
    ledOff();
    pendingHeaderValidUs = 0;
    // Sample background RSSI — only meaningful in WIN_CMD (rocket is RX).
    // Instantaneous read at a random point in the listen window; good enough
    // for a rough noise floor estimate.
    int16_t rssiDbm = 0;
    if (sx126x_get_rssi_inst(&radioCtx, &rssiDbm) == SX126X_STATUS_OK) {
      if (!rssiEmaInit) { rssiEma = rssiDbm; rssiEmaInit = true; }
      else rssiEma += RSSI_EMA_ALPHA * (rssiDbm - rssiEma);
    }
    radioState = RADIO_STANDBY;
  }

  // CRC_ERROR or HEADER_ERROR: discard and return to standby.
  if (irqFlags & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) {
    pendingHeaderValidUs = 0;
    radioState = RADIO_STANDBY;
    invalidRxCount++;
    Serial.println("RX: CRC/header error");
  }
}

// ===================== MAIN RADIO STATE MACHINE =====================

void nonblockingRadio() {
  if (!loraReady) return;

  // Always process DIO1 first, regardless of slot.
  if (dio1Fired) {
    radioHandleIrq();
    // After handling IRQ, fall through to slot logic only if now in STANDBY.
  }

  unsigned long now = micros();

  // ---- Bootstrap mode: continuous RX ----
  if (!radioSynced) {
    if (radioState == RADIO_STANDBY) {
      radioStartRx();
    }
    return;
  }

  // ---- Synced mode: slot-based scheduling ----

  unsigned long elapsed   = now - syncAnchorUs;
  uint32_t      slotNum   = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx    = (uint8_t)((syncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win       = SLOT_SEQUENCE[seqIdx];

  // Only act once per slot transition.
  if (slotNum == lastHandledSlotNum) return;

  // Don't preempt an in-progress operation — packet may run over its slot boundary.
  // Just miss this slot; the operation completes naturally.
  if (radioState == RADIO_TX_ACTIVE || radioState == RADIO_RX_ACTIVE) return;

  lastHandledSlotNum = slotNum;

  switch (win) {

    case WIN_TELEM: {
      if (!txSendingEnabled) {
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

    case WIN_CMD: {
      Serial.print("SLOT WIN_CMD RX pos="); Serial.print(posInSlot); Serial.println("us");
      ledOn();
      radioStartRxTimeout(ROCKET_RX_TIMEOUT_RAW);
      break;
    }

    case WIN_OFF:
    default:
      radioStandby();
      ledOff();
      break;
  }
}
