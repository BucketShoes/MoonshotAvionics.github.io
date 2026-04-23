// commands.cpp — HMAC authentication and command dispatch.
// See commands.h for the public API.

#include <Arduino.h>
#include <mbedtls/md.h>
#include "commands.h"
#include "radio.h"
#include "flight.h"
#include "pyro.h"
#include "log_store.h"
#include "globals.h"
#include "telemetry.h"
#include "ota.h"

// ===================== COMMAND STATE =====================

uint8_t  hmacKey[HMAC_KEY_LEN];
bool     hmacKeyValid = false;
uint32_t highestNonce = 0;

CommandAck lastAck = {0, CMD_ERR_UNKNOWN, 0, 0, 0, false};
unsigned long lastValidCmdUs = 0;

// ===================== HMAC-SHA256 =====================
// Covers payload bytes excluding the trailing HMAC itself.

static void computeHMAC(const uint8_t* data, size_t len, uint8_t* out) {
  uint8_t fullHmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, hmacKey, HMAC_KEY_LEN);
  mbedtls_md_hmac_update(&ctx, data, len);
  mbedtls_md_hmac_finish(&ctx, fullHmac);
  mbedtls_md_free(&ctx);
  memcpy(out, fullHmac, HMAC_TRUNC_LEN);
}

bool verifyCommandHMAC(const uint8_t* pkt, size_t pktLen) {
  if (!hmacKeyValid) return false;
  if (pktLen < HMAC_TRUNC_LEN + 7) return false;

  size_t dataLen = pktLen - HMAC_TRUNC_LEN;
  uint8_t computed[HMAC_TRUNC_LEN];
  computeHMAC(pkt, dataLen, computed);

  uint8_t diff = 0;
  for (int i = 0; i < HMAC_TRUNC_LEN; i++) {
    diff |= computed[i] ^ pkt[dataLen + i];
  }
  return (diff == 0);
}

uint32_t readU32(const uint8_t* buf) {
  return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

// ===================== PARAM LENGTH VALIDATION =====================

static int expectedParamLen(uint8_t cmdId) {
  switch (cmdId) {
    case CMD_ARM:           return -1;  // 0 (legacy), 9 (old params+flags), or 13 (new with fire durations)
    case CMD_DISARM:        return 0;
    case CMD_FIRE_PYRO:     return -1;  // 3 (channel + duration) or 4 (+ flags byte)
    case CMD_SET_TX_RATE:   return 1;   // rate(1)
    case CMD_SET_RADIO:     return 3;   // channel(1) + SF(1) + power(1)
    case CMD_OTA_BEGIN:     return 0;
    case CMD_OTA_FINALIZE:  return 36;  // fwSize(4) + fwHmac(32)
    case CMD_OTA_CONFIRM:   return 0;
    case CMD_SET_SYNC:      return 0;
    case CMD_PING:          return 0;
    case CMD_REBOOT:        return 0;
    case CMD_LOG_ERASE:     return 0;
    default:                return -1;
  }
}

// ===================== COMMAND DISPATCH =====================

void executeCommand(uint8_t cmdId, uint32_t nonce, const uint8_t* params, size_t paramsLen) {
  int expected = expectedParamLen(cmdId);
  if (expected >= 0 && paramsLen != (size_t)expected) {
    Serial.print("CMD 0x"); Serial.print(cmdId, HEX);
    Serial.print(": bad param len "); Serial.print(paramsLen);
    Serial.print(" expected "); Serial.println(expected);
    lastAck.nonce = nonce; lastAck.result = CMD_ERR_BAD_PARAMS; lastAck.pending = true;
    return;
  }

  uint8_t result = CMD_OK;

  switch (cmdId) {

    case CMD_ARM: {
      if (paramsLen != 0 && paramsLen != 9 && paramsLen != 13) { result = CMD_ERR_BAD_PARAMS; break; }
      uint8_t armResult = flightTryArm(params, paramsLen);
      if (armResult == ARM_OK) {
        if (logStoreOk) {
          logStore.clearProtectionPoint();
          logStore.setProtectionPoint();
          armRecordCounter = logStore.getRecordCounter();
        }
        peaks = {0, 0, 0};
        runningMaxLoopUs = 0;
        result = CMD_OK;
      } else if (armResult == ARM_ERR_ALREADY) {
        result = CMD_ERR_REFUSED;
      } else {
        result = armResult;
      }
      break;
    }

    case CMD_DISARM:
      flightDisarm();
      result = CMD_OK;
      break;

    case CMD_FIRE_PYRO: {
      if (paramsLen != 3 && paramsLen != 4) { result = CMD_ERR_BAD_PARAMS; break; }
      if (!isArmed) { result = CMD_ERR_REFUSED; break; }
      uint8_t  ch       = params[0];
      uint16_t dur      = (uint16_t)(params[1] | (params[2] << 8));
      uint8_t  flags    = (paramsLen >= 4) ? params[3] : 0;
      bool stayInPhase  = (flags & 0x01) != 0;
      if (ch < 1 || ch > 3 || dur == 0) { result = CMD_ERR_BAD_PARAMS; break; }
      pyroFire(ch, dur);
      if (!stayInPhase && flightState.phase != PHASE_GROUND_TEST) {
        // Transition to GROUND_TEST so launch detect can't misfire from shock
        unsigned long nowUs = micros();
        (void)nowUs;  // enterPhase is static in flight.cpp; call the public transition
        // Direct state write is safe here: GROUND_TEST has no entry actions in enterPhase
        flightState.phase = PHASE_GROUND_TEST;
      }
      result = CMD_OK;
      break;
    }

    case CMD_SET_TX_RATE: {
      int8_t rate = (int8_t)params[0];
      activeTxRate = rate;
      nvs.putChar("tx_rate", activeTxRate);
      txIntervalUs = txRateToIntervalUs(activeTxRate);
      txSendingEnabled = true;
      Serial.print("TX rate set: "); Serial.print(rate);
      Serial.print(" -> interval "); Serial.print(txIntervalUs); Serial.println("us");
      result = CMD_OK;
      break;
    }

    case CMD_SET_RADIO: {
      uint8_t ch    = params[0];
      uint8_t sf    = params[1];
      int8_t  power = (int8_t)params[2];

      float freq = channelToFreqMHz(ch);
      if (freq == 0.0f)           { result = CMD_ERR_BAD_PARAMS; break; }
      if (sf < 5 || sf > 12)      { result = CMD_ERR_BAD_PARAMS; break; }
      if (power < -9 || power > 22) { result = CMD_ERR_BAD_PARAMS; break; }

      activeChannel = ch;
      activeSF = sf;
      activePower = power;
      updateActiveFreqBw();

      nvs.putUChar("radio_ch", activeChannel);
      nvs.putUChar("radio_sf", activeSF);
      nvs.putChar("radio_pwr", activePower);

      // Apply new config. BLOCKING — radioApplyConfig_BLOCKING() calls
      // DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING (up to 100ms each).
      // TODO: @@@ refuse CMD_SET_RADIO while armed to prevent blocking the armed loop.
      radioStandby();
      radioApplyConfig_BLOCKING();
      radioStartRx();

      Serial.print("Radio set: ch"); Serial.print(ch);
      Serial.print(" "); Serial.print(activeFreqMHz, 1); Serial.print("MHz SF");
      Serial.print(sf); Serial.print(" BW"); Serial.print((int)activeBwKHz);
      Serial.print(" pwr="); Serial.println(power);
      result = CMD_OK;
      break;
    }

    case CMD_OTA_BEGIN:
      if (isArmed) {
        otaNotifyStatus(OTA_STATUS_REFUSED);
        result = CMD_ERR_REFUSED;
        break;
      }
      result = otaHandleBegin();
      // otaHandleBegin() notifies 0x00 on success (ready for chunks) and
      // returns CMD_ERR_REFUSED on failure — notify the refusal so JS can detect it.
      if (result != CMD_OK) otaNotifyStatus(OTA_STATUS_REFUSED);
      break;

    case CMD_OTA_FINALIZE: {
      if (isArmed) { result = CMD_ERR_REFUSED; break; }
      uint32_t fwSize  = readU32(&params[0]);
      const uint8_t* fwHmac = &params[4];
      result = otaHandleFinalize(fwHmac, fwSize);
      break;
    }

    case CMD_OTA_CONFIRM:
      result = otaHandleConfirm();
      break;

    case CMD_SET_SYNC:
      // Anchor the slot clock to this moment (RxDone = now).
      // slotIndex=1: the sync packet itself occupies slot 0 (WIN_TELEM from base's view),
      // so WIN_CMD starts immediately after.
      Serial.print("=======SYNC-====== dio1CaptureVal=");
      Serial.print(dio1CaptureVal);
      Serial.print("; micros()=");
      Serial.print(micros());
      radioSetSynced(dio1CaptureVal, 1); //TODO: wtf this shouldnt be micros() here - it should be captured eexactly at the point it comes in. check       dio1CaptureVal. although it should only be a matter of fraction of a millisecond
      result = CMD_OK;
      break;

    case CMD_PING:
      result = CMD_OK;
      break;

    case CMD_REBOOT:
      if (isArmed) { result = CMD_ERR_REFUSED; break; }
      Serial.println("REBOOTING");
      ESP.restart();
      break;

    case CMD_LOG_ERASE:
      if (isArmed)     { result = CMD_ERR_REFUSED; break; }
      if (logStoreOk)  { logStore.eraseLogs(); result = CMD_OK; }
      else             { result = CMD_ERR_REFUSED; }
      break;

    default:
      result = CMD_ERR_UNKNOWN;
      break;
  }

  lastAck.nonce = nonce;
  lastAck.result = result;
  lastAck.pending = true;
}

// ===================== PACKET RECEPTION =====================

void processReceivedPacket(const uint8_t* pkt, size_t pktLen, int8_t rssi, int8_t snr) {
  bool isBle = (rssi == LOG_SNR_LOCAL && snr == LOG_SNR_LOCAL);
  if (isBle) {
    Serial.print("BLE RX: ");
  } else {
    Serial.print("RX: ");
  }
  printPacketHex(pkt, pktLen);
  if (isBle) {
    Serial.print(" ("); Serial.print(pktLen); Serial.println("B BLE)");
  } else {
    Serial.print(" ("); Serial.print(pktLen);
    Serial.print("B RSSI="); Serial.print((int)rssi);
    Serial.print(" SNR="); Serial.print((float)snr, 2);
    Serial.println(")");
  }

  if (pktLen < 17) { invalidRxCount++; return; }
  if (pkt[0] != PKT_COMMAND) { invalidRxCount++; return; }

  if (pkt[1] != DEVICE_ID) {
    Serial.print("CMD: saw other target:"); Serial.print(pkt[1]);
    Serial.print(" we are:"); Serial.println(DEVICE_ID);
    return;
  }

  if (!verifyCommandHMAC(pkt, pktLen)) {
    lastAck.invalidHmacCount++;
    invalidRxCount++;
    Serial.println("CMD: HMAC fail");
    // Mark CMD ACK page fresh so BLE phone sees the updated invalidHmacCount.
    // Do NOT set lastAck.pending or overwrite lastAck.result — unsigned traffic
    // must not be able to prevent real acks from being seen.
    logPages[LOGI_CMD_ACK].freshMask |= 0xFF;
    return;
  }

  uint8_t cmdId = pkt[2];
  uint32_t nonce = readU32(&pkt[3]);

  if (nonce <= highestNonce) {
    Serial.println("CMD: stale nonce");
    invalidRxCount++;
    return;
  }

  highestNonce = nonce;
  nvs.putUInt("nonce", highestNonce);
  lastValidCmdUs = micros();

  lastAck.rssi = rssi;
  lastAck.snr  = (int8_t)((float)snr * 4);

  size_t paramsOffset = 7;
  size_t paramsLen = pktLen - paramsOffset - HMAC_TRUNC_LEN;
  const uint8_t* params = &pkt[paramsOffset];

  Serial.print("CMD: 0x"); Serial.print(cmdId, HEX);
  Serial.print(" nonce="); Serial.println(nonce);

  // Log raw command packet to flash. SNR stored in dB*4 format (spec).
  if (logStoreOk && pktLen > 0 && pktLen <= LOG_MAX_PAYLOAD) {
    logStore.writeRecord(pkt, (uint8_t)pktLen, (int8_t)((int)snr * 4), millis());
  }

  executeCommand(cmdId, nonce, params, paramsLen);
  // Mark cmd ack page fresh for all transports after any command execution
  logPages[LOGI_CMD_ACK].freshMask |= 0xFF;
}
