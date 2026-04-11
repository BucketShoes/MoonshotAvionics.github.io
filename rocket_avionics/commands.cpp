// commands.cpp — HMAC authentication, command dispatch, and log download.
// See commands.h for the public API.

#include <Arduino.h>
#include <mbedtls/md.h>
#include "commands.h"
#include "radio.h"
#include "flight.h"
#include "log_store.h"
#include "globals.h"
#include "telemetry.h"

// ===================== COMMAND STATE =====================

uint8_t  hmacKey[HMAC_KEY_LEN];
bool     hmacKeyValid = false;
uint32_t highestNonce = 0;

CommandAck lastAck = {0, CMD_ERR_UNKNOWN, 0, 0, 0, false};

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
    case CMD_ARM:           return -1;  // 0 (legacy) or 9 (params+flags)
    case CMD_DISARM:        return 0;
    case CMD_FIRE_PYRO:     return 3;   // channel(1) + duration(2)
    case CMD_SET_TX_RATE:   return 1;   // rate(1)
    case CMD_SET_RADIO:     return 3;   // channel(1) + SF(1) + power(1)
    case CMD_LOG_DOWNLOAD:  return 10;  // start(4)+count(2)+ch(1)+SF(1)+BW(1)+power(1)
    case CMD_PING:          return 0;
    case CMD_REBOOT:        return 0;
    case CMD_LOG_ERASE:     return 0;
    default:                return -1;
  }
}

// ===================== LOG DOWNLOAD =====================
// Blocking transfer of log records as 0xCA chunks over LoRa.
// Only called while disarmed. Takes over the radio completely.

static float bwCodeToKHz(uint8_t bwCode) {
  switch (bwCode & 0x0F) {
    case 0: return 500.0f;
    case 1: return 250.0f;
    case 2: return 125.0f;
    case 3: return 62.5f;
    case 4: return 41.7f;
    case 5: return 31.25f;
    case 6: return 20.8f;
    case 7: return 15.6f;
    case 8: return 10.4f;
    case 9: return 7.8f;
    default: return 0.0f;
  }
}

#define LORA_MAX_PAYLOAD  255
#define CHUNK_HDR_SIZE    7       // type(1)+deviceId(1)+startRecIdx(4)+count(1)
#define DOWNLOAD_IDLE_TIMEOUT_MS  5000

static bool sendLogChunk(uint32_t startRecIdx, uint8_t count,
                         const uint8_t* recordData, size_t recordDataLen) {
  uint8_t pkt[LORA_MAX_PAYLOAD];
  if (CHUNK_HDR_SIZE + recordDataLen > LORA_MAX_PAYLOAD) return false;

  size_t pos = 0;
  pkt[pos++] = PKT_LOG_CHUNK;
  pkt[pos++] = DEVICE_ID;
  pkt[pos++] = startRecIdx & 0xFF;
  pkt[pos++] = (startRecIdx >> 8) & 0xFF;
  pkt[pos++] = (startRecIdx >> 16) & 0xFF;
  pkt[pos++] = (startRecIdx >> 24) & 0xFF;
  pkt[pos++] = count;
  memcpy(pkt + pos, recordData, recordDataLen);
  pos += recordDataLen;

  int state = radio.transmit(pkt, pos);
  Serial.println("sent chunk");
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("DL TX err: "); Serial.println(state);
    return false;
  }
  return true;
}

static bool listenForDownloadRequest(uint32_t timeoutMs,
                                     uint8_t* outParams, size_t* outParamsLen,
                                     uint32_t* outNonce) {
  unsigned long deadline = millis() + timeoutMs;

  dio1Fired = false;
  radio.startReceive();

  while ((long)(deadline - millis()) > 0) {
    if (!dio1Fired) { delay(1); continue; }
    dio1Fired = false;

    uint8_t rxBuf[64];
    int state = radio.readData(rxBuf, sizeof(rxBuf));
    if (state != RADIOLIB_ERR_NONE) { radio.startReceive(); continue; }
    size_t rxLen = radio.getPacketLength();

    if (rxLen < 17 || rxBuf[0] != PKT_COMMAND || rxBuf[1] != DEVICE_ID) {
      radio.startReceive(); continue;
    }
    if (!verifyCommandHMAC(rxBuf, rxLen)) {
      lastAck.invalidHmacCount++;
      radio.startReceive(); continue;
    }

    uint32_t nonce = readU32(&rxBuf[3]);
    if (nonce <= highestNonce) { radio.startReceive(); continue; }

    highestNonce = nonce;
    nvs.putUInt("nonce", highestNonce);

    lastAck.rssi = (int8_t)radio.getRSSI(true);
    lastAck.snr  = (int8_t)(radio.getSNR() * 4);

    uint8_t cmdId = rxBuf[2];
    size_t paramsLen = rxLen - 7 - HMAC_TRUNC_LEN;

    if (cmdId == CMD_LOG_DOWNLOAD && paramsLen == 10) {
      memcpy(outParams, &rxBuf[7], paramsLen);
      *outParamsLen = paramsLen;
      *outNonce = nonce;
      return true;
    }

    // Not a download request — ack refused, continue listening
    lastAck.nonce = nonce;
    lastAck.result = CMD_ERR_REFUSED;
    lastAck.pending = true;
    radio.startReceive();
  }
  return false;
}

static uint32_t resolveRecordIndex(int32_t relStart, uint32_t armRec, uint32_t currentRec) {
  int64_t absIdx = (int64_t)armRec + (int64_t)relStart;
  if (absIdx < 0) absIdx = 0;
  if (absIdx > (int64_t)currentRec) absIdx = currentRec;
  return (uint32_t)absIdx;
}

static void streamRecords(uint32_t absStart, uint32_t absEnd) {
  uint32_t recIdx = absStart;
  while (recIdx < absEnd) {
    uint8_t recordBuf[LORA_MAX_PAYLOAD];
    size_t dataPos = 0;
    uint8_t recsInChunk = 0;
    uint32_t chunkStartRec = recIdx;
    size_t maxDataBytes = LORA_MAX_PAYLOAD - CHUNK_HDR_SIZE;

    while (recIdx < absEnd) {
      uint8_t tmpRec[LOG_HDR_SIZE + LOG_MAX_PAYLOAD];
      int recSize = logStore.readRecordRaw(recIdx, tmpRec, sizeof(tmpRec));
      if (recSize < 0) { recIdx++; continue; }
      if (dataPos + (size_t)recSize > maxDataBytes) break;
      memcpy(recordBuf + dataPos, tmpRec, recSize);
      dataPos += recSize;
      recsInChunk++;
      recIdx++;
    }

    if (recsInChunk > 0) {
      if (!sendLogChunk(chunkStartRec, recsInChunk, recordBuf, dataPos)) {
        Serial.println("DL: TX fail, aborting");
        return;
      }
      Serial.print("DL: sent "); Serial.print(recsInChunk);
      Serial.print(" recs from #"); Serial.println(chunkStartRec);
    }
  }
}

static void executeLogDownload(uint32_t cmdNonce, const uint8_t* params) {
  int32_t  relStart  = (int32_t)readU32(&params[0]);
  uint16_t reqCount  = (uint16_t)(params[4] | (params[5] << 8));
  uint8_t  dlChannel = params[6];
  uint8_t  dlSF      = params[7];
  uint8_t  dlBWcode  = params[8];
  int8_t   dlPower   = (int8_t)params[9];

  float dlBW = bwCodeToKHz(dlBWcode);
  if (dlBW == 0.0f) {
    lastAck.nonce = cmdNonce; lastAck.result = CMD_ERR_BAD_PARAMS; lastAck.pending = true;
    return;
  }
  if (dlSF < 5 || dlSF > 12) {
    lastAck.nonce = cmdNonce; lastAck.result = CMD_ERR_BAD_PARAMS; lastAck.pending = true;
    return;
  }
  float dlFreq = channelToFreqMHz(dlChannel);
  if (dlFreq == 0.0f) {
    lastAck.nonce = cmdNonce; lastAck.result = CMD_ERR_BAD_PARAMS; lastAck.pending = true;
    return;
  }

  uint32_t currentRec = logStore.getRecordCounter();
  uint32_t absStart = resolveRecordIndex(relStart, armRecordCounter, currentRec);
  uint32_t absEnd = absStart + reqCount;
  if (absEnd > currentRec) absEnd = currentRec;

  Serial.print("DL: rec "); Serial.print(absStart);
  Serial.print("-"); Serial.print(absEnd);
  Serial.print(" ch"); Serial.print(dlChannel);
  Serial.print(" "); Serial.print(dlFreq, 1); Serial.print("MHz");
  Serial.print(" SF"); Serial.print(dlSF);
  Serial.print(" BW"); Serial.print((int)dlBW);
  Serial.print(" pwr="); Serial.println(dlPower);

  lastAck.nonce = cmdNonce;
  lastAck.result = CMD_OK;
  lastAck.pending = true;

  if (!radioSetDownloadConfig(dlFreq, dlSF, dlBW, dlPower)) {
    Serial.println("DL: radio config fail, aborting");
    radioRestoreNormalConfig();
    radioStartRx();
    return;
  }

  delayMicroseconds(2000);  // allow both sides to switch (per spec) - NOTE: this is a blocking delay - this is allowed specifically in log download, which is blocking - this (and others) is the main reason log download in this form isnt allowed while armed

  streamRecords(absStart, absEnd);

  Serial.println("DL: batch done, listening for more requests");

  for (;;) {
    uint8_t nextParams[12];
    size_t nextParamsLen = 0;
    uint32_t nextNonce = 0;

    if (!listenForDownloadRequest(DOWNLOAD_IDLE_TIMEOUT_MS, nextParams, &nextParamsLen, &nextNonce)) {
      break;
    }

    int32_t  nextRelStart = (int32_t)readU32(&nextParams[0]);
    uint16_t nextReqCount = (uint16_t)(nextParams[4] | (nextParams[5] << 8));

    uint32_t nextAbsStart = resolveRecordIndex(nextRelStart, armRecordCounter, logStore.getRecordCounter());
    uint32_t nextAbsEnd = nextAbsStart + nextReqCount;
    if (nextAbsEnd > logStore.getRecordCounter()) nextAbsEnd = logStore.getRecordCounter();

    lastAck.nonce = nextNonce;
    lastAck.result = CMD_OK;
    lastAck.pending = true;

    Serial.print("DL: follow-up rec "); Serial.print(nextAbsStart);
    Serial.print("-"); Serial.println(nextAbsEnd);

    streamRecords(nextAbsStart, nextAbsEnd);
    Serial.println("DL: follow-up batch done");
  }

  Serial.println("DL: reverting to normal radio");
  radioRestoreNormalConfig();
  radioStartRx();
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
      if (paramsLen != 0 && paramsLen != 9) { result = CMD_ERR_BAD_PARAMS; break; }
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

    case CMD_FIRE_PYRO:
      if (!isArmed) { result = CMD_ERR_REFUSED; break; }
      // TODO: implement pyro fire (parse channel + duration, set GPIO high, main loop cuts off)
      result = CMD_ERR_UNKNOWN;
      break;

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

      radioRestoreNormalConfig();
      radioStartRx();

      Serial.print("Radio set: ch"); Serial.print(ch);
      Serial.print(" "); Serial.print(activeFreqMHz, 1); Serial.print("MHz SF");
      Serial.print(sf); Serial.print(" BW"); Serial.print((int)activeBwKHz);
      Serial.print(" pwr="); Serial.println(power);
      result = CMD_OK;
      break;
    }

    case CMD_LOG_DOWNLOAD:
      if (isArmed)     { result = CMD_ERR_REFUSED; break; }
      if (!logStoreOk) { result = CMD_ERR_REFUSED; break; }
      executeLogDownload(nonce, params);
      return;  // executeLogDownload handles ack internally

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
