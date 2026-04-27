// Base Station / Relay - Heltec Wireless Tracker V1.1
// Supports multiple transports: WiFi/WebSocket, BLE GATT, (future: USB Serial)
// BLE allows the phone to maintain mobile internet while connected to base station.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <mbedtls/md.h>
#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <esp_ota_ops.h>
#include "log_store.h"
#include "esp_wifi.h"
#include "radio.h"
//#include "tagged_serial.h"  // Serial wrapper that prefixes boot-relative micros

#define VEXT_CTRL_PIN 3
#define VBAT_ADC_PIN      1
#define VBAT_ADC_CTRL_PIN 2
#define VBAT_MULTIPLIER   4.9f

#define SERIAL_BAUD   115200
#include "secrets.h"  // gitignored — copy secrets_example.h to secrets.h
#define WIFI_CHANNEL  1

#define DEVICE_ID     157

// ===================== TAGGED SERIAL =====================
unsigned long bootMicros = 0;
//TaggedSerial taggedSerial(&Serial0);

// ===================== OTA COMMAND IDs (matching rocket_avionics/config.h) =====================
#define CMD_OTA_BEGIN     0x50
#define CMD_OTA_FINALIZE  0x51
#define CMD_OTA_CONFIRM   0x52

// ===================== TRANSPORT FLAGS (for 0x31/0x32 enable/disable) =====================
#define TRANSPORT_WIFI  0x01
#define TRANSPORT_BLE   0x02
#define TRANSPORT_USB   0x04  // reserved for future
#define TRANSPORT_ALL   (TRANSPORT_WIFI | TRANSPORT_BLE | TRANSPORT_USB)

bool wifiEnabled = true;
bool bleEnabled = true;

// ===================== HARDWARE =====================

AsyncWebServer httpServer(80);
AsyncWebSocket ws("/ws");
LogStore logStore;
bool logStoreOk = false;
uint16_t baseBattMv = 0;

void readBaseBattery() {
  digitalWrite(VBAT_ADC_CTRL_PIN, HIGH);
  delayMicroseconds(400);
  uint32_t adcMv = analogReadMilliVolts(VBAT_ADC_PIN);
  digitalWrite(VBAT_ADC_CTRL_PIN, LOW);
  baseBattMv = (uint16_t)(adcMv * VBAT_MULTIPLIER);
}

Preferences bsNvs;
uint32_t highestNonce = 0;

// ===================== BLE GATT SERVER =====================

#define BLE_SERVICE_UUID        "4d4f4f4e-5348-4f54-4253-000000000000"
#define BLE_TELEM_CHAR_UUID     "4d4f4f4e-5348-4f54-4253-000000000001"
#define BLE_CMD_CHAR_UUID       "4d4f4f4e-5348-4f54-4253-000000000002"
#define BLE_STATUS_CHAR_UUID    "4d4f4f4e-5348-4f54-4253-000000000003"
#define BLE_LOGFETCH_CHAR_UUID  "4d4f4f4e-5348-4f54-4253-000000000004"
#define BLE_OTA_CHAR_UUID       "4d4f4f4e-5348-4f54-4253-000000000006"  // WRITE|WRITE_NR|NOTIFY

NimBLEServer* bleServer = nullptr;
NimBLEAdvertising* bleAdvert = nullptr;
NimBLECharacteristic* bleTelemChar = nullptr;
NimBLECharacteristic* bleCmdChar = nullptr;
NimBLECharacteristic* bleStatusChar = nullptr;
NimBLECharacteristic* bleLogFetchChar = nullptr;
NimBLECharacteristic* bleOtaChar = nullptr;
bool bleClientConnected = false;

// ===================== OTA STATE MACHINE =====================
#define OTA_STATUS_OK            0x00
#define OTA_STATUS_NOT_ACTIVE    0x01
#define OTA_STATUS_WRITE_FAIL    0x02
#define OTA_STATUS_OFFSET_GAP    0x03
#define OTA_STATUS_HMAC_MISMATCH 0x04
#define OTA_STATUS_OTA_END_FAIL  0x05
#define OTA_STATUS_VERIFYING     0x06
#define OTA_STATUS_REFUSED       0x07
#define OTA_PROGRESS_MARKER      0xA0
#define OTA_PROGRESS_INTERVAL    1200

enum OtaState { OTA_LOCKED, OTA_ERASING, OTA_RECEIVING, OTA_VERIFYING };

struct OtaContext {
  OtaState              state;
  esp_ota_handle_t      handle;
  const esp_partition_t* partition;
  uint32_t              bytesWritten;
  uint32_t              chunkCount;
  bool                  notifyPending;
  uint8_t               notifyBuf[8];
  uint8_t               notifyLen;
} bsOta = { OTA_LOCKED, 0, nullptr, 0, 0, false, {0}, 0 };

static void bsOtaQueueNotify(uint8_t status) {
  bsOta.notifyBuf[0] = status;
  bsOta.notifyLen = 1;
  bsOta.notifyPending = true;
}

static void otaHandleChunk(uint32_t offset, const uint8_t* data, size_t len) {
  if (bsOta.state == OTA_VERIFYING) {
    bsOtaQueueNotify(OTA_STATUS_VERIFYING); return;
  }
  if (bsOta.state != OTA_RECEIVING) {
    bsOtaQueueNotify(OTA_STATUS_NOT_ACTIVE); return;
  }
  if (len == 0) { bsOtaQueueNotify(OTA_STATUS_NOT_ACTIVE); return; }
  if (offset != bsOta.bytesWritten) {
    bsOtaQueueNotify(OTA_STATUS_OFFSET_GAP); return;
  }
  if (esp_ota_write(bsOta.handle, data, len) != ESP_OK) {
    esp_ota_abort(bsOta.handle);
    bsOta.state = OTA_LOCKED;
    bsOtaQueueNotify(OTA_STATUS_WRITE_FAIL); return;
  }
  bsOta.bytesWritten += (uint32_t)len;
  bsOta.chunkCount++;
  if (bsOta.chunkCount % OTA_PROGRESS_INTERVAL == 0) {
    bsOta.notifyBuf[0] = OTA_PROGRESS_MARKER;
    bsOta.notifyBuf[1] = (uint8_t)(bsOta.bytesWritten);
    bsOta.notifyBuf[2] = (uint8_t)(bsOta.bytesWritten >> 8);
    bsOta.notifyBuf[3] = (uint8_t)(bsOta.bytesWritten >> 16);
    bsOta.notifyBuf[4] = (uint8_t)(bsOta.bytesWritten >> 24);
    bsOta.notifyLen = 5;
    bsOta.notifyPending = true;
  }
}

static uint8_t otaHandleBegin() {
  {
    esp_ota_img_states_t imgState;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &imgState) == ESP_OK
        && imgState == ESP_OTA_IMG_PENDING_VERIFY) {
      Serial.println("OTA: begin refused — boot pending confirmation (send 0x52 first)");
      return 0x07;
    }
  }
  if (bsOta.state != OTA_LOCKED) {
    Serial.println("OTA: begin refused — already active");
    return 0x02;
  }
  const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
  if (!part) { Serial.println("OTA: no update partition"); return 0x02; }
  bsOta.state = OTA_ERASING;
  bsOta.partition = part;
  Serial.printf("OTA: erasing partition '%s'...\n", part->label);
  for (uint32_t off = 0; off < part->size; off += 4096) {
    if (esp_partition_erase_range(part, off, 4096) != ESP_OK) {
      bsOta.state = OTA_LOCKED;
      return 0x02;
    }
    vTaskDelay(1);
  }
  Serial.println("OTA: erase done");
  esp_ota_handle_t handle;
  if (esp_ota_begin(part, part->size, &handle) != ESP_OK) {
    bsOta.state = OTA_LOCKED;
    return 0x02;
  }
  bsOta.handle = handle;
  bsOta.bytesWritten = 0;
  bsOta.chunkCount = 0;
  bsOta.state = OTA_RECEIVING;
  Serial.println("OTA: session open");
  return 0x00;
}

static uint8_t otaHandleFinalize(uint32_t expectedSize, const uint8_t* firmwareHmac) {
  if (bsOta.state != OTA_RECEIVING) return 0x02;
  bsOta.state = OTA_VERIFYING;
  if (bsOta.bytesWritten != expectedSize) {
    esp_ota_abort(bsOta.handle);
    bsOta.state = OTA_LOCKED;
    return 0x03;
  }
  uint8_t computed[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  uint8_t rbuf[256]; uint32_t pos = 0, rem = expectedSize; bool ok = true;
  while (rem > 0) {
    uint32_t n = rem < sizeof(rbuf) ? rem : (uint32_t)sizeof(rbuf);
    if (esp_partition_read(bsOta.partition, pos, rbuf, n) != ESP_OK) { ok = false; break; }
    mbedtls_md_hmac_update(&ctx, rbuf, n);
    pos += n; rem -= n;
    vTaskDelay(0);
  }
  mbedtls_md_hmac_finish(&ctx, computed);
  mbedtls_md_free(&ctx);
  if (!ok) { esp_ota_abort(bsOta.handle); bsOta.state = OTA_LOCKED; return 0x02; }
  uint8_t diff = 0;
  for (int i = 0; i < 32; i++) diff |= computed[i] ^ firmwareHmac[i];
  if (diff != 0) {
    Serial.println("OTA: HMAC mismatch");
    esp_ota_abort(bsOta.handle); bsOta.state = OTA_LOCKED;
    bsOtaQueueNotify(OTA_STATUS_HMAC_MISMATCH);
    return 0x02;
  }
  if (esp_ota_end(bsOta.handle) != ESP_OK) {
    bsOta.state = OTA_LOCKED;
    bsOtaQueueNotify(OTA_STATUS_OTA_END_FAIL);
    return 0x02;
  }
  if (esp_ota_set_boot_partition(bsOta.partition) != ESP_OK) {
    bsOta.state = OTA_LOCKED; return 0x02;
  }
  Serial.println("OTA: success — rebooting");
  bsOtaQueueNotify(OTA_STATUS_OK);
  delay(500);
  esp_restart();
  return 0x00;
}

static uint8_t otaHandleConfirm() {
  if (esp_ota_mark_app_valid_cancel_rollback() != ESP_OK) return 0x02;
  Serial.println("OTA: confirmed, rollback cancelled");
  return 0x00;
}

// ===================== BLE log fetch state machine — runs in main loop
struct BleLogFetch {
  bool active;
  uint32_t startRec;
  uint32_t endRec;      // exclusive
  uint32_t currentRec;
  LogStore::SeqReader seq;
  // Flow control: hold last built chunk until notify() succeeds. If notify
  // returns false (host queue full) we retry the same bytes next loop instead
  // of advancing the cursor — prevents holes in the fetched range.
  uint8_t  pendingBuf[500];
  uint16_t pendingLen;  // 0 = no chunk pending
  bool     endPending;  // 0-byte end marker not yet sent
} bleLogFetch = {false, 0, 0, 0, {}, {}, 0, false};

// Forward declaration — defined below, used in dispatchCmdTx
static bool BlockingReadyWait();

// ===================== TRANSPORT-AGNOSTIC COMMAND TX =====================

struct CmdTxState {
  uint8_t pkt[64];
  uint8_t pktLen;
  uint8_t sends;
  uint8_t sent;
  uint16_t waitMs;       // max ms to wait for next WIN_CMD before sending out of turn
  unsigned long lastSendMs;
  unsigned long queuedMs; // millis() when command was queued (for wait expiry)
  bool active;
} cmdTx = {.pktLen=0,.sends=0,.sent=0,.waitMs=0,.lastSendMs=0,.queuedMs=0,.active=false};

bool verifyCommandHMAC(const uint8_t* pkt, size_t pktLen) {
  if (pktLen < HMAC_TRUNC_LEN + 7) return false;
  size_t dataLen = pktLen - HMAC_TRUNC_LEN;
  uint8_t fullHmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  mbedtls_md_hmac_update(&ctx, pkt, dataLen);
  mbedtls_md_hmac_finish(&ctx, fullHmac);
  mbedtls_md_free(&ctx);
  uint8_t diff = 0;
  for (int i = 0; i < HMAC_TRUNC_LEN; i++) diff |= fullHmac[i] ^ pkt[dataLen + i];
  return (diff == 0);
}

// Returns true on success. On failure, errorMsg is set.
bool queueCommandTx(const uint8_t* body, size_t bodyLen, String& errorMsg) {
  if (bodyLen < 5) { errorMsg = "too short"; return false; }

  uint16_t waitMs = body[0] | ((uint16_t)body[1] << 8);
  uint8_t sends = body[2];
  uint8_t pktLen = body[3];
  if (pktLen == 0 || pktLen > 60 || (size_t)(4 + pktLen) > bodyLen) {
    errorMsg = "bad len"; return false;
  }
  if (sends == 0 || sends > 10) sends = 1;

  memcpy(cmdTx.pkt, body + 4, pktLen);

  if (!verifyCommandHMAC(cmdTx.pkt, pktLen)) {
    Serial.println("CMD TX rejected: HMAC fail");
    errorMsg = "hmac fail"; return false;
  }

  if (pktLen >= 7) {
    uint32_t pktNonce = (uint32_t)cmdTx.pkt[3] | ((uint32_t)cmdTx.pkt[4] << 8) |
                        ((uint32_t)cmdTx.pkt[5] << 16) | ((uint32_t)cmdTx.pkt[6] << 24);
    if (pktNonce <= highestNonce) {
      Serial.println("CMD TX rejected: stale nonce");
      errorMsg = "stale nonce"; return false;
    }
    highestNonce = pktNonce;
    bsNvs.putUInt("nonce", highestNonce);
  }

  cmdTx.pktLen = pktLen;
  cmdTx.sends = sends;
  cmdTx.sent = 0;
  cmdTx.waitMs = waitMs;
  cmdTx.lastSendMs = 0;
  cmdTx.queuedMs = millis();
  cmdTx.active = true;

  Serial.print("CMD TX queued: "); Serial.print(pktLen);
  Serial.print("B x"); Serial.print(sends);
  Serial.print(" wait="); Serial.println(waitMs);

  errorMsg = ""; return true;
}

// ===================== PUSH TELEMETRY TO ALL TRANSPORTS =====================

struct {
  uint8_t data[256]; size_t len; int8_t snr4; uint32_t timestamp; bool valid;
} latestTelem = {.len=0,.snr4=0,.timestamp=0,.valid=false};

void pushToAllTransports(const uint8_t* wsBuf, size_t wsLen) {
  if (wifiEnabled) {
    ws.binaryAll(wsBuf, wsLen);
  }
  if (bleEnabled && bleClientConnected && bleTelemChar) {
    bleTelemChar->notify((uint8_t*)wsBuf, wsLen);
  }
}

// ===================== PACKET RECEIVED CALLBACK =====================
// Invoked from radio.cpp bsHandleRxDone() for every valid received packet.

void bsOnPacketReceived(const uint8_t* buf, size_t len, float snrF, float rssiF,
                        int32_t signedPosInSlot, uint32_t slotNum, uint8_t seqIdx,
                        uint8_t win, uint32_t timeOnAirMs, float driftEmaUs, uint32_t timeSinceSyncMs) {
  int8_t snr4 = (int8_t)(snrF * 4);
  uint32_t nowMs = millis();

  int32_t recNum = -1;
  if (logStoreOk) recNum = logStore.writeRecord(buf, (uint8_t)len, snr4, nowMs);

  if (len >= 10 && buf[0] == 0xAF) {
    memcpy(latestTelem.data, buf, len);
    latestTelem.len = len; latestTelem.snr4 = snr4;
    latestTelem.timestamp = nowMs; latestTelem.valid = true;
  }

  if (len >= 5 && buf[0] == PKT_LONGRANGE) {
    uint32_t word   = (uint32_t)buf[2] | ((uint32_t)buf[3] << 8) | ((uint32_t)buf[4] << 16);
    uint16_t latFrac = (uint16_t)(word & 0x7FF);
    uint16_t lonFrac = (uint16_t)((word >> 11) & 0x7FF);
    bool     lowBatt = (word >> 22) & 1;
    bool     gpsErr  = (latFrac >= 2000 || lonFrac >= 2000);
    Serial.print("LR packet: lat="); Serial.print(gpsErr ? -1 : (int)latFrac);
    Serial.print(" lon="); Serial.print(gpsErr ? -1 : (int)lonFrac);
    Serial.print(" lowbatt="); Serial.println(lowBatt ? "YES" : "NO");
  }

  uint8_t wsBuf[268];
  memcpy(wsBuf, &snrF, 4);
  memcpy(wsBuf + 4, &rssiF, 4);
  memcpy(wsBuf + 8, &recNum, 4);
  memcpy(wsBuf + 12, buf, len);
  pushToAllTransports(wsBuf, 12 + len);
// Consolidated RX logging: signal + record num + slot timing + airtime + drift + packet size + hex dump
  //Slot:500=5(0)@1.234ms+2ms drift:7266us->ema Sync+123s 17B [AF230391098910831...]
  Serial.printf("BS OnPackRx: Sig:%.1f/%.0f #%d Slot:%lu=%u(%u)@%.2fms+%lums drift:%.1fms ema:%.1fms tss:%.1fs %dB: [",
                snrF, rssiF, recNum, slotNum,seqIdx,win, (signedPosInSlot/1000.0f-timeOnAirMs),timeOnAirMs, bsAnchorDriftUs/1000.0f, driftEmaUs/1000.0f, timeSinceSyncMs/1000.0f, len);
  // Serial.printf("BS OnPackRx: Sig:%.1f/%.0f #%d slot:%.3fms/%lu/%u/%u toa:%lums drift:%ldus ema:%.0f tss:%lums %dB: [",
  //               snrF, rssiF, recNum, signedPosInSlot/1000.0f, slotNum, seqIdx, win, timeOnAirMs, bsAnchorDriftUs, driftEmaUs, timeSinceSyncMs, len);
  size_t hexLen = (len < 14) ? len : 14;
  for (size_t i = 0; i < hexLen; i++) {
    Serial.printf("%02X", buf[i]);
  }
  if (hexLen<len) Serial.print("...");
  Serial.println("]");
}

// ===================== SYNC PACKET BUILDER =====================
// Builds a CMD_SET_SYNC (0x41) command packet signed with HMAC.
// Returns packet length (always 17).

size_t bsBuildSyncCmdPacket(uint8_t* buf) {
  highestNonce++;
  bsNvs.putUInt("nonce", highestNonce);

  buf[0] = 0x9A;               // PKT_COMMAND
  buf[1] = FAVORITE_ROCKET_DEVICE_ID;
  buf[2] = 0x41;               // CMD_SET_SYNC
  buf[3] = (uint8_t)(highestNonce);
  buf[4] = (uint8_t)(highestNonce >> 8);
  buf[5] = (uint8_t)(highestNonce >> 16);
  buf[6] = (uint8_t)(highestNonce >> 24);

  uint8_t fullHmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  mbedtls_md_hmac_update(&ctx, buf, 7);
  mbedtls_md_hmac_finish(&ctx, fullHmac);
  mbedtls_md_free(&ctx);
  memcpy(buf + 7, fullHmac, HMAC_TRUNC_LEN);

  return 17;
}

// ===================== PING PACKET BUILDER =====================
// Builds a CMD_PING (0x40) command packet signed with HMAC.
// Returns packet length (always 17).

size_t bsBuildPingCmdPacket(uint8_t* buf) {
  highestNonce++;
  bsNvs.putUInt("nonce", highestNonce);

  buf[0] = 0x9A;               // PKT_COMMAND
  buf[1] = FAVORITE_ROCKET_DEVICE_ID;
  buf[2] = 0x40;               // CMD_PING
  buf[3] = (uint8_t)(highestNonce);
  buf[4] = (uint8_t)(highestNonce >> 8);
  buf[5] = (uint8_t)(highestNonce >> 16);
  buf[6] = (uint8_t)(highestNonce >> 24);

  uint8_t fullHmac[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, HMAC_KEY, HMAC_KEY_LEN);
  mbedtls_md_hmac_update(&ctx, buf, 7);
  mbedtls_md_hmac_finish(&ctx, fullHmac);
  mbedtls_md_free(&ctx);
  memcpy(buf + 7, fullHmac, HMAC_TRUNC_LEN);

  return 17;
}

// ===================== CMD TX DISPATCH =====================
// Called from loop() when the slot machine signals WIN_CMD is ready (bsWinCmdReady)
// or when the wait window expires (out-of-turn fallback).

static void dispatchCmdTx() {
  if (!cmdTx.active || cmdTx.sent >= cmdTx.sends) return;
  // Stop RX if running — after standby, BUSY briefly goes high while the radio
  // transitions. We must wait for it before the TX SPI commands.
  // This is the base station (not rocket) and is only called when we intend to TX,
  // so a brief spin here (~300µs max) is acceptable — it is NOT in any armed path.
  if (bsRadioState == BS_RADIO_RX_ACTIVE) {
    Serial.println("CMD TX: stopping RX for TX");
    bsRadioStandby();
    // TODO: @@@ Blocking - BUSY spin after standby, expected <300µs, bounded at 2ms
    if (!BlockingReadyWait()) return;
  }
  if (bsRadioState != BS_RADIO_STANDBY) {
    Serial.print("CMD TX: not standby (state="); Serial.print(bsRadioState); Serial.println(") — skip");
    return;
  }

  Serial.print("CMD TX "); Serial.print(cmdTx.sent + 1);
  Serial.print("/"); Serial.print(cmdTx.sends);
  Serial.print(" len="); Serial.println(cmdTx.pktLen);

  // Detect CMD_SET_SYNC by byte inspection, not by source (internal vs browser-forwarded).
  // Both paths load cmdTx with the same 17-byte packet where pkt[2] = CMD_SET_SYNC (0x41).
  // This ensures the base re-anchors on TxDone for ANY sync TX — internally queued
  // (bsHandleSyncSend) or forwarded from the UI via queueCommandTx(). DO NOT narrow this check.
  bool isSyncPkt = (cmdTx.pktLen == 17 && cmdTx.pkt[2] == 0x41);
  if (isSyncPkt) bsSyncTxInFlight = true;

  // Commands are always sent on NORMAL config (not LR). Ensure target is set before TX.
  extern RadioSlotConfig bsTargetCfg;
  extern void bsApplyCfgIfNeeded();
  bsTargetCfg = RADIO_CFG_NORMAL;
  bsApplyCfgIfNeeded();

  if (!bsRadioStartTx(cmdTx.pkt, cmdTx.pktLen)) {
    bsSyncTxInFlight = false;
    Serial.println("CMD TX start fail — will retry next WIN_CMD slot");
    // Leave cmdTx.active true — packet stays queued and retries next slot.
    return;
  }

  extern unsigned long bsLastCmdSentMs;
  bsLastCmdSentMs = millis();

  cmdTx.sent++;
  cmdTx.lastSendMs = millis();

  if (cmdTx.sent >= cmdTx.sends) {
    cmdTx.active = false;
    Serial.print("CMD TX complete: all "); Serial.print(cmdTx.sends); Serial.println(" sends dispatched, waiting TxDone");
    if (logStoreOk) logStore.writeRecord(cmdTx.pkt, cmdTx.pktLen, 0x7F, millis());

    // OTA commands (0x50/0x51/0x52) targeted at this base station — apply locally
    if (cmdTx.pkt[1] == DEVICE_ID &&
        (cmdTx.pkt[2] == CMD_OTA_BEGIN || cmdTx.pkt[2] == CMD_OTA_FINALIZE || cmdTx.pkt[2] == CMD_OTA_CONFIRM)) {
      if (cmdTx.pkt[2] == CMD_OTA_BEGIN) {
        otaHandleBegin();
      } else if (cmdTx.pkt[2] == CMD_OTA_FINALIZE && cmdTx.pktLen >= 7 + 36 + HMAC_TRUNC_LEN) {
        uint32_t fwSize = (uint32_t)cmdTx.pkt[7] | ((uint32_t)cmdTx.pkt[8] << 8)
                        | ((uint32_t)cmdTx.pkt[9] << 16) | ((uint32_t)cmdTx.pkt[10] << 24);
        const uint8_t* fwHmac = &cmdTx.pkt[11];
        otaHandleFinalize(fwSize, fwHmac);
      } else if (cmdTx.pkt[2] == CMD_OTA_CONFIRM) {
        otaHandleConfirm();
      }
      return;
    }

    // SET RELAY RADIO (0x30) targeted at us — apply locally
    if (cmdTx.pktLen >= 23 && cmdTx.pkt[1] == DEVICE_ID && cmdTx.pkt[2] == 0x30) {
      uint8_t priCh  = cmdTx.pkt[7];
      uint8_t priSf  = cmdTx.pkt[8];
      int8_t  priPwr = (int8_t)cmdTx.pkt[9];
      uint8_t bhCh_  = cmdTx.pkt[10];
      uint8_t bhSf_  = cmdTx.pkt[11];
      int8_t  bhPwr_ = (int8_t)cmdTx.pkt[12];

      float priFreq = bsChannelToFreqMHz(priCh);
      if (priFreq != 0.0f && priSf >= 5 && priSf <= 12 && priPwr >= -9 && priPwr <= 22) {
        activeChannel = priCh; activeSF = priSf; activePower = priPwr;
        bsUpdateActiveFreqBw();
        bsNvs.putUChar("radio_ch", activeChannel);
        bsNvs.putUChar("radio_sf", activeSF);
        bsNvs.putChar("radio_pwr", activePower);
      }
      float bhFreq_ = bsChannelToFreqMHz(bhCh_);
      if (bhFreq_ != 0.0f && bhSf_ >= 5 && bhSf_ <= 12 && bhPwr_ >= -9 && bhPwr_ <= 22) {
        bhChannel = bhCh_; bhSF = bhSf_; bhPower = bhPwr_;
        bsNvs.putUChar("bh_ch", bhChannel);
        bsNvs.putUChar("bh_sf", bhSF);
        bsNvs.putChar("bh_pwr", bhPower);
      }
      bsRadioStandby();
      bsRadioApplyConfig_BLOCKING();
      bsRadioStartRx();
      return;
    }
  }
}

static bool BlockingReadyWait()
{
  // TODO: @@@ Blocking - spins on BUSY up to 2ms. Base station only, not in armed path.
  unsigned long t0 = micros();
  while (digitalRead(LORA_BUSY_PIN)) {
    if (micros() - t0 > 2000) {
      Serial.println("BS: BUSY stuck after standby (>2ms)");
      return false;
    }
  }
  return true;
}

// ===================== BUILD STATUS JSON =====================

size_t buildStatusJson(char* json, size_t maxLen) {
  readBaseBattery();
  return snprintf(json, maxLen,
    "{\"uptimeMs\":%lu,\"records\":%lu,\"oldest\":%lu,\"ringSize\":%lu,\"vpos\":%lu,"
    "\"logOk\":%s,\"nonce\":%lu,\"deviceId\":%d,\"baseBattMv\":%u,"
    "\"wifiOn\":%s,\"bleOn\":%s}",
    (unsigned long)millis(),
    (unsigned long)logStore.getRecordCounter(),
    (unsigned long)logStore.getOldestRecord(),
    (unsigned long)logStore.getRingSize(),
    (unsigned long)logStore.getVirtualPos(),
    logStoreOk ? "true" : "false",
    (unsigned long)highestNonce,
    DEVICE_ID,
    (unsigned)baseBattMv,
    wifiEnabled ? "true" : "false",
    bleEnabled ? "true" : "false");
}

// ===================== HTTP HANDLERS =====================

void handleApiCommand(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0) return;
  if (len < total) { req->send(400, "text/plain", "fragmented"); return; }
  String errorMsg;
  if (queueCommandTx(data, len, errorMsg)) {
    req->send(200, "text/plain", "ok");
  } else {
    int code = (errorMsg == "hmac fail" || errorMsg == "stale nonce") ? 403 : 400;
    req->send(code, "text/plain", errorMsg.c_str());
  }
}

void handleApiOtaChunk(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0) return;
  if (len < total) { req->send(400, "text/plain", "fragmented"); return; }
  if (len < 5) { req->send(400, "text/plain", "too short"); return; }
  uint32_t offset = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  bsOta.notifyPending = false;
  otaHandleChunk(offset, data + 4, len - 4);
  uint8_t status = bsOta.notifyPending ? bsOta.notifyBuf[0] : OTA_STATUS_OK;
  bsOta.notifyPending = false;
  req->send(200, "application/octet-stream", String((char)status));
}

void handleApiStatus(AsyncWebServerRequest *req) {
  char json[512];
  buildStatusJson(json, sizeof(json));
  req->send(200, "application/json", json);
}

void handleApiLogs(AsyncWebServerRequest *req) {
  if (!logStoreOk) { req->send(503, "text/plain", "no log"); return; }
  uint32_t startRec = 0, count = 100;
  if (req->hasParam("start")) startRec = req->getParam("start")->value().toInt();
  if (req->hasParam("count")) { count = req->getParam("count")->value().toInt(); if (count > 200) count = 200; }

  uint32_t oldest = logStore.getOldestRecord();
  uint32_t newest = logStore.getRecordCounter();
  if (startRec < oldest) startRec = oldest;
  if (startRec >= newest || newest == 0) { req->send(200, "application/octet-stream", ""); return; }
  if (startRec + count > newest) count = newest - startRec;

  size_t maxBuf = count * (10 + LOG_MAX_PAYLOAD);
  if (maxBuf > 60000) maxBuf = 60000;
  uint8_t* outBuf = (uint8_t*)malloc(maxBuf);
  if (!outBuf) { req->send(503, "text/plain", "OOM"); return; }

  size_t outPos = 0; uint8_t recBuf[LOG_MAX_PAYLOAD]; int8_t snr; uint32_t ts;
  LogStore::SeqReader seq = logStore.seqReader(startRec, startRec + count);
  while (seq.hasMore() && outPos + 10 + LOG_MAX_PAYLOAD <= maxBuf) {
    uint32_t rn = seq.currentRec();
    int pLen = seq.readNext(recBuf, sizeof(recBuf), &snr, &ts);
    if (pLen < 0) break;
    memcpy(outBuf+outPos, &rn, 4); outPos += 4;
    outBuf[outPos++] = (uint8_t)pLen;
    outBuf[outPos++] = (uint8_t)snr;
    memcpy(outBuf+outPos, &ts, 4); outPos += 4;
    memcpy(outBuf+outPos, recBuf, pLen); outPos += pLen;
  }
  AsyncWebServerResponse *resp = req->beginResponse_P(200, "application/octet-stream", outBuf, outPos);
  resp->addHeader("Cache-Control", "no-cache");
  req->send(resp);
  free(outBuf);
}

void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *a, uint8_t *d, size_t l) {
  if (t == WS_EVT_CONNECT) { Serial.print("WS+ #"); Serial.println(c->id()); }
  else if (t == WS_EVT_DISCONNECT) { Serial.print("WS- #"); Serial.println(c->id()); }
  else { Serial.print("WS evt="); Serial.println((int)t); }
}

// ===================== BLE CALLBACKS =====================

class BleServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    bleClientConnected = true;
    Serial.print("BLE+ addr:"); Serial.println(connInfo.getAddress().toString().c_str());
    //TODO: @@@ force 2m phy
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    bleClientConnected = false;
    bleLogFetch.active = false;
    Serial.print("BLE- reason:"); Serial.println(reason);
    NimBLEDevice::startAdvertising();
  }
};

class BleCmdCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    if (val.size() < 5) {
      uint8_t err = 0x01;
      chr->setValue(&err, 1);
      return;
    }
    String errorMsg;
    bool ok = queueCommandTx(val.data(), val.size(), errorMsg);
    uint8_t result = ok ? 0x00 : 0x02;
    chr->setValue(&result, 1);
    Serial.print("BLE CMD "); Serial.println(ok ? "OK" : errorMsg.c_str());
  }
};

class BleStatusCallbacks : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    char json[512];
    buildStatusJson(json, sizeof(json));
    chr->setValue((uint8_t*)json, strlen(json));
  }
};

class BleLogFetchCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    if (val.size() < 6 || !logStoreOk) return;

    uint32_t startRec = val.data()[0] | ((uint32_t)val.data()[1] << 8) |
                        ((uint32_t)val.data()[2] << 16) | ((uint32_t)val.data()[3] << 24);
    uint16_t count = val.data()[4] | ((uint16_t)val.data()[5] << 8);
    if (count > 2000) count = 2000;

    uint32_t oldest = logStore.getOldestRecord();
    uint32_t newest = logStore.getRecordCounter();
    if (startRec < oldest) startRec = oldest;
    if (startRec >= newest) {
      chr->notify((uint8_t*)"", 0, true);
      return;
    }
    uint32_t endRec = startRec + count;
    if (endRec > newest) endRec = newest;

    bleLogFetch.active = true;
    bleLogFetch.startRec = startRec;
    bleLogFetch.endRec = endRec;
    bleLogFetch.currentRec = startRec;
    bleLogFetch.seq = logStore.seqReader(startRec, endRec);
    bleLogFetch.pendingLen = 0;
    bleLogFetch.endPending = false;

    Serial.print("BLE fetch: "); Serial.print(startRec);
    Serial.print("-"); Serial.println(endRec);
  }
};

class BleOtaCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    if (val.size() < 5) return;
    uint32_t offset = (uint32_t)val.data()[0] | ((uint32_t)val.data()[1] << 8)
                    | ((uint32_t)val.data()[2] << 16) | ((uint32_t)val.data()[3] << 24);
    otaHandleChunk(offset, val.data() + 4, val.size() - 4);
  }
};

// ===================== BLE LOG FETCH STATE MACHINE =====================

void handleBleLogFetch() {
  if (!bleLogFetch.active || !bleClientConnected || !bleLogFetchChar) return;

  // Retry held chunk if previous notify() was dropped by congestion.
  if (bleLogFetch.pendingLen > 0) {
    if (!bleLogFetchChar->notify(bleLogFetch.pendingBuf, bleLogFetch.pendingLen, true)) return;
    bleLogFetch.pendingLen = 0;
  }

  // Retry the end-of-fetch marker if previously dropped.
  if (bleLogFetch.endPending) {
    if (!bleLogFetchChar->notify((uint8_t*)"", 0, true)) return;
    bleLogFetch.endPending = false;
    bleLogFetch.active = false;
    Serial.print("BLE fetch done @"); Serial.println(bleLogFetch.currentRec);
    return;
  }

  if (!bleLogFetch.seq.hasMore()) {
    if (!bleLogFetchChar->notify((uint8_t*)"", 0, true)) {
      bleLogFetch.endPending = true;
      return;
    }
    bleLogFetch.active = false;
    Serial.print("BLE fetch done @"); Serial.println(bleLogFetch.currentRec);
    return;
  }

  // Build a chunk directly into pendingBuf so we can retransmit on congestion.
  uint16_t chunkPos = 0;
  uint8_t recBuf[LOG_MAX_PAYLOAD];
  int8_t snr;
  uint32_t ts;

  while (bleLogFetch.seq.hasMore() && chunkPos + 10 + LOG_MAX_PAYLOAD <= sizeof(bleLogFetch.pendingBuf)) {
    uint32_t rn = bleLogFetch.seq.currentRec();
    int pLen = bleLogFetch.seq.readNext(recBuf, sizeof(recBuf), &snr, &ts);
    if (pLen < 0) break;

    memcpy(bleLogFetch.pendingBuf + chunkPos, &rn, 4); chunkPos += 4;
    bleLogFetch.pendingBuf[chunkPos++] = (uint8_t)pLen;
    bleLogFetch.pendingBuf[chunkPos++] = (uint8_t)snr;
    memcpy(bleLogFetch.pendingBuf + chunkPos, &ts, 4); chunkPos += 4;
    memcpy(bleLogFetch.pendingBuf + chunkPos, recBuf, pLen); chunkPos += pLen;
    bleLogFetch.currentRec = bleLogFetch.seq.currentRec();
  }

  if (chunkPos > 0) {
    if (!bleLogFetchChar->notify(bleLogFetch.pendingBuf, chunkPos, true)) {
      bleLogFetch.pendingLen = chunkPos;  // hold for retry
    }
  }
}

// ===================== BLE INIT / DEINIT =====================

void initBLE() {
  NimBLEDevice::init(WIFI_SSID);
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);  // +3 dBm — pocket range is plenty
  NimBLEDevice::setMTU(517);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new BleServerCallbacks());

  NimBLEService* svc = bleServer->createService(BLE_SERVICE_UUID);

  bleTelemChar = svc->createCharacteristic(BLE_TELEM_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  bleCmdChar = svc->createCharacteristic(BLE_CMD_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  bleCmdChar->setCallbacks(new BleCmdCallbacks());
  bleStatusChar = svc->createCharacteristic(BLE_STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ);
  bleStatusChar->setCallbacks(new BleStatusCallbacks());
  bleLogFetchChar = svc->createCharacteristic(BLE_LOGFETCH_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  bleLogFetchChar->setCallbacks(new BleLogFetchCallbacks());

  bleOtaChar = svc->createCharacteristic(BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
  bleOtaChar->setCallbacks(new BleOtaCallbacks());

  svc->start();

  bleAdvert = NimBLEDevice::getAdvertising();
  bleAdvert->addServiceUUID(BLE_SERVICE_UUID);
  bleAdvert->setName("NimBLE");
  bleAdvert->enableScanResponse(true);
  bleAdvert->setMinInterval(0x20);
  bleAdvert->setMaxInterval(0x40);
  bleAdvert->start();

  Serial.println("BLE GATT started");
}

void deinitBLE() {
  if (!bleServer) return;
  NimBLEDevice::deinit(true);
  bleServer = nullptr;
  bleTelemChar = nullptr;
  bleCmdChar = nullptr;
  bleStatusChar = nullptr;
  bleLogFetchChar = nullptr;
  bleOtaChar = nullptr;
  bleClientConnected = false;
  bleLogFetch.active = false;
  Serial.println("BLE deinited");
}

// ===================== SETUP =====================

void setup() {
  bootMicros = micros();  // Capture boot time for tagged serial timestamps
  //taggedSerial.begin(SERIAL_BAUD);
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n=== Rocket Base Station ===");

  initBLE();

  ledcAttach(LED_PIN, 1000, 11);
  ledcWrite(LED_PIN, 50);

  pinMode(VEXT_CTRL_PIN, OUTPUT); digitalWrite(VEXT_CTRL_PIN, HIGH);
  pinMode(VBAT_ADC_CTRL_PIN, OUTPUT); digitalWrite(VBAT_ADC_CTRL_PIN, LOW);
  analogSetAttenuation(ADC_11db);
  readBaseBattery();

  if (!LittleFS.begin(false, "/littlefs", 10, "fs")) Serial.println("LittleFS mount failed — run LittleFS Data Upload");
  else Serial.println("LittleFS mounted ok");

  logStoreOk = logStore.begin("log_data", "log_index", "bs_log");
  Serial.print("logStore: "); Serial.println(logStoreOk ? "OK" : "FAIL");
  if (logStoreOk) {
    Serial.print("  rec="); Serial.print(logStore.getRecordCounter());
    Serial.print(" old="); Serial.print(logStore.getOldestRecord());
    Serial.print(" vpos=0x"); Serial.println(logStore.getVirtualPos(), HEX);
  }

  bsNvs.begin("basestation", false);
  highestNonce = bsNvs.getUInt("nonce", 0);

  // Load radio settings from NVS
  if (bsNvs.isKey("radio_ch")) { uint8_t c = bsNvs.getUChar("radio_ch", DEFAULT_CHANNEL); activeChannel = (bsChannelToFreqMHz(c) != 0.0f) ? c : DEFAULT_CHANNEL; }
  if (bsNvs.isKey("radio_sf")) { uint8_t s = bsNvs.getUChar("radio_sf", DEFAULT_SF); activeSF = (s >= 5 && s <= 12) ? s : DEFAULT_SF; }
  if (bsNvs.isKey("radio_pwr")) { int8_t p = bsNvs.getChar("radio_pwr", DEFAULT_POWER); activePower = (p >= -9 && p <= 22) ? p : DEFAULT_POWER; }
  bsUpdateActiveFreqBw();

  if (bsNvs.isKey("bh_ch")) { uint8_t c = bsNvs.getUChar("bh_ch", DEFAULT_BH_CHANNEL); bhChannel = (bsChannelToFreqMHz(c) != 0.0f) ? c : DEFAULT_BH_CHANNEL; }
  if (bsNvs.isKey("bh_sf")) { uint8_t s = bsNvs.getUChar("bh_sf", DEFAULT_BH_SF); bhSF = (s >= 5 && s <= 12) ? s : DEFAULT_BH_SF; }
  if (bsNvs.isKey("bh_pwr")) { int8_t p = bsNvs.getChar("bh_pwr", DEFAULT_BH_POWER); bhPower = (p >= -9 && p <= 22) ? p : DEFAULT_BH_POWER; }

  Serial.print("Radio pri: ch"); Serial.print(activeChannel);
  Serial.print(" "); Serial.print(activeFreqMHz, 1);
  Serial.print("MHz SF"); Serial.print(activeSF);
  Serial.print(" BW"); Serial.print((int)activeBwKHz);
  Serial.print(" pwr="); Serial.println(activePower);
  Serial.print("Device ID: "); Serial.println(DEVICE_ID);

  // WiFi AP
  Serial.println("WiFi: setting mode...");
  WiFi.mode(WIFI_AP);
  Serial.println("WiFi: starting softAP...");
  bool apOk = WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
  Serial.print("WiFi: softAP result="); Serial.println(apOk);
  Serial.print("AP: "); Serial.println(WiFi.softAPIP());

  httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
    Serial.println("HTTP GET /");
    AsyncWebServerResponse *resp = r->beginResponse(LittleFS, "/index.html.gz", "text/html");
    if (!resp) { r->send(503, "text/plain", "LittleFS not mounted"); return; }
    resp->addHeader("Content-Encoding", "gzip");
    r->send(resp);
  });
  httpServer.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncWebServerResponse *resp = r->beginResponse(LittleFS, "/style.css.gz", "text/css");
    if (!resp) { r->send(503, "text/plain", "LittleFS not mounted"); return; }
    resp->addHeader("Content-Encoding", "gzip");
    r->send(resp);
  });
  httpServer.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *r){
    AsyncWebServerResponse *resp = r->beginResponse(LittleFS, "/app.js.gz", "application/javascript");
    if (!resp) { r->send(503, "text/plain", "LittleFS not mounted"); return; }
    resp->addHeader("Content-Encoding", "gzip");
    r->send(resp);
  });
  httpServer.on("/api/status", HTTP_GET, handleApiStatus);
  httpServer.on("/api/logs", HTTP_GET, handleApiLogs);
  httpServer.on("/api/command", HTTP_POST,
    [](AsyncWebServerRequest *req){ }, NULL, handleApiCommand);
  httpServer.on("/api/ota/chunk", HTTP_PUT,
    [](AsyncWebServerRequest *req){ }, NULL, handleApiOtaChunk);
  ws.onEvent(onWsEvent);
  httpServer.addHandler(&ws);
  httpServer.begin();

  // LoRa
  bsLoraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  Serial.print("LoRa init... ");
  if (bsRadioInit()) {
    Serial.println("ok");
    bsLoraReady = true;
    bsRadioStartRx();  // start listening immediately after init// TODO: we dont need to init on boot - the first window will do that for us - why is this here? casn we delete it? will this interfere with the first loop because the radio is already running?
  } else {
    Serial.println("FAIL");
  }

  Serial.println("=== Ready ===");

  NimBLEDevice::getAdvertising()->setAdvertisingInterval(1600);
  esp_wifi_set_max_tx_power(20);
  esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
  setCpuFrequencyMhz(160);
}

// ===================== MAIN LOOP =====================

void loop() {
  if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::getAdvertising()->start(0);
  }

  // ---- Radio state machine ----
  if (bsLoraReady) {
    bsHandleRadio();

    // Auto-sync: bsHandleSyncSend sets bsSyncNeedsQueue while never-synced (tight walk-retries
    // then slow backoff). After TxDone, bsSynced=true and no further automatic sync is queued.
    // Ping is queued separately on a 60s cadence.
    bsHandleSyncSend();

    // Build and load sync packet when flagged. Only load if no TX already queued.
    // Internally generated sync always fires immediately (waitMs=0) because by definition
    // the rocket's slot timing isn't knowable yet — we want the send to land as soon as the
    // radio is standby, and subsequent walk-retries will land at different phases of the
    // rocket's slot cycle. User-initiated sync via queueCommandTx() uses caller-supplied waitMs.
    if (bsSyncNeedsQueue && !cmdTx.active) {
      bsSyncNeedsQueue = false;
      uint8_t syncPkt[17];
      size_t syncLen = bsBuildSyncCmdPacket(syncPkt);
      memcpy(cmdTx.pkt, syncPkt, syncLen);
      cmdTx.pktLen   = (uint8_t)syncLen;
      cmdTx.sends    = 1;
      cmdTx.sent     = 0;
      cmdTx.waitMs   = 0;
      cmdTx.queuedMs = millis();
      cmdTx.active   = true;
      Serial.print("SYNC loaded nonce="); Serial.println(highestNonce);
    }

    // Build and load ping packet when flagged. Only load if no TX already queued.
    // Ping waits up to 4s for the next WIN_CMD slot so it lands on the rocket's listen window.
    // If the wait expires (shouldn't normally happen — WIN_CMD comes every ~840ms), send out-of-slot.
    if (bsPingNeedsQueue && !cmdTx.active) {
      bsPingNeedsQueue = false;
      uint8_t pingPkt[17];
      size_t pingLen = bsBuildPingCmdPacket(pingPkt);
      memcpy(cmdTx.pkt, pingPkt, pingLen);
      cmdTx.pktLen   = (uint8_t)pingLen;
      cmdTx.sends    = 1;
      cmdTx.sent     = 0;
      cmdTx.waitMs   = 4000;
      cmdTx.queuedMs = millis();
      cmdTx.active   = true;
      Serial.print("PING loaded nonce="); Serial.println(highestNonce);
    }

    // Dispatch: slot machine signals WIN_CMD (synced path)
    if (bsWinCmdReady) {
      bsWinCmdReady = false;
      dispatchCmdTx();
    }

    // Dispatch: send immediately when not synced (waitMs=0) or wait expired.
    // Never interrupt a TX in progress — if the radio is already transmitting,
    // wait for TxDone IRQ (handled in bsHandleRadio) before sending next.
    if (cmdTx.active && cmdTx.sent < cmdTx.sends && bsRadioState != BS_RADIO_TX_ACTIVE) {
      bool waitExpired = (cmdTx.waitMs == 0) || ((millis() - cmdTx.queuedMs) >= cmdTx.waitMs);
      if (waitExpired) {
        if (cmdTx.waitMs > 0) Serial.println("CMD TX: wait expired, sending out-of-slot");
        dispatchCmdTx();
      }
    }
  }

  handleBleLogFetch();

  // OTA notify drain
  if (bsOta.notifyPending && bleOtaChar && bleClientConnected) {
    bleOtaChar->notify(bsOta.notifyBuf, bsOta.notifyLen);
    bsOta.notifyPending = false;
  }

  if (wifiEnabled) ws.cleanupClients();

}


