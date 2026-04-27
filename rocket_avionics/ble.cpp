// ble.cpp — BLE (NimBLE) transport implementation for rocket avionics.
// See ble.h for the public API and wire format documentation.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "ble.h"
#include "config.h"
#include "telemetry.h"
#include "commands.h"
#include "globals.h"
#include "flight.h"
#include "ota.h"

// ===================== GLOBALS =====================

BleState bleState;

volatile uint32_t bleCallbackTotalUs = 0;
volatile uint32_t bleCallbackPeakUs  = 0;

static NimBLEServer*         pServer      = nullptr;
static NimBLECharacteristic* pTelemChar   = nullptr;
static NimBLECharacteristic* pCmdChar     = nullptr;
static NimBLECharacteristic* pStatusChar  = nullptr;
static NimBLECharacteristic* pConnSetChar = nullptr;
static NimBLECharacteristic* pFetchChar   = nullptr;
static NimBLECharacteristic* pOtaChar     = nullptr;
static NimBLEAdvertising*    pAdvert      = nullptr;

// ===================== CALLBACK TIMING HELPER =====================
// Use in every callback: record start time, do work, accumulate on exit.

#define BLE_CB_START()  unsigned long _cbT0 = micros()
#define BLE_CB_END()    do { \
  unsigned long _cbEl = micros() - _cbT0; \
  bleCallbackTotalUs += _cbEl; \
  if (_cbEl > bleCallbackPeakUs) bleCallbackPeakUs = _cbEl; \
} while(0)

// ===================== PHY / CONNECTION PARAM HELPERS =====================

static void applyPhyParams(uint16_t connHandle, uint8_t phy) {
  // phy: 0=1M, 1=2M, 2=Coded-S2, 3=Coded-S8
  switch (phy) {
    case 1:  // 2M
      pServer->updatePhy(connHandle,
        BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
      pServer->updateConnParams(connHandle,
        BLE_CI_FAST_MIN, BLE_CI_FAST_MAX, BLE_CI_FAST_LATENCY, BLE_CI_FAST_TIMEOUT);
      bleState.currentTxPhy = 2;
      bleTxPhy = 2;
      break;
    case 2:  // Coded S=2
      pServer->updatePhy(connHandle,
        BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_S2);
      pServer->updateConnParams(connHandle,
        BLE_CI_SLOW_MIN, BLE_CI_SLOW_MAX, BLE_CI_SLOW_LATENCY, BLE_CI_SLOW_TIMEOUT);
      bleState.currentTxPhy = 3;
      bleTxPhy = 3;
      break;
    case 3:  // Coded S=8
      pServer->updatePhy(connHandle,
        BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_S8);
      pServer->updateConnParams(connHandle,
        BLE_CI_SLOW_MIN, BLE_CI_SLOW_MAX, BLE_CI_SLOW_LATENCY, BLE_CI_SLOW_TIMEOUT);
      bleState.currentTxPhy = 3;
      bleTxPhy = 3;
      break;
    default:  // 0 = 1M
      pServer->updatePhy(connHandle,
        BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_1M_MASK, 0);
      pServer->updateConnParams(connHandle,
        BLE_CI_FAST_MIN, BLE_CI_FAST_MAX, BLE_CI_FAST_LATENCY, BLE_CI_FAST_TIMEOUT);
      bleState.currentTxPhy = 1;
      bleTxPhy = 1;
      break;
  }
}

// ===================== SERVER CALLBACKS =====================

class BleServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    BLE_CB_START();

    // Single connection: any new connection clears previous state
    bleState.connected       = true;
    bleState.telemSubscribed = false;
    bleState.fetchActive     = false;
    bleState.ovfLen          = 0;
    bleState.ovfSent         = 0;
    bleState.fetchCurrentRec = 0;
    bleState.fetchEndRec     = 0;
    bleState.nextCaptureUs   = micros();
    bleState.currentTxPhy    = 1;
    bleState.subPageMask     = BLE_DEFAULT_PAGE_MASK;
    bleSubPageMask           = BLE_DEFAULT_PAGE_MASK;  // mirror into globals

    // Mark all pages fresh for the new connection
    for (int i = 0; i < LOGI_COUNT; i++) {
      logPages[i].freshMask |= FRESH_BLE;
    }

    // Default: 2M PHY + fast connection interval
    applyPhyParams(info.getConnHandle(), 1);

    Serial.printf("BLE: connected. handle=%d interval=%.2fms\n",
      info.getConnHandle(), info.getConnInterval() * 1.25f);

    BLE_CB_END();
  }

  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    BLE_CB_START();

    bleState.connected       = false;
    bleState.telemSubscribed = false;
    bleState.fetchActive     = false;
    bleState.ovfLen          = 0;
    bleState.ovfSent         = 0;

    Serial.printf("BLE: disconnected. reason=%d\n", reason);

    BLE_CB_END();
  }

  void onConnParamsUpdate(NimBLEConnInfo& info) override {
    BLE_CB_START();
    Serial.printf("BLE: conn params updated. interval=%.2fms latency=%d\n",
      info.getConnInterval() * 1.25f, info.getConnLatency());
    BLE_CB_END();
  }

  void onPhyUpdate(NimBLEConnInfo& info, uint8_t txPhy, uint8_t rxPhy) override {
    BLE_CB_START();
    const char* n[] = { "?", "1M", "2M", "Coded" };
    Serial.printf("BLE: PHY tx:%s rx:%s\n",
      txPhy < 4 ? n[txPhy] : "?",
      rxPhy < 4 ? n[rxPhy] : "?");
    if (txPhy <= 3) {
      bleState.currentTxPhy = txPhy;
      bleTxPhy = txPhy;  // mirror into globals for telemetry.cpp use
    }
    BLE_CB_END();
  }
};

// ===================== COMMAND CHAR CALLBACKS =====================

class BleCmdCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    BLE_CB_START();

    // Format: [waitMs u16][sends u8][pktLen u8][packet bytes...]
    // waitMs and sends are ignored for BLE (reliable transport).
    // Inner packet is passed directly to processReceivedPacket().
    NimBLEAttValue val = c->getValue();
    const uint8_t* data = val.data();
    size_t len = val.size();

    if (len < 5) {
      Serial.println("BLE CMD: too short");
      BLE_CB_END();
      return;
    }

    uint8_t pktLen = data[3];
    if ((size_t)(pktLen + 4) > len) {
      Serial.println("BLE CMD: pktLen overflow");
      BLE_CB_END();
      return;
    }

    const uint8_t* pkt = data + 4;
    // Use LOG_SNR_LOCAL (0x7F) as sentinel for BLE-sourced commands
    processReceivedPacket(pkt, pktLen, LOG_SNR_LOCAL, LOG_SNR_LOCAL);

    BLE_CB_END();
  }
};

// ===================== STATUS CHAR CALLBACKS =====================

class BleStatusCB : public NimBLECharacteristicCallbacks {
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    BLE_CB_START();

    char buf[128];
    uint32_t logIdx = logStoreOk ? logStore.getRecordCounter() : 0;
    snprintf(buf, sizeof(buf),
      "{\"logIdx\":%lu,\"batt\":%u,\"armed\":%d,\"uptime\":%lu,\"nonce\":%lu}",
      (unsigned long)logIdx,
      (unsigned)batteryMv,
      isArmed ? 1 : 0,
      (unsigned long)(millis() / 1000UL),
      (unsigned long)highestNonce);

    c->setValue((uint8_t*)buf, strlen(buf));

    BLE_CB_END();
  }
};

// ===================== CONNSET CHAR CALLBACKS =====================

class BleConnSetCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    BLE_CB_START();

    NimBLEAttValue val = c->getValue();
    const uint8_t* data = val.data();
    size_t len = val.size();

    if (len < 2) { BLE_CB_END(); return; }

    uint8_t type = data[0];
    switch (type) {
      case BLE_CONNSET_INTERVAL:  // [0x01][intervalUs u32]
        if (len >= 5) {
          uint32_t iv = (uint32_t)data[1] | ((uint32_t)data[2] << 8) |
                        ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 24);
          bleState.subIntervalUs = iv;
          Serial.printf("BLE: interval set to %lu us\n", (unsigned long)iv);
        }
        break;

      case BLE_CONNSET_PAGEMASK:  // [0x02][mask u64]
        if (len >= 9) {
          uint64_t mask = 0;
          for (int i = 0; i < 8; i++) mask |= ((uint64_t)data[1 + i] << (i * 8));
          bleState.subPageMask = mask;
          bleSubPageMask = mask;  // mirror into globals for telemetry.cpp / flight.cpp
          // Recalculate thrust ring active state based on new subscription
          thrustBufActive = (isArmed || ((bleSubPageMask & (1ULL << PAGE_THRUST_CURVE)) != 0))
                            && !thrustLoraForce;
          Serial.printf("BLE: page mask set to 0x%08lX%08lX, thrust buf %s\n",
            (unsigned long)(mask >> 32), (unsigned long)(mask & 0xFFFFFFFF),
            thrustBufActive ? "active" : "inactive");
        }
        break;

      case BLE_CONNSET_PHY:  // [0x03][phy u8]  0=1M 1=2M 2=Coded-S2 3=Coded-S8
        if (len >= 2) {
          Serial.printf("BLE: PHY change requested to %d\n", data[1]);
          applyPhyParams(info.getConnHandle(), data[1]);
        }
        break;

      default:
        Serial.printf("BLE ConnSet: unknown type 0x%02X\n", type);
        break;
    }

    BLE_CB_END();
  }
};

// ===================== LOG FETCH CHAR CALLBACKS =====================

class BleFetchCB : public NimBLECharacteristicCallbacks {
  // Write: [startRec u32][count u16]
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    BLE_CB_START();

    NimBLEAttValue val = c->getValue();
    const uint8_t* data = val.data();
    size_t len = val.size();
    Serial.printf("[BLEFetch] onWrite size=%u logStoreOk=%d active=%d\n",
      (unsigned)len, logStoreOk?1:0, bleState.fetchActive?1:0);

    if (len < 6) { Serial.println("[BLEFetch] onWrite ignored (size<6)"); BLE_CB_END(); return; }

    uint32_t startRec = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    uint16_t count    = (uint16_t)data[4] | ((uint16_t)data[5] << 8);

    uint32_t totalRecs = logStoreOk ? logStore.getRecordCounter() : 0;
    uint32_t endRec    = startRec + count;
    if (endRec > totalRecs) endRec = totalRecs;
    Serial.printf("[BLEFetch] req start=%lu count=%u | totalRecs=%lu endRec=%lu\n",
      (unsigned long)startRec, (unsigned)count, (unsigned long)totalRecs, (unsigned long)endRec);

    // Disable in-flight fetch so loopTask can't observe a half-built state
    // while we re-init the seq below. Set active LAST.
    bleState.fetchActive     = false;
    bleState.fetchCurrentRec = startRec;
    bleState.fetchEndRec     = endRec;
    bleState.fetchPendingLen = 0;
    bleState.fetchEndPending = false;
    rktFetchNotifyOk = 0;
    rktFetchNotifyDrop = 0;
    rktFetchBytesSent = 0;
    if (logStoreOk) {
      bleState.fetchSeq = logStore.seqReader(startRec, endRec);
    }
    Serial.printf("[BLEFetch] BLE fetch: %lu-%lu — going active\n",
      (unsigned long)startRec, (unsigned long)endRec);
    // Set active LAST so loopTask sees a fully-initialized state.
    bleState.fetchActive = true;

    BLE_CB_END();
  }

  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& info, uint16_t subValue) override {
    BLE_CB_START();
    // subValue 1 = notifications enabled
    BLE_CB_END();
  }
};

// ===================== OTA CHAR CALLBACK =====================
// Receives raw firmware chunks: [offset u32 LE][data: up to 512 bytes].
// Uses WRITE_NR — no ATT response. Relies on LL-level ACK for flow control.
// Notifies errors and periodic progress via the OTA characteristic (drained by main loop).

class BleOtaCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    BLE_CB_START();
    NimBLEAttValue val = c->getValue();
    const uint8_t* data = val.data();
    size_t len = val.size();
    if (len < 5) { BLE_CB_END(); return; }  // need at least offset(4) + 1 byte

    uint32_t offset = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                    | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    otaHandleChunk(offset, data + 4, len - 4);
    BLE_CB_END();
  }
};

// ===================== OTA NOTIFY HELPERS =====================
// Called from the NimBLE FreeRTOS callback task or from ota.cpp.
// The main loop drains these to the OTA characteristic.

void otaQueueNotify(uint8_t status) {
  bleState.otaNotifyBuf[0] = status;
  bleState.otaNotifyLen     = 1;
  bleState.otaNotifyPending = true;
}

void otaQueueNotifyBytes(const uint8_t* data, uint8_t len) {
  if (len > 8) len = 8;
  memcpy((void*)bleState.otaNotifyBuf, data, len);
  bleState.otaNotifyLen     = len;
  bleState.otaNotifyPending = true;
}

// ===================== TELEM CHAR SUBSCRIPTION TRACKING =====================

class BleTelemCB : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& info, uint16_t subValue) override {
    BLE_CB_START();
    bleState.telemSubscribed = (subValue == 1);
    Serial.printf("BLE telem: %s\n", bleState.telemSubscribed ? "subscribed" : "unsubscribed");
    BLE_CB_END();
  }
};

// ===================== INIT =====================

void initBLE() {
  memset(&bleState, 0, sizeof(BleState));
  bleState.subIntervalUs = BLE_DEFAULT_INTERVAL_US;
  bleState.subPageMask   = BLE_DEFAULT_PAGE_MASK;
  bleState.nextCaptureUs = micros();
  bleState.currentTxPhy  = 1;

  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // +9 dBm — max BLE TX power
  NimBLEDevice::setMTU(517);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new BleServerCB());

  NimBLEService* svc = pServer->createService(BLE_SERVICE_UUID);

  pTelemChar = svc->createCharacteristic(
    BLE_TELEM_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  pTelemChar->setCallbacks(new BleTelemCB());

  pCmdChar = svc->createCharacteristic(
    BLE_CMD_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pCmdChar->setCallbacks(new BleCmdCB());

  pStatusChar = svc->createCharacteristic(
    BLE_STATUS_CHAR_UUID, NIMBLE_PROPERTY::READ);
  pStatusChar->setCallbacks(new BleStatusCB());

  pConnSetChar = svc->createCharacteristic(
    BLE_CONNSET_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pConnSetChar->setCallbacks(new BleConnSetCB());

  pFetchChar = svc->createCharacteristic(
    BLE_LOGFETCH_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
  pFetchChar->setCallbacks(new BleFetchCB());

  pOtaChar = svc->createCharacteristic(
    BLE_OTA_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY);
  pOtaChar->setCallbacks(new BleOtaCB());

  svc->start();

  pAdvert = NimBLEDevice::getAdvertising();
  pAdvert->addServiceUUID(BLE_SERVICE_UUID);
  pAdvert->setName(BLE_DEVICE_NAME);
  pAdvert->setMinInterval(BLE_ADV_INTERVAL);
  pAdvert->setMaxInterval(BLE_ADV_INTERVAL);
  pAdvert->start(0);

  Serial.println("BLE: initialised");
}

// ===================== TELEM CAPTURE & DRAIN =====================
// Captures all fresh pages into ovfBuf at once. Drains one PDU per loop.

// Maps LOGI_* index to page type byte. Must match LOG_PAGE_TYPE in telemetry.cpp.
static const uint8_t BLE_LOGI_TO_PAGE[LOGI_COUNT] = {
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E
};

static void bleCapture() {
  unsigned long now = micros();

  // Advance nextCaptureUs by subIntervalUs to maintain consistent pacing.
  // Skip missed cycles (don't try to catch up).
  if (bleState.subIntervalUs > 0) {
    while (bleState.nextCaptureUs <= now) {
      bleState.nextCaptureUs += bleState.subIntervalUs;
    }
  }
  // subIntervalUs == 0 means capture every loop — fall through immediately.

  // Store capture timestamp. First 8 bytes of ovfBuf are the uint64 capture
  // timestamp (microseconds since boot). Records begin at offset 8.
  bleState.captureUs = (uint64_t)now;
  memcpy(bleState.ovfBuf, &bleState.captureUs, 8);
  bleState.ovfLen  = 8;  // records start after timestamp
  bleState.ovfSent = 8;

  // Write 0xAF header record first
  size_t written = buildHeaderRecord(bleState.ovfBuf + 8, BLE_OVF_BUF_SIZE - 8);
  bleState.ovfLen = 8 + (uint16_t)written;

  // Write each fresh page in the subscription mask (or forced thrust page)
  for (int i = 0; i < LOGI_COUNT; i++) {
    uint8_t pageType = BLE_LOGI_TO_PAGE[i];

    // Thrust curve page can be forced regardless of subscription (coast entry send)
    bool forced = (pageType == PAGE_THRUST_CURVE && thrustBleForce);
    bool subscribed = (bleState.subPageMask & (1ULL << pageType)) != 0;

    if (!forced && !subscribed) continue;
    if (!forced && !(logPages[i].freshMask & FRESH_BLE)) continue;

    size_t remaining = BLE_OVF_BUF_SIZE - bleState.ovfLen;
    size_t rec = buildDataPageRecord(pageType,
      bleState.ovfBuf + bleState.ovfLen, remaining);
    if (rec == 0) continue;
    bleState.ovfLen += (uint16_t)rec;

    // Clear force flag after building the record
    if (forced) thrustBleForce = false;
  }

  // Clear FRESH_BLE for all subscribed pages
  for (int i = 0; i < LOGI_COUNT; i++) {
    uint8_t pageType = BLE_LOGI_TO_PAGE[i];
    if (bleState.subPageMask & (1ULL << pageType)) {
      logPages[i].freshMask &= ~FRESH_BLE;
    }
  }
}

static void bleDrainOnePdu() {
  if (bleState.ovfSent >= bleState.ovfLen) return;
  if (!pTelemChar) return;

  // Records always start at offset 8 (first 8 bytes reserved for capture timestamp).
  // If ovfSent somehow fell below 8, reset to 8.
  if (bleState.ovfSent < 8) bleState.ovfSent = 8;
  if (bleState.ovfSent >= bleState.ovfLen) return;

  // Walk records starting at ovfSent to fit as many whole records as possible
  // into one PDU (max BLE_LOGFETCH_MAX_PDU - 8 bytes of records, plus 8-byte timestamp prefix).
  uint16_t maxRecBytes = (BLE_LOGFETCH_MAX_PDU > 8) ? (BLE_LOGFETCH_MAX_PDU - 8) : 0;
  uint16_t pduRecLen = 0;
  uint16_t pos = bleState.ovfSent;
  while (pos < bleState.ovfLen) {
    uint8_t recLen = bleState.ovfBuf[pos];  // length byte
    uint16_t recTotal = 1 + recLen;          // len byte + data
    if (pduRecLen + recTotal > maxRecBytes) break;
    pduRecLen += recTotal;
    pos += recTotal;
  }

  if (pduRecLen == 0) {
    // Single record larger than maxRecBytes — shouldn't happen; skip to avoid loop
    uint8_t recLen = bleState.ovfBuf[bleState.ovfSent];
    bleState.ovfSent += 1 + recLen;
    return;
  }

  // Write capture timestamp into the 8 bytes immediately before ovfSent.
  // This is always valid because records start at offset 8, so ovfSent >= 8.
  memcpy(bleState.ovfBuf + bleState.ovfSent - 8, &bleState.captureUs, 8);

  // Send: timestamp prefix + record bytes as one contiguous PDU
  bool ok = pTelemChar->notify(bleState.ovfBuf + bleState.ovfSent - 8, 8 + pduRecLen);
  if (ok) {
    bleState.ovfSent += pduRecLen;
  }
  // If !ok (congested): don't advance, retry next loop
}

// ===================== LOG FETCH DRAIN =====================

// Per-fetch diagnostics (rocket-side) — reset on each new request via onWrite.
static uint32_t rktFetchNotifyOk = 0, rktFetchNotifyDrop = 0, rktFetchBytesSent = 0;

static void bleFetchOnePdu() {
  if (!bleState.fetchActive || !pFetchChar) return;
  if (!logStoreOk) {
    bleState.fetchActive = false;
    return;
  }

  // Flow control is the notify() return value, not a timer.
  if (bleState.fetchPendingLen > 0) {
    if (!pFetchChar->notify(bleState.fetchPendingBuf, bleState.fetchPendingLen)) {
      rktFetchNotifyDrop++;
      yield();
      return;
    }
    rktFetchNotifyOk++;
    rktFetchBytesSent += bleState.fetchPendingLen;
    bleState.fetchPendingLen = 0;
  }

  // Retry the end-of-fetch marker if previously dropped.
  if (bleState.fetchEndPending) {
    if (!pFetchChar->notify((uint8_t*)"", 0)) { yield(); return; }
    bleState.fetchEndPending = false;
    bleState.fetchActive = false;
    Serial.printf("[BLEFetch] done @%lu notifyOk=%lu drop=%lu bytes=%lu (endPending path)\n",
      (unsigned long)bleState.fetchCurrentRec, (unsigned long)rktFetchNotifyOk,
      (unsigned long)rktFetchNotifyDrop, (unsigned long)rktFetchBytesSent);
    return;
  }

  // End of range — or cursor became invalid (e.g. hit erased marker)
  if (!bleState.fetchSeq.hasMore()) {
    if (!pFetchChar->notify((uint8_t*)"", 0)) {
      bleState.fetchEndPending = true;
      Serial.println("[BLEFetch] end marker dropped — will retry");
      return;
    }
    bleState.fetchActive = false;
    Serial.printf("[BLEFetch] done @%lu notifyOk=%lu drop=%lu bytes=%lu\n",
      (unsigned long)bleState.fetchCurrentRec, (unsigned long)rktFetchNotifyOk,
      (unsigned long)rktFetchNotifyDrop, (unsigned long)rktFetchBytesSent);
    return;
  }

  // Build directly into pendingBuf so we can retransmit on congestion.
  // Format: [recNum u32 LE][len u8][snr i8][ts u32 LE][payload: len bytes]
  uint16_t pduLen = 0;
  uint8_t* pdu = bleState.fetchPendingBuf;

  uint8_t recBuf[LOG_MAX_PAYLOAD];
  int8_t  snr;
  uint32_t ts;

  while (bleState.fetchSeq.hasMore()) {
    if (pduLen + 10 + LOG_MAX_PAYLOAD > BLE_LOGFETCH_MAX_PDU) break;

    uint32_t recNum = bleState.fetchSeq.currentRec();
    int rLen = bleState.fetchSeq.readNext(recBuf, sizeof(recBuf), &snr, &ts);
    if (rLen < 0) break;

    pdu[pduLen + 0] = (uint8_t)(recNum);
    pdu[pduLen + 1] = (uint8_t)(recNum >> 8);
    pdu[pduLen + 2] = (uint8_t)(recNum >> 16);
    pdu[pduLen + 3] = (uint8_t)(recNum >> 24);
    pdu[pduLen + 4] = (uint8_t)rLen;
    pdu[pduLen + 5] = (uint8_t)snr;
    pdu[pduLen + 6] = (uint8_t)(ts);
    pdu[pduLen + 7] = (uint8_t)(ts >> 8);
    pdu[pduLen + 8] = (uint8_t)(ts >> 16);
    pdu[pduLen + 9] = (uint8_t)(ts >> 24);
    memcpy(pdu + pduLen + 10, recBuf, rLen);
    pduLen += 10 + (uint16_t)rLen;
    bleState.fetchCurrentRec = bleState.fetchSeq.currentRec();
  }

  if (pduLen > 0) {
    if (!pFetchChar->notify(pdu, pduLen)) {
      bleState.fetchPendingLen = pduLen;  // hold for retry
      rktFetchNotifyDrop++;
      yield();
    } else {
      rktFetchNotifyOk++;
      rktFetchBytesSent += pduLen;
    }
  }
}

// ===================== NON-BLOCKING BLE MAIN ENTRY =====================

void nonblockingBle() {
  // Keep advertising alive — NimBLE may stop it unexpectedly
  if (pAdvert && !pAdvert->isAdvertising()) {
    pAdvert->start(0);
  }

  if (!bleState.connected) return;

  unsigned long now = micros();

  // Priority 1: drain telem overflow buffer
  if (bleState.ovfSent < bleState.ovfLen) {
    bleDrainOnePdu();
    return;  // one PDU per loop
  }

  // Priority 2: capture new telem if subscribed and interval elapsed, OR forced thrust send
  bool forcedCapture = thrustBleForce;  // coast entry: send thrust page immediately
  if (bleState.telemSubscribed || forcedCapture) {
    bool captureNow = forcedCapture ||
                      (bleState.subIntervalUs == 0) ||
                      (now >= bleState.nextCaptureUs);
    if (captureNow) {
      bleCapture();
      // Start draining immediately if we captured anything
      if (bleState.ovfLen > 0) {
        bleDrainOnePdu();
        return;
      }
    }
  }

  // Priority 3: log fetch (lower priority than telem)
  if (bleState.fetchActive) {
    bleFetchOnePdu();
  }

  // OTA notify drain — send queued error/progress byte(s) from OTA callbacks.
  if (bleState.otaNotifyPending && pOtaChar) {
    pOtaChar->notify((uint8_t*)bleState.otaNotifyBuf, bleState.otaNotifyLen);
    bleState.otaNotifyPending = false;
  }
}
