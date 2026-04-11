/**
 * BLE Throughput Test - ESP32-S3 — 8-characteristic parallel stream
 * NimBLE-Arduino 2.4.0+
 *
 * Firmware round-robins 500-byte packets across 8 notify characteristics.
 * Each characteristic has its own sequence counter (increments by 1 per
 * packet on that stream). JS subscribes to all 8 in parallel to maximise
 * Web Bluetooth event dispatch throughput.
 *
 * UUIDs: ...ab0 through ...ab7 (data), ...ab8 (mode)
 *
 * Mode characteristic write:
 *   0x00 = 2M PHY    — max throughput
 *   0x01 = Coded S=8 — max range
 */

#define ADV_MODE_LEGACY
//#define ADV_MODE_CODED

#if defined(ADV_MODE_CODED)
#include <NimBLEExtAdvertising.h>
#endif
#include <NimBLEDevice.h>

static const char* DEVICE_NAME  = "BLE-Speed-Test";
static const char* SERVICE_UUID = "12345678-1234-1234-1234-123456789abc";
static const char* MODE_UUID    = "12345678-1234-1234-1234-123456789ab8";

static const int NUM_STREAMS = 8;
static const char* DATA_UUIDS[NUM_STREAMS] = {
    "12345678-1234-1234-1234-123456789ab0",
    "12345678-1234-1234-1234-123456789ab1",
    "12345678-1234-1234-1234-123456789ab2",
    "12345678-1234-1234-1234-123456789ab3",
    "12345678-1234-1234-1234-123456789ab4",
    "12345678-1234-1234-1234-123456789ab5",
    "12345678-1234-1234-1234-123456789ab6",
    "12345678-1234-1234-1234-123456789ab7",
};

static const int PACKET_SIZE = 500;

static const uint16_t CI_FAST_MIN = 6;
static const uint16_t CI_FAST_MAX = 6;
static const uint16_t CI_FAST_LAT = 0;
static const uint16_t CI_FAST_TO  = 800;

static const uint16_t CI_SLOW_MIN = 24;
static const uint16_t CI_SLOW_MAX = 40;
static const uint16_t CI_SLOW_LAT = 4;
static const uint16_t CI_SLOW_TO  = 3200;

static NimBLECharacteristic* pDataChar[NUM_STREAMS] = {};
static NimBLECharacteristic* pModeChar = nullptr;

#if defined(ADV_MODE_CODED)
static NimBLEExtAdvertising* pExtAdv = nullptr;
#endif

static bool     connected  = false;
static bool     subscribed = false;  // true when at least one stream subscribed
static bool     sending    = false;
static uint8_t  subCount   = 0;      // how many streams are subscribed

static uint32_t seqNum[NUM_STREAMS] = {};  // per-stream sequence counters
static uint8_t  nextStream = 0;            // round-robin index

static uint32_t bytesSent    = 0;
static uint32_t lastReportMs = 0;
static uint8_t  pktBuf[PACKET_SIZE];

// ── Advertising ───────────────────────────────────────────────────────────────

void startAdvertising() {
#if defined(ADV_MODE_CODED)
    pExtAdv->start(0);
    Serial.println("Advertising: Coded PHY");
#else
    NimBLEDevice::getAdvertising()->start(0);
    Serial.println("Advertising: legacy 1M");
#endif
}

// ── Callbacks ─────────────────────────────────────────────────────────────────

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        connected = true;
        subscribed = false;
        sending   = true;
        subCount  = 0;
        nextStream = 0;
        memset(seqNum, 0, sizeof(seqNum));
        bytesSent = 0;
        s->updateConnParams(info.getConnHandle(),
            CI_FAST_MIN, CI_FAST_MAX, CI_FAST_LAT, CI_FAST_TO);
        Serial.printf("Connected. Interval: %.2f ms\n",
            info.getConnInterval() * 1.25f);
    }

    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        connected = subscribed = sending = false;
        subCount = 0;
        Serial.printf("Disconnected: %d\n", reason);
        startAdvertising();
    }

    void onConnParamsUpdate(NimBLEConnInfo& info) override {
        Serial.printf("Interval: %.2f ms  latency: %d\n",
            info.getConnInterval() * 1.25f, info.getConnLatency());
    }

    void onPhyUpdate(NimBLEConnInfo& info, uint8_t txPhy, uint8_t rxPhy) override {
        const char* n[] = { "?", "1M", "2M", "Coded" };
        Serial.printf("PHY — tx:%s rx:%s\n",
            txPhy < 4 ? n[txPhy] : "?", rxPhy < 4 ? n[rxPhy] : "?");
    }
};

class ModeCharCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
        uint8_t mode = c->getValue<uint8_t>();
        NimBLEServer* s = NimBLEDevice::getServer();
        uint16_t h = info.getConnHandle();
        if (mode == 0x01) {
            Serial.println("Mode: Coded S=8");
            s->updatePhy(h, BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_MASK, BLE_GAP_LE_PHY_CODED_S8);
            s->updateConnParams(h, CI_SLOW_MIN, CI_SLOW_MAX, CI_SLOW_LAT, CI_SLOW_TO);
        } else {
            Serial.println("Mode: 2M PHY");
            s->updatePhy(h, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
            s->updateConnParams(h, CI_FAST_MIN, CI_FAST_MAX, CI_FAST_LAT, CI_FAST_TO);
        }
        sending = true;
    }
};

class DataCharCB : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* c, NimBLEConnInfo& info, uint16_t subValue) override {
        if (subValue == 1) {
            subCount++;
            Serial.printf("Stream subscribed (%d/%d)\n", subCount, NUM_STREAMS);
            if (subCount == NUM_STREAMS) {
                subscribed = true;
                Serial.println("All streams subscribed");
            }
        } else {
            if (subCount > 0) subCount--;
        }
    }
};

// ── Setup ─────────────────────────────────────────────────────────────────────

void bleSetup() {
    memset(pktBuf, 0xAA, sizeof(pktBuf));

    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(517);

    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());

    NimBLEService* svc = pServer->createService(SERVICE_UUID);

    DataCharCB* dataCB = new DataCharCB();
    for (int i = 0; i < NUM_STREAMS; i++) {
        pDataChar[i] = svc->createCharacteristic(DATA_UUIDS[i], NIMBLE_PROPERTY::NOTIFY);
        pDataChar[i]->setCallbacks(dataCB);
    }

    pModeChar = svc->createCharacteristic(MODE_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    pModeChar->setCallbacks(new ModeCharCB());

    svc->start();

#if defined(ADV_MODE_CODED)
    NimBLEExtAdvertisement advCoded;
    advCoded.setConnectable(true);
    advCoded.setPrimaryPhy(BLE_HCI_LE_PHY_CODED);
    advCoded.setSecondaryPhy(BLE_HCI_LE_PHY_CODED);
    advCoded.addServiceUUID(SERVICE_UUID);
    advCoded.setName(DEVICE_NAME);
    pExtAdv = new NimBLEExtAdvertising();
    pExtAdv->setInstanceData(0, advCoded);
#else
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setName(DEVICE_NAME);
#endif

    startAdvertising();
}

// ── Main loop ─────────────────────────────────────────────────────────────────

bool bleTrySend() {
    if (!connected) {
#if defined(ADV_MODE_CODED)
        if (!pExtAdv->isActive(0)) pExtAdv->start(0);
#else
        if (!NimBLEDevice::getAdvertising()->isAdvertising())
            NimBLEDevice::getAdvertising()->start(0);
#endif
        return false;
    }

    if (!subscribed || !sending) return false;

    // Round-robin across all 8 streams
    uint8_t s = nextStream;
    nextStream = (nextStream + 1) % NUM_STREAMS;

    uint32_t now = millis();
    // Header: [streamId:1][seq:4][timestamp:4] = 9 bytes, rest 0xAA
    pktBuf[0] = s;
    memcpy(pktBuf + 1, &seqNum[s], 4);
    memcpy(pktBuf + 5, &now, 4);

    bool ok = pDataChar[s]->notify(pktBuf, PACKET_SIZE);
    if (ok) {
        seqNum[s]++;
        bytesSent += PACKET_SIZE;
    }

    uint32_t nowMs = millis();
    if (nowMs - lastReportMs >= 1000) {
        float elapsed = (nowMs - lastReportMs) / 1000.0f;
        uint32_t totalSeq = 0;
        for (int i = 0; i < NUM_STREAMS; i++) totalSeq += seqNum[i];
        Serial.printf("BLE: %.1f KB/s | total pkts:%lu\n",
            (bytesSent / 1024.0f) / elapsed, totalSeq);
        bytesSent    = 0;
        lastReportMs = nowMs;
    }

    return ok;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("BLE Throughput Test — 8 parallel streams");
    bleSetup();
}

void loop() {
    bleTrySend();
}
