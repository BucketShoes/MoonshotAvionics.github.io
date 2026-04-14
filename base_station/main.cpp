// Base Station / Relay - Heltec Wireless Tracker V1.1
// Supports multiple transports: WiFi/WebSocket, BLE GATT, (future: USB Serial)
// BLE allows the phone to maintain mobile internet while connected to base station.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <RadioLib.h>
#include <Preferences.h>
#include <mbedtls/md.h>
#include <NimBLEDevice.h>
#include <LittleFS.h>
#include <esp_ota_ops.h>
#include "log_store.h"
#include "esp_wifi.h"

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14
#define LED_PIN       18
#define VEXT_CTRL_PIN 3
#define VBAT_ADC_PIN      1
#define VBAT_ADC_CTRL_PIN 2
#define VBAT_MULTIPLIER   4.9f

#define SERIAL_BAUD   115200
#include "secrets.h"  // gitignored — copy secrets_example.h to secrets.h
#define WIFI_CHANNEL  1

#define LORA_CR       5
#define LORA_PREAMBLE 6
#define LORA_SYNCWORD 0x12

#define DEVICE_ID     157

// ===================== OTA COMMAND IDs (matching rocket_avionics/config.h) =====================
#define CMD_OTA_BEGIN     0x50  // Open OTA write session (erases inactive partition)
#define CMD_OTA_FINALIZE  0x51  // Verify image HMAC, set boot partition, reboot
#define CMD_OTA_CONFIRM   0x52  // After reboot: confirm new firmware, cancel rollback

// ===================== TRANSPORT FLAGS (for 0x31/0x32 enable/disable) =====================
// When 0x31/0x32 has no params, all transports are enabled/disabled together.
// When a param byte is present, it's a bitmask of which transports to control.
// LoRa is NOT controlled by these — use TX rate commands for LoRa.
#define TRANSPORT_WIFI  0x01
#define TRANSPORT_BLE   0x02
#define TRANSPORT_USB   0x04  // reserved for future
#define TRANSPORT_ALL   (TRANSPORT_WIFI | TRANSPORT_BLE | TRANSPORT_USB)

bool wifiEnabled = true;
bool bleEnabled = true;

// ===================== CHANNEL TABLE (AU915-aligned) =====================
#define CHANNEL_COUNT 72

static float channelToFreqMHz(uint8_t ch) {
  if (ch < 64) return 915.2f + ch * 0.2f;
  if (ch < 72) return 915.9f + (ch - 64) * 1.6f;
  return 0.0f;
}

// ===================== SLOT TIMING SYNC =====================
// Mirror of rocket_avionics/config.h. Keep in sync manually.

enum WindowMode : uint8_t {
  WIN_TELEM  = 0,  // rocket TX telemetry / base RX
  WIN_RX     = 1,  // base TX commands / rocket RX
  WIN_OFF    = 2,  // radio off
  WIN_LR     = 3,  // future: long-range low-rate TX
  WIN_FINDME = 4,  // future: long-preamble beacon
};

static const WindowMode SLOT_SEQUENCE[] = { WIN_TELEM, WIN_RX };
#define SLOT_SEQUENCE_LEN   2
#define SLOT_DURATION_US    2'000'000UL  // µs
// startReceive(timeout) takes raw SX1262 timer units (1 unit = 15.625µs).
// Set the _US or _MS constants; _RAW values are derived automatically.
// Keep in sync with rocket_avionics/config.h.
#define BS_RX_EARLY_US        25'000UL   // µs before slot boundary
#define BS_RX_TIMEOUT_MS      50UL      // ms
#define BS_RX_TIMEOUT_RAW     (BS_RX_TIMEOUT_MS * 1000UL / 15.625f)
// Aim command TX this many µs after WIN_RX slot start.
// Gives rocket time to arm startReceive. Also the drift calibration reference point.
#define BS_CMD_TX_OFFSET_US   5'000UL  // µs after WIN_RX start

#define CMD_SET_SYNC           0x41    // sync command ID (matches rocket config.h)
// HMAC_KEY_LEN and HMAC_TRUNC_LEN come from secrets.h (already included above)

#define BS_SYNC_BOOT_DELAY_MS  2'000    // send first CMD_SET_SYNC 2s after boot
#define BS_SYNC_LOSS_SLOTS     5       // resync if no telem for this many WIN_TELEM slots

// ===================== ACTIVE RADIO CONFIG =====================
#define DEFAULT_CHANNEL  65
#define DEFAULT_SF       5
#define DEFAULT_POWER    -9

uint8_t activeChannel = DEFAULT_CHANNEL;
uint8_t activeSF      = DEFAULT_SF;
int8_t  activePower   = DEFAULT_POWER;
float   activeFreqMHz = 917.5f;
float   activeBwKHz   = 500.0f;

#define DEFAULT_BH_CHANNEL 67
#define DEFAULT_BH_SF      5
#define DEFAULT_BH_POWER   -9

uint8_t bhChannel = DEFAULT_BH_CHANNEL;
uint8_t bhSF      = DEFAULT_BH_SF;
int8_t  bhPower   = DEFAULT_BH_POWER;

static void updateActiveFreqBw() {
  activeFreqMHz = channelToFreqMHz(activeChannel);
  activeBwKHz = (activeChannel < 64) ? 125.0f : 500.0f;
}

#include "secrets.h"  // gitignored — copy secrets_example.h to secrets.h and fill in your key

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

// ===================== HARDWARE =====================

SPIClass loraSPI(FSPI);
SX1262 radio = new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_RST_PIN,
                          LORA_BUSY_PIN, loraSPI);
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

volatile bool dio1Fired = false;
void IRAM_ATTR onDio1() { dio1Fired = true; }

// ===================== BASE SLOT CLOCK STATE =====================

bool          bsSynced           = false;
unsigned long bsSyncAnchorUs     = 0;
uint32_t      bsSyncSlotIndex    = 0;
uint32_t      bsLastHandledSlot  = 0xFFFFFFFF;

// Radio state for slot machine
enum BsRadioState { BS_RADIO_IDLE, BS_RADIO_RX_ACTIVE, BS_RADIO_TX_ACTIVE };
BsRadioState bsRadioState = BS_RADIO_IDLE;

// Sync auto-send bookkeeping
unsigned long bsSyncBootMs    = 0;   // millis() at boot (set in setup())
bool          bsSyncSent      = false;
unsigned long bsMissedTelemSlots = 0; // consecutive WIN_TELEM slots with no RX (for resync)

// Pending sync send (deferred until slot machine can TX it)
bool          bsSyncPending   = false;

// Set true once the command TX window has been attempted in the current WIN_RX slot,
// so we don't attempt again in the same slot.
bool          bsCmdSentThisSlot = false;

// Set true when we've called startReceive for the early-listen window,
// so we don't spam startReceive every loop until DIO1 fires.
// Also set true after a telem-fallback listen is started, to prevent
// re-arming within the same WIN_TELEM slot after a timeout.
bool          bsEarlyRxArmed  = false;

// LED helpers — base station uses ledcAttach, so must use ledcWrite
static void bsLedOn()  { ledcWrite(LED_PIN, 255); }
static void bsLedOff() { ledcWrite(LED_PIN, 0);   }

struct { uint8_t data[256]; size_t len; int8_t snr4; uint32_t timestamp; bool valid;
} latestTelem = {.len=0,.snr4=0,.timestamp=0,.valid=false};

// ===================== BLE GATT SERVER =====================
// One service with 4 characteristics:
//   Telemetry Notify (0x0001): subscribe for real-time LoRa packets
//     Push format: [float32 snr LE][float32 rssi LE][int32 recNum LE][raw packet]
//     Uses BLE notify (unreliable/fast). JS tracks record numbers for gap detection.
//   Command Write (0x0002): write-with-response (reliable) to queue LoRa TX
//     Write format: [uint16 waitMs LE][uint8 sends][uint8 pktLen][packet bytes]
//     Same body format as POST /api/command.
//   Status Read (0x0003): read to get status JSON (same as GET /api/status)
//   Log Fetch (0x0004): write request, receive records via notify (fast/unreliable)
//   OTA Chunks (0x0006): WRITE|WRITE_NR raw firmware chunks; NOTIFY for errors/progress
//     Chunk format: [offset u32 LE][data: up to 512 bytes, must not cross 512B sector boundary]
//     Errors and progress reported via notify (see OTA_STATUS_* defines)
//     Write format: [uint32 startRec LE][uint16 count LE]
//     Notify format: concatenated records, same as /api/logs response binary.
//     Empty notify = end marker.
//     JS drives flow control: request a batch, receive it, request more.

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
// Status codes (same as rocket ota.h for consistency)
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
      return 0x07;  // OTA_STATUS_REFUSED
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
  return 0x00;  // CMD_OK
}

static uint8_t otaHandleFinalize(uint32_t expectedSize, const uint8_t* firmwareHmac) {
  if (bsOta.state != OTA_RECEIVING) return 0x02;
  bsOta.state = OTA_VERIFYING;
  if (bsOta.bytesWritten != expectedSize) {
    esp_ota_abort(bsOta.handle);
    bsOta.state = OTA_LOCKED;
    return 0x03;  // CMD_ERR_BAD_PARAMS
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
} bleLogFetch = {false, 0, 0, 0, {}};

// ===================== TRANSPORT-AGNOSTIC COMMAND TX =====================

struct CmdTxState {
  uint8_t pkt[64];
  uint8_t pktLen;
  uint8_t sends;
  uint8_t sent;
  uint16_t waitMs;       // max ms to wait for next WIN_RX before sending out of turn
  unsigned long lastSendMs;
  unsigned long queuedMs; // millis() when command was queued (for wait expiry)
  bool active;
} cmdTx = {.pktLen=0,.sends=0,.sent=0,.waitMs=0,.lastSendMs=0,.queuedMs=0,.active=false};

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

// ===================== LOG DOWNLOAD RADIO STATE =====================

static float bwCodeToFloat(uint8_t code) {
  switch (code) {
    case 0: return 500.0; case 1: return 250.0; case 2: return 125.0;
    case 3: return 62.5;  case 4: return 41.7;  case 5: return 31.25;
    case 6: return 20.8;  case 7: return 15.6;  case 8: return 10.4;
    case 9: return 7.8;   default: return 125.0;
  }
}

struct LogDlState {
  bool active;
  unsigned long lastRxMs;
  unsigned long enteredMs;
  static const unsigned long TIMEOUT_MS = 2000;
} logDl = {.active=false, .lastRxMs=0, .enteredMs=0};

void switchToDownloadRadio(float freqMHz, uint8_t sf, float bw, int8_t pwr) {
  radio.standby();
  radio.setFrequency(freqMHz);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setOutputPower(pwr);
  radio.startReceive();
}

void revertToNormalRadio() {
  radio.standby();
  radio.setFrequency(activeFreqMHz);
  radio.setSpreadingFactor(activeSF);
  radio.setBandwidth(activeBwKHz);
  radio.setOutputPower(activePower);
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  radio.startReceive();
  logDl.active = false;
  logDl.enteredMs = 0;
  Serial.println("DL radio reverted");
}

// ===================== PUSH TELEMETRY TO ALL TRANSPORTS =====================

void pushToAllTransports(const uint8_t* wsBuf, size_t wsLen) {
  if (wifiEnabled) {
    ws.binaryAll(wsBuf, wsLen);
  }
  if (bleEnabled && bleClientConnected && bleTelemChar) {
    bleTelemChar->notify((uint8_t*)wsBuf, wsLen);
  }
}

// ===================== SYNC PACKET BUILDER =====================
// Builds a CMD_SET_SYNC (0x41) command packet signed with HMAC.
// Returns packet length. Target device ID = rocket's DEVICE_ID (0x92 per config.h).
// Uses highestNonce++ (stored to NVS) same as all other commands.

#define ROCKET_DEVICE_ID  0x92   // must match rocket config.h DEVICE_ID

static size_t buildSyncCmdPacket(uint8_t* buf) {
  // Packet format: [type(1)][targetId(1)][cmdId(1)][nonce(4)][HMAC(10)]
  // = 17 bytes, 0 param bytes.
  highestNonce++;
  bsNvs.putUInt("nonce", highestNonce);

  buf[0] = 0x9A;               // PKT_COMMAND
  buf[1] = ROCKET_DEVICE_ID;
  buf[2] = CMD_SET_SYNC;
  buf[3] = (uint8_t)(highestNonce);
  buf[4] = (uint8_t)(highestNonce >> 8);
  buf[5] = (uint8_t)(highestNonce >> 16);
  buf[6] = (uint8_t)(highestNonce >> 24);

  // HMAC-SHA256 over bytes 0..6, truncated to 10 bytes
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

// ===================== LoRa RX HANDLER =====================

void handleLoRaRx() {
  if (!dio1Fired) return;
  dio1Fired    = false;
  bsRadioState = BS_RADIO_IDLE;
  uint8_t buf[256];
  size_t len = radio.getPacketLength();
  if (len == 0 || len > sizeof(buf)) { radio.startReceive(); return; }
  int state = radio.readData(buf, len);
  if (state != RADIOLIB_ERR_NONE) { radio.startReceive(); return; }
  float snrF = radio.getSNR();
  float rssiF = radio.getRSSI(true);
  int8_t snr4 = (int8_t)(snrF * 4);
  uint32_t nowMs = millis();

  int32_t recNum = -1;
  if (logStoreOk) recNum = logStore.writeRecord(buf, len, snr4, nowMs);

  if (logDl.active) logDl.lastRxMs = nowMs;

  if (len >= 10 && buf[0] == 0xAF) {
    memcpy(latestTelem.data, buf, len);
    latestTelem.len = len; latestTelem.snr4 = snr4;
    latestTelem.timestamp = nowMs; latestTelem.valid = true;
  }

  uint8_t wsBuf[268];
  memcpy(wsBuf, &snrF, 4);
  memcpy(wsBuf + 4, &rssiF, 4);
  memcpy(wsBuf + 8, &recNum, 4);
  memcpy(wsBuf + 12, buf, len);
  pushToAllTransports(wsBuf, 12 + len);

  Serial.print("RX "); Serial.print(len);
  Serial.print("B snr:"); Serial.print(snrF, 1);
  Serial.print(" rssi:"); Serial.print(rssiF, 0);
  Serial.print(" #"); Serial.println(recNum);
  // Note: radio.startReceive() is called by the slot machine (or bootstrap path) after this returns.
}

// ===================== COMMAND TX STATE MACHINE =====================

void handleCmdTx() {
  if (!cmdTx.active) return;
  unsigned long now = millis();
  if (cmdTx.sent > 0 && (now - cmdTx.lastSendMs) < cmdTx.waitMs) return;

  radio.standby();
  int st = radio.transmit(cmdTx.pkt, cmdTx.pktLen);
  cmdTx.lastSendMs = millis();
  cmdTx.sent++;

  Serial.print("CMD TX #"); Serial.print(cmdTx.sent);
  Serial.print("/"); Serial.print(cmdTx.sends);
  Serial.print(" st="); Serial.println(st);

  if (cmdTx.sent >= cmdTx.sends) {
    cmdTx.active = false;
    if (logStoreOk) logStore.writeRecord(cmdTx.pkt, cmdTx.pktLen, 0x7F, millis());

    // LOG DOWNLOAD (0x20) — switch to download radio
    if (cmdTx.pktLen >= 17 && cmdTx.pkt[2] == 0x20) {
      uint8_t dlCh  = cmdTx.pkt[7 + 6];
      uint8_t dlSF  = cmdTx.pkt[7 + 7];
      uint8_t dlBw  = cmdTx.pkt[7 + 8];
      int8_t  dlPwr = (int8_t)cmdTx.pkt[7 + 9];
      float dlFreq = channelToFreqMHz(dlCh);
      if (dlFreq == 0.0f) dlFreq = activeFreqMHz;
      switchToDownloadRadio(dlFreq, dlSF, bwCodeToFloat(dlBw), dlPwr);
      logDl.active = true;
      if (logDl.enteredMs == 0) logDl.enteredMs = millis();
      logDl.lastRxMs = millis();
    }

    // OTA commands (0x50/0x51/0x52) targeted at us — apply locally, do not forward over LoRa
    if (cmdTx.pkt[1] == DEVICE_ID &&
        (cmdTx.pkt[2] == CMD_OTA_BEGIN || cmdTx.pkt[2] == CMD_OTA_FINALIZE || cmdTx.pkt[2] == CMD_OTA_CONFIRM)) {
      uint8_t r = 0x00;
      if (cmdTx.pkt[2] == CMD_OTA_BEGIN) {
        r = otaHandleBegin();
      } else if (cmdTx.pkt[2] == CMD_OTA_FINALIZE && cmdTx.pktLen >= 7 + 36 + 10) {
        // params start at pkt[7]: [fwSize u32 LE][fwHmac 32 bytes]
        uint32_t fwSize = (uint32_t)cmdTx.pkt[7] | ((uint32_t)cmdTx.pkt[8] << 8)
                        | ((uint32_t)cmdTx.pkt[9] << 16) | ((uint32_t)cmdTx.pkt[10] << 24);
        const uint8_t* fwHmac = &cmdTx.pkt[11];
        r = otaHandleFinalize(fwSize, fwHmac);
      } else if (cmdTx.pkt[2] == CMD_OTA_CONFIRM) {
        r = otaHandleConfirm();
      }
      (void)r;
      revertToNormalRadio();
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

      float priFreq = channelToFreqMHz(priCh);
      if (priFreq != 0.0f && priSf >= 5 && priSf <= 12 && priPwr >= -9 && priPwr <= 22) {
        activeChannel = priCh; activeSF = priSf; activePower = priPwr;
        updateActiveFreqBw();
        bsNvs.putUChar("radio_ch", activeChannel);
        bsNvs.putUChar("radio_sf", activeSF);
        bsNvs.putChar("radio_pwr", activePower);
      }
      float bhFreq_ = channelToFreqMHz(bhCh_);
      if (bhFreq_ != 0.0f && bhSf_ >= 5 && bhSf_ <= 12 && bhPwr_ >= -9 && bhPwr_ <= 22) {
        bhChannel = bhCh_; bhSF = bhSf_; bhPower = bhPwr_;
        bsNvs.putUChar("bh_ch", bhChannel);
        bsNvs.putUChar("bh_sf", bhSF);
        bsNvs.putChar("bh_pwr", bhPower);
      }
      revertToNormalRadio();
      return;
    }
  }

  radio.startReceive();
}

// ===================== BUILD STATUS JSON =====================

size_t buildStatusJson(char* json, size_t maxLen) {
  readBaseBattery();
  return snprintf(json, maxLen,
    "{\"uptimeMs\":%lu,\"records\":%lu,\"oldest\":%lu,\"ringSize\":%lu,\"vpos\":%lu,"
    "\"logOk\":%s,\"nonce\":%lu,\"dlActive\":%s,\"deviceId\":%d,\"baseBattMv\":%u,"
    "\"wifiOn\":%s,\"bleOn\":%s}",
    (unsigned long)millis(),
    (unsigned long)logStore.getRecordCounter(),
    (unsigned long)logStore.getOldestRecord(),
    (unsigned long)logStore.getRingSize(),
    (unsigned long)logStore.getVirtualPos(),
    logStoreOk ? "true" : "false",
    (unsigned long)highestNonce,
    logDl.active ? "true" : "false",
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

// PUT /api/ota/chunk — body: [offset u32 LE][data: up to 512 bytes]
// No authentication on individual chunks; authentication is on the finalize command.
// Returns single-byte status or 4xx on bad request.
void handleApiOtaChunk(AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index != 0) return;
  if (len < total) { req->send(400, "text/plain", "fragmented"); return; }
  if (len < 5) { req->send(400, "text/plain", "too short"); return; }
  uint32_t offset = (uint32_t)data[0] | ((uint32_t)data[1] << 8)
                  | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
  // Call handler and flush the queued notify as the HTTP response
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

    //TODO: @@@ force 2m phy - is this the right place?
    //NimBLEConnection::setPhy(BLE_GAP_LE_PHY_2M, BLE_GAP_LE_PHY_2M)

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

    Serial.print("BLE fetch: "); Serial.print(startRec);
    Serial.print("-"); Serial.println(endRec);
  }
};

class BleOtaCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    NimBLEAttValue val = chr->getValue();
    if (val.size() < 5) return;  // need at least offset(4) + 1 data byte
    uint32_t offset = (uint32_t)val.data()[0] | ((uint32_t)val.data()[1] << 8)
                    | ((uint32_t)val.data()[2] << 16) | ((uint32_t)val.data()[3] << 24);
    otaHandleChunk(offset, val.data() + 4, val.size() - 4);
  }
};

// ===================== BLE LOG FETCH STATE MACHINE =====================
// Sends one MTU-full of records per main loop iteration (fast, no delay).
// Uses same binary format as /api/logs: [recNum u32][pLen u8][snr i8][ts u32][payload]

void handleBleLogFetch() {
  if (!bleLogFetch.active || !bleClientConnected || !bleLogFetchChar) return;

  if (!bleLogFetch.seq.hasMore()) {
    bleLogFetchChar->notify((uint8_t*)"", 0, true);
    bleLogFetch.active = false;
    Serial.print("BLE fetch done @"); Serial.println(bleLogFetch.currentRec);
    return;
  }

  uint8_t chunkBuf[500];
  size_t chunkPos = 0;
  uint8_t recBuf[LOG_MAX_PAYLOAD];
  int8_t snr;
  uint32_t ts;

  while (bleLogFetch.seq.hasMore() && chunkPos + 10 + LOG_MAX_PAYLOAD <= sizeof(chunkBuf)) {
    uint32_t rn = bleLogFetch.seq.currentRec();
    int pLen = bleLogFetch.seq.readNext(recBuf, sizeof(recBuf), &snr, &ts);
    if (pLen < 0) break;

    memcpy(chunkBuf + chunkPos, &rn, 4); chunkPos += 4;
    chunkBuf[chunkPos++] = (uint8_t)pLen;
    chunkBuf[chunkPos++] = (uint8_t)snr;
    memcpy(chunkBuf + chunkPos, &ts, 4); chunkPos += 4;
    memcpy(chunkBuf + chunkPos, recBuf, pLen); chunkPos += pLen;
    bleLogFetch.currentRec = bleLogFetch.seq.currentRec();
  }

  if (chunkPos > 0) {
    bleLogFetchChar->notify(chunkBuf, chunkPos, true);
  }
}

// ===================== BLE INIT / DEINIT =====================

void basicNimbleSetup() {
    NimBLEDevice::init("Moonshot");
    
    NimBLEServer *pServer = NimBLEDevice::createServer();
    NimBLEService *pService = pServer->createService("ABCD");
    NimBLECharacteristic *pCharacteristic = pService->createCharacteristic("1234");
    
    pService->start();
    pCharacteristic->setValue("Hello BLE");
    //TODO: @@@ switching to NimBLEDevice::setOwnAddrType and negotiating 2M PHY via setPreferredPhy is worth trying.
    
    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID("ABCD"); // advertise the UUID of our service
    pAdvertising->setName("NimBLE"); // advertise the device name
    pAdvertising->start(); 
    Serial.println("basicNimbleSetup started");
}

void initBLE() {
  //basicNimbleSetup();
  //return;

  NimBLEDevice::init(WIFI_SSID);
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

  // Ensure service UUID is in the main advertisement (not just scan response)
  // so Web Bluetooth filters can find it
  bleAdvert->setMinInterval(0x20);  // 20ms * 0.625 = 12.5ms
  bleAdvert->setMaxInterval(0x40);  // 40ms * 0.625 = 25ms
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

// ===================== SYNC SEND =====================
// Sends CMD_SET_SYNC 2s after boot (and on demand / resync).
// In the slot machine, the actual TX is deferred to the slot machine's
// IDLE state so we don't interrupt an active WIN_TELEM RX.

void triggerSyncSend() {
  bsSyncPending = true;
}

// Called from loop() — fires the sync TX when the radio is idle and
// there is no currently active slot RX/TX.
// This runs as a blocking transmit so we can capture the exact TxDone time.
static void doSendSync() {
  uint8_t pkt[17];
  size_t len = buildSyncCmdPacket(pkt);

  Serial.print("SYNC TX: sending CMD_SET_SYNC nonce="); Serial.println(highestNonce);
  radio.standby();
  unsigned long t0 = micros();
  int st = radio.transmit(pkt, len);
  unsigned long txDoneUs = micros();

  if (st == RADIOLIB_ERR_NONE) {
    // Anchor the base slot clock to TxDone.
    bsSyncAnchorUs    = txDoneUs;
    bsSyncSlotIndex   = 1;  // WIN_RX starts now (sync pkt was slot 0 = WIN_TELEM)
    bsLastHandledSlot = 0xFFFFFFFF;
    bsSynced          = true;
    bsSyncSent        = true;
    bsMissedTelemSlots = 0;
    Serial.print("SYNC anchor="); Serial.print(txDoneUs);
    Serial.print("us (tx took "); Serial.print(txDoneUs - t0); Serial.println("us)");
  } else {
    Serial.print("SYNC TX fail: "); Serial.println(st);
  }

  // Leave radio idle — the slot machine will arm RX at the right time.
  // Clear IRQ flags left by the blocking transmit before handing off to slot machine.
  radio.standby();
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  dio1Fired    = false;
  bsRadioState = BS_RADIO_IDLE;
}

void handleSyncSend() {
  unsigned long now = millis();

  // Auto-send 2s after boot
  if (!bsSyncSent && (now - bsSyncBootMs) >= BS_SYNC_BOOT_DELAY_MS) {
    bsSyncPending = true;
  }

  // Resync if too many missed telem slots
  if (bsSynced && bsMissedTelemSlots >= BS_SYNC_LOSS_SLOTS) {
    Serial.println("SYNC: too many missed telem slots, resyncing");
    bsSyncPending = true;
    bsMissedTelemSlots = 0;
  }

  if (!bsSyncPending) return;

  // Only fire when the radio is idle (not mid-RX or mid-TX in a slot)
  if (bsRadioState != BS_RADIO_IDLE && bsRadioState != BS_RADIO_RX_ACTIVE) return;

  bsSyncPending = false;
  doSendSync();
}

// ===================== SYNCED RADIO SLOT MACHINE =====================
// Replaces handleLoRaRx() and handleCmdTx() when bsSynced == true.
// Called from loop() every iteration.
//
// Slot layout (base station view):
//   WIN_TELEM: rocket is TX-ing; base listens. Early-listen started from prior WIN_RX.
//   WIN_RX:    base may TX a queued command; rocket is listening.
//              200ms before WIN_RX ends, arm early-listen for the next WIN_TELEM.

static void bsHandleRxDone() {
  // Called when DIO1 fires while in BS_RADIO_RX_ACTIVE.
  // readData returns RADIOLIB_ERR_NONE on good packet, RADIOLIB_ERR_RX_TIMEOUT on timeout.
  size_t len = radio.getPacketLength();
  uint8_t buf[256];
  int st = radio.readData(buf, (len > 0 && len <= sizeof(buf)) ? len : sizeof(buf));

  if (st == RADIOLIB_ERR_NONE) {
    float snrF  = radio.getSNR();
    float rssiF = radio.getRSSI(true);
    int8_t snr4 = (int8_t)(snrF * 4);
    uint32_t nowMs = millis();

    int32_t recNum = -1;
    if (logStoreOk) recNum = logStore.writeRecord(buf, len, snr4, nowMs);

    if (len >= 10 && buf[0] == 0xAF) {
      memcpy(latestTelem.data, buf, len);
      latestTelem.len = len; latestTelem.snr4 = snr4;
      latestTelem.timestamp = nowMs; latestTelem.valid = true;
      bsMissedTelemSlots = 0;
    }

    uint8_t wsBuf[268];
    memcpy(wsBuf, &snrF, 4);
    memcpy(wsBuf + 4, &rssiF, 4);
    memcpy(wsBuf + 8, &recNum, 4);
    memcpy(wsBuf + 12, buf, len);
    pushToAllTransports(wsBuf, 12 + len);

    Serial.print("SLOT RX "); Serial.print(len);
    Serial.print("B snr:"); Serial.print(snrF, 1);
    Serial.print(" rssi:"); Serial.print(rssiF, 0);
    Serial.print(" #"); Serial.println(recNum);
  } else if (st == RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.println("SLOT RX timeout");
  } else {
    Serial.print("SLOT RX err: "); Serial.println(st);
  }
}

static void bsStartListening(const char* label) {
  // Clear any stale IRQ flags on the SX1262 before arming RX,
  // otherwise a leftover flag fires DIO1 immediately after startReceive.
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  dio1Fired = false;
  int st = radio.startReceive(BS_RX_TIMEOUT_RAW);
  if (st == RADIOLIB_ERR_NONE) {
    bsRadioState   = BS_RADIO_RX_ACTIVE;
    bsEarlyRxArmed = true;
    bsLedOn();
    unsigned long nowUs = micros();
    unsigned long posInSlot = (nowUs - bsSyncAnchorUs) % SLOT_DURATION_US;
    Serial.print("RX start ["); Serial.print(label);
    Serial.print("] pos="); Serial.print(posInSlot); Serial.println("us");
  } else {
    Serial.print("RX start fail ["); Serial.print(label);
    Serial.print("]: "); Serial.println(st);
  }
}

void handleSyncedRadio() {
  unsigned long now = micros();

  // ---- DIO1: handle completion of any in-progress RX or TX ----
  if (dio1Fired) {
    dio1Fired = false;

    // Read and clear IRQ flags so we know what actually fired,
    // and so the flags don't re-trigger DIO1 on the next startReceive.
    uint32_t irqFlags = radio.getIrqFlags();
    radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);

    unsigned long posInSlot = (now - bsSyncAnchorUs) % SLOT_DURATION_US;
    Serial.print("DIO1 irq=0x"); Serial.print(irqFlags, HEX);
    Serial.print(" state="); Serial.print((int)bsRadioState);
    Serial.print(" pos="); Serial.print(posInSlot); Serial.println("us");

    if (bsRadioState == BS_RADIO_RX_ACTIVE) {
      bsLedOff();
      // Note: do NOT clear bsEarlyRxArmed here. It is reset on slot transition.
      // Keeping it true after a timeout prevents re-arming within the same slot.
      bsHandleRxDone();
      // finishReceive() clears IRQ flags then calls standby() — prevents DIO1 re-firing
      // before the next startReceive (which would also clear flags, but the gap matters).
      radio.finishReceive();
      bsRadioState = BS_RADIO_IDLE;

    } else if (bsRadioState == BS_RADIO_TX_ACTIVE) {
      bsLedOff();
      // finishTransmit() clears IRQ flags then calls standby() — same reason.
      radio.finishTransmit();
      bsRadioState = BS_RADIO_IDLE;
      Serial.println("SLOT TxDone");

      if (cmdTx.active) {
        cmdTx.sent++;
        if (cmdTx.sent >= cmdTx.sends) {
          if (logStoreOk) logStore.writeRecord(cmdTx.pkt, cmdTx.pktLen, 0x7F, millis());
          cmdTx.active = false;
        }
      }
    }
    return;
  }

  // ---- Compute current slot ----
  unsigned long elapsed    = now - bsSyncAnchorUs;
  uint32_t      slotNum    = (uint32_t)(elapsed / SLOT_DURATION_US);
  uint32_t      posInSlot  = (uint32_t)(elapsed % SLOT_DURATION_US);
  uint8_t       seqIdx     = (uint8_t)((bsSyncSlotIndex + slotNum) % SLOT_SEQUENCE_LEN);
  WindowMode    win        = SLOT_SEQUENCE[seqIdx];
  uint32_t      timeToNext = SLOT_DURATION_US - posInSlot;

  // Reset early-RX armed flag on slot transition so it can re-arm next cycle
  if (slotNum != bsLastHandledSlot && win != WIN_TELEM) {
    bsEarlyRxArmed = false;
  }

  switch (win) {

    case WIN_TELEM: {
      // Log slot entry once
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        bsMissedTelemSlots++;
        Serial.print("SLOT WIN_TELEM pos="); Serial.print(posInSlot); Serial.println("us");
      }
      // Start listening if: radio is idle AND we haven't already listened this slot.
      // bsEarlyRxArmed is set by bsStartListening() and only cleared on WIN_RX slot entry,
      // so it remains true for the entire WIN_TELEM slot after any listen attempt.
      if (!bsEarlyRxArmed && bsRadioState == BS_RADIO_IDLE) {
        bsStartListening("telem-fallback");
      }
      break;
    }

    case WIN_RX: {
      // On first entry: reset per-slot flags and do idle flash if no command pending.
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot    = slotNum;
        bsEarlyRxArmed       = false;
        bsCmdSentThisSlot   = false;
        if (!cmdTx.active || cmdTx.sent >= cmdTx.sends) {
          // No command — 5ms LED flash as proof-of-sync
          bsLedOn();
          delayMicroseconds(5000);
          bsLedOff();
          Serial.print("SLOT WIN_RX idle pos="); Serial.print(posInSlot); Serial.println("us");
        }
      }

      // Fire command TX on the first loop after BS_CMD_TX_OFFSET_US is reached.
      // Exactly one attempt per slot: mark fired (hit or miss) the moment the offset is
      // crossed. If the radio isn't idle at that instant, the slot is skipped entirely —
      // no retrying later in the same slot when the rocket may no longer be listening.
      if (!bsCmdSentThisSlot && cmdTx.active && cmdTx.sent < cmdTx.sends
          && posInSlot >= BS_CMD_TX_OFFSET_US) {
        bsCmdSentThisSlot = true;
        if (bsRadioState == BS_RADIO_IDLE) {
          Serial.print("SLOT WIN_RX TX cmd "); Serial.print(cmdTx.sent + 1);
          Serial.print("/"); Serial.print(cmdTx.sends);
          Serial.print(" pos="); Serial.print(posInSlot); Serial.println("us");
          bsLedOn();
          radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
          dio1Fired = false;
          int st = radio.startTransmit(cmdTx.pkt, cmdTx.pktLen);
          if (st == RADIOLIB_ERR_NONE) {
            bsRadioState = BS_RADIO_TX_ACTIVE;
          } else {
            bsLedOff();
            Serial.print("WIN_RX TX fail: "); Serial.println(st);
          }
        } else {
          Serial.print("SLOT WIN_RX cmd skipped (radio busy) pos="); Serial.print(posInSlot); Serial.println("us");
        }
      }

      // Arm early-listen when close to WIN_TELEM. Guard prevents re-arming after timeout.
      if (!bsEarlyRxArmed && bsRadioState == BS_RADIO_IDLE && timeToNext <= BS_RX_EARLY_US) {
        bsStartListening("early");
      }
      break;
    }

    case WIN_OFF:
    default:
      if (bsRadioState != BS_RADIO_TX_ACTIVE && bsRadioState != BS_RADIO_RX_ACTIVE) {
        radio.standby();
        bsRadioState = BS_RADIO_IDLE;
        bsLedOff();
      }
      if (slotNum != bsLastHandledSlot) {
        bsLastHandledSlot = slotNum;
        Serial.println("SLOT WIN_OFF");
      }
      break;
  }
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println("\n=== Rocket Base Station ===");

  // BLE
  initBLE();

  ledcAttach(LED_PIN, 1000, 8);  // pin, freq, resolution
  ledcWrite(LED_PIN, 5);
  //pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

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
  if (bsNvs.isKey("radio_ch")) { uint8_t c = bsNvs.getUChar("radio_ch", DEFAULT_CHANNEL); activeChannel = (channelToFreqMHz(c) != 0.0f) ? c : DEFAULT_CHANNEL; }
  if (bsNvs.isKey("radio_sf")) { uint8_t s = bsNvs.getUChar("radio_sf", DEFAULT_SF); activeSF = (s >= 5 && s <= 12) ? s : DEFAULT_SF; }
  if (bsNvs.isKey("radio_pwr")) { int8_t p = bsNvs.getChar("radio_pwr", DEFAULT_POWER); activePower = (p >= -9 && p <= 22) ? p : DEFAULT_POWER; }
  updateActiveFreqBw();

  if (bsNvs.isKey("bh_ch")) { uint8_t c = bsNvs.getUChar("bh_ch", DEFAULT_BH_CHANNEL); bhChannel = (channelToFreqMHz(c) != 0.0f) ? c : DEFAULT_BH_CHANNEL; }
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
  loraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
  int st = radio.begin(activeFreqMHz, activeBwKHz, activeSF, LORA_CR, LORA_SYNCWORD, activePower, LORA_PREAMBLE);
  Serial.print("LoRa: "); Serial.println(st == RADIOLIB_ERR_NONE ? "ok" : "FAIL");
  radio.setDio2AsRfSwitch(true);
  radio.setDio1Action(onDio1);
  // startReceive(timeout) automatically enables RxTimeout on DIO1 when timeout != INF.
  // No manual IRQ mask setup needed — RadioLib handles it in startReceiveCommon().
  radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
  radio.startReceive();
  dio1Fired      = false;
  bsRadioState   = BS_RADIO_RX_ACTIVE;
  bsSyncBootMs   = millis();

Serial.println("Radio Done");



    //TODO: lower power usage:
    NimBLEDevice::getAdvertising()->setAdvertisingInterval(1600);// in 0.6125ms units
esp_wifi_set_max_tx_power(20);//8=2dbm, 0.25dbm units 80=20dbm
esp_wifi_set_ps(WIFI_PS_MAX_MODEM); // WIFI_PS_MIN_MODEM or WIFI_PS_MAX_MODEM for more aggressive - only wakes at DTIM beacons so drops first packet, should drop wifi from 111ma to 30ma
setCpuFrequencyMhz(160); //tune this original 240 - rocket can also slow down - dont need speed, only need consistency, and the flash delay is our worst, not impacted by speed. clock should drop from 40-80 down by 20-30ma
//TODO: @@@when we do hopping, also schedule recieve, at least when landed - agt least shrink it when clocks are syncd. keep 100% rx when no gps time



  Serial.println("=== Ready === new3");
}

// ===================== MAIN LOOP =====================

void loop() {
  if (!NimBLEDevice::getAdvertising()->isAdvertising()) {
    NimBLEDevice::getAdvertising()->start(0);
  }

  // Log download mode: blocking transfer, use old handlers while active
  if (logDl.active) {
    handleLoRaRx();   // uses dio1Fired, pushes telem, calls radio.startReceive()
    if ((millis() - logDl.lastRxMs) >= LogDlState::TIMEOUT_MS) {
      revertToNormalRadio();
      bsRadioState = BS_RADIO_RX_ACTIVE;
      dio1Fired = false;
    }
  } else {
    // Normal operation: slot-based radio machine
    handleSyncSend();      // auto-send CMD_SET_SYNC at boot + resync
    if (bsSynced) {
      handleSyncedRadio();
    } else {
      // Bootstrap: continuous RX, waiting for sync to be sent and anchored
      handleLoRaRx();
      // After handleLoRaRx() consumes a packet, re-arm RX if radio went idle
      if (bsRadioState == BS_RADIO_IDLE) {
        radio.startReceive();
        bsRadioState = BS_RADIO_RX_ACTIVE;
        dio1Fired = false;
      }
    }
  }

  // Command TX: not synced → old immediate path; synced → out-of-turn if wait expired
  if (!logDl.active) {
    if (!bsSynced) {
      handleCmdTx();
    } else if (bsSynced && cmdTx.active && cmdTx.sent < cmdTx.sends
               && bsRadioState == BS_RADIO_IDLE
               && (millis() - cmdTx.queuedMs) > cmdTx.waitMs) {
      // Wait window expired — send out of turn rather than drop the command.
      Serial.print("CMD TX out-of-turn (wait expired) pos=");
      unsigned long posNow = (micros() - bsSyncAnchorUs) % SLOT_DURATION_US;
      Serial.println(posNow);
      bsLedOn();
      radio.clearIrqFlags(RADIOLIB_SX126X_IRQ_ALL);
      dio1Fired = false;
      int st = radio.startTransmit(cmdTx.pkt, cmdTx.pktLen);
      if (st == RADIOLIB_ERR_NONE) {
        bsRadioState = BS_RADIO_TX_ACTIVE;
        bsCmdSentThisSlot = true;
      } else {
        bsLedOff();
        Serial.print("Out-of-turn TX fail: "); Serial.println(st);
      }
      cmdTx.queuedMs = millis();  // reset so it doesn't fire again next loop
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



void NotUsed(){
  //led current drive strength - not good given series resistor
   //gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  // Try weaker drive strength
  //gpio_set_drive_capability(LED_PIN, GPIO_DRIVE_CAP_0); // 0=weakest, 5ma, 10ma, 20ma, 40ma=GPIO_DRIVE_CAP_3  
  //gpio_set_level(LED_PIN, 1); // LED on
}