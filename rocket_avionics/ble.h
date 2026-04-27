// ble.h — BLE (NimBLE) transport for rocket avionics.
//
// Provides two transport functions called from the main loop:
//   initBLE()        — call once in setup after NVS/log init
//   nonblockingBle() — call every loop iteration
//
// GATT layout (separate service UUID from base station):
//   Service:     524f434b-4554-5354-424c-000000000000
//   Char 0001:   NOTIFY              — realtime telemetry records
//   Char 0002:   WRITE|WRITE_NR      — commands (same binary format as base station, waitMs/sends ignored)
//   Char 0003:   READ                — status JSON (logIdx, batt, armed, uptime)
//   Char 0004:   WRITE|WRITE_NR      — connection settings (interval, page mask, PHY)
//   Char 0005:   WRITE|NOTIFY        — log fetch (request then stream records)
//   Char 0006:   WRITE|WRITE_NR|NOTIFY — OTA firmware chunks in, error/progress notifications out
//
// Record wire format (used for both telem and fetch):
//   [len u8][data: len bytes]
//   data[0] is always the type byte.
//   len includes the type byte: e.g. len=8 → 1 type byte + 7 data bytes.
//   Types 0x00-0x7F = data pages; 0x80-0xFF = packet/header types (e.g. 0xAF).
//   Max PDU payload for log fetch: BLE_LOGFETCH_MAX_PDU bytes (defined in config.h). Whole records only.
//
// Fresh flags: ble.cpp clears FRESH_BLE (bit 2) from logPages[i].freshMask after
// sending each page. Sensor code sets freshMask |= 0xFF on each new reading.
//
// BLE callback timing: bleCallbackTotalUs / bleCallbackPeakUs accumulate time
// spent in NimBLE callbacks (which run on a separate FreeRTOS task). Monitor
// these to decide whether BLE is safe during armed flight.

#ifndef BLE_H
#define BLE_H

#include <Arduino.h>
#include "config.h"
#include "telemetry.h"

// ===================== BLE STATE =====================

struct BleState {
  bool connected;
  bool telemSubscribed;
  volatile bool fetchActive;  // volatile: written by NimBLE task, read by loopTask

  uint32_t subIntervalUs;      // 0 = every loop; default BLE_DEFAULT_INTERVAL_US
  uint64_t subPageMask;        // bit N = include page type N (bits 1-13 default)
  unsigned long nextCaptureUs; // consistent-pacing: advance by subIntervalUs each cycle

  // Overflow buffer: all fresh page records captured at once into here.
  // The first 8 bytes are a reserved uint64 capture timestamp (microseconds since
  // boot at capture time). Records always start at offset 8. When draining, the
  // timestamp is written into the 8 bytes immediately before ovfSent so every
  // PDU begins with the capture timestamp without copying any record data.
  uint8_t  ovfBuf[BLE_OVF_BUF_SIZE];
  uint16_t ovfLen;             // valid bytes currently in buffer (0 when empty, else >= 8)
  uint16_t ovfSent;            // bytes already notified; records start at 8
  uint64_t captureUs;          // micros() at time of capture (written into PDU prefix)

  // Log fetch state
  uint32_t fetchCurrentRec;
  uint32_t fetchEndRec;
  LogStore::SeqReader fetchSeq;  // sequential cursor — avoids O(N²) re-scan per record
  // Flow control: hold last built PDU until notify() succeeds — prevents
  // dropped records when the BLE host queue is full.
  uint8_t  fetchPendingBuf[BLE_LOGFETCH_MAX_PDU];
  uint16_t fetchPendingLen;     // 0 = no chunk pending
  bool     fetchEndPending;     // 0-byte end marker not yet sent
  unsigned long fetchLastTryMs; // millis() of last notify attempt — throttles retries

  uint8_t currentTxPhy;        // 1=1M 2=2M 3=Coded (for serial debug / status)

  // OTA notify-back: set by otaQueueNotify() from the NimBLE callback task;
  // drained by nonblockingBle() in the main loop.
  volatile bool    otaNotifyPending;
  volatile uint8_t otaNotifyBuf[8];   // up to 8 bytes (progress reports are 5 bytes)
  volatile uint8_t otaNotifyLen;
};

extern BleState bleState;

// ===================== CALLBACK TIMING COUNTERS =====================
// Volatile because they are written from the NimBLE FreeRTOS task and
// read from the main loop. 32-bit writes are atomic on ESP32 (aligned).

extern volatile uint32_t bleCallbackTotalUs;  // accumulated since last reset
extern volatile uint32_t bleCallbackPeakUs;   // worst single callback duration

// ===================== PUBLIC API =====================

// Call once in setup, after logStore is initialised.
void initBLE();

// Call every main loop iteration.
void nonblockingBle();

// Queue a 1-byte OTA status notify. Safe to call from the NimBLE callback task.
void otaQueueNotify(uint8_t status);

// Queue a multi-byte OTA notification (e.g. progress report). len <= 8.
void otaQueueNotifyBytes(const uint8_t* data, uint8_t len);

#endif // BLE_H
