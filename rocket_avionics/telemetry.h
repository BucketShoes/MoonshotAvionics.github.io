// telemetry.h — Telemetry packet building, data page assembly, and flash log writing.
// Owns the page cycle, log page config, and all data page format code.
// Changes to telemetry wire format, page layout, or log format only touch
// telemetry.h + telemetry.cpp.

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include "config.h"
#include "log_store.h"

// ===================== FRESH MASK BITS =====================
// Per-transport freshness flags. Sensor updates set all bits (0xFF).
// Each transport clears its own bit after consuming a page.
// Unreliable transports (LoRa) do not consume freshness — they fire on their
// own schedule regardless.

#define FRESH_LOG  0x01   // bit 0: log transport
#define FRESH_LORA 0x02   // bit 1: LoRa (set by sensor update, never cleared)
#define FRESH_BLE  0x04   // bit 2: BLE realtime telem
// bits 3-7 reserved for USB, WebSocket, etc.

// ===================== LOG PAGE CONFIG =====================
// Each page type has an independent log interval. intervalUs=0 = disabled.

struct LogPageConfig {
  uint32_t intervalUs;       // minimum microseconds between logs (0 = disabled)
  unsigned long lastLogUs;   // micros() of last log for this page
  uint8_t freshMask;         // bitmask of transports that have new data pending
};

// Page indices for the config array (not the page type IDs themselves)
enum LogPageIdx {
  LOGI_GPS_POS = 0,   // 0x01
  LOGI_BARO,          // 0x02
  LOGI_MAG,           // 0x03
  LOGI_ACCEL,         // 0x04
  LOGI_GYRO,          // 0x05
  LOGI_GPS_EXTRA,     // 0x06
  LOGI_KALMAN,        // 0x07
  LOGI_SYS_HEALTH,    // 0x08
  LOGI_PEAKS,         // 0x09
  LOGI_CMD_ACK,       // 0x0A (event-driven + periodic)
  LOGI_FLIGHT_STATUS, // 0x0B
  LOGI_RADIO_HEALTH,  // 0x0C
  LOGI_TIMESTAMP,     // 0x0D
  LOGI_COUNT
};

extern LogPageConfig logPages[LOGI_COUNT];

// ===================== PUBLIC API =====================

// Build a complete telemetry packet into buf (LoRa format). Returns packet length.
size_t buildTelemetryPacket(uint8_t* buf);

// Build a telemetry header as a BLE record: [len][0xAF][deviceId][gpsFrac1][gpsFrac2][fusionAlt s16][stateFlags u16]
// len=10 (type byte + 9 data bytes). Returns total bytes written (len byte + data).
size_t buildHeaderRecord(uint8_t* buf, size_t maxLen);

// Build one data page as a BLE record: [len][pageType][pageData...]
// len includes the type byte. Returns total bytes written (0 if not implemented or buf too small).
size_t buildDataPageRecord(uint8_t pageType, uint8_t* buf, size_t maxLen);

// Build one data page into a temp buffer and write it to flash.
void logPage(LogPageIdx idx);

// Write a data page directly to flash (type byte + payload).
void logDataPage(uint8_t pageType, const uint8_t* pageData, uint8_t pageDataLen);

// Write a raw received command packet to flash (for audit trail).
void logReceivedCommand(const uint8_t* pkt, size_t pktLen, int8_t snr);

// Check each enabled log page and write any that are due.
void nonblockingLogging();

// Print a packet as hex to Serial.
void printPacketHex(const uint8_t* buf, size_t len);

#endif // TELEMETRY_H
