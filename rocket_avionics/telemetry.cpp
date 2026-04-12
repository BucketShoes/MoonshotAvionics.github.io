// telemetry.cpp — Telemetry packet building, data page assembly, and flash log writing.
// See telemetry.h for the public API.

#include <Arduino.h>
#include "telemetry.h"
#include "gps.h"
#include "sensors.h"
#include "flight.h"
#include "commands.h"
#include "radio.h"
#include "globals.h"

// ===================== LOG PAGE CONFIG =====================
//back reference index
static const uint8_t LOG_PAGE_TYPE[] = {
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D
};

LogPageConfig logPages[LOGI_COUNT] = {
  { 1000000, 0, 0 },  // GPS pos
  { 1000000, 0, 0 },  // Baro
  { 1000000, 0, 0 },  // Mag
  { 1000000, 0, 0 },  // Accel
  { 1000000, 0, 0 },  // Gyro
  { 1000000, 0, 0 },  // GPS extra
  { 0,       0, 0 },  // Kalman: disabled (not implemented)
  { 1000000, 0, 0 },  // System health
  {10000000, 0, 0 },  // Peaks
  {10000000, 0, 0 },  // Cmd ack (also event-driven)
  { 1000000, 0, 0 },  // Flight status
  {10000000, 0, 0 },  // Radio health
  {10000000, 0, 0 },  // Timestamp
};

// ===================== PAGE CYCLE =====================
// Order in which data pages rotate in regular telemetry packets.

static const uint8_t PAGE_CYCLE[] = {
  0x01, 0x02, 0x03, 0x02, 0x04, 0x02, 0x05, 0x02,
  0x06, 0x02, 0x08, 0x02, 0x09, 0x02, 0x0A, 0x02,
  0x0B, 0x02, 0x0C, 0x02, 0x0D
};
static const uint8_t PAGE_CYCLE_COUNT = sizeof(PAGE_CYCLE) / sizeof(PAGE_CYCLE[0]);
static uint8_t pageIndex = 0;

// ===================== BINARY WRITE HELPERS =====================

static void writeU8(uint8_t* buf, size_t* pos, uint8_t val) {
  buf[(*pos)++] = val;
}

static void writeU16(uint8_t* buf, size_t* pos, uint16_t val) {
  buf[(*pos)++] = val & 0xFF;
  buf[(*pos)++] = (val >> 8) & 0xFF;
}

static void writeS16(uint8_t* buf, size_t* pos, int16_t val) {
  writeU16(buf, pos, (uint16_t)val);
}

static void writeS32(uint8_t* buf, size_t* pos, int32_t val) {
  buf[(*pos)++] = val & 0xFF;
  buf[(*pos)++] = (val >> 8) & 0xFF;
  buf[(*pos)++] = (val >> 16) & 0xFF;
  buf[(*pos)++] = (val >> 24) & 0xFF;
}

static void writeU32(uint8_t* buf, size_t* pos, uint32_t val) {
  writeS32(buf, pos, (int32_t)val);
}

// ===================== ENCODING HELPERS =====================

static uint16_t encodeGpsFrac(double degrees) {
  if (initState < INIT_GPS_UART) return GPS_FRAC_NOT_POWERED;
  if (!gpsUartStarted)           return GPS_FRAC_INITIALISING;
  if (!gps.valid)                return GPS_FRAC_NO_FIX;
  double absDeg = fabs(degrees);
  double frac = absDeg - floor(absDeg);
  uint32_t steps = (uint32_t)(frac / 0.00002);
  if (steps > GPS_FRAC_VALID_MAX) steps = GPS_FRAC_VALID_MAX;
  return (uint16_t)steps;
}

static int16_t clampToInt16(double val, bool valid, int16_t invalidVal) {
  if (!valid) return invalidVal;
  if (val > 32767.0) return 32767;
  if (val < -32767.0) return -32767;
  return (int16_t)val;
}

static uint16_t buildStateFlags() {
  uint16_t flags = 0;
  flags |= ((uint16_t)flightGetPhase() & 0x0F);  // [3:0] flight phase
  if (isArmed) flags |= (1 << 4);                // [4] armed
  // [5] pyro ch1 fired — TODO
  // [6] pyro ch2 fired — TODO
  // [7] chute released — TODO
  if (batteryMv > 0 && batteryMv < 3300) flags |= (1 << 8);  // [8] low battery
  if (flightBaroOk())    flags |= (1 << 9);   // [9] baro ok
  if (flightAccelOk())   flags |= (1 << 10);  // [10] accel ok
  if (flightIsArmReady()) flags |= (1 << 11); // [11] arm ready
  return flags;
}

// ===================== DATA PAGES =====================

// Page 0x01: Raw GPS position (10 bytes)
static void buildPage01(uint8_t* buf, size_t* pos) {
  int32_t lat1e7 = gps.valid ? (int32_t)(gps.lat * 1e7) : 0;
  int32_t lon1e7 = gps.valid ? (int32_t)(gps.lon * 1e7) : 0;
  writeS32(buf, pos, lat1e7);
  writeS32(buf, pos, lon1e7);
  uint8_t hdop10 = 255;
  if (gps.valid) {
    double h = gps.hdop * 10.0;
    hdop10 = (uint8_t)constrain(h, 0.0, 255.0);
  }
  writeU8(buf, pos, hdop10);
  writeU8(buf, pos, gps.valid ? gps.sats : 0);
}

// Page 0x02: Baro + filtered vertical velocity (8 bytes)
static void buildPage02(uint8_t* buf, size_t* pos) {
  int32_t altCmMSL = baroData.valid ? baroData.altCmMSL : 0;
  writeS32(buf, pos, altCmMSL);
  int16_t vvel = baroData.valid ? baroData.vvel10 : 0;
  writeS16(buf, pos, vvel);
  int16_t groundM = baroData.valid ? (int16_t)(baroData.groundAltCm / 100) : 0;
  writeS16(buf, pos, groundM);
}

// Page 0x03: Magnetometer (6 bytes)
static void buildPage03(uint8_t* buf, size_t* pos) {
  writeS16(buf, pos, magData.valid ? magData.x : 0);
  writeS16(buf, pos, magData.valid ? magData.y : 0);
  writeS16(buf, pos, magData.valid ? magData.z : 0);
}

// Page 0x04: Accelerometer 3-axis (6 bytes)
static void buildPage04(uint8_t* buf, size_t* pos) {
  writeS16(buf, pos, accelData.valid ? accelData.x : 0);
  writeS16(buf, pos, accelData.valid ? accelData.y : 0);
  writeS16(buf, pos, accelData.valid ? accelData.z : 0);
}

// Page 0x05: Gyroscope 3-axis (6 bytes)
static void buildPage05(uint8_t* buf, size_t* pos) {
  writeS16(buf, pos, gyroData.valid ? gyroData.x : 0);
  writeS16(buf, pos, gyroData.valid ? gyroData.y : 0);
  writeS16(buf, pos, gyroData.valid ? gyroData.z : 0);
}

// Page 0x06: GPS extra + full-res altitude (10 bytes)
static void buildPage06(uint8_t* buf, size_t* pos) {
  uint16_t speedCms = 0;
  if (gps.rmcValid) {
    double cms = gps.groundSpeedKnots * 51.4444;
    speedCms = (uint16_t)constrain(cms, 0.0, 65535.0);
  }
  writeU16(buf, pos, speedCms);

  uint16_t course100 = 0;
  if (gps.rmcValid && gps.groundSpeedKnots > 0.5) {
    course100 = (uint16_t)constrain(gps.courseDeg * 100.0, 0.0, 36000.0);
  }
  writeU16(buf, pos, course100);

  uint8_t fixAge = 255;
  if (gps.valid && gps.lastFixUs != 0) {
    unsigned long age100ms = (micros() - gps.lastFixUs) / 100000UL;
    fixAge = (uint8_t)min(age100ms, 255UL);
  }
  writeU8(buf, pos, fixAge);

  uint8_t vdop10 = 255;
  if (gps.valid) vdop10 = (uint8_t)constrain(gps.vdop * 10.0, 0.0, 255.0);
  writeU8(buf, pos, vdop10);

  int32_t altCmMSL = gps.valid ? (int32_t)gps.altCm : 0;
  writeS32(buf, pos, altCmMSL);
}

// Page 0x08: System health (11 bytes)
static void buildPage08(uint8_t* buf, size_t* pos) {
  float tempC = temperatureRead();
  writeU8(buf, pos, (uint8_t)(int8_t)constrain((int)tempC, -128, 127));
  writeU16(buf, pos, (uint16_t)(ESP.getFreeHeap() / 1024));
  writeU16(buf, pos, (uint16_t)((millis() / 1000UL) & 0xFFFF));
  writeU16(buf, pos, batteryMv);
  writeU32(buf, pos, logStoreOk ? logStore.getRecordCounter() : 0);
}

// Page 0x09: Peak values since arm (8 bytes)
static void buildPage09(uint8_t* buf, size_t* pos) {
  writeS32(buf, pos, peaks.maxAltCmMSL);
  writeU16(buf, pos, peaks.maxAccel100g);
  writeS16(buf, pos, peaks.maxVvel10);
}

// Page 0x0A: Command ack + signal quality (9 bytes)
static void buildPage0A(uint8_t* buf, size_t* pos) {
  writeU32(buf, pos, lastAck.nonce);
  writeU8(buf, pos, lastAck.result);
  writeU8(buf, pos, (uint8_t)lastAck.rssi);
  writeU8(buf, pos, (uint8_t)lastAck.snr);
  writeU16(buf, pos, lastAck.invalidHmacCount);
}

// Page 0x0B: Flight status (8 bytes)
static void buildPage0B(uint8_t* buf, size_t* pos) {
  writeS32(buf, pos, flightState.msSinceLaunch);
  writeU16(buf, pos, 0);  // pyro/chute flags TODO
}

// Page 0x0C: Radio health (5 bytes)
static void buildPage0C(uint8_t* buf, size_t* pos) {
  writeU16(buf, pos, delayedTxCount);
  writeU16(buf, pos, invalidRxCount);
  int8_t noiseFloor = (int8_t)constrain((int)rssiEma, -128, 127);
  writeU8(buf, pos, (uint8_t)noiseFloor);
}

// Page 0x0D: Timestamp (8 bytes) — UTC ms since 1970-01-01
static void buildPage0D(uint8_t* buf, size_t* pos) {
  uint64_t t = utcTimeMs;
  for (int i = 0; i < 8; i++) {
    buf[(*pos)++] = (uint8_t)(t & 0xFF);
    t >>= 8;
  }
}

static void dispatchBuildPage(uint8_t pageType, uint8_t* buf, size_t* pos) {
  switch (pageType) {
    case 0x01: buildPage01(buf, pos); break;
    case 0x02: buildPage02(buf, pos); break;
    case 0x03: buildPage03(buf, pos); break;
    case 0x04: buildPage04(buf, pos); break;
    case 0x05: buildPage05(buf, pos); break;
    case 0x06: buildPage06(buf, pos); break;
    case 0x08: buildPage08(buf, pos); break;
    case 0x09: buildPage09(buf, pos); break;
    case 0x0A: buildPage0A(buf, pos); break;
    case 0x0B: buildPage0B(buf, pos); break;
    case 0x0C: buildPage0C(buf, pos); break;
    case 0x0D: buildPage0D(buf, pos); break;
    // 0x07 (Kalman) not yet implemented
  }
}

// ===================== BLE RECORD HELPERS =====================
// Records are [len u8][data: len bytes], where data[0] is always the type byte.
// len includes the type byte (e.g. len=8 means 1 type byte + 7 data bytes).

// Build the 0xAF header as a BLE record. Returns total bytes written (len byte + data).
size_t buildHeaderRecord(uint8_t* buf, size_t maxLen) {
  // data: [0xAF][deviceId][gpsFracLat u16][gpsFracLon u16][fusionAlt s16][stateFlags u16]
  // = 1 + 1 + 2 + 2 + 2 + 2 = 10 bytes of data, so len=10
  if (maxLen < 11) return 0;  // need 1 (len byte) + 10 (data)

  int16_t fusionAlt = -32768;
  if (flightState.baroFast.valid) {
    fusionAlt = clampToInt16(flightState.baroFast.valueCm / 100.0, true, -32768);
  } else if (gps.valid) {
    fusionAlt = clampToInt16(gps.alt, true, -32768);
  }

  size_t pos = 0;
  buf[pos++] = 10;  // len: 10 bytes of data follow
  writeU8(buf, &pos, PKT_TELEMETRY);
  writeU8(buf, &pos, DEVICE_ID);
  writeU16(buf, &pos, encodeGpsFrac(gps.lat));
  writeU16(buf, &pos, encodeGpsFrac(gps.lon));
  writeS16(buf, &pos, fusionAlt);
  writeU16(buf, &pos, buildStateFlags());
  return pos;  // 11
}

// Build one data page as a BLE record: [len][pageType][pageData...]
// len includes the type byte. Returns total bytes written (0 if not implemented or buf too small).
size_t buildDataPageRecord(uint8_t pageType, uint8_t* buf, size_t maxLen) {
  if (maxLen < 2) return 0;

  uint8_t stageBuf[32];
  size_t stagePos = 0;
  dispatchBuildPage(pageType, stageBuf, &stagePos);

  if (stagePos == 0) return 0;           // not implemented
  if (stagePos + 2 > maxLen) return 0;   // won't fit

  buf[0] = (uint8_t)(1 + stagePos);  // len = type byte + data bytes
  buf[1] = pageType;
  memcpy(buf + 2, stageBuf, stagePos);
  return 2 + stagePos;
}

// ===================== TELEMETRY PACKET =====================

size_t buildTelemetryPacket(uint8_t* buf) {
  // LoRa format: flat binary, no length-prefixed records.
  // Header fields written directly, followed by one data page.
  size_t pos = 0;

  writeU8(buf, &pos, PKT_TELEMETRY);
  writeU8(buf, &pos, DEVICE_ID);
  writeU16(buf, &pos, encodeGpsFrac(gps.lat));
  writeU16(buf, &pos, encodeGpsFrac(gps.lon));

  // Fusion altitude: fast baro EMA (MSL metres), GPS fallback during first ~50ms
  int16_t fusionAlt = -32768;
  if (flightState.baroFast.valid) {
    fusionAlt = clampToInt16(flightState.baroFast.valueCm / 100.0, true, -32768);
  } else if (gps.valid) {
    fusionAlt = clampToInt16(gps.alt, true, -32768);
  }
  writeS16(buf, &pos, fusionAlt);
  writeU16(buf, &pos, buildStateFlags());

  // Choose data page: force 0x0A after a command, otherwise round-robin
  uint8_t pageType;
  if (lastAck.pending) {
    pageType = 0x0A;
    lastAck.pending = false;
  } else {
    pageType = PAGE_CYCLE[pageIndex];
    pageIndex = (pageIndex + 1) % PAGE_CYCLE_COUNT;
  }

  writeU8(buf, &pos, pageType);
  dispatchBuildPage(pageType, buf, &pos);

  return pos;
}

// ===================== LOG WRITING =====================

void logDataPage(uint8_t pageType, const uint8_t* pageData, uint8_t pageDataLen) {
  if (!logStoreOk) return;
  uint8_t payload[1 + LOG_MAX_PAYLOAD];
  payload[0] = pageType;
  if (pageDataLen > LOG_MAX_PAYLOAD - 1) return;
  memcpy(payload + 1, pageData, pageDataLen);
  logStore.writeRecord(payload, 1 + pageDataLen, LOG_SNR_LOCAL, millis());
}

void logPage(LogPageIdx idx) {
  uint8_t buf[32];
  size_t pos = 0;
  uint8_t pageType = LOG_PAGE_TYPE[idx];
  dispatchBuildPage(pageType, buf, &pos);
  if (pos > 0) logDataPage(pageType, buf, (uint8_t)pos);
}

void logReceivedCommand(const uint8_t* pkt, size_t pktLen, int8_t snr) {
  if (!logStoreOk || pktLen == 0 || pktLen > LOG_MAX_PAYLOAD) return;
  logStore.writeRecord(pkt, (uint8_t)pktLen, snr, millis());
}

// ===================== NON-BLOCKING LOGGING =====================
// Iterates enabled log page types and writes any that are due.

void nonblockingLogging() {
  if (!logStoreOk) return;
  if (initState != INIT_DONE) return;

  unsigned long now = micros();
  for (int i = 0; i < LOGI_COUNT; i++) {
    if (logPages[i].intervalUs == 0) continue;
    if ((now - logPages[i].lastLogUs) < logPages[i].intervalUs) continue;
    logPages[i].lastLogUs = now;
    logPage((LogPageIdx)i);
  }
}

// ===================== SERIAL DEBUG =====================

void printPacketHex(const uint8_t* buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print("0");
    Serial.print(buf[i], HEX);
  }
}
