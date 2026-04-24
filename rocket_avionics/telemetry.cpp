// telemetry.cpp — Telemetry packet building, data page assembly, and flash log writing.
// See telemetry.h for the public API.

#include <Arduino.h>
#include "telemetry.h"
#include "gps.h"
#include "sensors.h"
#include "flight.h"
#include "pyro.h"
#include "commands.h"
#include "radio.h"
#include "globals.h"

// ===================== LOG PAGE CONFIG =====================
//back reference index
static const uint8_t LOG_PAGE_TYPE[] = {
  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
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
  { 0,       0, 0 },  // Thrust curve: intervalUs=0, never written to flash
  { 1000000, 0, 0 },  // Pyro status
};

// ===================== PAGE CYCLE =====================
// Order in which data pages rotate in regular telemetry packets.
static const uint8_t PAGE_CYCLE[] = {
  0x01, 0x02, 0x03, 0x04, 0x05, 
  0x06, 0x08, 0x09, 0x0A, 
  0x0B, 0x0C, 0x0D, 0x0F,
};

static const uint8_t PAGE_CYCLEbaros[] = {
  0x01, 0x02, 0x03, 0x02, 0x04, 0x02, 0x05, 0x02,
  0x06, 0x02, 0x08, 0x02, 0x09, 0x02, 0x0A, 0x02,
  0x0B, 0x02, 0x0C, 0x02, 0x0D, 0x02, 0x0F
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
  if (pyroState.ch1Fired) flags |= (1 << 5);    // [5] pyro ch1 fired
  if (pyroState.ch2Fired) flags |= (1 << 6);    // [6] pyro ch2 fired
  if (pyroState.ch3Fired) flags |= (1 << 7);    // [7] pyro ch3 fired
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

// Page 0x0A: Command ack + signal quality (15 bytes)
static void buildPage0A(uint8_t* buf, size_t* pos) {
  writeU32(buf, pos, lastAck.nonce);
  writeU8(buf, pos, lastAck.result);
  writeU8(buf, pos, (uint8_t)lastAck.rssi);
  writeU8(buf, pos, (uint8_t)lastAck.snr);
  writeU16(buf, pos, lastAck.invalidHmacCount);
  writeU32(buf, pos, lastAck.rxTimeUs);
  writeU16(buf, pos, lastAck.rxPosInSlot);
}

// Page 0x0B: Flight status (6 bytes)
static void buildPage0B(uint8_t* buf, size_t* pos) {
  writeS32(buf, pos, flightState.msSinceLaunch);
  uint16_t pyroFlags = 0;
  if (pyroState.ch1Fired) pyroFlags |= (1 << 0);
  if (pyroState.ch2Fired) pyroFlags |= (1 << 1);
  if (pyroState.ch3Fired) pyroFlags |= (1 << 2);
  if (pyroState.ch1Continuity) pyroFlags |= (1 << 4);
  if (pyroState.ch2Continuity) pyroFlags |= (1 << 5);
  if (pyroState.ch3Continuity) pyroFlags |= (1 << 6);
  if (pyroState.hvPresent)     pyroFlags |= (1 << 8);
  writeU16(buf, pos, pyroFlags);
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

// Page 0x0F: Pyro status (6 bytes)
// uint8 flags: [0]=ch1_continuity [1]=ch2_continuity [2]=ch3_continuity [3]=hv_present
//              [4]=ch1_fired [5]=ch2_fired [6]=ch3_fired [7]=any_active (CPU estimate)
// uint8 active_channel (0/1/2/3)
// uint16 active_duration_ms
// uint16 hv_millivolts
static void buildPage0F(uint8_t* buf, size_t* pos) {
  uint8_t flags = 0;
  if (pyroState.ch1Continuity) flags |= (1 << 0);
  if (pyroState.ch2Continuity) flags |= (1 << 1);
  if (pyroState.ch3Continuity) flags |= (1 << 2);
  if (pyroState.hvPresent)     flags |= (1 << 3);
  if (pyroState.ch1Fired)      flags |= (1 << 4);
  if (pyroState.ch2Fired)      flags |= (1 << 5);
  if (pyroState.ch3Fired)      flags |= (1 << 6);
  if (pyroState.activeChannel) flags |= (1 << 7);
  writeU8(buf, pos, flags);
  writeU8(buf, pos, pyroState.activeChannel);
  uint16_t durMs = (pyroState.fireDurationUs > 0)
    ? (uint16_t)constrain(pyroState.fireDurationUs / 1000UL, 0UL, 65535UL) : 0;
  writeU16(buf, pos, durMs);
  writeU16(buf, pos, pyroState.hvMillivolts);
}

// Page 0x0E: Thrust curve — X-axis acceleration ring buffer snapshot (variable length)
// Wire format (after type byte):
//   uint16  durationMs  — window span in milliseconds
//   int16   minAccel8   — min sample in 8mg units (actual mg = val × 8)
//   int16   maxAccel8   — max sample in 8mg units
//   uint8[] samples     — N bytes; count inferred from record/packet length
//                         each = ((val_mg − min8×8) × 255) / ((max8 − min8) × 8)
// Two-pass ring walk: pass 1 finds min/max, pass 2 encodes. No intermediate buffer.
static void buildPage0E(uint8_t* buf, size_t* pos, uint16_t numSamples) {
  const unsigned long MAX_WINDOW_US =
    (unsigned long)THRUST_BUF_SIZE * 1000000UL / THRUST_SAMPLE_RATE_HZ;

  // Determine window duration and sample count
  unsigned long windowUs = MAX_WINDOW_US;

  if (flightState.thrustCoastEntryUs != 0) {
    // Coast snapshot: window = from (boostEntry - 1s) to coastEntry, or full buffer
    if (flightState.thrustViaBoost && flightState.boostEntryUs > 0) {
      unsigned long startUs = (flightState.boostEntryUs > 1000000UL)
                              ? flightState.boostEntryUs - 1000000UL : 0;
      unsigned long coast = flightState.thrustCoastEntryUs;
      windowUs = (coast > startUs) ? (coast - startUs) : MAX_WINDOW_US;
    }
    // else direct coast (PAD_READY→COAST): use full buffer
    if (windowUs > MAX_WINDOW_US) windowUs = MAX_WINDOW_US;
  }
  // else rolling subscription: full buffer

  uint16_t windowSamples = (uint16_t)(windowUs * THRUST_SAMPLE_RATE_HZ / 1000000UL);
  if (windowSamples > THRUST_BUF_SIZE) windowSamples = THRUST_BUF_SIZE;
  if (windowSamples == 0) windowSamples = 1;

  uint16_t N = (numSamples < windowSamples) ? numSamples : windowSamples;
  if (N == 0) N = 1;

  uint16_t durationMs = (uint16_t)(windowUs / 1000UL);

  // --- Pass 1: find min/max ---
  int16_t min16 = 0, max16 = 0;
  bool first = true;
  uint16_t step = (N > 1) ? (windowSamples / N) : 1;
  if (step == 0) step = 1;
  for (uint16_t i = 0; i < N; i++) {
    uint16_t ringIdx = (uint16_t)((thrustBufHead + THRUST_BUF_SIZE - windowSamples + i * step) % THRUST_BUF_SIZE);
    int16_t val = thrustBuf[ringIdx];
    if (!flightState.orientXPositive) val = -val;
    if (first) { min16 = max16 = val; first = false; }
    else { if (val < min16) min16 = val; if (val > max16) max16 = val; }
  }
  // Flat-range guard: avoid division by zero in encoding
  if (max16 == min16) max16 = min16 + 1;

  // Header: durationMs, min8, max8 (8mg units)
  writeU16(buf, pos, durationMs);
  writeS16(buf, pos, (int16_t)(min16 / 8));
  writeS16(buf, pos, (int16_t)(max16 / 8));

  // --- Pass 2: encode ---
  int32_t range8 = (int32_t)(max16 / 8) - (int32_t)(min16 / 8);
  for (uint16_t i = 0; i < N; i++) {
    uint16_t ringIdx = (uint16_t)((thrustBufHead + THRUST_BUF_SIZE - windowSamples + i * step) % THRUST_BUF_SIZE);
    int16_t val = thrustBuf[ringIdx];
    if (!flightState.orientXPositive) val = -val;
    int32_t encoded = ((int32_t)val - (int32_t)(min16 / 8) * 8) * 255 / (range8 * 8);
    if (encoded < 0) encoded = 0;
    if (encoded > 255) encoded = 255;
    buf[(*pos)++] = (uint8_t)encoded;
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
    case 0x0F: buildPage0F(buf, pos); break;
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

  // Page 0x0E (thrust curve): variable-length, bypasses stageBuf.
  // numSamples chosen based on current TX PHY: 489 on 1M/2M, 200 on Coded.
  // ATT limit is 517; record header is 2 bytes (len+type), payload header is 6 bytes.
  if (pageType == PAGE_THRUST_CURVE) {
    uint16_t n = (bleTxPhy == 3) ? 200 : 210;// 489;
    // record: [len u8][0x0E][6-byte header][n bytes samples] = 2 + 6 + n
    if (maxLen < (size_t)(2 + 6 + n)) return 0;
    size_t recPos = 0;
    buf[recPos++] = 0;        // len placeholder
    buf[recPos++] = 0x0E;
    buildPage0E(buf, &recPos, n);
    buf[0] = (uint8_t)(recPos - 1);  // len = everything after the len byte
    return recPos;
  }

  uint8_t stageBuf[255];  // large enough for any standard page (max 254 bytes payload)
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

  // Choose data page: cmd ack > thrust curve force > round-robin
  uint8_t pageType;
  uint16_t thrustSamples = 0;
  if (lastAck.pending) {
    pageType = 0x0A;
    lastAck.pending = false;
  } else if (thrustLoraForce) {
    pageType = PAGE_THRUST_CURVE;
    thrustLoraForce = false;
    // Resume ring after send: active if armed OR BLE subscribed to 0x0E
    thrustBufActive = isArmed || ((bleSubPageMask & (1ULL << PAGE_THRUST_CURVE)) != 0);
    // Sample count depends on SF: SF≤7 = 200, SF8 = 100, SF9+ = 50
    thrustSamples = (activeSF <= 7) ? 200 : (activeSF == 8) ? 100 : 50;
  } else {
    pageType = PAGE_CYCLE[pageIndex];
    pageIndex = (pageIndex + 1) % PAGE_CYCLE_COUNT;
  }

  writeU8(buf, &pos, pageType);
  if (pageType == PAGE_THRUST_CURVE) {
    buildPage0E(buf, &pos, thrustSamples);
  } else {
    dispatchBuildPage(pageType, buf, &pos);
  }

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

// ===================== NON-BLOCKING THRUST CAPTURE =====================
// Samples accel.x into the ring buffer at THRUST_SAMPLE_RATE_HZ.
// Only runs when thrustBufActive. Non-blocking — just a timestamp check.

void nonblockingThrust() {
  if (!thrustBufActive) return;
  if (!accelData.valid) return;

  static unsigned long thrustNextUs = 0;
  unsigned long now = micros();
  const unsigned long THRUST_CAPTURE_US = 1000000UL / THRUST_SAMPLE_RATE_HZ;  // 5000µs @ 200Hz

  if ((now - thrustNextUs) < THRUST_CAPTURE_US) return;
  thrustNextUs += THRUST_CAPTURE_US;
  // Catch-up guard: if we're more than one period behind, reset (e.g. after long sleep)
  if ((long)(now - thrustNextUs) > (long)THRUST_CAPTURE_US) thrustNextUs = now;

  thrustBuf[thrustBufHead] = accelData.x;
  thrustBufHead = (uint16_t)((thrustBufHead + 1) % THRUST_BUF_SIZE);
  logPages[LOGI_THRUST_CURVE].freshMask = 0xFF;  // fresh for all transports
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
