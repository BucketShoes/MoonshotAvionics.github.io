// gps.cpp — GPS serial port, state, and NMEA parsing.
// Parses GGA (position/fix), RMC (speed/course/UTC time), GSA (VDOP).

#include <Arduino.h>
#include "gps.h"
#include "globals.h"
#include "telemetry.h"

// ===================== GPS STATE =====================

HardwareSerial gpsSerial(1);

GPSState gps = {0, 0, 0, 99.9f, 0, 0, false, 0,
                0, 0, false,
                99.9f, 99.9f,
                0};

char nmeaBuf[128];
uint8_t nmeaIdx = 0;

uint64_t utcTimeMs = 0;

// ===================== NMEA PARSING =====================

static bool nmeaField(const char* sentence, int fieldNum, char* out, size_t outSize) {
  out[0] = '\0';
  const char* p = sentence;
  int commas = 0;
  while (*p && commas < fieldNum) {
    if (*p == ',') commas++;
    p++;
  }
  if (!*p || commas < fieldNum) return false;
  size_t i = 0;
  while (*p && *p != ',' && *p != '*' && i < outSize - 1) {
    out[i++] = *p++;
  }
  out[i] = '\0';
  return (i > 0);
}

static double nmeaToDecimal(const char* raw, const char* dir) {
  if (raw[0] == '\0') return 0.0;
  double val = strtod(raw, NULL);
  int deg = (int)(val / 100.0);
  double min = val - (deg * 100.0);
  double result = deg + (min / 60.0);
  if (dir[0] == 'S' || dir[0] == 'W') result = -result;
  return result;
}

static void parseGGA(const char* sentence) {
  char rawLat[20], latDir[4], rawLon[20], lonDir[4];
  char fixStr[4], satStr[4], hdopStr[8], altStr[12];

  nmeaField(sentence, 2, rawLat, sizeof(rawLat));
  nmeaField(sentence, 3, latDir, sizeof(latDir));
  nmeaField(sentence, 4, rawLon, sizeof(rawLon));
  nmeaField(sentence, 5, lonDir, sizeof(lonDir));
  nmeaField(sentence, 6, fixStr, sizeof(fixStr));
  nmeaField(sentence, 7, satStr, sizeof(satStr));
  nmeaField(sentence, 8, hdopStr, sizeof(hdopStr));
  nmeaField(sentence, 9, altStr, sizeof(altStr));

  gps.fix  = atoi(fixStr);
  gps.sats = atoi(satStr);
  gps.hdop = atof(hdopStr);
  gps.lat  = nmeaToDecimal(rawLat, latDir);
  gps.lon  = nmeaToDecimal(rawLon, lonDir);
  gps.alt  = atof(altStr);
  gps.altCm = gps.alt * 100.0;
  gps.valid = (gps.fix > 0);
  if (gps.valid) {
    gps.lastFixUs = micros();
    logPages[LOGI_GPS_POS].freshMask |= 0xFF;
    logPages[LOGI_GPS_EXTRA].freshMask |= 0xFF;
    logPages[LOGI_TIMESTAMP].freshMask |= 0xFF;
  }
}

static void parseRMC(const char* sentence) {
  char statusStr[4], sogStr[12], cogStr[12], timeStr[16], dateStr[12];
  nmeaField(sentence, 1, timeStr, sizeof(timeStr));
  nmeaField(sentence, 2, statusStr, sizeof(statusStr));
  nmeaField(sentence, 7, sogStr, sizeof(sogStr));
  nmeaField(sentence, 8, cogStr, sizeof(cogStr));
  nmeaField(sentence, 9, dateStr, sizeof(dateStr));
  gps.rmcValid = (statusStr[0] == 'A');
  if (sogStr[0] != '\0') gps.groundSpeedKnots = atof(sogStr);
  if (cogStr[0] != '\0') gps.courseDeg = atof(cogStr);

  // Extract UTC time from RMC: time=hhmmss.ss date=ddmmyy
  if (gps.rmcValid && timeStr[0] != '\0' && dateStr[0] != '\0' && strlen(dateStr) >= 6) {
    int hh = (timeStr[0]-'0')*10 + (timeStr[1]-'0');
    int mm = (timeStr[2]-'0')*10 + (timeStr[3]-'0');
    int ss = (timeStr[4]-'0')*10 + (timeStr[5]-'0');
    int ms = 0;
    if (timeStr[6] == '.') ms = atoi(&timeStr[7]) * (strlen(&timeStr[7])==1 ? 100 : strlen(&timeStr[7])==2 ? 10 : 1);
    int dd = (dateStr[0]-'0')*10 + (dateStr[1]-'0');
    int mo = (dateStr[2]-'0')*10 + (dateStr[3]-'0');
    int yy = (dateStr[4]-'0')*10 + (dateStr[5]-'0');
    int year = 2000 + yy;

    static const uint16_t mdays[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    // Days since 1970-01-01 using closed-form leap-year count (no loop).
    int32_t y0 = year - 1;
    int32_t leaps = (y0/4) - (y0/100) + (y0/400) - (1969/4 - 1969/100 + 1969/400);
    int32_t days = (year - 1970) * 365 + leaps;
    if (mo >= 1 && mo <= 12) days += mdays[mo-1];
    if (mo > 2 && ((year%4==0 && year%100!=0) || year%400==0)) days++;
    days += dd - 1;

    utcTimeMs = (uint64_t)days * 86400000ULL +
                (uint64_t)hh * 3600000ULL +
                (uint64_t)mm * 60000ULL +
                (uint64_t)ss * 1000ULL +
                (uint64_t)ms;
  }
}

static void parseGSA(const char* sentence) {
  char pdopStr[8], hdopStr[8], vdopStr[8];
  nmeaField(sentence, 15, pdopStr, sizeof(pdopStr));
  nmeaField(sentence, 16, hdopStr, sizeof(hdopStr));
  nmeaField(sentence, 17, vdopStr, sizeof(vdopStr));
  if (pdopStr[0] != '\0') gps.pdop = atof(pdopStr);
  if (vdopStr[0] != '\0') gps.vdop = atof(vdopStr);
}

static void parseNMEA(const char* sentence) {
  if (sentence[0] != '$' || strlen(sentence) < 6) return;
  const char* id = sentence + 3;
  if      (strncmp(id, "GGA", 3) == 0) parseGGA(sentence);
  else if (strncmp(id, "RMC", 3) == 0) parseRMC(sentence);
  else if (strncmp(id, "GSA", 3) == 0) parseGSA(sentence);
}

// ===================== NONBLOCKING UPDATE =====================

void nonblockingGPS() {
  if (!gpsUartStarted) return;
  // Read all available bytes in one call — one mutex acquisition vs one per byte.
  // available()+read() each take a FreeRTOS mutex; the uart_event_task on the other
  // core holds it intermittently, causing multi-ms stalls per byte under poor signal.
  uint8_t chunk[64];
  int n;
  while ((n = gpsSerial.read(chunk, sizeof(chunk))) > 0) {
    for (int i = 0; i < n; i++) {
      char c = (char)chunk[i];
      if (c == '$') nmeaIdx = 0;
      if (nmeaIdx < sizeof(nmeaBuf) - 1) nmeaBuf[nmeaIdx++] = c;
      if (c == '\n') {
        nmeaBuf[nmeaIdx] = '\0';
        parseNMEA(nmeaBuf);
        nmeaIdx = 0;
      }
    }
  }
}
