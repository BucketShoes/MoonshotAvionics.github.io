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

// ===================== INTEGER NMEA PARSERS =====================
// Replacing strtod/atof with integer arithmetic — each atof on soft-float Xtensa
// costs ~1.5-3ms; GGA alone had 4 of them, causing the recurring ~9-12ms GPS spike.

// Parse a decimal string into integer and fractional parts scaled to fracScale.
// e.g. "3.14" with fracScale=10000 → *intPart=3, *fracPart=1400, returns true.
// Handles missing or empty fields safely (returns false).
static bool parseDecimalParts(const char* s, int32_t* intPart, int32_t* fracPart, int32_t fracScale) {
  if (!s || s[0] == '\0') return false;
  bool neg = (*s == '-');
  if (neg) s++;
  int32_t ip = 0;
  while (*s >= '0' && *s <= '9') ip = ip * 10 + (*s++ - '0');
  int32_t fp = 0;
  if (*s == '.') {
    s++;
    int32_t scale = fracScale;
    while (*s >= '0' && *s <= '9' && scale > 1) {
      scale /= 10;
      fp += (*s++ - '0') * scale;
    }
  }
  if (neg) { ip = -ip; fp = -fp; }
  *intPart = ip;
  *fracPart = fp;
  return true;
}

// Parse NMEA lat/lon field (DDDMM.MMMMM or DDMM.MMMMM) + direction into degrees×1e7.
// Returns 0 on empty/invalid.
static int32_t nmeaCoordTo1e7(const char* raw, char dir) {
  if (!raw || raw[0] == '\0') return 0;
  // Find decimal point to split degrees vs minutes
  const char* dot = raw;
  while (*dot && *dot != '.') dot++;
  // Degrees are all chars before the last 2 digits before the dot
  int degDigits = (int)(dot - raw) - 2;
  if (degDigits < 1) return 0;
  int32_t deg = 0;
  for (int i = 0; i < degDigits; i++) deg = deg * 10 + (raw[i] - '0');
  // Minutes: 2 digits before dot + up to 5 after
  int32_t minInt = (raw[degDigits] - '0') * 10 + (raw[degDigits+1] - '0');
  int32_t minFrac = 0; // ×100000
  if (dot[0] == '.') {
    const char* p = dot + 1;
    int32_t scale = 100000;
    while (*p >= '0' && *p <= '9' && scale > 1) {
      scale /= 10;
      minFrac += (*p++ - '0') * scale;
    }
  }
  // degrees×1e7 = (deg + min/60) × 1e7 = deg×1e7 + min×1e7/60
  // min×1e7/60: minInt×166667 + minFrac×(1e7/60/1e5) = minFrac×100000/60 ≈ minFrac×1667
  int32_t result = deg * 10000000L + minInt * 166667L + (minFrac * 1667L) / 1000L;
  if (dir == 'S' || dir == 'W') result = -result;
  return result;
}

// Parse a small unsigned decimal like hdop/vdop/pdop into ×10 integer (e.g. "1.2" → 12).
// Returns 255 on empty/invalid (sentinel for "unknown").
static uint8_t parseX10(const char* s) {
  if (!s || s[0] == '\0') return 255;
  int32_t ip = 0, fp = 0;
  parseDecimalParts(s, &ip, &fp, 10);
  int32_t v = ip * 10 + fp;
  if (v < 0) return 255;
  return (uint8_t)(v > 254 ? 254 : v);
}

// Parse altitude string (metres, possibly fractional) into centimetres.
static int32_t parseAltCm(const char* s) {
  if (!s || s[0] == '\0') return 0;
  int32_t ip = 0, fp = 0;
  parseDecimalParts(s, &ip, &fp, 100);
  return ip * 100 + fp;
}

// Parse speed/course strings for RMC — kept as float since these aren't on the hot path.
// Speed is knots (rarely used in flight decisions), course is degrees.
// These fire once per second and only need one atof each — acceptable.
static float parseFloat(const char* s) {
  if (!s || s[0] == '\0') return 0.0f;
  return atof(s);
}

// ===================== NMEA FIELD SPLITTER =====================

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

// ===================== SENTENCE PARSERS =====================

static void parseGGA(const char* sentence) {
  char rawLat[12], latDir[4], rawLon[13], lonDir[4];
  char fixStr[4], satStr[4], hdopStr[8], altStr[12];

  nmeaField(sentence, 2, rawLat,  sizeof(rawLat));
  nmeaField(sentence, 3, latDir,  sizeof(latDir));
  nmeaField(sentence, 4, rawLon,  sizeof(rawLon));
  nmeaField(sentence, 5, lonDir,  sizeof(lonDir));
  nmeaField(sentence, 6, fixStr,  sizeof(fixStr));
  nmeaField(sentence, 7, satStr,  sizeof(satStr));
  nmeaField(sentence, 8, hdopStr, sizeof(hdopStr));
  nmeaField(sentence, 9, altStr,  sizeof(altStr));

  gps.fix  = fixStr[0] ? (fixStr[0] - '0') : 0;
  gps.sats = satStr[0] ? (uint8_t)atoi(satStr) : 0;

  int32_t lat1e7 = nmeaCoordTo1e7(rawLat, latDir[0]);
  int32_t lon1e7 = nmeaCoordTo1e7(rawLon, lonDir[0]);
  gps.lat   = lat1e7 * 1e-7;
  gps.lon   = lon1e7 * 1e-7;
  gps.hdop  = parseX10(hdopStr) * 0.1f;

  int32_t altCm = parseAltCm(altStr);
  gps.altCm = altCm;
  gps.alt   = altCm * 0.01;

  gps.valid = (gps.fix > 0);
  if (gps.valid) {
    gps.lastFixUs = micros();
    logPages[LOGI_GPS_POS].freshMask    |= 0xFF;
    logPages[LOGI_GPS_EXTRA].freshMask  |= 0xFF;
    logPages[LOGI_TIMESTAMP].freshMask  |= 0xFF;
  }
}

static void parseRMC(const char* sentence) {
  char statusStr[4], sogStr[12], cogStr[12], timeStr[16], dateStr[12];
  nmeaField(sentence, 1, timeStr,    sizeof(timeStr));
  nmeaField(sentence, 2, statusStr,  sizeof(statusStr));
  nmeaField(sentence, 7, sogStr,     sizeof(sogStr));
  nmeaField(sentence, 8, cogStr,     sizeof(cogStr));
  nmeaField(sentence, 9, dateStr,    sizeof(dateStr));
  gps.rmcValid = (statusStr[0] == 'A');
  // Speed and course: one atof each, once per second — acceptable on the non-critical path.
  if (sogStr[0] != '\0') gps.groundSpeedKnots = parseFloat(sogStr);
  if (cogStr[0] != '\0') gps.courseDeg        = parseFloat(cogStr);

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
    int32_t y0 = year - 1;
    int32_t leaps = (y0/4) - (y0/100) + (y0/400) - (1969/4 - 1969/100 + 1969/400);
    int32_t days = (year - 1970) * 365 + leaps;
    if (mo >= 1 && mo <= 12) days += mdays[mo-1];
    if (mo > 2 && ((year%4==0 && year%100!=0) || year%400==0)) days++;
    days += dd - 1;

    utcTimeMs = (uint64_t)days * 86400000ULL +
                (uint64_t)hh  * 3600000ULL  +
                (uint64_t)mm  * 60000ULL    +
                (uint64_t)ss  * 1000ULL     +
                (uint64_t)ms;
  }
}

static void parseGSA(const char* sentence) {
  char pdopStr[8], hdopStr[8], vdopStr[8];
  nmeaField(sentence, 15, pdopStr, sizeof(pdopStr));
  nmeaField(sentence, 16, hdopStr, sizeof(hdopStr));
  nmeaField(sentence, 17, vdopStr, sizeof(vdopStr));
  gps.pdop = parseX10(pdopStr) * 0.1f;
  gps.vdop = parseX10(vdopStr) * 0.1f;
}

static void parseNMEA(const char* sentence) {
  if (sentence[0] != '$' || sentence[1] == '\0' || sentence[2] == '\0' || sentence[3] == '\0') return;
  const char* id = sentence + 3;
  if      (id[0]=='G' && id[1]=='G' && id[2]=='A') parseGGA(sentence);
  else if (id[0]=='R' && id[1]=='M' && id[2]=='C') parseRMC(sentence);
  else if (id[0]=='G' && id[1]=='S' && id[2]=='A') parseGSA(sentence);
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
