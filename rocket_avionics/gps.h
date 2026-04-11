// gps.h — GPS state and NMEA parsing for rocket avionics.
// Owns the GPS serial port object and all parsed GPS state.
// Changes to GPS parsing, state fields, or NMEA handling only touch gps.h + gps.cpp.

#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

// ===================== GPS STATE =====================

struct GPSState {
  double lat;
  double lon;
  double alt;         // metres MSL
  float  hdop;
  uint8_t sats;
  uint8_t fix;        // 0=none, 1=GPS, 2=DGPS
  bool   valid;
  unsigned long lastFixUs;

  float  groundSpeedKnots;
  float  courseDeg;
  bool   rmcValid;

  float  vdop;
  float  pdop;

  double altCm;       // cm MSL full resolution
};

extern GPSState gps;
extern HardwareSerial gpsSerial;
extern uint64_t utcTimeMs;  // UTC ms since 1970-01-01 from RMC; 0 = no valid time

// ===================== PUBLIC API =====================

void nonblockingGPS();

#endif // GPS_H
