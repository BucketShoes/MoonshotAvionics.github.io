// globals.h — extern declarations for main-owned cross-cutting state.
// These globals are defined in rocket_avionics.ino and accessed by multiple modules.
// Include this wherever you need to read or write system-level state.

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "log_store.h"

// ===================== INIT STATE MACHINE =====================

enum InitState {
  INIT_VEXT_ON,
  INIT_GPS_RST_LOW,
  INIT_GPS_RST_HIGH,
  INIT_GPS_UART,
  INIT_LORA,
  INIT_NVS,
  INIT_SENSORS,
  INIT_DONE
};

extern InitState initState;
extern bool gpsUartStarted;

// ===================== NVS + LOG STORAGE =====================

extern Preferences nvs;
extern LogStore logStore;
extern bool logStoreOk;

// ===================== BATTERY =====================

extern uint16_t batteryMv;

// ===================== PEAK VALUES =====================

struct PeakValues {
  int32_t  maxAltCmMSL;
  uint16_t maxAccel100g;
  int16_t  maxVvel10;
};

extern PeakValues peaks;

// ===================== ARM STATE =====================

// Record counter at arming — used as reference for log download relative offsets.
extern uint32_t armRecordCounter;

// ===================== TX CONFIG =====================

extern bool txSendingEnabled;   // toggled by button (not stored to NVS)
extern int8_t activeTxRate;     // NVS-backed rate; positive=Hz, negative=seconds
extern unsigned long txIntervalUs;

// ===================== LOOP TIMING =====================

extern unsigned long runningMaxLoopUs;  // all-time worst loop iteration (reset on arm)

#endif // GLOBALS_H
