// flight.h — Flight phase state machine for Moonshot rocket avionics
// Handles arming stability checks, launch detection, coast/apogee/deploy/landing.
// All functions are non-blocking, called from the main loop.
//
// Dependencies: sensors.h (accelData, gyroData, magData, baroData, sensorState)
//
// Usage:
//   #include "flight.h"
//   In loop: nonblockingFlight();
//   ARM command: flightTryArm(forceArm, params...)
//   DISARM command: flightDisarm()
//   Read state: flightState, flightPhase, etc.
//
// ╔═════════════════════════════════════════════════════════════════════╗
// ║                  *** SAFETY-CRITICAL CODE ***                       ║
// ║                                                                     ║
// ║  Phase transitions in this file directly trigger pyro charges and   ║
// ║  chute release mechanisms on a live rocket:                         ║
// ║                                                                     ║
// ║  PHASE_DROGUE     — fires pyro channel 1 (drogue chute ejection).   ║
// ║                     Early trigger: chute ejects while still         ║
// ║                     climbing — could explode into ground crew.      ║
// ║                     Late/missed: rocket descends ballistically,     ║
// ║                     potentially at hundreds of m/s.                 ║
// ║                                                                     ║
// ║  PHASE_MAIN_DEPLOY — fires pyro channel 2 and/or energises          ║
// ║                      nichrome wire or servo for main chute.         ║
// ║                      Nichrome stays hot while the GPIO is high —    ║
// ║                      if the main loop stalls it can burn through    ║
// ║                      structure. Missed deploy means fast descent.   ║
// ║                                                                     ║
// ║  REVIEW POLICY: any change to phase transition conditions,          ║
// ║  timing constants, EMA parameters, or the order of operations in    ║
// ║  nonblockingFlight() must be reviewed against the flight phase      ║
// ║  spec ("design documents/flight phases.txt") before committing.     ║
// ║                                                                     ║
// ║  If AI makes any changes which could effect the logic for any phase ║
// ║  transitions, or the data which is used to control them (such as    ║
// ║  around barometer logic), then the message MUST                     ║
// ║  end with "THIS TURN CHANGES FLIGHT PHASE LOGIC" with               ║
// ║  nothing after it. (before the end, describe the changes).          ║
// ║  Following turns should also check if previous turns made changes   ║
// ║  without a warning (e.g. if this note wasnt yet in context)         ║
// ╚═════════════════════════════════════════════════════════════════════╝

#ifndef FLIGHT_H
#define FLIGHT_H

#include <Arduino.h>
#include "sensors.h"

// ===================== FLIGHT PHASES =====================
// These values are used in the telemetry state flags [3:0]
enum FlightPhase : uint8_t {
  PHASE_IDLE        = 0,
  PHASE_ARMED       = 1,
  PHASE_BOOST       = 2,
  PHASE_COAST       = 3,
  PHASE_APOGEE      = 4,   // reserved/skipped — transitions straight to drogue
  PHASE_DROGUE      = 5,
  PHASE_MAIN_DEPLOY = 6,
  PHASE_DESCENT     = 7,
  PHASE_LANDED      = 8,
  PHASE_ERROR       = 15
};

// ===================== ARM RESULT CODES =====================
// Returned by flightTryArm() to give feedback in the command ack
#define ARM_OK                0x00
#define ARM_ERR_BARO          0x10  // baro sensor error or not ready
#define ARM_ERR_ACCEL         0x11  // accel sensor error or not ready
#define ARM_ERR_STABILITY     0x12  // sensors not stable long enough
#define ARM_ERR_ALREADY       0x13  // already armed (not an error, but can't re-arm without disarm)

// ===================== THRESHOLD TRACKER =====================
// Reusable struct for detecting sustained conditions:
// counts consecutive qualifying samples and tracks elapsed duration.
// Reset on any failing sample. "Triggered" when both sample count
// and duration thresholds are met simultaneously.

struct ThresholdTracker {
  uint16_t sampleCount;           // consecutive qualifying samples
  unsigned long firstQualifyUs;   // micros() of first qualifying sample in current run
  unsigned long lastQualifyUs;    // micros() of most recent qualifying sample

  void reset() {
    sampleCount = 0;
    firstQualifyUs = 0;
    lastQualifyUs = 0;
  }

  // Call when a sample qualifies. Returns true if trigger thresholds are now met.
  bool qualify(unsigned long nowUs, uint16_t minSamples, unsigned long minDurationUs) {
    if (sampleCount == 0) {
      firstQualifyUs = nowUs;
    }
    sampleCount++;
    lastQualifyUs = nowUs;
    return (sampleCount >= minSamples) &&
           ((nowUs - firstQualifyUs) >= minDurationUs);
  }

  // Call when a sample fails. Resets the tracker.
  void fail() {
    reset();
  }

  // Check if triggered without adding a sample
  bool isTriggered(uint16_t minSamples, unsigned long minDurationUs) const {
    if (sampleCount < minSamples) return false;
    return (lastQualifyUs - firstQualifyUs) >= minDurationUs;
  }

  // Duration since first qualifying sample (0 if no samples)
  unsigned long durationUs() const {
    if (sampleCount == 0) return 0;
    return lastQualifyUs - firstQualifyUs;
  }
};

// ===================== STABILITY STATE =====================
// Per-sensor stability tracking for arming checks

struct SensorStability {
  // Error tracking
  unsigned long lastErrorUs;      // micros() of last error on this sensor
  unsigned long lastReadingUs;    // micros() of last successful reading

  // Stability tracking (consecutive good readings)
  ThresholdTracker stable;

  void reset() {
    lastErrorUs = 0;
    lastReadingUs = 0;
    stable.reset();
  }

  // Record an error event
  void recordError(unsigned long nowUs) {
    lastErrorUs = nowUs;
    stable.reset();  // any error resets stability timer
  }

  // Record a successful reading that passes stability checks
  void recordStable(unsigned long nowUs) {
    lastReadingUs = nowUs;
    stable.qualify(nowUs, 1, 0);  // just accumulate; arming checks duration separately
  }

  // Record a successful reading that fails stability thresholds
  void recordUnstable(unsigned long nowUs) {
    lastReadingUs = nowUs;
    stable.reset();
  }

  // Has this sensor had an error in the last N microseconds?
  bool hasRecentError(unsigned long nowUs, unsigned long windowUs) const {
    if (lastErrorUs == 0) return false;
    return (nowUs - lastErrorUs) < windowUs;
  }

  // Has this sensor gone too long without a reading? (>1 second = error)
  bool isStale(unsigned long nowUs) const {
    if (lastReadingUs == 0) return true;  // never read
    return (nowUs - lastReadingUs) > 1000000UL;
  }
};

// ===================== FLIGHT CONFIG =====================
// Parameters from the ARM command (or defaults)

struct FlightConfig {
  // From ARM params (uint16 milli-g and uint16 whole meters)
  uint16_t boostAccelMg;     // total accel threshold for boost detect (pythagoras)
  uint16_t boostAltM;        // altitude AGL threshold for boost alt trigger
  uint16_t coastAltM;        // altitude AGL for coast alt trigger / alt-only launch
  uint16_t mainDeployAltM;   // altitude AGL for main chute deploy

  // Hardcoded stability thresholds (not configurable)
  // Arming: accel total 800-1200mg, single axis 700-1300mg
  // Arming: gyro < 50 deg/s per axis
  // Arming: baro within 10m of slow EMA, MSL between -1000m and +2500m
  // Arming: GPS good fix, speed<10m/s, hdop<30, sats>=5
};

// Default flight config (used when ARM has no params, or as fallback)
#define DEFAULT_BOOST_ACCEL_MG   3000
#define DEFAULT_BOOST_ALT_M      100
#define DEFAULT_COAST_ALT_M      200
#define DEFAULT_MAIN_DEPLOY_ALT_M 100

// Arming stability duration requirement
#define ARM_STABILITY_DURATION_US 10000000UL  // 10 seconds
#define ARM_ERROR_WINDOW_US       10000000UL  // errors in last 10s block arming

// ===================== BARO EMA STATE =====================
// Two EMAs: slow (30s period) for arming/calibration, fast (1s) for flight decisions

struct BaroEMA {
  float valueCm;        // current EMA value in cm MSL
  bool valid;
  unsigned long lastUs; // timestamp of last update

  void reset() { valid = false; valueCm = 0; lastUs = 0; }

  void update(float newCm, unsigned long nowUs, float tauUs) {
    if (!valid) {
      valueCm = newCm;
      valid = true;
      lastUs = nowUs;
      return;
    }
    float dt = (float)(nowUs - lastUs);
    if (dt < 100.0f) return;  // too fast, skip
    float alpha = dt / tauUs;
    if (alpha > 1.0f) alpha = 1.0f;
    valueCm += alpha * (newCm - valueCm);
    lastUs = nowUs;
  }
};

#define BARO_SLOW_TAU_US  30000000.0f   // 30 seconds
#define BARO_FAST_TAU_US   1000000.0f   // 1 second

// ===================== FLIGHT STATE =====================
// The main flight state struct. Exposed globally for telemetry access.

struct FlightState {
  FlightPhase phase;
  bool armed;

  FlightConfig config;

  // Orientation: true = X+ is up, false = X- is up. Locked at arming.
  bool orientXPositive;
  bool orientLocked;
  bool orientSeen;       // true once we've seen at least one accel reading (for consistency check)

  // Baro EMAs
  BaroEMA baroSlow;  // 30s period — for arming stability and ground level cal
  BaroEMA baroFast;  // 1s period  — for flight decisions (apogee, deploy, etc)

  // Previous fast baro EMA value (for apogee detection: new < old = descending)
  float prevBaroFastCm;
  bool prevBaroFastValid;


  // Sensor stability trackers
  SensorStability accelStab;
  SensorStability gyroStab;
  SensorStability baroStab;
  SensorStability gpsStab;     // tracked but not required for arming

  // Launch detection trackers
  ThresholdTracker launchAccelTracker;  // trigger B: sustained high accel
  ThresholdTracker launchAltTracker;    // trigger C: sustained high altitude (boost alt)
  ThresholdTracker launchCoastAltTracker; // trigger A: sustained coast altitude (alt-only)
  unsigned long lastAccelTriggerUs;     // time trigger B was last satisfied
  unsigned long lastAltTriggerUs;       // time trigger C was last satisfied

  // Coast detection trackers
  ThresholdTracker coastAltTracker;     // at coast altitude
  ThresholdTracker coastLowAccelTracker; // low acceleration

  // Phase timing
  unsigned long boostEntryUs;           // micros() when first entered boost or coast
  unsigned long apogeeEntryUs;          // micros() when entered drogue phase
  unsigned long mainDeployEntryUs;      // micros() when entered main_deploy phase
  unsigned long launchDetectUs;         // micros() of launch detection

  // Landing detection trackers (separate from arming trackers)
  ThresholdTracker landAccelX;
  ThresholdTracker landAccelY;
  ThresholdTracker landAccelZ;
  ThresholdTracker landGyro;
  ThresholdTracker landBaro;
  // Per-axis accel EMAs for landing (1s period)
  float landAccelEmaX, landAccelEmaY, landAccelEmaZ;
  bool landAccelEmaValid;
  unsigned long landAccelEmaLastUs;

  // Arm readiness flag (for telemetry)
  bool armReady;

  // ms since launch detect (-1 if not launched)
  int32_t msSinceLaunch;

  // ARM command flags byte layout:
  // bit 0: force arm (ignore stability/error checks)
  // bits 1-7: reserved
  uint8_t armFlags;
};

extern FlightState flightState;

// Convenience macro — matches the armed field in flightState.
#define isArmed (flightState.armed)

// ===================== PUBLIC API =====================

// Initialize flight state. Call once at startup.
void flightInit();

// Main non-blocking update. Call every loop iteration.
// Reads sensor data, updates EMAs, checks phase transitions.
void nonblockingFlight();

// Attempt to arm. Returns ARM_OK or ARM_ERR_* code.
// params: pointer to 9 bytes of ARM command params, or nullptr for defaults+force.
// paramsLen: 0 for legacy (defaults+force), 9 for full params.
uint8_t flightTryArm(const uint8_t* params, size_t paramsLen);

// Disarm. Always succeeds (caller must check pyro state before calling).
void flightDisarm();

// Get current flight phase (for telemetry state flags)
FlightPhase flightGetPhase();

// Get arm readiness (for telemetry)
bool flightIsArmReady();

// Get baro/accel error status (for telemetry flags)
bool flightBaroOk();
bool flightAccelOk();

#endif // FLIGHT_H
