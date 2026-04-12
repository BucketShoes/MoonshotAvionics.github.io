// flight.cpp — Flight phase state machine implementation
// See flight.h for API documentation and struct definitions.

#include "flight.h"

// ===================== GLOBAL STATE =====================

FlightState flightState;

// ===================== HELPERS =====================

// Total acceleration magnitude in milli-g (pythagoras of all axes)
static uint32_t accelTotalMg() {
  int32_t ax = accelData.x;
  int32_t ay = accelData.y;
  int32_t az = accelData.z;
  float magSq = (float)(ax*ax + ay*ay + az*az);
  return (uint32_t)sqrtf(magSq);
}

// Current baro altitude AGL in whole meters (from baroData)
static int32_t baroAltAglM() {
  if (!baroData.valid || !baroData.altCalibrated) return 0;
  return baroData.altCmAGL / 100;
}

// Current baro altitude MSL in cm
static int32_t baroAltMslCm() {
  if (!baroData.valid) return 0;
  return baroData.altCmMSL;
}

// ===================== INIT =====================

void flightInit() {
  memset(&flightState, 0, sizeof(FlightState));
  flightState.phase = PHASE_IDLE;
  flightState.armed = false;
  flightState.orientLocked = false;
  flightState.prevBaroFastValid = false;

  flightState.baroSlow.reset();
  flightState.baroFast.reset();

  flightState.accelStab.reset();
  flightState.gyroStab.reset();
  flightState.baroStab.reset();
  flightState.gpsStab.reset();

  flightState.config.boostAccelMg = DEFAULT_BOOST_ACCEL_MG;
  flightState.config.boostAltM = DEFAULT_BOOST_ALT_M;
  flightState.config.coastAltM = DEFAULT_COAST_ALT_M;
  flightState.config.mainDeployAltM = DEFAULT_MAIN_DEPLOY_ALT_M;

  flightState.msSinceLaunch = -1;
  flightState.armReady = false;
  flightState.landAccelEmaValid = false;
}

// ===================== STABILITY CHECKS =====================
// Called every loop to update sensor stability trackers.
// These run in ALL phases (idle and armed) so arm_ready is always current.

static void updateAccelStability(unsigned long nowUs) {
  SensorStability& s = flightState.accelStab;

  // Check for stale readings (>1s = error)
  if (s.isStale(nowUs) && accelData.lastReadUs > 0) {
    s.recordError(nowUs);
    return;
  }

  if (!accelData.valid) return;
  if (accelData.lastReadUs == s.lastReadingUs) return;  // no new data

  uint32_t totalMg = accelTotalMg();
  int16_t ax = accelData.x;

  // Total accel must be 800-1200 mg
  bool totalOk = (totalMg >= 800 && totalMg <= 1200);

  // One axis (X+ or X-) must be 700-1300 mg (detecting gravity = rocket is vertical)
  bool xPosOk = (ax >= 700 && ax <= 1300);
  bool xNegOk = (ax >= -1300 && ax <= -700);
  bool axisOk = xPosOk || xNegOk;

  // Orientation consistency: must be same as previous loop
  bool currentOrientPositive = (ax > 0);
  if (flightState.orientLocked) {
    // While armed, orientation is locked — just check
    if (currentOrientPositive != flightState.orientXPositive) {
      axisOk = false;  // wrong orientation for stability
    }
  } else {
    // Not armed yet: track orientation, but require consistency
    if (flightState.orientSeen) {
      // We've seen accel before; check if orientation flipped
      if (currentOrientPositive != flightState.orientXPositive) {
        axisOk = false;
        s.recordUnstable(nowUs);
        flightState.orientXPositive = currentOrientPositive;
        return;
      }
    }
    flightState.orientXPositive = currentOrientPositive;
    flightState.orientSeen = true;
  }

  if (totalOk && axisOk) {
    s.recordStable(nowUs);
  } else {
    s.recordUnstable(nowUs);
  }
}

static void updateGyroStability(unsigned long nowUs) {
  SensorStability& s = flightState.gyroStab;

  if (s.isStale(nowUs) && gyroData.lastReadUs > 0) {
    s.recordError(nowUs);
    return;
  }

  if (!gyroData.valid) return;
  if (gyroData.lastReadUs == s.lastReadingUs) return;

  // Each axis must be under ±50 deg/s. gyroData is in 0.1 deg/s, so ±500 units.
  bool ok = (abs(gyroData.x) < 500) && (abs(gyroData.y) < 500) && (abs(gyroData.z) < 500);

  if (ok) {
    s.recordStable(nowUs);
  } else {
    s.recordUnstable(nowUs);
  }
}

static void updateBaroStability(unsigned long nowUs) {
  SensorStability& s = flightState.baroStab;

  if (s.isStale(nowUs) && baroData.lastReadUs > 0) {
    s.recordError(nowUs);
    return;
  }

  if (!baroData.valid) return;
  if (baroData.lastReadUs == s.lastReadingUs) return;

  float altMsl = (float)baroData.altCmMSL / 100.0f;  // metres MSL

  // MSL must be between -1000m and +2500m
  bool rangeOk = (altMsl >= -1000.0f && altMsl <= 2500.0f);

  // Must be within 10m of the slow baro EMA
  bool emaOk = true;
  if (flightState.baroSlow.valid) {
    float emaMsl = flightState.baroSlow.valueCm / 100.0f;
    float diff = fabsf(altMsl - emaMsl);
    emaOk = (diff <= 10.0f);
  }

  if (rangeOk && emaOk) {
    s.recordStable(nowUs);
  } else {
    s.recordUnstable(nowUs);
  }
}

static void updateGpsStability(unsigned long nowUs) {
  SensorStability& s = flightState.gpsStab;
//TODO: @@@@ GPS stability isnt actually checked yet.

  // GPS is tracked for errors but not required for arming.
  // "Stale" for GPS is also 1 second without a reading, but only track if GPS has started.
  // GPS doesn't have a .lastReadUs in the same way — we use gps validity.
  // For now, just track whether GPS is in a good state when we have data.
  // GPS stability is informational only (not blocking arming).

  // We don't have a direct lastReadUs for GPS in the same struct.
  // Skip stale check for GPS — it's advisory only.
  // (GPS data comes from NMEA parsing, timing is less predictable)
}

// ===================== BARO EMA UPDATES =====================

// EMAs run every flight loop with the latest baro reading.
// Their purpose is to estimate altitude *between* sensor readings and filter
// noise — they should keep moving toward the last known value even when no
// new sample has arrived. prevBaroFastCm is saved each loop so apogee
// detection compares the last two loop values of the EMA.
static void updateBaroEMAs(unsigned long nowUs) {
  if (!baroData.valid) return;

  float altCm = (float)baroData.altCmMSL;

  flightState.baroSlow.update(altCm, nowUs, BARO_SLOW_TAU_US);
  flightState.baroFast.update(altCm, nowUs, BARO_FAST_TAU_US);

  logPages[LOGI_FLIGHT_STATUS].freshMask |= 0xFF;
}

// ===================== ARM READINESS =====================
// Checks whether arming would succeed right now (for telemetry flag).
// Does NOT modify state — pure read.

static bool checkArmReady(unsigned long nowUs) {
  // Baro sensor must be ok and have no recent errors
  if (!sensorState.baroOk) return false;
  if (flightState.baroStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return false;
  if (flightState.baroStab.isStale(nowUs)) return false;

  // Accel sensor must be ok and have no recent errors
  if (!sensorState.accelOk) return false;
  if (flightState.accelStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return false;
  if (flightState.accelStab.isStale(nowUs)) return false;

  // Gyro/mag/GPS: track errors but don't block arming
  // (gyro errors are tracked for information only per spec)

  // All required sensors must be stable for the duration
  if (flightState.accelStab.stable.durationUs() < ARM_STABILITY_DURATION_US) return false;
  if (flightState.baroStab.stable.durationUs() < ARM_STABILITY_DURATION_US) return false;
  // Gyro must be stable too (stability check, not error check)
  if (sensorState.gyroOk) {
    if (flightState.gyroStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return false;
    if (flightState.gyroStab.stable.durationUs() < ARM_STABILITY_DURATION_US) return false;
  }

  return true;
}

// ===================== PHASE TRANSITIONS =====================

static void enterPhase(FlightPhase newPhase, unsigned long nowUs) {
  FlightPhase oldPhase = flightState.phase;
  flightState.phase = newPhase;

  Serial.print("FLIGHT: ");
  static const char* phaseNames[] = {
    "IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
    "DROGUE", "MAIN_DEPLOY", "DESCENT", "LANDED"
  };
  if (oldPhase <= 8) { Serial.print(phaseNames[oldPhase]); }
  else { Serial.print(oldPhase); }
  Serial.print(" -> ");
  if (newPhase <= 8) { Serial.println(phaseNames[newPhase]); }
  else { Serial.println(newPhase); }
}

// --- Launch detection (ARMED -> BOOST) ---
// Two paths:
//   A: Over coast altitude AGL for 4 samples / 250ms -> immediate launch
//   B+C: High accel (5 samples/200ms) AND high altitude (4 samples/250ms) within 5s window

static void checkLaunchDetect(unsigned long nowUs) {
  if (!accelData.valid || !baroData.valid || !baroData.altCalibrated) return;

  int32_t altAglM = baroAltAglM();
  uint32_t totalMg = accelTotalMg();

  // Trigger A: altitude above coast altitude AGL (alt-only launch detect)
  if (altAglM >= (int32_t)flightState.config.coastAltM) {
    if (flightState.launchCoastAltTracker.qualify(nowUs, 4, 250000UL)) {
      // Alt-only trigger: immediate launch detect
      flightState.launchDetectUs = nowUs;
      flightState.boostEntryUs = nowUs;
      enterPhase(PHASE_BOOST, nowUs);
      return;
    }
  } else {
    flightState.launchCoastAltTracker.fail();
  }

  // Trigger B: high acceleration (total g > boost threshold)
  if (totalMg >= flightState.config.boostAccelMg) {
    if (flightState.launchAccelTracker.qualify(nowUs, 5, 200000UL)) {
      flightState.lastAccelTriggerUs = nowUs;
    }
  } else {
    flightState.launchAccelTracker.fail();
  }

  // Trigger C: altitude above boost altitude AGL
  if (altAglM >= (int32_t)flightState.config.boostAltM) {
    if (flightState.launchAltTracker.qualify(nowUs, 4, 250000UL)) {
      flightState.lastAltTriggerUs = nowUs;
    }
  } else {
    flightState.launchAltTracker.fail();
  }

  // B+C combination: both triggered within 5 seconds of each other
  bool bTriggered = flightState.launchAccelTracker.isTriggered(5, 200000UL);
  bool cTriggered = flightState.launchAltTracker.isTriggered(4, 250000UL);

  if (bTriggered && cTriggered) {
    // Check 5-second window between the two triggers
    bool bRecent = (flightState.lastAccelTriggerUs > 0) &&
                   (nowUs - flightState.lastAccelTriggerUs) < 5000000UL;
    bool cRecent = (flightState.lastAltTriggerUs > 0) &&
                   (nowUs - flightState.lastAltTriggerUs) < 5000000UL;

    if (bRecent && cRecent) {
      flightState.launchDetectUs = nowUs;
      flightState.boostEntryUs = nowUs;
      enterPhase(PHASE_BOOST, nowUs);
      return;
    }
  }

  // Also: if B is triggered and C was recent (within 5s), or vice versa
  if (bTriggered && flightState.lastAltTriggerUs > 0 &&
      (nowUs - flightState.lastAltTriggerUs) < 5000000UL) {
    flightState.launchDetectUs = nowUs;
    flightState.boostEntryUs = nowUs;
    enterPhase(PHASE_BOOST, nowUs);
    return;
  }
  if (cTriggered && flightState.lastAccelTriggerUs > 0 &&
      (nowUs - flightState.lastAccelTriggerUs) < 5000000UL) {
    flightState.launchDetectUs = nowUs;
    flightState.boostEntryUs = nowUs;
    enterPhase(PHASE_BOOST, nowUs);
    return;
  }
}

// --- Coast detection (BOOST -> COAST, or ARMED -> COAST) ---
// (in boost OR at coast altitude) AND low accel (<2000mg for 5 samples/500ms)

static void checkCoastDetect(unsigned long nowUs) {
  if (!accelData.valid) return;

  uint32_t totalMg = accelTotalMg();
  int32_t altAglM = baroAltAglM();
  bool inBoost = (flightState.phase == PHASE_BOOST);

  // Coast altitude check (can substitute for boost phase)
  bool atCoastAlt = false;
  if (baroData.valid && baroData.altCalibrated) {
    if (altAglM >= (int32_t)flightState.config.coastAltM) {
      if (flightState.coastAltTracker.qualify(nowUs, 4, 250000UL)) {
        atCoastAlt = true;
      }
    } else {
      flightState.coastAltTracker.fail();
    }
  }

  // Low acceleration check
  if (totalMg < 2000) {
    if (flightState.coastLowAccelTracker.qualify(nowUs, 5, 500000UL)) {
      // Low accel confirmed — check if we can transition to coast
      if (inBoost || atCoastAlt) {
        if (flightState.boostEntryUs == 0) flightState.boostEntryUs = nowUs;
        enterPhase(PHASE_COAST, nowUs);
        return;
      }
    }
  } else {
    flightState.coastLowAccelTracker.fail();
  }
}

// --- Apogee detection (COAST -> DROGUE) ---
// Fast baro EMA (1s period) is falling vs its value from the previous loop.
// prevBaroFastCm is saved at the top of nonblockingFlight(), before the EMA
// update, so this is always a genuine loop-to-loop comparison.
// 3s minimum after boost/coast entry covers baro noise during boost and the
// supersonic-to-subsonic pressure spike right after burnout.

static void checkApogeeDetect(unsigned long nowUs) {
  if (flightState.boostEntryUs == 0) return;
  if ((nowUs - flightState.boostEntryUs) < 3000000UL) return;

  if (!flightState.baroFast.valid || !flightState.prevBaroFastValid) return;

  if (flightState.baroFast.valueCm <= flightState.prevBaroFastCm) {
    flightState.apogeeEntryUs = nowUs;
    enterPhase(PHASE_DROGUE, nowUs);
  }
}

// --- Main deploy detection (DROGUE -> MAIN_DEPLOY) ---
// At least 3 seconds after apogee, AND fast baro EMA below main deploy alt AGL

static void checkMainDeploy(unsigned long nowUs) {
  if (flightState.apogeeEntryUs == 0) return;
  if ((nowUs - flightState.apogeeEntryUs) < 3000000UL) return;

  if (!flightState.baroFast.valid || !baroData.altCalibrated) return;

  // Fast baro EMA in AGL metres
  float fastAglCm = flightState.baroFast.valueCm - (float)baroData.groundAltCm;
  float fastAglM = fastAglCm / 100.0f;

  if (fastAglM <= (float)flightState.config.mainDeployAltM) {
    flightState.mainDeployEntryUs = nowUs;
    enterPhase(PHASE_MAIN_DEPLOY, nowUs);
  }
}

// --- Landing detection (MAIN_DEPLOY -> LANDED) ---
// 120s timeout OR (10s after main deploy AND not moving much)

static void updateLandingAccelEma(unsigned long nowUs) {
  if (!accelData.valid) return;

  if (!flightState.landAccelEmaValid) {
    flightState.landAccelEmaX = (float)accelData.x;
    flightState.landAccelEmaY = (float)accelData.y;
    flightState.landAccelEmaZ = (float)accelData.z;
    flightState.landAccelEmaValid = true;
    flightState.landAccelEmaLastUs = nowUs;
    return;
  }

  float dt = (float)(nowUs - flightState.landAccelEmaLastUs);
  if (dt < 100.0f) return;
  float alpha = dt / 1000000.0f;  // 1-second time constant
  if (alpha > 1.0f) alpha = 1.0f;

  flightState.landAccelEmaX += alpha * ((float)accelData.x - flightState.landAccelEmaX);
  flightState.landAccelEmaY += alpha * ((float)accelData.y - flightState.landAccelEmaY);
  flightState.landAccelEmaZ += alpha * ((float)accelData.z - flightState.landAccelEmaZ);
  flightState.landAccelEmaLastUs = nowUs;
}

static void checkLanding(unsigned long nowUs) {
  if (flightState.mainDeployEntryUs == 0) return;

  unsigned long sinceMainDeploy = nowUs - flightState.mainDeployEntryUs;

  // 120-second timeout: automatic landing declaration
  if (sinceMainDeploy >= 120000000UL) {
    enterPhase(PHASE_LANDED, nowUs);
    return;
  }

  // Early landing: at least 10 seconds after main deploy AND not moving
  if (sinceMainDeploy < 10000000UL) return;

  // Update landing accel EMAs
  updateLandingAccelEma(nowUs);

  // Check accel: each axis within ±300mg of its EMA, for 10 samples / 5 seconds
  bool accelStill = false;
  if (accelData.valid && flightState.landAccelEmaValid) {
    bool xOk = fabsf((float)accelData.x - flightState.landAccelEmaX) <= 300.0f;
    bool yOk = fabsf((float)accelData.y - flightState.landAccelEmaY) <= 300.0f;
    bool zOk = fabsf((float)accelData.z - flightState.landAccelEmaZ) <= 300.0f;

    if (xOk) { flightState.landAccelX.qualify(nowUs, 1, 0); }
    else     { flightState.landAccelX.fail(); }
    if (yOk) { flightState.landAccelY.qualify(nowUs, 1, 0); }
    else     { flightState.landAccelY.fail(); }
    if (zOk) { flightState.landAccelZ.qualify(nowUs, 1, 0); }
    else     { flightState.landAccelZ.fail(); }

    accelStill = flightState.landAccelX.isTriggered(10, 5000000UL) &&
                 flightState.landAccelY.isTriggered(10, 5000000UL) &&
                 flightState.landAccelZ.isTriggered(10, 5000000UL);
  }

  // Check gyro: each axis under ±50 deg/s for 5 seconds
  bool gyroStill = false;
  if (gyroData.valid) {
    bool gOk = (abs(gyroData.x) < 500) && (abs(gyroData.y) < 500) && (abs(gyroData.z) < 500);
    if (gOk) { flightState.landGyro.qualify(nowUs, 1, 0); }
    else     { flightState.landGyro.fail(); }
    gyroStill = flightState.landGyro.isTriggered(1, 5000000UL);
  }

  // Check baro: within 5m of fast EMA continuously for 5 seconds, at least 4 samples
  bool baroStill = false;
  if (baroData.valid && flightState.baroFast.valid) {
    float diffM = fabsf((float)baroData.altCmMSL - flightState.baroFast.valueCm) / 100.0f;
    if (diffM <= 5.0f) {
      flightState.landBaro.qualify(nowUs, 1, 0);
    } else {
      flightState.landBaro.fail();
    }
    baroStill = flightState.landBaro.isTriggered(4, 5000000UL);
  }

  if (accelStill && gyroStill && baroStill) {
    enterPhase(PHASE_LANDED, nowUs);
  }
}

// ===================== MAIN UPDATE =====================

void nonblockingFlight() {
  unsigned long nowUs = micros();

  // Save previous fast baro EMA before updating — apogee detection compares
  // this loop's EMA value against last loop's to detect a falling trend.
  if (flightState.baroFast.valid) {
    flightState.prevBaroFastCm = flightState.baroFast.valueCm;
    flightState.prevBaroFastValid = true;
  }

  // Always update baro EMAs (needed for both arming and flight)
  updateBaroEMAs(nowUs);

  // Always update stability trackers (so arm_ready telemetry flag is current)
  updateAccelStability(nowUs);
  updateGyroStability(nowUs);
  updateBaroStability(nowUs);
  updateGpsStability(nowUs);

  // Update arm readiness flag
  flightState.armReady = !flightState.armed && checkArmReady(nowUs);

  // Update ms since launch
  if (flightState.launchDetectUs > 0) {
    flightState.msSinceLaunch = (int32_t)((nowUs - flightState.launchDetectUs) / 1000UL);
  } else {
    flightState.msSinceLaunch = -1;
  }

  // Phase-specific checks
  switch (flightState.phase) {
    case PHASE_IDLE:
      // Nothing to do — waiting for ARM command
      break;

    case PHASE_ARMED:
      // Check for launch detection
      checkLaunchDetect(nowUs);
      // Also check for direct-to-coast (skipping boost)
      if (flightState.phase == PHASE_ARMED) {
        checkCoastDetect(nowUs);
      }
      break;

    case PHASE_BOOST:
      // Check for coast transition
      checkCoastDetect(nowUs);
      break;

    case PHASE_COAST:
      // Check for apogee
      checkApogeeDetect(nowUs);
      break;

    case PHASE_DROGUE:
      // Check for main deploy
      checkMainDeploy(nowUs);
      break;

    case PHASE_MAIN_DEPLOY:
      // Check for landing
      checkLanding(nowUs);
      break;

    case PHASE_LANDED:
      // Still armed — waiting for DISARM command
      break;

    default:
      break;
  }
}

// ===================== ARM / DISARM =====================

uint8_t flightTryArm(const uint8_t* params, size_t paramsLen) {
  unsigned long nowUs = micros();

  bool forceArm = false;
  FlightConfig cfg;
  cfg.boostAccelMg = DEFAULT_BOOST_ACCEL_MG;
  cfg.boostAltM = DEFAULT_BOOST_ALT_M;
  cfg.coastAltM = DEFAULT_COAST_ALT_M;
  cfg.mainDeployAltM = DEFAULT_MAIN_DEPLOY_ALT_M;

  if (paramsLen == 0 || params == nullptr) {
    // Legacy: no params = force arm with defaults
    forceArm = true;
  } else if (paramsLen == 9) {
    // Full params: 8 bytes config + 1 byte flags
    cfg.boostAccelMg = (uint16_t)(params[0] | (params[1] << 8));
    cfg.boostAltM    = (uint16_t)(params[2] | (params[3] << 8));
    cfg.coastAltM    = (uint16_t)(params[4] | (params[5] << 8));
    cfg.mainDeployAltM = (uint16_t)(params[6] | (params[7] << 8));
    uint8_t flags    = params[8];
    forceArm = (flags & 0x01) != 0;
  } else {
    // Bad param length — shouldn't reach here if caller validates, but just force defaults
    forceArm = true;
  }

  // Already armed?
  if (flightState.armed) {
    return ARM_ERR_ALREADY;
  }

  if (!forceArm) {
    // Check baro sensor
    if (!sensorState.baroOk) return ARM_ERR_BARO;
    if (flightState.baroStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return ARM_ERR_BARO;
    if (flightState.baroStab.isStale(nowUs)) return ARM_ERR_BARO;

    // Check accel sensor
    if (!sensorState.accelOk) return ARM_ERR_ACCEL;
    if (flightState.accelStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return ARM_ERR_ACCEL;
    if (flightState.accelStab.isStale(nowUs)) return ARM_ERR_ACCEL;

    // Check stability duration
    if (flightState.accelStab.stable.durationUs() < ARM_STABILITY_DURATION_US) return ARM_ERR_STABILITY;
    if (flightState.baroStab.stable.durationUs() < ARM_STABILITY_DURATION_US) return ARM_ERR_STABILITY;
    if (sensorState.gyroOk) {
      if (flightState.gyroStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return ARM_ERR_STABILITY;
      if (flightState.gyroStab.stable.durationUs() < ARM_STABILITY_DURATION_US) return ARM_ERR_STABILITY;
    }
  }

  // --- ARMING ---
  flightState.config = cfg;
  flightState.armed = true;
  flightState.phase = PHASE_ARMED;
  flightState.armFlags = forceArm ? 0x01 : 0x00;

  // Lock orientation based on current accel X axis
  if (accelData.valid) {
    flightState.orientXPositive = (accelData.x > 0);
    flightState.orientLocked = true;
  }

  // Set ground level from slow baro EMA (or current baro if EMA not ready)
  if (flightState.baroSlow.valid) {
    baroData.groundAltCm = (int32_t)flightState.baroSlow.valueCm;
    baroData.altCalibrated = true;
    baroData.altCmAGL = baroData.altCmMSL - baroData.groundAltCm;
  } else if (baroData.valid) {
    baroData.groundAltCm = baroData.altCmMSL;
    baroData.altCalibrated = true;
    baroData.altCmAGL = 0;
  }

  // Reset all flight trackers
  flightState.launchAccelTracker.reset();
  flightState.launchAltTracker.reset();
  flightState.launchCoastAltTracker.reset();
  flightState.lastAccelTriggerUs = 0;
  flightState.lastAltTriggerUs = 0;
  flightState.coastAltTracker.reset();
  flightState.coastLowAccelTracker.reset();
  flightState.boostEntryUs = 0;
  flightState.apogeeEntryUs = 0;
  flightState.mainDeployEntryUs = 0;
  flightState.launchDetectUs = 0;
  flightState.prevBaroFastValid = false;
  flightState.msSinceLaunch = -1;

  // Reset landing trackers
  flightState.landAccelX.reset();
  flightState.landAccelY.reset();
  flightState.landAccelZ.reset();
  flightState.landGyro.reset();
  flightState.landBaro.reset();
  flightState.landAccelEmaValid = false;

  Serial.print("ARMED: boostG="); Serial.print(cfg.boostAccelMg);
  Serial.print(" boostAlt="); Serial.print(cfg.boostAltM);
  Serial.print(" coastAlt="); Serial.print(cfg.coastAltM);
  Serial.print(" mainAlt="); Serial.print(cfg.mainDeployAltM);
  Serial.print(" force="); Serial.print(forceArm);
  Serial.print(" orient="); Serial.println(flightState.orientXPositive ? "X+" : "X-");

  return ARM_OK;
}

void flightDisarm() {
  // Only clear the armed flag and set phase to idle.
  // Everything else (orientation, launch time, ground level, trackers, EMAs)
  // stays intact for post-flight telemetry and analysis.
  // Re-arming (flightTryArm) is what resets flight state.
  flightState.armed = false;
  enterPhase(PHASE_IDLE, micros());
  Serial.println("FLIGHT: disarmed (state preserved)");
}

// ===================== ACCESSORS =====================

FlightPhase flightGetPhase() {
  return flightState.phase;
}

bool flightIsArmReady() {
  return flightState.armReady;
}

bool flightBaroOk() {
  if (!sensorState.baroOk) return false;
  unsigned long nowUs = micros();
  if (flightState.baroStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return false;
  if (flightState.baroStab.isStale(nowUs)) return false;
  return true;
}

bool flightAccelOk() {
  if (!sensorState.accelOk) return false;
  unsigned long nowUs = micros();
  if (flightState.accelStab.hasRecentError(nowUs, ARM_ERROR_WINDOW_US)) return false;
  if (flightState.accelStab.isStale(nowUs)) return false;
  return true;
}
