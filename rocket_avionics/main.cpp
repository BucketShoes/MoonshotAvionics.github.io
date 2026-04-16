// Rocket GPS + LoRa Telemetry - Heltec Wireless Tracker V1.1
// Heltec wireless tracker (ESP32, LoRa, WiFi, GPS)
// and a SEN0140 v2 (ADXL345 accelerometer, ITG3200 gyro, VCM5883L compass, BMP280 pressure sensor/barometer)
// Sends 0xAF telemetry packets with rotating data pages.
// Receives 0x9A command packets with HMAC-SHA256 authentication.
// Logs to flash and transfers logs over radio.
// Parses GPS NMEA messages GGA (position), RMC (speed/course), GSA (VDOP).
// For bench testing, User button (GPIO0) toggles TX sending on/off (radio always listens). LED shows TX state.
//
// Board: "Heltec Wireless Tracker" in Arduino IDE
//
// *** CRITICAL SAFETY REQUIREMENT ***
// This main loop runs avionics for a rocket. While armed, the loop controls
// pyro charges and chute releases using set-high / check-each-loop / set-low
// timing. If the loop stalls, a pyro charge stays energised too long (fire
// hazard, structural damage) or a chute deploys late (ballistic descent onto
// people), and framework will eventually handle active aerodynamic control surfaces for guidance to stay pointed upwards, so could swerve if control loop stalls.
// NOTHING in this loop may block. Target: 1000Hz+ update rate. Nothing over 1ms even in worst case, even if only occasional.
//
// Rules:
//   - No delay() anywhere, ever.
//   - No blocking I/O (no Serial.flush, no while-waiting-for-X).
//   - All periodic work uses timestamp-and-check patterns.
//   - Every function called from loop() while armed has "nonblocking" in its name as a reminder and grep target.
//   - Radio TX is async (startTransmit + poll DIO1).
//   - The radio is in RX mode whenever not transmitting. TX is deferred if a preamble is detected
//     (incoming command), and the delayed TX count is tracked for stats.

#include <Arduino.h>
#include <Preferences.h>
#include <mbedtls/md.h>
#include "log_store.h"
#include "sensors.h"
#include "flight.h"
#include "config.h"
#include "globals.h"
#include "gps.h"
#include "radio.h"
#include "commands.h"
#include "telemetry.h"
#include "ble.h"

// ===================== GLOBAL DEFINITIONS =====================
// These match the extern declarations in globals.h.

InitState  initState    = INIT_VEXT_ON;
bool       gpsUartStarted = false;

Preferences nvs;
LogStore logStore;
bool logStoreOk = false;

uint16_t batteryMv = 0;
PeakValues peaks = {0, 0, 0};
uint32_t armRecordCounter = 0;

bool txSendingEnabled = true;
int8_t activeTxRate   = -5;
unsigned long txIntervalUs = 5000000UL;

unsigned long runningMaxLoopUs = 0;

// ===================== DEFAULT HMAC KEY =====================
// Written to NVS on first boot. Replace via CMD_SET_KEY in production.
// Key is loaded from secrets.h (gitignored). Copy secrets_example.h to secrets.h and fill in your key.
#include "secrets.h"

// ===================== INIT STATE MACHINE =====================
// Non-blocking hardware init. Sequences GPS reset, LoRa init, NVS load, sensors.

unsigned long initTimestamp = 0;

void nonblockingInit() {
  if (initState == INIT_DONE) return;
  unsigned long now = micros();

  switch (initState) {
    case INIT_VEXT_ON:
      pinMode(VEXT_CTRL_PIN, OUTPUT);
      digitalWrite(VEXT_CTRL_PIN, HIGH);
      pinMode(VBAT_ADC_CTRL_PIN, OUTPUT);
      digitalWrite(VBAT_ADC_CTRL_PIN, LOW);
      pinMode(VBAT_ADC_PIN, INPUT);
      initTimestamp = now;
      initState = INIT_GPS_RST_LOW;
      break;

    case INIT_GPS_RST_LOW:
      if (now - initTimestamp >= 100000UL) {
        pinMode(GPS_RST_PIN, OUTPUT);
        digitalWrite(GPS_RST_PIN, LOW);
        initTimestamp = now;
        initState = INIT_GPS_RST_HIGH;
      }
      break;

    case INIT_GPS_RST_HIGH:
      if (now - initTimestamp >= 100000UL) {
        digitalWrite(GPS_RST_PIN, HIGH);
        initTimestamp = now;
        initState = INIT_GPS_UART;
      }
      break;

    case INIT_GPS_UART:
      if (now - initTimestamp >= 100000UL) {
        gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
        gpsUartStarted = true;
        Serial.println("GNSS UART started");
        initState = INIT_LORA;
      }
      break;

    case INIT_LORA: {
      loraSPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_NSS_PIN);
      Serial.print("LoRa init... ");
      updateActiveFreqBw();
      if (radioInit()) {
        Serial.println("ok");
        loraReady = true;
      } else {
        Serial.println("FAIL");
      }
      initState = INIT_NVS;
      break;
    }

    case INIT_NVS: {
      nvs.begin(NVS_NAMESPACE, false);
      highestNonce = nvs.getUInt("nonce", 0);
      // Pre-populate CMD ACK page with current nonce so BLE phone sees it on first connect
      lastAck.nonce = highestNonce;
      lastAck.result = 0;
      lastAck.pending = false;
      logPages[LOGI_CMD_ACK].freshMask |= 0xFF;

      // Load or write HMAC key
      size_t keyLen = nvs.getBytesLength("hmac_key");
      if (keyLen == HMAC_KEY_LEN) {
        nvs.getBytes("hmac_key", hmacKey, HMAC_KEY_LEN);
        bool allFF = true;
        for (int i = 0; i < HMAC_KEY_LEN; i++) {
          if (hmacKey[i] != 0xFF) { allFF = false; break; }
        }
        if (allFF) {
          memcpy(hmacKey, DEFAULT_HMAC_KEY, HMAC_KEY_LEN);
          nvs.putBytes("hmac_key", hmacKey, HMAC_KEY_LEN);
          Serial.println("Default HMAC key written to NVS");
        } else {
          Serial.println("HMAC key loaded from NVS");
        }
      } else {
        memcpy(hmacKey, DEFAULT_HMAC_KEY, HMAC_KEY_LEN);
        nvs.putBytes("hmac_key", hmacKey, HMAC_KEY_LEN);
        Serial.println("Default HMAC key written to NVS (first boot)");
      }
      hmacKeyValid = true;
      Serial.print("Highest nonce: "); Serial.println(highestNonce);
      Serial.println("---");

      // Load radio config from NVS
      if (nvs.isKey("radio_ch")) {
        uint8_t nvsCh = nvs.getUChar("radio_ch", DEFAULT_CHANNEL);
        activeChannel = (channelToFreqMHz(nvsCh) != 0.0f) ? nvsCh : DEFAULT_CHANNEL;
      } else {
        activeChannel = DEFAULT_CHANNEL;
      }
      if (nvs.isKey("radio_sf")) {
        uint8_t nvsSf = nvs.getUChar("radio_sf", DEFAULT_SF);
        activeSF = (nvsSf >= 5 && nvsSf <= 12) ? nvsSf : DEFAULT_SF;
      } else {
        activeSF = DEFAULT_SF;
      }
      if (nvs.isKey("radio_pwr")) {
        int8_t nvsPwr = nvs.getChar("radio_pwr", DEFAULT_POWER);
        activePower = (nvsPwr >= -9 && nvsPwr <= 22) ? nvsPwr : DEFAULT_POWER;
      } else {
        activePower = DEFAULT_POWER;
      }
      updateActiveFreqBw();
      // Re-apply config with NVS values if radio is ready (radioInit already called).
      if (loraReady) {
        radioApplyConfig();
      }
      Serial.print("Radio config: ch"); Serial.print(activeChannel);
      Serial.print(" "); Serial.print(activeFreqMHz, 1); Serial.print("MHz SF");
      Serial.print(activeSF); Serial.print(" BW"); Serial.print((int)activeBwKHz);
      Serial.print(" pwr="); Serial.println(activePower);

      // Load TX rate from NVS
      int8_t loadedRate = nvs.isKey("tx_rate") ? nvs.getChar("tx_rate", 0) : 0;
      activeTxRate = (loadedRate == 0) ? -5 : loadedRate;
      txIntervalUs = txRateToIntervalUs(activeTxRate);
      Serial.print("TX rate: "); Serial.print(activeTxRate);
      Serial.print(" -> "); Serial.print(txIntervalUs); Serial.println("us interval");

      // Enable radio
      txSendingEnabled = true;
      if (loraReady) {
        radioStartRx();
        ledcWrite(LED_PIN, 10);
        Serial.println("Radio enabled — bootstrap RX, waiting for sync");
      } else {
        Serial.println("Radio FAILED — no LoRa");
      }

      logStoreOk = logStore.begin("log_data", "log_index", "rkt_log");
      Serial.print("LogStore: "); Serial.println(logStoreOk ? "OK" : "FAIL");
      if (logStoreOk) {
        Serial.print("  rec="); Serial.print(logStore.getRecordCounter());
        Serial.print(" vpos=0x"); Serial.println(logStore.getVirtualPos(), HEX);
      }

      initState = INIT_SENSORS;
      break;
    }

    case INIT_SENSORS:
      sensorsInit();
      flightInit();
      initState = INIT_DONE;
      break;

    case INIT_DONE:
      break;
  }
}

// ===================== BUTTON =====================
// Toggles TX sending on/off. Radio always keeps listening.

bool lastBtnRaw       = true;
bool btnStableState   = true;
unsigned long btnLastChangeUs = 0;

void nonblockingButton() {
  bool raw = digitalRead(USER_BTN_PIN);
  unsigned long now = micros();

  if (raw != lastBtnRaw) {
    btnLastChangeUs = now;
    lastBtnRaw = raw;
  }

  if ((now - btnLastChangeUs) >= BTN_DEBOUNCE_US) {
    if (raw != btnStableState) {
      btnStableState = raw;
      if (!btnStableState) {
        txSendingEnabled = !txSendingEnabled;
        ledcWrite(LED_PIN, txSendingEnabled ? 10 : 0);
        Serial.print("TX sending: ");
        Serial.println(txSendingEnabled ? "ON" : "OFF");
      }
    }
  }
}

// ===================== BATTERY =====================

enum VbatPhase { VBAT_IDLE, VBAT_SETTLING };
VbatPhase vbatPhase = VBAT_IDLE;
unsigned long vbatPhaseStartUs = 0;
unsigned long lastVbatCompleteMs = 0;

void nonblockingBattery() {
  unsigned long nowUs = micros();
  switch (vbatPhase) {
    case VBAT_IDLE:
      if ((millis() - lastVbatCompleteMs) >= VBAT_READ_INTERVAL_MS) {
        digitalWrite(VBAT_ADC_CTRL_PIN, HIGH);
        vbatPhaseStartUs = nowUs;
        vbatPhase = VBAT_SETTLING;
      }
      break;
    case VBAT_SETTLING:
      if ((nowUs - vbatPhaseStartUs) >= VBAT_SETTLE_US) {
        uint32_t adcMv = analogReadMilliVolts(VBAT_ADC_PIN);
        digitalWrite(VBAT_ADC_CTRL_PIN, LOW);
        if (batteryMv == 0) batteryMv = (uint16_t)(adcMv * VBAT_MULTIPLIER); else batteryMv = (uint16_t)(((((uint32_t)batteryMv)*900) + ((uint32_t)(adcMv * VBAT_MULTIPLIER)*100))/1000);
        lastVbatCompleteMs = millis();
        vbatPhase = VBAT_IDLE;
      }
      break;
  }
}

// ===================== PEAK TRACKING =====================

void nonblockingPeakTracking() {
  if (gps.valid) {
    int32_t altCm = (int32_t)gps.altCm;
    if (altCm > peaks.maxAltCmMSL) {
      peaks.maxAltCmMSL = altCm;
      logPages[LOGI_PEAKS].freshMask |= 0xFF;
    }
    //TODO: should use baro EMA 1-sec, or fusion alt
  }
  if (accelData.valid) {
    uint32_t magSq = (uint32_t)((int32_t)accelData.x * accelData.x +
                                 (int32_t)accelData.y * accelData.y +
                                 (int32_t)accelData.z * accelData.z);
    uint16_t accel100g = (uint16_t)(sqrtf((float)magSq) / 10.0f);
    if (accel100g > peaks.maxAccel100g) {
      peaks.maxAccel100g = accel100g;
      logPages[LOGI_PEAKS].freshMask |= 0xFF;
    }
  }
  if (baroData.valid) {
    int16_t vv = baroData.vvel10;
    if (vv > peaks.maxVvel10) {
      peaks.maxVvel10 = vv;
      logPages[LOGI_PEAKS].freshMask |= 0xFF;
    }
  }
  // Sys health data (heap, uptime, battery) changes every loop — mark fresh
  logPages[LOGI_SYS_HEALTH].freshMask |= 0xFF;
}

// ===================== LOOP STATS =====================

unsigned long loopCount       = 0;
unsigned long loopStatStartUs = 0;
unsigned long recentMaxLoopUs = 0;

void nonblockingLoopStats() {
  loopCount++;
  unsigned long now = micros();
  if (now - loopStatStartUs >= 10000000UL) {
    unsigned long elapsed = now - loopStatStartUs;
    float hz = (float)loopCount / ((float)elapsed / 1000000.0f);

    Serial.print("Loop: "); Serial.print((int)hz);
    Serial.print(" Hz  Batt: "); Serial.print(batteryMv);
    Serial.print("mV  DelayedTX: "); Serial.print(delayedTxCount);
    Serial.print("  InvalidRX: "); Serial.print(invalidRxCount);
    Serial.print("  recentMaxLoopUs: "); Serial.print(recentMaxLoopUs);
    Serial.print("  runningMaxLoopUs: "); Serial.print(runningMaxLoopUs);
    Serial.print("  bleCbTotal: "); Serial.print(bleCallbackTotalUs);
    Serial.print("us  bleCbPeak: "); Serial.print(bleCallbackPeakUs);
    Serial.println("us. new2");

    if (recentMaxLoopUs > runningMaxLoopUs) runningMaxLoopUs = recentMaxLoopUs;
    recentMaxLoopUs = 0;
    loopCount = 0;
    loopStatStartUs = now;
    // Reset BLE callback accumulators each stats period
    bleCallbackTotalUs = 0;
    bleCallbackPeakUs  = 0;
  }
}

// ===================== SETUP =====================

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(USER_BTN_PIN, INPUT_PULLUP);
  ledcAttach(LED_PIN, 1000, 8);
  ledcWrite(LED_PIN, 25);
  setCpuFrequencyMhz(240);
  Serial.println("Rocket Telemetry - Heltec Wireless Tracker");
  loopStatStartUs = micros();
  initBLE();
}

// ===================== MAIN LOOP =====================
// Every call is non-blocking. While armed, must run at 1000Hz+ for pyro safety.
// Any new function added here MUST be non-blocking (name it nonblockingXxx).

unsigned long lastLoopUs = 0;

// Slow-loop blame tracking: when a loop iteration exceeds the threshold, report
// which subsystem consumed the most time. Only active when not armed.
enum LoopSlot {
  SLOT_INIT = 0, SLOT_GPS, SLOT_BUTTON, SLOT_BATTERY,
  SLOT_SENSORS, SLOT_FLIGHT, SLOT_PEAKS, SLOT_LOGGING, SLOT_RADIO,
  SLOT_BLE, SLOT_STATS, SLOT_COUNT
};
static const char* slotNames[] = {
  "init", "gps", "btn", "batt", "sens", "flight", "peak", "log", "radio", "ble", "stats"
};
unsigned long slotUs[SLOT_COUNT];

void loop() {
  unsigned long now = micros();
  unsigned long elapsed = now - lastLoopUs;
  if (elapsed < MIN_LOOP_US) return;
  if (elapsed > recentMaxLoopUs) recentMaxLoopUs = elapsed;
  lastLoopUs = now;

  unsigned long t0, t1;

  t0 = micros();
  nonblockingInit();
  t1 = micros(); slotUs[SLOT_INIT] = t1 - t0; t0 = t1;

  nonblockingGPS();
  t1 = micros(); slotUs[SLOT_GPS] = t1 - t0; t0 = t1;

  nonblockingButton();
  t1 = micros(); slotUs[SLOT_BUTTON] = t1 - t0; t0 = t1;

  nonblockingBattery();
  t1 = micros(); slotUs[SLOT_BATTERY] = t1 - t0; t0 = t1;

  nonblockingSensors();
  t1 = micros(); slotUs[SLOT_SENSORS] = t1 - t0; t0 = t1;

  nonblockingFlight();
  t1 = micros(); slotUs[SLOT_FLIGHT] = t1 - t0; t0 = t1;

  nonblockingPeakTracking();
  t1 = micros(); slotUs[SLOT_PEAKS] = t1 - t0; t0 = t1;

  nonblockingLogging();
  t1 = micros(); slotUs[SLOT_LOGGING] = t1 - t0; t0 = t1;

  nonblockingRadio();
  t1 = micros(); slotUs[SLOT_RADIO] = t1 - t0; t0 = t1;

  nonblockingBle();
  t1 = micros(); slotUs[SLOT_BLE] = t1 - t0; t0 = t1;

  nonblockingLoopStats();
  t1 = micros(); slotUs[SLOT_STATS] = t1 - t0;

  unsigned long totalUs = t1 - lastLoopUs;
  if (!isArmed && totalUs > SLOW_LOOP_THRESHOLD_US) {
    Serial.print("SLOW LOOP "); Serial.print(totalUs); Serial.print("us: ");
    for (int i = 0; i < SLOT_COUNT; i++) {
      if (slotUs[i] > 100) {
        Serial.print(slotNames[i]); Serial.print("="); Serial.print(slotUs[i]); Serial.print(" ");
      }
    }
    // Report BLE callback overhead (runs on separate FreeRTOS task, not in slots)
    if (bleCallbackTotalUs > 0) {
      Serial.print(" ble_cb_total="); Serial.print(bleCallbackTotalUs);
      Serial.print(" ble_cb_peak="); Serial.print(bleCallbackPeakUs);
    }
    Serial.println();
  }
}
