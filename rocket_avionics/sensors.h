// sensors.h — SEN0140 v2 10DOF IMU driver for Moonshot rocket avionics
// ADXL345 (accelerometer), ITG3200 (gyro), VCM5883L (mag), BMP280 (baro)
// All I2C, non-blocking reads via state machines for main loop integration.
// No external libraries — direct register access over Wire.
//
// Usage:
//   #include "sensors.h"
//   In setup (after Wire.begin): sensorsInit();
//   In loop: nonblockingSensors();
//   Read data from: accelData, gyroData, magData, baroData
//
// I2C bus note: Wire.requestFrom / Wire.read are fast (<200us at 400kHz for
// 6-byte reads). The BMP280 forced-mode conversion takes ~44ms at max
// oversampling, handled by the state machine waiting between request and read.

#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include "telemetry.h"

// ===================== I2C PIN CONFIG =====================
// Choose two free GPIOs from the Heltec Wireless Tracker headers.
// GPIO4 (SDA) and GPIO5 (SCL) are on J3 pins 8 and 7 — both are
// ADC1/TOUCH pins with no conflicting connected peripherals.
// Change these if your wiring differs.
#define SENSOR_SDA_PIN  4
#define SENSOR_SCL_PIN  5

// ===================== I2C ADDRESSES =====================
#define ADXL345_ADDR    0x53  // SDO grounded on SEN0140
#define ITG3200_ADDR    0x68  // AD0 grounded on SEN0140
#define HMC5883L_ADDR   0x1E  // genuine Honeywell HMC5883L (v1 boards only)
#define VCM5883L_ADDR   0x0C  // VCM5883L on DFRobot SEN0140 v2
#define BMP280_ADDR     0x77  // SDO pulled high on SEN0140 v2 (could be 0x76 if SDO low)

// ===================== SENSOR DATA STRUCTS =====================

struct AccelData {
  int16_t x, y, z;          // raw ADC values in milli-g (after scaling)
  bool    valid;
  unsigned long lastReadUs;
};

struct GyroData {
  int16_t x, y, z;          // raw ADC values in 0.1 deg/s (after scaling)
  bool    valid;
  unsigned long lastReadUs;
};

struct MagData {
  int16_t x, y, z;          // milligauss (after scaling)
  bool    valid;
  unsigned long lastReadUs;
};

struct BaroData {
  int32_t pressurePa;       // pascals
  int16_t tempC10;          // temperature in 0.1°C
  int32_t altCmAGL;         // cm above ground level (0 = launch site)
  bool    altCalibrated;    // true once ground-level is set (at arming)
  int16_t vvel10;           // filtered vertical velocity, 0.1 m/s
  bool    valid;
  int32_t groundAltCm;      // ground level pressure altitude, cm MSL
  int32_t altCmMSL;         // pressure altitude cm MSL (before AGL offset)
  unsigned long lastReadUs;
};

// Global sensor data — read these from anywhere
extern AccelData accelData;
extern GyroData  gyroData;
extern MagData   magData;
extern BaroData  baroData;

// ===================== POLL INTERVALS =====================
// How often each sensor is read. Can be changed at runtime.
// These are defaults for bench testing — flight rates will be faster.
#define ACCEL_POLL_US    10000UL   // 100 Hz
#define GYRO_POLL_US     10000UL   // 100 Hz
#define MAG_POLL_US      50000UL   // 20 Hz
#define BARO_POLL_US     25000UL   // 40 Hz (BMP280 forced mode ~44ms at max oversampling,
                                   // we use x16 pressure / x2 temp ≈ 38ms typical)

// ===================== BARO VERTICAL VELOCITY EMA =====================
// Simple EMA on baro altitude for filtered vertical velocity.
// This is the safety-critical path — intentionally simple.
#define BARO_VVEL_ALPHA  0.15f     // EMA smoothing factor (higher = more responsive)

// ===================== REGISTER DEFINITIONS =====================

// --- ADXL345 ---
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_DATA_FORMAT 0x31
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_DATAX0      0x32  // 6 bytes: X0,X1,Y0,Y1,Z0,Z1

// --- ITG3200 ---
#define ITG3200_REG_DLPF_FS     0x16
#define ITG3200_REG_SMPLRT_DIV  0x15
#define ITG3200_REG_PWR_MGM     0x3E
#define ITG3200_REG_GYRO_XOUT_H 0x1D  // 6 bytes: XH,XL,YH,YL,ZH,ZL

// --- HMC5883L (kept for v1 board compatibility) ---
#define HMC5883L_REG_CONFIG_A   0x00
#define HMC5883L_REG_CONFIG_B   0x01
#define HMC5883L_REG_MODE       0x02
#define HMC5883L_REG_DATA_X_H   0x03  // 6 bytes: XH,XL,ZH,ZL,YH,YL (note Z before Y!)

// --- VCM5883L ---
#define VCM5883L_REG_CTRL1      0x09
#define VCM5883L_REG_CTRL2      0x0A
#define VCM5883L_REG_SET_RESET  0x0B
#define VCM5883L_REG_DATA_X_L   0x00  // 6 bytes: XL,XH,YL,YH,ZL,ZH

// --- BMP280 ---
#define BMP280_REG_CHIP_ID      0xD0  // should read 0x58
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_PRESS_MSB    0xF7  // 6 bytes: P_MSB,P_LSB,P_XLSB, T_MSB,T_LSB,T_XLSB
#define BMP280_REG_CALIB        0x88  // 26 bytes of calibration data (0x88-0xA1)

// ===================== INTERNAL STATE =====================

// Which magnetometer variant was detected
enum MagType { MAG_NONE, MAG_HMC5883L, MAG_VCM5883L };

// BMP280 state machine phases (forced mode: start → wait → read → idle)
enum BMP280Phase {
  BMP_IDLE,
  BMP_MEAS_WAIT,     // waiting for forced-mode conversion to complete
};

// BMP280 calibration coefficients (read once at init from sensor)
// All little-endian in the sensor, unlike BMP085
struct BMP280Cal {
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
  bool     valid;
};

// ===================== SENSOR STATE (internal) =====================

struct SensorState {
  bool accelOk;
  bool gyroOk;
  MagType magType;
  bool baroOk;

  BMP280Cal bmpCal;
  BMP280Phase bmpPhase;
  unsigned long bmpPhaseStartUs;

  unsigned long accelNextUs;
  unsigned long gyroNextUs;
  unsigned long magNextUs;
  unsigned long baroNextUs;

  // Baro vvel state
  float lastAltM;          // last altitude in metres for vvel calc
  unsigned long lastAltUs; // timestamp of last altitude
  float vvelEma;           // filtered vertical velocity m/s
  bool vvelValid;
};

extern SensorState sensorState;

// ===================== I2C HELPERS =====================

static inline void i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline bool i2cReadRegs(uint8_t addr, uint8_t startReg, uint8_t* buf, uint8_t count) {
  Wire.beginTransmission(addr);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;  // NACK
  Wire.requestFrom(addr, count);
  if (Wire.available() < count) return false;
  for (uint8_t i = 0; i < count; i++) buf[i] = Wire.read();
  return true;
}

static inline bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

// ===================== INIT FUNCTIONS =====================

static bool initADXL345() {
  if (!i2cProbe(ADXL345_ADDR)) return false;
  // Full resolution, ±16g range
  i2cWriteReg(ADXL345_ADDR, ADXL345_REG_DATA_FORMAT, 0x0B);  // FULL_RES | ±16g
  // 200Hz output data rate
  i2cWriteReg(ADXL345_ADDR, ADXL345_REG_BW_RATE, 0x0B);      // 200Hz
  // Measure mode
  i2cWriteReg(ADXL345_ADDR, ADXL345_REG_POWER_CTL, 0x08);    // Measure bit
  return true;
}

static bool initITG3200() {
  if (!i2cProbe(ITG3200_ADDR)) return false;
  // Use PLL with X gyro reference, not internal oscillator
  i2cWriteReg(ITG3200_ADDR, ITG3200_REG_PWR_MGM, 0x01);
  // Full scale ±2000°/s, DLPF bandwidth 42Hz (gives ~100Hz internal rate)
  i2cWriteReg(ITG3200_ADDR, ITG3200_REG_DLPF_FS, 0x1B);      // FS_SEL=3, DLPF=3
  // Sample rate divider: 0 = full rate (internal_rate / (1+0))
  i2cWriteReg(ITG3200_ADDR, ITG3200_REG_SMPLRT_DIV, 0x00);
  return true;
}

static MagType initMag() {
  // Try HMC5883L first (genuine Honeywell — only on v1 boards)
  if (i2cProbe(HMC5883L_ADDR)) {
    // Check identification registers (reg 10-12 should be 'H', '4', '3')
    uint8_t id[3];
    if (i2cReadRegs(HMC5883L_ADDR, 10, id, 3)) {
      if (id[0] == 'H' && id[1] == '4' && id[2] == '3') {
        // 8-average, 75Hz output rate, normal measurement
        i2cWriteReg(HMC5883L_ADDR, HMC5883L_REG_CONFIG_A, 0x78);
        // Gain: ±1.3 Ga (1090 LSB/Gauss) — good general range
        i2cWriteReg(HMC5883L_ADDR, HMC5883L_REG_CONFIG_B, 0x20);
        // Continuous measurement mode
        i2cWriteReg(HMC5883L_ADDR, HMC5883L_REG_MODE, 0x00);
        return MAG_HMC5883L;
      }
    }
  }

  // Try VCM5883L (DFRobot SEN0140 v2 board, address 0x0C)
  if (i2cProbe(VCM5883L_ADDR)) {
    // Set/Reset period register (recommended)
    i2cWriteReg(VCM5883L_ADDR, VCM5883L_REG_SET_RESET, 0x01);
    // Continuous mode, 200Hz ODR, 8G range, 512 oversampling
    i2cWriteReg(VCM5883L_ADDR, VCM5883L_REG_CTRL1, 0x1D);
    // Disable interrupt, pointer auto-increment
    i2cWriteReg(VCM5883L_ADDR, VCM5883L_REG_CTRL2, 0x40);
    return MAG_VCM5883L;
  }

  return MAG_NONE;
}

static bool initBMP280(BMP280Cal& cal) {
  if (!i2cProbe(BMP280_ADDR)) return false;

  // Verify chip ID — BMP280 should return 0x58
  uint8_t chipId;
  if (!i2cReadRegs(BMP280_ADDR, BMP280_REG_CHIP_ID, &chipId, 1)) return false;
  if (chipId != 0x58) {
    Serial.print("BMP280: unexpected chip ID 0x");
    Serial.println(chipId, HEX);
    return false;
  }

  // Soft reset
  i2cWriteReg(BMP280_ADDR, BMP280_REG_RESET, 0xB6);
  delay(5);  // blocking at init is fine — reset needs ~2ms

  // Read 26 bytes of calibration data from 0x88-0xA1
  // All values are little-endian in BMP280 (unlike BMP085 which was big-endian)
  uint8_t buf[26];
  if (!i2cReadRegs(BMP280_ADDR, BMP280_REG_CALIB, buf, 26)) return false;

  cal.dig_T1 = (uint16_t)((buf[1]  << 8) | buf[0]);
  cal.dig_T2 = (int16_t) ((buf[3]  << 8) | buf[2]);
  cal.dig_T3 = (int16_t) ((buf[5]  << 8) | buf[4]);
  cal.dig_P1 = (uint16_t)((buf[7]  << 8) | buf[6]);
  cal.dig_P2 = (int16_t) ((buf[9]  << 8) | buf[8]);
  cal.dig_P3 = (int16_t) ((buf[11] << 8) | buf[10]);
  cal.dig_P4 = (int16_t) ((buf[13] << 8) | buf[12]);
  cal.dig_P5 = (int16_t) ((buf[15] << 8) | buf[14]);
  cal.dig_P6 = (int16_t) ((buf[17] << 8) | buf[16]);
  cal.dig_P7 = (int16_t) ((buf[19] << 8) | buf[18]);
  cal.dig_P8 = (int16_t) ((buf[21] << 8) | buf[20]);
  cal.dig_P9 = (int16_t) ((buf[23] << 8) | buf[22]);

  // Sanity: dig_T1 should not be 0 or 0xFFFF
  if (cal.dig_T1 == 0 || cal.dig_T1 == 0xFFFF) return false;

  // Config register: standby 0.5ms, IIR filter off, SPI disabled
  // (filter off — we handle filtering externally; and we use forced mode anyway)
  i2cWriteReg(BMP280_ADDR, BMP280_REG_CONFIG, 0x00);

  // ctrl_meas: temp oversampling x2, pressure oversampling x16, forced mode
  // Forced mode means the sensor goes back to sleep after one measurement.
  // We trigger each measurement explicitly. Conversion time ~38ms.
  // osrs_t[7:5]=010 (x2), osrs_p[4:2]=101 (x16), mode[1:0]=01 (forced)
  i2cWriteReg(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x55);

  cal.valid = true;
  return true;
}

// ===================== SENSOR READ FUNCTIONS =====================

static void readADXL345() {
  uint8_t buf[6];
  if (!i2cReadRegs(ADXL345_ADDR, ADXL345_REG_DATAX0, buf, 6)) return;

  // ADXL345 full-resolution: 4mg/LSB regardless of range
  // Data is little-endian in the sensor
  int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

  // Convert to milli-g: 4mg per LSB
  accelData.x = rawX * 4;   // milli-g (clamps at ±16000 = ±16g)
  accelData.y = rawY * 4;
  accelData.z = rawZ * 4;
  accelData.valid = true;
  accelData.lastReadUs = micros();
  logPages[LOGI_ACCEL].freshMask |= 0xFF;
}

static void readITG3200() {
  uint8_t buf[6];
  if (!i2cReadRegs(ITG3200_ADDR, ITG3200_REG_GYRO_XOUT_H, buf, 6)) return;

  // ITG3200: 14.375 LSB per deg/s, data is big-endian
  // Target unit: 0.1 deg/s → multiply raw by (10 / 14.375) ≈ 0.6957
  // Best compromise: raw * 100 / 144 (error ~0.17%)
  int16_t rawX = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t rawY = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t rawZ = (int16_t)((buf[4] << 8) | buf[5]);

  // Convert to 0.1 deg/s using 32-bit intermediate
  gyroData.x = (int16_t)((int32_t)rawX * 100 / 144);
  gyroData.y = (int16_t)((int32_t)rawY * 100 / 144);
  gyroData.z = (int16_t)((int32_t)rawZ * 100 / 144);
  gyroData.valid = true;
  gyroData.lastReadUs = micros();
  logPages[LOGI_GYRO].freshMask |= 0xFF;
}

static void readMagHMC5883L() {
  uint8_t buf[6];
  if (!i2cReadRegs(HMC5883L_ADDR, HMC5883L_REG_DATA_X_H, buf, 6)) return;

  // HMC5883L register order: XH,XL, ZH,ZL, YH,YL (Z before Y!)
  // At gain=1 (±1.3Ga): 1090 LSB/Gauss = 1.09 LSB/milliGauss
  // milliGauss = raw * 1000 / 1090 ≈ raw * 100 / 109
  int16_t rawX = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t rawZ = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t rawY = (int16_t)((buf[4] << 8) | buf[5]);

  magData.x = (int16_t)((int32_t)rawX * 100 / 109);
  magData.y = (int16_t)((int32_t)rawY * 100 / 109);
  magData.z = (int16_t)((int32_t)rawZ * 100 / 109);
  magData.valid = true;
  magData.lastReadUs = micros();
  logPages[LOGI_MAG].freshMask |= 0xFF;
}

static void readMagVCM5883L() {
  uint8_t buf[6];
  if (!i2cReadRegs(VCM5883L_ADDR, VCM5883L_REG_DATA_X_L, buf, 6)) return;

  // VCM5883L register order: XL,XH, YL,YH, ZL,ZH (little-endian)
  // At 8 Gauss range: 3000 LSB/Gauss → milliGauss = raw * 1000/3000 = raw/3
  int16_t rawX = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t rawY = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t rawZ = (int16_t)((buf[5] << 8) | buf[4]);

  magData.x = rawX / 3;
  magData.y = rawY / 3;
  magData.z = rawZ / 3;
  magData.valid = true;
  magData.lastReadUs = micros();
  logPages[LOGI_MAG].freshMask |= 0xFF;
}

// ===================== BMP280 COMPENSATION =====================
// Standard Bosch BMP280 algorithm from the datasheet (integer version).
// Uses the calibration coefficients read at init time.
// t_fine is shared between temp and pressure calculations.

static int32_t bmp280_t_fine;  // set by temp calc, used by pressure calc

static int32_t bmp280CompensateTemp(const BMP280Cal& cal, int32_t adcT) {
  int32_t var1 = ((((adcT >> 3) - ((int32_t)cal.dig_T1 << 1))) * ((int32_t)cal.dig_T2)) >> 11;
  int32_t var2 = (((((adcT >> 4) - ((int32_t)cal.dig_T1)) *
                    ((adcT >> 4) - ((int32_t)cal.dig_T1))) >> 12) *
                  ((int32_t)cal.dig_T3)) >> 14;
  bmp280_t_fine = var1 + var2;
  // Returns temperature in 0.01°C (e.g. 5123 = 51.23°C)
  return (bmp280_t_fine * 5 + 128) >> 8;
}

static uint32_t bmp280CompensatePressure(const BMP280Cal& cal, int32_t adcP) {
  int64_t var1 = ((int64_t)bmp280_t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)cal.dig_P6;
  var2 = var2 + ((var1 * (int64_t)cal.dig_P5) << 17);
  var2 = var2 + (((int64_t)cal.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)cal.dig_P3) >> 8) +
         ((var1 * (int64_t)cal.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)cal.dig_P1) >> 33;
  if (var1 == 0) return 0;  // avoid division by zero

  int64_t p = 1048576 - adcP;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)cal.dig_P8) * p) >> 19;
  // Returns pressure in Pa as Q24.8 fixed point (divide by 256 for Pa)
  p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7) << 4);
  return (uint32_t)p;
}

// Convert pressure to altitude using barometric formula (ISA)
// Returns altitude in cm. Uses float — acceptable for non-time-critical path.
static int32_t pressureToAltCm(int32_t pressurePa) {
  // Hypsometric formula: alt = 44330 * (1 - (P/P0)^0.19029)
  // P0 = 101325 Pa (standard sea level)
  float altM = 44330.0f * (1.0f - powf((float)pressurePa / 101325.0f, 0.19029f));
  return (int32_t)(altM * 100.0f);
}

// ===================== BMP280 STATE MACHINE =====================

static void bmp280StartMeasurement() {
  // Write ctrl_meas to trigger forced mode measurement
  // osrs_t[7:5]=010 (x2), osrs_p[4:2]=101 (x16), mode[1:0]=01 (forced)
  i2cWriteReg(BMP280_ADDR, BMP280_REG_CTRL_MEAS, 0x55);
}

static bool bmp280ReadResult(const BMP280Cal& cal,
                             int32_t& outPressurePa, int16_t& outTempC10) {
  // Check if measurement is still in progress
  uint8_t status;
  if (!i2cReadRegs(BMP280_ADDR, BMP280_REG_STATUS, &status, 1)) return false;
  if (status & 0x08) return false;  // bit 3 = measuring, still busy

  // Read 6 bytes: press[19:12], press[11:4], press[3:0]<<4, temp[19:12], temp[11:4], temp[3:0]<<4
  uint8_t buf[6];
  if (!i2cReadRegs(BMP280_ADDR, BMP280_REG_PRESS_MSB, buf, 6)) return false;

  int32_t adcP = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
  int32_t adcT = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);

  // Skip obviously invalid readings (all zeros or all ones = sensor not ready)
  if (adcT == 0 || adcT == 0xFFFFF || adcP == 0 || adcP == 0xFFFFF) return false;

  // Temperature first (sets t_fine for pressure calc)
  int32_t tempC100 = bmp280CompensateTemp(cal, adcT);  // 0.01°C
  outTempC10 = (int16_t)(tempC100 / 10);               // convert to 0.1°C

  // Pressure (Q24.8 format, divide by 256 for Pa)
  uint32_t pressQ24_8 = bmp280CompensatePressure(cal, adcP);
  outPressurePa = (int32_t)(pressQ24_8 / 256);

  return true;
}

// ===================== PUBLIC API =====================

// Call once during init (after Wire.begin). This does blocking I2C reads
// for sensor detection and BMP280 calibration — acceptable at startup.
inline void sensorsInit() {
  Wire.begin(SENSOR_SDA_PIN, SENSOR_SCL_PIN);
  Wire.setClock(400000);  // 400kHz fast mode

  sensorState.accelOk = initADXL345();
  Serial.print("ADXL345: ");
  Serial.println(sensorState.accelOk ? "OK" : "NOT FOUND");

  sensorState.gyroOk = initITG3200();
  Serial.print("ITG3200: ");
  Serial.println(sensorState.gyroOk ? "OK" : "NOT FOUND");

  sensorState.magType = initMag();
  Serial.print("Mag: ");
  switch (sensorState.magType) {
    case MAG_HMC5883L:  Serial.println("HMC5883L"); break;
    case MAG_VCM5883L:  Serial.println("VCM5883L"); break;
    default:            Serial.println("NOT FOUND"); break;
  }

  sensorState.baroOk = initBMP280(sensorState.bmpCal);
  Serial.print("BMP280: ");
  Serial.println(sensorState.baroOk ? "OK" : "NOT FOUND");
  if (sensorState.baroOk) {
    Serial.print("  dig_T1="); Serial.print(sensorState.bmpCal.dig_T1);
    Serial.print(" dig_T2="); Serial.print(sensorState.bmpCal.dig_T2);
    Serial.print(" dig_P1="); Serial.println(sensorState.bmpCal.dig_P1);
  }

  sensorState.bmpPhase = BMP_IDLE;
  sensorState.vvelValid = false;

  unsigned long now = micros();
  sensorState.accelNextUs = now;
  sensorState.gyroNextUs  = now;
  sensorState.magNextUs   = now;
  sensorState.baroNextUs  = now;
}

// Call every loop iteration. Reads sensors at their configured rates.
// Each sensor read is a single I2C transaction (~100-200us at 400kHz).
inline void nonblockingSensors() {
  unsigned long now = micros();

  // --- Accelerometer ---
  if (sensorState.accelOk && (now - sensorState.accelNextUs) >= ACCEL_POLL_US) {
    sensorState.accelNextUs += ACCEL_POLL_US;
    // Prevent drift if we fell behind
    if ((long)(now - sensorState.accelNextUs) > (long)ACCEL_POLL_US)
      sensorState.accelNextUs = now;
    readADXL345();
  }

  // --- Gyroscope ---
  if (sensorState.gyroOk && (now - sensorState.gyroNextUs) >= GYRO_POLL_US) {
    sensorState.gyroNextUs += GYRO_POLL_US;
    if ((long)(now - sensorState.gyroNextUs) > (long)GYRO_POLL_US)
      sensorState.gyroNextUs = now;
    readITG3200();
  }

  // --- Magnetometer ---
  if (sensorState.magType != MAG_NONE && (now - sensorState.magNextUs) >= MAG_POLL_US) {
    sensorState.magNextUs += MAG_POLL_US;
    if ((long)(now - sensorState.magNextUs) > (long)MAG_POLL_US)
      sensorState.magNextUs = now;
    if (sensorState.magType == MAG_HMC5883L) readMagHMC5883L();
    else                                      readMagVCM5883L();
  }

  // --- Barometer (BMP280 forced mode: idle → start → wait → read → idle) ---
  if (sensorState.baroOk) {
    switch (sensorState.bmpPhase) {
      case BMP_IDLE:
        if ((now - sensorState.baroNextUs) >= BARO_POLL_US) {
          bmp280StartMeasurement();
          sensorState.bmpPhase = BMP_MEAS_WAIT;
          sensorState.bmpPhaseStartUs = now;
        }
        break;

      case BMP_MEAS_WAIT:
        // BMP280 forced mode at osrs_t=x2, osrs_p=x16: typical ~38ms, max ~44ms
        if ((now - sensorState.bmpPhaseStartUs) >= 44000UL) {
          int32_t pressurePa;
          int16_t tempC10;
          if (bmp280ReadResult(sensorState.bmpCal, pressurePa, tempC10)) {
            baroData.pressurePa = pressurePa;
            baroData.tempC10 = tempC10;

            baroData.altCmMSL = pressureToAltCm(baroData.pressurePa);

            if (baroData.altCalibrated) {
              baroData.altCmAGL = baroData.altCmMSL - baroData.groundAltCm;
            } else {
              baroData.altCmAGL = 0;
            }

            // Vertical velocity EMA
            if (sensorState.vvelValid) {
              float dt = (float)(now - sensorState.lastAltUs) / 1000000.0f;
              if (dt > 0.001f && dt < 1.0f) {  // sanity: 1ms to 1s
                float altM = (float)baroData.altCmMSL / 100.0f;
                float instantVvel = (altM - sensorState.lastAltM) / dt;
                sensorState.vvelEma += BARO_VVEL_ALPHA * (instantVvel - sensorState.vvelEma);
                baroData.vvel10 = (int16_t)(sensorState.vvelEma * 10.0f);
              }
            } else {
              sensorState.vvelEma = 0.0f;
              sensorState.vvelValid = true;
            }
            sensorState.lastAltM = (float)baroData.altCmMSL / 100.0f;
            sensorState.lastAltUs = now;

            baroData.valid = true;
            baroData.lastReadUs = now;
            logPages[LOGI_BARO].freshMask |= 0xFF;
          }

          sensorState.baroNextUs = now;
          sensorState.bmpPhase = BMP_IDLE;
        }
        break;
    }
  }
}

// Call when arming to set the ground-level reference for AGL altitude.
// Uses the current baro altitude as ground level.
inline void sensorsSetGroundLevel() {
  if (baroData.valid) {
    baroData.groundAltCm = baroData.altCmMSL;
    baroData.altCalibrated = true;
    baroData.altCmAGL = 0;
    Serial.print("Ground level set: ");
    Serial.print(baroData.groundAltCm);
    Serial.println(" cm MSL");
  }
}

// Reset ground level (e.g. on disarm if you want to recalibrate next arm)
inline void sensorsClearGroundLevel() {
  baroData.altCalibrated = false;
}

#endif // SENSORS_H
