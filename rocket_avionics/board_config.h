// board_config.h — Board-specific pin assignments.
// Included by config.h. All other code gets pins from config.h as before.
// Select a board by setting -DBOARD_WIRELESS_TRACKER or -DBOARD_LORA32_V4 in build_flags.
//
// External wiring for BOARD_LORA32_V4 (pins that require manual wiring):
//   Pyro CH1 (drogue gate)    GPIO 45
//   Pyro CH2 (main gate)      GPIO 47   (NOT 46 — FEM_PA)
//   Pyro CH3 (cut gate)       GPIO 48   (verify not locked to PSRAM SubSPI before committing)
//   Pyro sense CH1            GPIO 15
//   Pyro sense CH2            GPIO 16
//   Pyro sense CH3            GPIO 6
//   HV sense                  GPIO 3
//   Sensor I2C SDA            GPIO 4
//   Sensor I2C SCL            GPIO 5
//   GPS RST (L76K RESET_N)    GPIO 19   (other GPS signals via GNSS connector on 33/34)

#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#if defined(BOARD_WIRELESS_TRACKER)

  #define VEXT_CTRL_PIN      3
  #define LED_PIN            18
  #define VBAT_ADC_PIN       1
  #define VBAT_ADC_CTRL_PIN  2
  #define VBAT_MULTIPLIER    4.9f
  #define GPS_BAUD           115200   // UC6580 default
  #define GPS_RX_PIN         33
  #define GPS_TX_PIN         34
  #define GPS_RST_PIN        35
  #define SENSOR_SDA_PIN     4
  #define SENSOR_SCL_PIN     5
  #define PYRO_CH1_PIN       45
  #define PYRO_CH2_PIN       46
  #define PYRO_CH3_PIN       6
  #define PYRO_SENSE_CH1_PIN 39
  #define PYRO_SENSE_CH2_PIN 40
  #define PYRO_SENSE_CH3_PIN 41
  #define PYRO_HV_SENSE_PIN  42

#elif defined(BOARD_LORA32_V4)

  #define VEXT_CTRL_PIN      36
  #define LED_PIN            35
  #define VBAT_ADC_PIN       1
  #define VBAT_ADC_CTRL_PIN  37
  #define VBAT_MULTIPLIER    4.9f   // same BJT+100k/390k divider — verify against V4 schematic PDF
  #define GPS_BAUD           9600   // L76K default baud
  #define GPS_RX_PIN         33
  #define GPS_TX_PIN         34
  #define GPS_RST_PIN        19     // 35=LED, 42=GNSS_RST on connector
  #define SENSOR_SDA_PIN     4
  #define SENSOR_SCL_PIN     5     // free on V4 (FEM_CTL=7, FEM_EN=2)
  #define PYRO_CH1_PIN       45
  #define PYRO_CH2_PIN       47    // 46=FEM_PA on V4
  #define PYRO_CH3_PIN       48    // verify GPIO47/48 free (not locked to PSRAM SubSPI)
  #define PYRO_SENSE_CH1_PIN 15    // ADC-capable, free
  #define PYRO_SENSE_CH2_PIN 16    // ADC-capable, free
  #define PYRO_SENSE_CH3_PIN 6     // ADC-capable, free
  #define PYRO_HV_SENSE_PIN  3     // ADC-capable, free
  // FEM front-end amp — must be driven for radio to work
  #define LORA_FEM_EN_PIN    2
  #define LORA_FEM_PA_PIN    46
  #define LORA_FEM_CTL_PIN   7

#else
  #error "No board defined. Add -DBOARD_WIRELESS_TRACKER or -DBOARD_LORA32_V4 to build_flags."
#endif

// LoRa SPI — identical on both boards
#define LORA_NSS_PIN   8
#define LORA_SCK_PIN   9
#define LORA_MOSI_PIN  10
#define LORA_MISO_PIN  11
#define LORA_RST_PIN   12
#define LORA_BUSY_PIN  13
#define LORA_DIO1_PIN  14
#define USER_BTN_PIN   0

#endif // BOARD_CONFIG_H
