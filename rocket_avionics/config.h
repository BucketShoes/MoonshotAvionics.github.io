// config.h — All compile-time constants for rocket avionics.
// No variables, no non-inline functions. Pure #defines, enums, structs, and inline utilities.
// Change hardware pins, protocol constants, or timing here without touching anything else.

#ifndef CONFIG_H
#define CONFIG_H

// ===================== PIN DEFINITIONS =====================

#define GPS_RX_PIN    33
#define GPS_TX_PIN    34
#define GPS_RST_PIN   35
#define VEXT_CTRL_PIN 3

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

#define USER_BTN_PIN  0
#define LED_PIN       18

// Battery voltage divider
#define VBAT_ADC_PIN      1
#define VBAT_ADC_CTRL_PIN 2
#define VBAT_MULTIPLIER   4.9f

// ===================== GENERAL CONFIG =====================

#define GPS_BAUD      115200
#define SERIAL_BAUD   115200
#define DEVICE_ID     0x92

// ===================== LORA RADIO DEFAULTS =====================
// BW is derived from channel index (ch<64 = 125kHz, ch>=64 = 500kHz).

#define LORA_CR         5       // 4/5 coding rate (fixed, not command-configurable)
#define LORA_PREAMBLE   6       // preamble symbols
#define LORA_SYNCWORD   0x12    // private network

// Default channel if NVS is erased (0xFF). Channel 65 = BW500, 917.5 MHz.
#define DEFAULT_CHANNEL  65
#define DEFAULT_SF       5
#define DEFAULT_POWER    -9     // dBm (bench testing)

// ===================== CHANNEL TABLE (AU915-aligned) =====================
// Channel 0-63:  BW125, 915.2 + ch*0.2 MHz
// Channel 64-71: BW500, 915.9 + (ch-64)*1.6 MHz

#define CHANNEL_COUNT 72

// ===================== SLOT TIMING SYNC =====================
// Slot-based scheduling for time-sync between rocket and base station.
// Both sides share the same slot sequence and duration.
// After CMD_SET_SYNC, the base anchors to TxDone of that packet; the rocket
// anchors to RxDone. Both compute slot position from that anchor.

enum WindowMode : uint8_t {
  WIN_TELEM  = 0,  // rocket TX telemetry / base RX
  WIN_CMD     = 1,  // base TX commands / rocket RX
  WIN_OFF    = 2,  // radio off — neither side active
  WIN_LR     = 3,  // future: long-range low-rate TX with very small payload
  WIN_FINDME = 4,  // future: long-preamble beacon for passive scan without bootstrap
};

// Compile-time slot sequence. Edit here to change the pattern.
static const WindowMode SLOT_SEQUENCE[] = { WIN_TELEM, WIN_CMD };
#define SLOT_SEQUENCE_LEN  2
#define SLOT_DURATION_US   1'000'000UL  // µs

// Rocket WIN_CMD RX timeouts (converted to RTC steps via /15.625 at use site).
#define ROCKET_RX_TIMEOUT_US           100'000UL              // short: synced, heard command recently
#define ROCKET_LONG_RX_TIMEOUT_US      (SLOT_DURATION_US - 200'000UL)  // long: synced, no command in 2min
#define ROCKET_PRESYNC_RX_TIMEOUT_US   (SLOT_DURATION_US - 20'000UL)   // pre-sync: nearly full slot

// When to switch from short to long listen (no verified command heard).
#define ROCKET_CMD_SILENCE_THRESHOLD_US  120'000'000UL  // 2 minutes




// ===================== SLOT CONFIG =====================
// Per-slot radio parameters. Enables per-slot frequency, SF, BW, and header mode.
// All slots currently use the same config (populated from activeChannel/SF/power).
// SlotConfig is used by radioApplySlotConfig() when parameters differ between slots.

#include "sx126x.h"  // sx126x_lora_sf_t, sx126x_lora_bw_t, etc.

struct SlotConfig {
  uint32_t              freqHz;
  sx126x_lora_sf_t      sf;
  sx126x_lora_bw_t      bw;
  sx126x_lora_cr_t      cr;
  uint16_t              preambleSymbols;
  sx126x_lora_pkt_len_modes_t headerType;  // explicit or implicit
  uint8_t               payloadLen;        // used when implicit header
};

// ===================== TIMING =====================

#define BTN_DEBOUNCE_US        50000UL
#define VBAT_READ_INTERVAL_MS  2000
#define VBAT_SETTLE_US         400
// Minimum loop iteration time (prevents micros() math issues at >100kHz).
#define MIN_LOOP_US            10UL
// Slow loop blame threshold: only report iterations longer than this (unarmed).
#define SLOW_LOOP_THRESHOLD_US 10000UL

// ===================== HMAC / NVS =====================

#define HMAC_KEY_LEN    32
#define HMAC_TRUNC_LEN  10   // truncated HMAC in command packets
#define NVS_NAMESPACE   "rocket"

// ===================== PACKET TYPE CONSTANTS =====================

#define PKT_TELEMETRY  0xAF
#define PKT_COMMAND    0x9A
#define PKT_BACKHAUL   0xE2
#define PKT_LOG_CHUNK  0xCA

// ===================== DATA PAGE TYPE CONSTANTS =====================

#define PAGE_THRUST_CURVE      0x0E   // X-axis accel ring buffer (variable length, not logged to flash)

// ===================== THRUST CURVE =====================

#define THRUST_BUF_SIZE        2048   // int16_t samples; 4096 bytes RAM
#define THRUST_SAMPLE_RATE_HZ  200    // target capture rate (5000µs period)

// ===================== COMMAND IDs =====================

#define CMD_ARM           0x01
#define CMD_DISARM        0x02
#define CMD_FIRE_PYRO     0x03
#define CMD_SET_TX_RATE   0x10
#define CMD_SET_RADIO     0x12
// CMD 0x20 (LOG_DOWNLOAD) removed — superseded by BLE log fetch
#define CMD_OTA_BEGIN     0x50  // Open OTA write session (erases inactive partition)
#define CMD_OTA_FINALIZE  0x51  // Verify image HMAC, set boot partition, reboot
#define CMD_OTA_CONFIRM   0x52  // After reboot: confirm new firmware, cancel rollback
#define CMD_PING          0x40
#define CMD_SET_SYNC      0x41  // Set slot-clock sync point (0 params; anchor = RxDone of this packet)
#define CMD_REBOOT        0xF0
#define CMD_LOG_ERASE     0xF1

// ===================== COMMAND ACK RESULT CODES =====================

#define CMD_OK              0x00
#define CMD_ERR_UNKNOWN     0x01
#define CMD_ERR_REFUSED     0x02
#define CMD_ERR_BAD_PARAMS  0x03

// ===================== GPS FRACTIONAL ENCODING STATUS CODES =====================

#define GPS_FRAC_VALID_MAX    49999
#define GPS_FRAC_NO_FIX       65533
#define GPS_FRAC_INITIALISING 65534
#define GPS_FRAC_NOT_POWERED  65535

// ===================== LOG =====================

#define LOG_SNR_LOCAL  0x7F   // SNR value meaning "locally generated, not received"

// ===================== BLE =====================

// Service UUID: "ROCKETSTBL" — 52 4F 43 4B-4554-5354-424C-000000000000
#define BLE_DEVICE_NAME        "Moonshot-Rocket"
#define BLE_SERVICE_UUID       "524f434b-4554-5354-424c-000000000000"
#define BLE_TELEM_CHAR_UUID    "524f434b-4554-5354-424c-000000000001"  // NOTIFY
#define BLE_CMD_CHAR_UUID      "524f434b-4554-5354-424c-000000000002"  // WRITE|WRITE_NR
#define BLE_STATUS_CHAR_UUID   "524f434b-4554-5354-424c-000000000003"  // READ
#define BLE_CONNSET_CHAR_UUID  "524f434b-4554-5354-424c-000000000004"  // WRITE|WRITE_NR
#define BLE_LOGFETCH_CHAR_UUID "524f434b-4554-5354-424c-000000000005"  // WRITE|NOTIFY
#define BLE_OTA_CHAR_UUID      "524f434b-4554-5354-424c-000000000006"  // WRITE|WRITE_NR|NOTIFY

// ConnSet command types
#define BLE_CONNSET_INTERVAL  0x01   // [intervalUs u32]
#define BLE_CONNSET_PAGEMASK  0x02   // [mask u64]
#define BLE_CONNSET_PHY       0x03   // [phy u8] 0=1M 1=2M 2=Coded-S2 3=Coded-S8

// Max BLE PDU payload for log fetch in bytes. 517 MTU - 3 ATT header - 12 LL overhead = ~502.
// Using 502 allows 2 LL segments; 517 would spill into a 3rd.
// NOTE: for thrust curve telem the real ATT limit (517) applies — use 517 directly there.
#define BLE_LOGFETCH_MAX_PDU   502

// Advertising interval in 0.625ms units. 1600 = 1000ms.
#define BLE_ADV_INTERVAL       1600

// Connection interval parameters (units: 1.25ms)
#define BLE_CI_FAST_MIN        6     // 7.5ms — for 2M and 1M PHY
#define BLE_CI_FAST_MAX        6
#define BLE_CI_FAST_LATENCY    0
#define BLE_CI_FAST_TIMEOUT    800   // 1000ms

#define BLE_CI_SLOW_MIN        24    // 30ms — for Coded PHY
#define BLE_CI_SLOW_MAX        40    // 50ms
#define BLE_CI_SLOW_LATENCY    4
#define BLE_CI_SLOW_TIMEOUT    3200  // 4000ms

// Default BLE subscription interval (microseconds). 0 = every loop.
#define BLE_DEFAULT_INTERVAL_US  1000000UL  // 1Hz

// Default page mask: pages 0x01-0x0D (bits 1-13 set). 0x07 included for future Kalman.
// Bit N corresponds to page type N.
#define BLE_DEFAULT_PAGE_MASK  0x0000000000003FFEULL

// Overflow buffer size for BLE multi-PDU captures
#define BLE_OVF_BUF_SIZE       4096

// ===================== TX RATE UTILITY =====================
// TX rate encoding: positive = Hz, negative = seconds-between-packets, 0 = disabled.
// +1 and -1 both mean 1Hz (1s interval).

static inline unsigned long txRateToIntervalUs(int8_t rate) {
  if (rate == 0) return 0;
  if (rate > 0)  return 1000000UL / (unsigned long)rate;
  return (unsigned long)(-(int)rate) * 1000000UL;
}

#endif // CONFIG_H
