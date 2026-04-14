// config.h — All compile-time constants for rocket avionics.
// No variables, no non-inline functions. Pure #defines and one inline utility.
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
// After CMD_SET_SYNC, both boards anchor their slot clock to RxDone/TxDone of that packet.

enum WindowMode : uint8_t {
  WIN_TELEM  = 0,  // rocket TX telemetry / base RX
  WIN_RX     = 1,  // base TX commands / rocket RX
  WIN_OFF    = 2,  // radio off — neither side active
  WIN_LR     = 3,  // future: long-range low-rate TX
  WIN_FINDME = 4,  // future: long-preamble beacon for passive scan without bootstrap
};

// Compile-time slot sequence. Edit here to change the pattern.
static const WindowMode SLOT_SEQUENCE[] = { WIN_TELEM, WIN_RX };
#define SLOT_SEQUENCE_LEN  2
#define SLOT_DURATION_US   4000000UL   // 4 seconds per slot

// startReceive(timeout) takes raw SX1262 timer units (1 unit = 15.625µs).
// Formula: us / 15.625  or  ms * 64

// Base station starts listening this many µs before WIN_TELEM slot start.
#define BS_RX_EARLY_US        50000UL    // 50ms early-listen
// Base RX window: 100ms = 100000µs / 15.625 = 6400 units.
#define BS_RX_TIMEOUT_RAW     6400UL
// Base TX aim point: transmit this many µs after WIN_RX slot start.
// Gives rocket time to arm startReceive before preamble arrives.
// Also the drift calibration reference — aim slightly after slot boundary.
#define BS_CMD_TX_OFFSET_US   10000UL   // 10ms after WIN_RX start

// Rocket WIN_RX listen window: 300ms = 300000µs / 15.625 = 19200 units.
#define ROCKET_RX_TIMEOUT_RAW 19200UL

// ===================== TX SCHEDULING (legacy, used pre-sync) =====================

// If TX is delayed >5ms past its scheduled time, count it as a delayed TX for stats.
#define TX_LATE_THRESHOLD_US  5000UL
// Hard ceiling: TX anyway after 1000ms of deferral regardless of channel state.
#define TX_MAX_DEFER_US       1000000UL
// Post-TX keepout: random gap before next TX to listen for incoming commands.
#define TX_KEEPOUT_MIN_US     10000UL
#define TX_KEEPOUT_RANGE_US   10000UL

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

// ===================== RSSI EMA =====================

#define RSSI_EMA_TAU_US         60000000UL  // 60 seconds
#define RSSI_BUSY_THRESHOLD_DB  20.0        // dB above EMA = channel busy
#define RSSI_SAMPLE_INTERVAL_US 500UL       // max sample rate
#define RSSI_HIGH_HOLDOFF_US    10000UL     // holdoff after high-RSSI event

// ===================== PACKET TYPE CONSTANTS =====================

#define PKT_TELEMETRY  0xAF
#define PKT_COMMAND    0x9A
#define PKT_BACKHAUL   0xE2
#define PKT_LOG_CHUNK  0xCA

// ===================== COMMAND IDs =====================

#define CMD_ARM           0x01
#define CMD_DISARM        0x02
#define CMD_FIRE_PYRO     0x03
#define CMD_SET_TX_RATE   0x10
#define CMD_SET_RADIO     0x12
#define CMD_LOG_DOWNLOAD  0x20
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

// Max BLE PDU payload in bytes. 517 MTU - 3 ATT header - 12 LL overhead = ~502.
// Using 502 allows 2 LL segments; 517 would spill into a 3rd.
#define BLE_MAX_PDU            502

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
