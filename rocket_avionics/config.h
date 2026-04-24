// config.h — All compile-time constants for rocket avionics.
// No variables, no non-inline functions. Pure #defines, enums, structs, and inline utilities.
// Change hardware pins, protocol constants, or timing here without touching anything else.

#ifndef CONFIG_H
#define CONFIG_H

// Board-specific pin assignments (switchable per -DBOARD_* build flag)
#include "board_config.h"

// ===================== PIN DEFINITIONS (shared across all boards) =====================

#define LORA_NSS_PIN  8
#define LORA_SCK_PIN  9
#define LORA_MOSI_PIN 10
#define LORA_MISO_PIN 11
#define LORA_RST_PIN  12
#define LORA_BUSY_PIN 13
#define LORA_DIO1_PIN 14

#define USER_BTN_PIN  0

// Battery voltage divider multiplier (applies to all boards)
#define VBAT_MULTIPLIER   4.9f

// ===================== GENERAL CONFIG =====================

// GPS_BAUD is defined in board_config.h (board-specific: 115200 for Tracker, 9600 for V4)
#define SERIAL_BAUD   115200
#define DEVICE_ID     0x92

// ===================== LORA RADIO DEFAULTS =====================
// BW is derived from channel index (ch<64 = 125kHz, ch>=64 = 500kHz).

#define LORA_CR         5       // 4/5 coding rate (fixed, not command-configurable)
#define LORA_PREAMBLE   6       // preamble symbols
#define LORA_SYNCWORD   0x12    // private network
// WIN_LR slot config. BW follows activeChannel (same as normal slots — changeable via CMD_SET_RADIO,
// and BW500 is fine for bench testing). SF is separate because LR uses a higher SF than normal
// telem but you may want to reduce it during bench testing to shorten air time.
// WIN_LR spreading factor. Change here for bench testing (lower SF = shorter airtime).
#define LORA_LR_SF    11
// WIN_LR coding rate. 0x05 = 4/5 long-interleave (LI). Not in sx126x_driver enum; cast at use site.
// TODO: @@@ confirm whether LI is causing the stuck-BUSY seen in testing, or find real cause.
// If LI is the problem, replace with SX126X_LORA_CR_4_5 (standard 4/5 = 0x01).
#define LORA_LR_CR    SX126X_LORA_CR_4_5 //SX126X_LORA_CR_4_5 for normal 4/5 or li4/5 is 0x05

// HV sense threshold: ADC mV above which high-side power is considered present
// Assumes 10k:100k divider (11× attenuation). 3V on high side → ~273mV at ADC.
// We use 250mV (≈2.75V high-side) as the threshold to be slightly conservative.
#define PYRO_HV_PRESENT_MV    2750   // high-side mV threshold for hvPresent flag
#define PYRO_HV_DIVIDER_RATIO 11     // (10k + 100k) / 10k

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
  WIN_TELEM  = 0,  // rocket TX telemetry / base RX - using hopping channel with user-configurable telem modulation
  WIN_CMD     = 1, // base TX commands / rocket RX - using the command channel, with command modulation params (typically same modulation as telem)
  WIN_OFF    = 2,  // radio off — neither side active, power save
  WIN_LR     = 3,  // long-range low-rate TX at high sf, implicit headers, etc
  WIN_FINDME = 4,  //  long-preamble beacon for passive scan without bootstrap, on specific modulation regardless of radio settings
  WIN_BACKHAUL = 5, //timeslot reserved for relay backhaul beween multiple bae stations - rocket stays quiet, but listens similar to a WIN_CMD but on hop channel with backhaul modulation. not valid for sending normal commands, but can help sync
  WIN_MULTIPURPOSE = 6,//long slow cycle between various functions. full anchor timing and state machine required to know which modulation to use.
  WIN_RDF = 7, //for radio distance/direction finding. send multiple short packets at variable signal strength and changing SF/BW for improved distance estimation
  WIN_GFSK = 8,//high data rate, mutiple packets in succession
};

// Compile-time slot sequence. Edit here to change the pattern.
static const WindowMode SLOT_SEQUENCE[] = { WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_TELEM, WIN_CMD, WIN_LR, WIN_CMD, };
#define SLOT_SEQUENCE_LEN   (sizeof(SLOT_SEQUENCE) / sizeof(SLOT_SEQUENCE[0]))
#define SLOT_DURATION_US    420'000UL //how long between the timing points where messages are sent/listened for. note that this may change in futue, and some comments incorrectly assume itll always be this long.


// Rocket WIN_CMD RX timeouts (converted to RTC steps via /15.625 at use site).
#define ROCKET_RX_TIMEOUT_US           100'000UL                        // short: synced + heard base recently, save battery
#define ROCKET_LONG_RX_TIMEOUT_US      (SLOT_DURATION_US - 20'000UL)    // long: pre-sync, or base-silent lost-rocket fallback

// Safety cutoff: force standby if RX has been active for more than this many slot durations.
// Indicates a missed DIO1 IRQ or stuck DIO1 line. Mirrors base station's RX_STUCK_MAX_SLOTS.
#define RX_STUCK_MAX_SLOTS             20UL

// When to widen RX window: no valid command heard from base for this long.
// "Silence" is the wrong framing — signal can be fine but we may have lost sync
// (base reboot, drift past tolerance, etc). Opening a long RX window lets the
// base's next CMD_SET_SYNC land even when short-window timing has drifted.
// CRITICAL for lost-rocket recovery: if the rocket is out in a field, this is
// the only way it can re-sync without manual power-cycle.
// 124s = slightly more than 2 ping intervals (60s each), so rounding/jitter
// cannot make this fire after missing only one ping.
// Widening the window does NOT clear radioSynced and does NOT trigger any TX.
// The main putrpose of this time is to safe power when we know we are safe.
#define ROCKET_NO_BASE_HEARD_THRESHOLD_US  130'000'000UL  //  (>2 ping intervals)


#define LOG_RX_DONE true
#define LOG_RX_TIMEOUT false
#define LOG_RX_START false
#define LOG_TX_START true
#define LOG_TX_DONE true



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
#define SLOW_LOOP_THRESHOLD_US 2000UL

// ===================== HMAC / NVS =====================

#define HMAC_KEY_LEN    32
#define HMAC_TRUNC_LEN  10   // truncated HMAC in command packets
#define NVS_NAMESPACE   "rocket"

// ===================== PACKET TYPE CONSTANTS =====================

#define PKT_TELEMETRY  0xAF
#define PKT_COMMAND    0x9A
#define PKT_BACKHAUL   0xE2
#define PKT_LOG_CHUNK  0xCA
#define PKT_LONGRANGE  0xBB

// ===================== DATA PAGE TYPE CONSTANTS =====================

#define PAGE_THRUST_CURVE      0x0E   // X-axis accel ring buffer (variable length, not logged to flash)
#define PAGE_PYRO_STATUS       0x0F   // Pyro channel state, continuity, HV sense (6 bytes)

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
