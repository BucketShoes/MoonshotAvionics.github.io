// ota.h — OTA firmware update state machine for rocket_avionics.
//
// Flow:
//   CMD_OTA_BEGIN (0x50)    — erases inactive OTA partition, opens write session
//   OTA char writes         — 512-byte chunks written sequentially (no app-level ack)
//   CMD_OTA_FINALIZE (0x51) — verifies HMAC of written image, sets boot partition, reboots
//   CMD_OTA_CONFIRM (0x52)  — after reboot: cancels rollback, keeps new firmware
//
// Arm/OTA mutual exclusion: OTA commands/chunks refused while armed;
// ARM refused if OTA state != OTA_LOCKED (enforced in flight.cpp).

#ifndef OTA_H
#define OTA_H

#include <Arduino.h>
#include <esp_ota_ops.h>

// OTA status bytes — notified on the OTA BLE characteristic (errors only;
// no per-chunk ack on success). Also used as HTTP response for base station.
#define OTA_STATUS_OK            0x00  // chunk accepted / operation succeeded
#define OTA_STATUS_NOT_ACTIVE    0x01  // CMD_OTA_BEGIN not sent yet
#define OTA_STATUS_WRITE_FAIL    0x02  // esp_ota_write / begin / erase failed
#define OTA_STATUS_OFFSET_GAP    0x03  // offset != bytesWritten
#define OTA_STATUS_HMAC_MISMATCH 0x04  // image HMAC verification failed
#define OTA_STATUS_OTA_END_FAIL  0x05  // esp_ota_end failed
#define OTA_STATUS_VERIFYING     0x06  // finalize in progress, no more chunks
#define OTA_STATUS_REFUSED       0x07  // armed or rollback pending

// Progress report prefix (5 bytes: [0xA0][bytesWritten u32 LE]) sent every ~1200 chunks.
#define OTA_PROGRESS_MARKER      0xA0
#define OTA_PROGRESS_INTERVAL    1200  // chunks between progress reports

enum OtaState {
  OTA_LOCKED,     // default; no session open
  OTA_ERASING,    // erasing inactive partition (blocking, inside CMD_OTA_BEGIN)
  OTA_RECEIVING,  // session open, accepting chunks
  OTA_VERIFYING,  // CMD_OTA_FINALIZE received, chunks blocked, verifying
};

struct OtaContext {
  OtaState              state;
  esp_ota_handle_t      handle;
  const esp_partition_t* partition;
  uint32_t              bytesWritten;
  uint32_t              chunkCount;   // used for periodic progress reporting
};

// Public API ---------------------------------------------------------------

// Called from executeCommand(CMD_OTA_BEGIN). Erases inactive partition and
// opens an esp_ota_begin session. Returns CMD_OK or CMD_ERR_*.
uint8_t otaHandleBegin();

// Called from the BLE OTA characteristic callback and HTTP handler.
// offset:    byte offset of this chunk in the firmware image
// data:      chunk payload (up to 508 bytes)
// len:       number of bytes in this chunk
// Queues a status notify via otaQueueNotify() on errors; sends a progress
// report (OTA_PROGRESS_MARKER + bytesWritten) every OTA_PROGRESS_INTERVAL chunks.
void otaHandleChunk(uint32_t offset, const uint8_t* data, size_t len);

// Called from executeCommand(CMD_OTA_FINALIZE).
// firmwareHmac: 32-byte HMAC-SHA256(hmacKey, firmware_binary)
// expectedSize: total byte count of the firmware image
// Returns CMD_OK or CMD_ERR_*.
uint8_t otaHandleFinalize(const uint8_t* firmwareHmac, uint32_t expectedSize);

// Called from executeCommand(CMD_OTA_CONFIRM).
// Marks the running firmware as valid, cancelling the pending rollback.
// Returns CMD_OK.
uint8_t otaHandleConfirm();

// Returns the current OTA state (used by flight.cpp for arm guard).
OtaState otaGetState();

// Notify a single status byte on the OTA BLE characteristic.
// Wraps ble.h otaQueueNotify() so callers (commands.cpp) don't need to include ble.h.
void otaNotifyStatus(uint8_t status);

#endif // OTA_H
