// ota.cpp — OTA firmware update implementation for rocket_avionics.
// See ota.h for the public API and flow description.

#include <Arduino.h>
#include <esp_ota_ops.h>
#include <mbedtls/md.h>
#include "ota.h"
#include "ble.h"       // otaQueueNotify()
#include "commands.h"  // hmacKey, HMAC_KEY_LEN, CMD_OK, CMD_ERR_*
#include "config.h"

// ===================== STATE =====================

static OtaContext otaCtx = {
  OTA_LOCKED, 0, nullptr, 0, 0
};

OtaState otaGetState() { return otaCtx.state; }

// ===================== BEGIN =====================

uint8_t otaHandleBegin() {
  // Refuse if a rollback is pending (another OTA slot booted but not confirmed).
  // esp_ota_check_rollback_is_possible() returns true when the running app was
  // set via esp_ota_set_boot_partition and has not yet called
  // esp_ota_mark_app_valid_cancel_rollback().
  if (esp_ota_check_rollback_is_possible()) {
    Serial.println("OTA: begin refused — rollback pending, confirm first");
    return CMD_ERR_REFUSED;
  }

  if (otaCtx.state != OTA_LOCKED) {
    Serial.println("OTA: begin refused — session already active");
    return CMD_ERR_REFUSED;
  }

  // Determine inactive OTA partition (alternates app0/app1 depending on running slot).
  const esp_partition_t* part = esp_ota_get_next_update_partition(NULL);
  if (!part) {
    Serial.println("OTA: no update partition found");
    return CMD_ERR_REFUSED;
  }

  Serial.printf("OTA: erasing partition '%s' (0x%05lx, %lu bytes)...\n",
    part->label, (unsigned long)part->address, (unsigned long)part->size);
  otaCtx.state = OTA_ERASING;
  otaCtx.partition = part;

  // Erase the entire target partition in 4KB sectors.
  // vTaskDelay(1) every sector feeds the task watchdog (same pattern as log_store.h).
  for (uint32_t off = 0; off < part->size; off += 4096) {
    esp_err_t err = esp_partition_erase_range(part, off, 4096);
    if (err != ESP_OK) {
      Serial.printf("OTA: erase failed at 0x%05lx: %d\n", (unsigned long)off, err);
      otaCtx.state = OTA_LOCKED;
      return CMD_ERR_REFUSED;
    }
    vTaskDelay(1);
  }
  Serial.println("OTA: erase complete");

  esp_ota_handle_t handle;
  esp_err_t err = esp_ota_begin(part, part->size, &handle);
  if (err != ESP_OK) {
    Serial.printf("OTA: esp_ota_begin failed: %d\n", err);
    otaCtx.state = OTA_LOCKED;
    return CMD_ERR_REFUSED;
  }

  otaCtx.handle = handle;
  otaCtx.bytesWritten = 0;
  otaCtx.chunkCount = 0;
  otaCtx.state = OTA_RECEIVING;
  Serial.println("OTA: session open, ready for chunks");
  return CMD_OK;
}

// ===================== CHUNK HANDLER =====================

void otaHandleChunk(uint32_t offset, const uint8_t* data, size_t len) {
  if (otaCtx.state == OTA_VERIFYING) {
    otaQueueNotify(OTA_STATUS_VERIFYING);
    return;
  }
  if (otaCtx.state != OTA_RECEIVING) {
    otaQueueNotify(OTA_STATUS_NOT_ACTIVE);
    return;
  }

  if (len == 0) {
    otaQueueNotify(OTA_STATUS_NOT_ACTIVE);
    return;
  }

  if (offset != otaCtx.bytesWritten) {
    Serial.printf("OTA: offset gap: expected %lu got %lu\n",
      (unsigned long)otaCtx.bytesWritten, (unsigned long)offset);
    otaQueueNotify(OTA_STATUS_OFFSET_GAP);
    return;
  }

  esp_err_t err = esp_ota_write(otaCtx.handle, data, len);
  if (err != ESP_OK) {
    Serial.printf("OTA: write failed: %d\n", err);
    esp_ota_abort(otaCtx.handle);
    otaCtx.state = OTA_LOCKED;
    otaQueueNotify(OTA_STATUS_WRITE_FAIL);
    return;
  }

  otaCtx.bytesWritten += (uint32_t)len;
  otaCtx.chunkCount++;

  // Send periodic progress report every OTA_PROGRESS_INTERVAL chunks.
  // Format: [0xA0][bytesWritten u32 LE] (5 bytes)
  if (otaCtx.chunkCount % OTA_PROGRESS_INTERVAL == 0) {
    uint8_t progress[5];
    progress[0] = OTA_PROGRESS_MARKER;
    progress[1] = (uint8_t)(otaCtx.bytesWritten);
    progress[2] = (uint8_t)(otaCtx.bytesWritten >> 8);
    progress[3] = (uint8_t)(otaCtx.bytesWritten >> 16);
    progress[4] = (uint8_t)(otaCtx.bytesWritten >> 24);
    otaQueueNotifyBytes(progress, 5);
  }
  // No per-chunk success notify — rely on LL-level ACK for flow control.
}

// ===================== FINALIZE =====================

uint8_t otaHandleFinalize(const uint8_t* firmwareHmac, uint32_t expectedSize) {
  if (otaCtx.state != OTA_RECEIVING) return CMD_ERR_REFUSED;

  // Block further chunks immediately.
  otaCtx.state = OTA_VERIFYING;

  if (otaCtx.bytesWritten != expectedSize) {
    Serial.printf("OTA: size mismatch: written %lu, expected %lu\n",
      (unsigned long)otaCtx.bytesWritten, (unsigned long)expectedSize);
    esp_ota_abort(otaCtx.handle);
    otaCtx.state = OTA_LOCKED;
    return CMD_ERR_BAD_PARAMS;
  }

  // Read back the written image and compute HMAC-SHA256(hmacKey, data[0..expectedSize)).
  uint8_t computed[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, hmacKey, HMAC_KEY_LEN);

  uint8_t readBuf[256];
  uint32_t pos = 0;
  uint32_t remaining = expectedSize;
  bool readOk = true;
  while (remaining > 0) {
    uint32_t n = (remaining < sizeof(readBuf)) ? remaining : (uint32_t)sizeof(readBuf);
    esp_err_t err = esp_partition_read(otaCtx.partition, pos, readBuf, n);
    if (err != ESP_OK) { readOk = false; break; }
    mbedtls_md_hmac_update(&ctx, readBuf, n);
    pos += n;
    remaining -= n;
    vTaskDelay(0);  // yield without delay to keep watchdog fed during long read
  }
  mbedtls_md_hmac_finish(&ctx, computed);
  mbedtls_md_free(&ctx);

  if (!readOk) {
    Serial.println("OTA: partition read failed during verify");
    esp_ota_abort(otaCtx.handle);
    otaCtx.state = OTA_LOCKED;
    return CMD_ERR_REFUSED;
  }

  // Constant-time comparison of all 32 bytes.
  uint8_t diff = 0;
  for (int i = 0; i < 32; i++) diff |= computed[i] ^ firmwareHmac[i];
  if (diff != 0) {
    Serial.println("OTA: HMAC mismatch — rejecting image");
    esp_ota_abort(otaCtx.handle);
    otaCtx.state = OTA_LOCKED;
    otaQueueNotify(OTA_STATUS_HMAC_MISMATCH);
    return CMD_ERR_REFUSED;
  }

  esp_err_t err = esp_ota_end(otaCtx.handle);
  if (err != ESP_OK) {
    Serial.printf("OTA: esp_ota_end failed: %d\n", err);
    otaCtx.state = OTA_LOCKED;
    otaQueueNotify(OTA_STATUS_OTA_END_FAIL);
    return CMD_ERR_REFUSED;
  }

  err = esp_ota_set_boot_partition(otaCtx.partition);
  if (err != ESP_OK) {
    Serial.printf("OTA: set_boot_partition failed: %d\n", err);
    otaCtx.state = OTA_LOCKED;
    return CMD_ERR_REFUSED;
  }

  Serial.println("OTA: image verified, boot partition set — rebooting");
  otaQueueNotify(OTA_STATUS_OK);
  delay(500);  // allow the notify to drain before reboot
  esp_restart();
  return CMD_OK;  // unreachable
}

// ===================== CONFIRM =====================

uint8_t otaHandleConfirm() {
  esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
  if (err != ESP_OK) {
    Serial.printf("OTA: confirm failed: %d\n", err);
    return CMD_ERR_REFUSED;
  }
  Serial.println("OTA: firmware confirmed, rollback cancelled");
  return CMD_OK;
}
