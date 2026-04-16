// radio_hal.cpp — SX126x HAL implementation for ESP32-S3 (Arduino SPI).
// Implements the four HAL functions declared in Lora-net/sx126x_driver's sx126x_hal.h,
// plus MCPWM hardware capture for jitter-free DIO1 timestamps.

#include <Arduino.h>
#include <SPI.h>
#include "driver/mcpwm_prelude.h"  // ESP-IDF MCPWM capture API
#include "radio_hal.h"             // our header: sx126x_hal_context_t, dio1Fired, radioMcpwmInit
#include "sx126x_hal.h"            // driver's header: sx126x_hal_status_t, SX126X_NOP, and the
                                   // four HAL function prototypes we implement here

// ===================== DIO1 CAPTURE GLOBALS =====================

volatile bool     dio1Fired      = false;
volatile uint32_t dio1CaptureVal = 0;

// ===================== MCPWM CAPTURE =====================

static bool IRAM_ATTR mcpwmCaptureIsr(mcpwm_cap_channel_handle_t /*chan*/,
                                      const mcpwm_capture_event_data_t* data,
                                      void* /*user_data*/) {
    dio1CaptureVal = (uint32_t)data->cap_value;
    dio1Fired      = true;
    return false;  // no higher-priority task woken
}

void radioMcpwmInit(uint8_t dio1Pin) {
    // Capture timer at 80 MHz (APB clock, no prescaler).
    mcpwm_cap_timer_handle_t capTimer = nullptr;
    mcpwm_capture_timer_config_t timerCfg = {};
    timerCfg.clk_src    = MCPWM_CAPTURE_CLK_SRC_DEFAULT;  // APB ~80 MHz
    timerCfg.group_id   = 0;
    timerCfg.resolution_hz = 0;  // 0 = use clock source directly (no divider)
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&timerCfg, &capTimer));

    // Capture channel on DIO1, rising edge only.
    mcpwm_cap_channel_handle_t capChan = nullptr;
    mcpwm_capture_channel_config_t chanCfg = {};
    chanCfg.gpio_num      = (int)dio1Pin;
    chanCfg.prescale      = 1;
    chanCfg.flags.pos_edge = true;
    chanCfg.flags.neg_edge = false;
    chanCfg.flags.pull_up  = false;
    chanCfg.flags.pull_down = false;
    chanCfg.flags.invert_cap_signal = false;
    chanCfg.flags.io_loop_back = false;
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(capTimer, &chanCfg, &capChan));

    // Register ISR callback (runs in IRAM, no FreeRTOS heap).
    mcpwm_capture_event_callbacks_t cbs = {};
    cbs.on_cap = mcpwmCaptureIsr;
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(capChan, &cbs, nullptr));

    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(capChan));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(capTimer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(capTimer));
}

// ===================== SPI HELPERS =====================

static inline const sx126x_hal_context_t* ctx_cast(const void* ctx) {
    return reinterpret_cast<const sx126x_hal_context_t*>(ctx);
}

// ===================== HAL WRITE =====================
// Called by sx126x_driver for every command sent to the radio.
// Sequence: check BUSY → assert NSS → send command bytes → send data bytes → deassert NSS.

extern "C" sx126x_hal_status_t sx126x_hal_write(
    const void*    context,
    const uint8_t* command,
    const uint16_t command_length,
    const uint8_t* data,
    const uint16_t data_length)
{
    const sx126x_hal_context_t* c = ctx_cast(context);

    if (digitalRead(c->busy)) {
        // Radio still processing — missed the window. Drop, don't spin.
        Serial.println("HAL: BUSY on write — dropped");
        return SX126X_HAL_STATUS_ERROR;
    }

    c->spi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(c->nss, LOW);

    for (uint16_t i = 0; i < command_length; i++) {
        c->spi->transfer(command[i]);
    }
    for (uint16_t i = 0; i < data_length; i++) {
        c->spi->transfer(data[i]);
    }

    digitalWrite(c->nss, HIGH);
    c->spi->endTransaction();

    return SX126X_HAL_STATUS_OK;
}

// ===================== HAL READ =====================
// Called by sx126x_driver to read responses (IRQ status, FIFO data, packet status, etc.).
// Sequence: check BUSY → assert NSS → send command bytes → read data bytes → deassert NSS.
// The SX126x read protocol inserts one status byte between command and data; the driver
// handles this by including the NOP byte in the command buffer it passes us.

extern "C" sx126x_hal_status_t sx126x_hal_read(
    const void*    context,
    const uint8_t* command,
    const uint16_t command_length,
    uint8_t*       data,
    const uint16_t data_length)
{
    const sx126x_hal_context_t* c = ctx_cast(context);

    if (digitalRead(c->busy)) {
        Serial.println("HAL: BUSY on read — dropped");
        return SX126X_HAL_STATUS_ERROR;
    }

    c->spi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(c->nss, LOW);

    for (uint16_t i = 0; i < command_length; i++) {
        c->spi->transfer(command[i]);
    }
    for (uint16_t i = 0; i < data_length; i++) {
        data[i] = c->spi->transfer(SX126X_NOP);
    }

    digitalWrite(c->nss, HIGH);
    c->spi->endTransaction();

    return SX126X_HAL_STATUS_OK;
}

// ===================== HAL RESET =====================
// Pulse RST low for 1 ms then release. The chip needs ~3 ms after RST high
// before it accepts SPI commands — caller must wait (via non-blocking timer,
// not a spin-wait) before proceeding.

extern "C" sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    const sx126x_hal_context_t* c = ctx_cast(context);
    pinMode(c->rst, OUTPUT);
    digitalWrite(c->rst, LOW);
    delay(1);  // 1 ms — only called during init, not from main loop
    digitalWrite(c->rst, HIGH);
    // Caller waits for BUSY to go low before the next SPI call.
    return SX126X_HAL_STATUS_OK;
}

// ===================== HAL WAKEUP =====================
// Wake the radio from sleep by toggling NSS. The chip drives BUSY high
// during the wake sequence; the HAL's BUSY check in subsequent calls
// handles the wait naturally (returns error if still busy, caller retries).

extern "C" sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    const sx126x_hal_context_t* c = ctx_cast(context);
    // A brief NSS toggle (without BUSY check) wakes the chip from sleep.
    c->spi->beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
    digitalWrite(c->nss, LOW);
    delayMicroseconds(1);
    digitalWrite(c->nss, HIGH);
    c->spi->endTransaction();
    return SX126X_HAL_STATUS_OK;
}
