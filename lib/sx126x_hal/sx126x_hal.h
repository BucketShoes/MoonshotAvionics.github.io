// sx126x_hal.h — SX126x hardware abstraction layer for ESP32-S3 (Arduino/SPI).
// Implements the four HAL functions required by Lora-net/sx126x_driver:
//   sx126x_hal_write, sx126x_hal_read, sx126x_hal_reset, sx126x_hal_wakeup
// Also owns MCPWM hardware capture on DIO1 for jitter-free timestamps.
//
// Usage:
//   1. Fill an sx126x_hal_context_t with your SPI instance and pin numbers.
//   2. Call radioMcpwmInit(dio1Pin) once during init.
//   3. Pass &context as the first argument to all sx126x_* calls.
//
// BUSY handling:
//   hal_write/hal_read check BUSY before each SPI transaction. If BUSY is high
//   the call returns SX126X_HAL_STATUS_ERROR immediately — no spinning. The
//   caller must ensure the radio is ready; a BUSY error means the window was
//   missed and the operation should be dropped (logged, not retried in-place).
//
// MCPWM capture:
//   DIO1 is connected to an MCPWM capture channel at 80 MHz. On every rising
//   edge the counter value is latched by hardware before the ISR runs, so the
//   timestamp has ~12.5 ns resolution and zero ISR-jitter.
//   Read with dio1TimestampUs() after checking dio1Fired.

#ifndef RADIO_HAL_H
#define RADIO_HAL_H

#include <Arduino.h>
#include <SPI.h>

// ===================== HAL CONTEXT =====================

struct sx126x_hal_context_t {
    SPIClass* spi;
    uint8_t   nss;   // chip-select (active low, driven by HAL)
    uint8_t   busy;  // BUSY pin — GPIO only, checked before every SPI transaction
    uint8_t   rst;   // reset pin (active low, held low 1 ms then released)
};

// ===================== DIO1 CAPTURE STATE =====================
// Written by MCPWM capture ISR (IRAM-safe); read from main loop.

extern volatile bool     dio1Fired;       // set by capture ISR on each DIO1 rising edge
extern volatile uint32_t dio1CaptureVal;  // raw 80 MHz capture timer value at that edge

// Microsecond timestamp of the most recent DIO1 rising edge.
// Valid only while dio1Fired is true (or immediately after clearing it, before
// the next edge could arrive).
inline uint64_t dio1TimestampUs() {
    return (uint64_t)dio1CaptureVal / 80;
}

// ===================== PUBLIC API =====================

// Initialise MCPWM capture timer and channel on the given DIO1 GPIO pin.
// Call once during radio init, after SPI and pins are configured.
// Replaces attachInterrupt / radio.setDio1Action().
void radioMcpwmInit(uint8_t dio1Pin);

#endif // RADIO_HAL_H
