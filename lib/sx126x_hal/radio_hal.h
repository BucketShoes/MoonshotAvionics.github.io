// radio_hal.h — ESP32 HAL context and DIO1 interrupt capture for the SX126x driver.
// The Lora-net/sx126x_driver library declares the four HAL function prototypes
// (sx126x_hal_write/read/reset/wakeup) in its own sx126x_hal.h. This file adds
// what the driver leaves to the user: the context struct, the DIO1 interrupt
// capture state, and the init function.
//
// Usage:
//   1. Fill an sx126x_hal_context_t with your SPI instance and pin numbers.
//   2. Call radioMcpwmInit(dio1Pin) once during radio init.
//   3. Pass &context as the first argument to all sx126x_* calls.
//
// BUSY handling:
//   BUSY is an OUTPUT from the SX1262 that goes high while it is processing a
//   command. The ESP32 only reads this pin — never writes it.
//   hal_write/hal_read check BUSY before each SPI transaction. If BUSY is high
//   the call returns SX126X_HAL_STATUS_ERROR immediately — no spinning. The
//   caller must ensure the radio is ready; a BUSY error means the window was
//   missed and the operation should be dropped (logged, not retried in-place).
//   The no-spin policy is intentional: slot windows are exact and mandatory.
//   Missing one is correct behaviour; spinning to catch up would corrupt timing.
//   Exception: initMode=true enables spinning during radio init (setup() only).
//
// DIO1 interrupt:
//   DIO1 is connected to a GPIO interrupt (RISING edge). The ISR captures
//   micros() into dio1CaptureVal and sets dio1Fired. Resolution is ~1 µs.

#ifndef RADIO_HAL_H
#define RADIO_HAL_H

#include <Arduino.h>
#include <SPI.h>

// ===================== HAL CONTEXT =====================

struct sx126x_hal_context_t {
    SPIClass* spi;
    uint8_t   nss;      // chip-select (active low, driven by HAL)
    uint8_t   busy;     // BUSY pin — SX1262 output, ESP32 reads only, never writes
    uint8_t   rst;      // reset pin (active low, held low 1 ms then released)
    bool      initMode; // true during radio init: HAL spins on BUSY instead of dropping.
                        // Set to false before returning from radioInit()/bsRadioInit().
                        // NEVER set true outside of init — spinning blocks the main loop.
};

// ===================== DIO1 CAPTURE STATE =====================
// Written by GPIO ISR (IRAM-safe); read from main loop.

extern volatile bool     dio1Fired;       // set by ISR on each DIO1 rising edge
extern volatile uint32_t dio1CaptureVal;  // micros() at that edge (~1 µs resolution)

// Microsecond timestamp of the most recent DIO1 rising edge.
// Valid only while dio1Fired is true (or immediately after clearing it).
inline uint64_t dio1TimestampUs() {
    return (uint64_t)dio1CaptureVal;
}

// ===================== PUBLIC API =====================

// Attach GPIO interrupt on the given DIO1 pin (RISING edge).
// Call once during radio init, after SPI and pins are configured.
void radioMcpwmInit(uint8_t dio1Pin);

// *** BLOCKING — INIT ONLY. NEVER CALL WHILE ARMED OR FROM LOOP() ***
//
// radioWaitBusy() spins reading the SX1262 BUSY pin (an output from the radio,
// read-only to the ESP32) until the radio deasserts it, or timeoutMs elapses.
// This can block for up to timeoutMs milliseconds (default 100ms).
//
// WHY THIS EXISTS: the SX1262 reasserts BUSY briefly after every SPI command
// while it processes it. During radio init, commands are sent back-to-back; each
// one must wait for BUSY to clear before the next is sent, or the HAL will drop
// it (BUSY=high -> immediate error return, no retry). During normal operation the
// slot machine ensures BUSY is low before issuing any command, so this is not needed.
//
// SAFETY: This firmware targets >1000 Hz loop rate while armed. Pyro channels
// are driven by the main loop; any block >1ms risks a misfire or failure to fire.
// radioWaitBusy() can block up to 100ms — NEVER call it from any code path
// reachable while armed=true. radioInit() and bsRadioInit() run in setup() only
// and are safe. If radio power-cycling while armed is ever added, a non-blocking
// approach (state machine with timeout) must be used instead.
inline bool radioWaitBusy(const sx126x_hal_context_t* ctx, uint32_t timeoutMs = 100) {
    unsigned long t0 = millis();
    while (digitalRead(ctx->busy)) {
        if ((millis() - t0) >= timeoutMs) return false;
    }
    return true;
}

#endif // RADIO_HAL_H
