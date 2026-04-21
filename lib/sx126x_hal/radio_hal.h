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
//   BUSY is an OUTPUT from the SX1262. It goes high both during SPI command
//   processing (~µs) AND for the entire duration of any active RX or TX operation
//   (hundreds of ms). The ESP32 only reads this pin — never writes it.
//   hal_write checks BUSY and drops immediately if high (no spinning at runtime).
//   hal_read does the same, UNLESS ctx->allowBusyRead is set — that flag permits
//   one read while BUSY (for commands valid during RX, e.g. get_rssi_inst) and is
//   cleared by the HAL after each use. All radio calls are from loop() — no ISR
//   concurrency — so this flag is safe without atomics.
//   initMode=true enables spinning during radio init (setup() only).
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
    uint8_t   nss;           // chip-select (active low, driven by HAL)
    uint8_t   busy;          // BUSY pin — SX1262 output, ESP32 reads only, never writes
    uint8_t   rst;           // reset pin (active low, held low 1 ms then released)
    bool      initMode;      // true during radio init: HAL spins on BUSY instead of dropping.
                             // Set to false before returning from radioInit()/bsRadioInit().
                             // NEVER set true outside of init — spinning blocks the main loop.
    bool      allowBusyRead; // true: skip BUSY check for the next hal_read call only.
                             // Use for read-only commands valid while radio is mid-RX/TX
                             // (e.g. get_rssi_inst). HAL clears this flag after each use.
                             // Safe: all radio calls are from loop() — no concurrency.
};

// ===================== DIO1 CAPTURE STATE =====================
// Written by GPIO ISR (IRAM-safe); read from main loop.

extern volatile bool     dio1Fired;       // set by ISR on each DIO1 rising edge
extern volatile uint32_t dio1CaptureVal;  // micros() at that edge (~1 µs resolution)
extern volatile uint32_t dio1IsrCount;    // total ISR fires since boot (diagnostic)

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
// DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING() spins reading
// the SX1262 BUSY pin until the radio deasserts it, or timeoutMs elapses.
// This can block for up to timeoutMs milliseconds (default 100ms).
//
// WHY THIS EXISTS: the SX1262 reasserts BUSY briefly after every SPI command
// while it processes it. During radio init, commands are sent back-to-back; each
// one must wait for BUSY to clear before the next is sent, or the HAL will drop
// it (BUSY=high -> immediate error return, no retry). During normal operation the
// slot machine checks BUSY before issuing commands (non-blocking drop on high).
//
// SAFETY: This firmware targets >1000 Hz loop rate while armed. Pyro channels
// and future active control surfaces are driven by the main loop; any block >1ms
// risks a misfire, failure to fire, or loss of control authority. 100ms is
// ABSOLUTELY NOT acceptable while armed.
// ONLY call from functions named *_BLOCKING, which are ONLY called from setup().
// Per-slot radio config switching MUST use the nonblockingApplyCfg state machine
// (one SPI command per loop iteration, BUSY-check-and-skip, no spinning).
// If radio power-cycling while armed is ever added, use a non-blocking state machine.
inline bool DO_NOT_CALL_WHILE_ARMED_radioWaitBusy_WARNING_LONG_BLOCKING(
    const sx126x_hal_context_t* ctx, uint32_t timeoutMs = 100) {
    unsigned long t0 = millis();
    while (digitalRead(ctx->busy)) {
        if ((millis() - t0) >= timeoutMs) return false;
    }
    return true;
}

#endif // RADIO_HAL_H
