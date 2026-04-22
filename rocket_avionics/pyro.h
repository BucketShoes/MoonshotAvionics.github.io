// pyro.h — Pyro channel control: RMT-timed single-shot pulses, continuity sense, HV sense.
// Three RMT TX channels, one pre-bound per GPIO. Clock: APB/80 = 1 MHz (1 µs/tick).
// 1ms symbols repeated N times in hardware (loop_count). Pulse cutoff is hardware-guaranteed.

#ifndef PYRO_H
#define PYRO_H

#include <stdint.h>
#include <stddef.h>

// ===================== STATE =====================

struct PyroState {
  bool     ready;              // true after pyroInit() — gates continuity/HV reads
  bool     ch1Fired;           // latch: fired since last arm (cleared on next arm)
  bool     ch2Fired;
  bool     ch3Fired;
  uint8_t  activeChannel;      // 0 = none; 1/2/3 = RMT pulse in progress (CPU estimate)
  unsigned long fireStartUs;   // micros() when current pulse was started
  unsigned long fireDurationUs;// configured pulse duration

  // Continuity sense (digitalRead with internal pulldown; high = wire connected)
  bool     ch1Continuity;
  bool     ch2Continuity;
  bool     ch3Continuity;

  // High-side voltage sense (ADC on PYRO_HV_SENSE_PIN via 10k:100k divider)
  uint16_t hvMillivolts;       // estimated high-side voltage in mV
  bool     hvPresent;          // true if hvMillivolts > PYRO_HV_PRESENT_MV
};

extern PyroState pyroState;

// ===================== PUBLIC API =====================

// Call once from setup(). Initialises GPIO outputs LOW, sense inputs, ADC, and encoder.
void pyroInit();

// Non-blocking update. Call every loop iteration (after nonblockingFlight).
// Updates continuity sense and HV ADC at ~10 Hz. Clears activeChannel estimate when done.
void nonblockingPyro();

// Fire the specified channel for durationMs milliseconds.
// Non-blocking: queues a single-shot RMT symbol and returns immediately.
// Each channel has its own RMT hardware; firing one has no effect on others.
// channel: 1 = PYRO_CH1_PIN, 2 = PYRO_CH2_PIN, 3 = PYRO_CH3_PIN
// durationMs: 1 ms resolution. 0 is ignored. Max INT_MAX ms (loop_count).
void pyroFire(uint8_t channel, uint16_t durationMs);

// Clear ch1/ch2/ch3 fired latches. Call on arm.
void pyroClearLatches();

// Return the GPIO pin number for a channel (1/2/3). Returns -1 for invalid channel.
int pyroChannelPin(uint8_t channel);

#endif // PYRO_H
