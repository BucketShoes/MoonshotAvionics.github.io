// pyro.cpp — Pyro channel implementation.
//
// Three RMT TX channels, one pre-bound per output GPIO, created at pyroInit().
// Clock: RMT_CLK_SRC_RC_FAST (~17.5 MHz, survives light sleep, immune to APB gating).
// Resolution: 100 Hz → 10 ms per tick. Duration accuracy ±10 ms — acceptable for igniters.
// Maximum pulse: 32767 ticks = ~327 s.
//
// pyroFire() queues a single-shot pulse via rmt_transmit() and returns immediately.
// The RMT peripheral drives the pin HIGH then LOW in hardware. The CPU is not involved
// after the call — freezes, logging erases, sleep, and watchdog resets cannot extend the pulse.
// On any reset, all GPIOs go high-impedance; external pulldowns on MOSFET gates hold them low.
//
// ADC: uses Arduino analogReadMilliVolts() (legacy IDF ADC driver on ADC1).
// Do NOT call adc_oneshot_new_unit(ADC_UNIT_1) — it claims exclusive ADC1 ownership and
// breaks analogReadMilliVolts() used for battery sense elsewhere in the system.
//
// SAFETY: pyroFire() is called from loop() while armed. It must not block.
// All heavy work (channel/encoder allocation) is done once in pyroInit() during setup().

#include "pyro.h"
#include "config.h"

#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_common.h>

// ===================== MODULE STATE =====================

PyroState pyroState = {};

struct PyroChannel {
  rmt_channel_handle_t rmt;      // nullptr if init failed
  rmt_encoder_handle_t encoder;
  int                  pin;
};
static PyroChannel pyroChannels[3] = {
  { nullptr, nullptr, PYRO_CH1_PIN },
  { nullptr, nullptr, PYRO_CH2_PIN },
  { nullptr, nullptr, PYRO_CH3_PIN },
};

static rmt_symbol_word_t pyroSyms[3] = {};

// RMT_CLK_SRC_RC_FAST is ~17.5 MHz. We use a resolution of 100 Hz (10 ms/tick).
// 17,500,000 / 100 = 175,000 — this is set as the prescaler target; the IDF driver
// finds the nearest achievable integer division. At 100 Hz resolution the tick is
// 10 ms ±some RC drift; acceptable for igniter durations (seconds range).
#define PYRO_RMT_RESOLUTION_HZ  100    // 10 ms per tick
#define PYRO_MS_PER_TICK        10     // must match 1000/PYRO_RMT_RESOLUTION_HZ

// ===================== INIT =====================

void pyroInit() {
  // Output pins: drive LOW before handing to RMT so the gate is never left floating
  pinMode(PYRO_CH1_PIN, OUTPUT); digitalWrite(PYRO_CH1_PIN, LOW);
  pinMode(PYRO_CH2_PIN, OUTPUT); digitalWrite(PYRO_CH2_PIN, LOW);
  pinMode(PYRO_CH3_PIN, OUTPUT); digitalWrite(PYRO_CH3_PIN, LOW);

  // Continuity sense: input with internal pulldown; high = wire connected
  pinMode(PYRO_SENSE_CH1_PIN, INPUT_PULLDOWN);
  pinMode(PYRO_SENSE_CH2_PIN, INPUT_PULLDOWN);
  pinMode(PYRO_SENSE_CH3_PIN, INPUT_PULLDOWN);

  // HV sense: plain ADC input, no internal pull (external divider provides bias)
  pinMode(PYRO_HV_SENSE_PIN, INPUT);

  // Create one RMT TX channel per output pin. All are pre-enabled here so pyroFire()
  // never needs to allocate, configure, or enable anything at runtime.
  rmt_copy_encoder_config_t enc_cfg = {};
  for (int i = 0; i < 3; i++) {
    rmt_tx_channel_config_t ch_cfg = {};
    ch_cfg.gpio_num          = (gpio_num_t)pyroChannels[i].pin;
    ch_cfg.clk_src           = RMT_CLK_SRC_RC_FAST;  // survives light sleep; immune to APB gating
    ch_cfg.resolution_hz     = PYRO_RMT_RESOLUTION_HZ;
    ch_cfg.mem_block_symbols = 2;   // minimum; we only ever queue 1 symbol
    ch_cfg.trans_queue_depth = 1;
    ch_cfg.flags.invert_out  = 0;
    ch_cfg.flags.with_dma    = 0;

    if (rmt_new_tx_channel(&ch_cfg, &pyroChannels[i].rmt) != ESP_OK) {
      pyroChannels[i].rmt = nullptr;
      Serial.printf("PYRO: RMT ch%d init failed\n", i + 1);
      continue;
    }
    if (rmt_new_copy_encoder(&enc_cfg, &pyroChannels[i].encoder) != ESP_OK) {
      rmt_del_channel(pyroChannels[i].rmt);
      pyroChannels[i].rmt     = nullptr;
      pyroChannels[i].encoder = nullptr;
      Serial.printf("PYRO: encoder ch%d init failed\n", i + 1);
      continue;
    }
    rmt_enable(pyroChannels[i].rmt);
  }

  pyroState = {};
}

// ===================== NON-BLOCKING UPDATE =====================

void nonblockingPyro() {
  unsigned long now = micros();

  // CPU-side estimate of pulse completion (for telemetry only; has no effect on hardware)
  if (pyroState.activeChannel != 0) {
    if ((now - pyroState.fireStartUs) >= pyroState.fireDurationUs) {
      pyroState.activeChannel = 0;
    }
  }

  // Continuity: digitalRead on all three sense pins. Each call is one register read (~1 µs).
  pyroState.ch1Continuity = (digitalRead(PYRO_SENSE_CH1_PIN) != 0);
  pyroState.ch2Continuity = (digitalRead(PYRO_SENSE_CH2_PIN) != 0);
  pyroState.ch3Continuity = (digitalRead(PYRO_SENSE_CH3_PIN) != 0);

  // HV sense: Arduino ADC read (blocking, ~20–50 µs max — acceptable per-loop cost).
  // 10k:100k divider → ADC pin sees V_hv / PYRO_HV_DIVIDER_RATIO.
  // EMA with 10% weight smooths noise while tracking slow supply changes.
  uint32_t rawMv = analogReadMilliVolts(PYRO_HV_SENSE_PIN);
  uint32_t hvMv  = (uint32_t)rawMv * PYRO_HV_DIVIDER_RATIO;
  if (pyroState.hvMillivolts == 0) {
    // First reading: seed the EMA rather than starting from 0
    pyroState.hvMillivolts = (uint16_t)hvMv;
  } else {
    // 10% weight on new sample
    pyroState.hvMillivolts = (uint16_t)((pyroState.hvMillivolts * 9 + hvMv) / 10);
  }
  pyroState.hvPresent = (pyroState.hvMillivolts >= PYRO_HV_PRESENT_MV);
}

// ===================== FIRE =====================

void pyroFire(uint8_t channel, uint16_t durationMs) {
  if (durationMs == 0) return;
  if (channel < 1 || channel > 3) return;

  PyroChannel& ch = pyroChannels[channel - 1];
  if (ch.rmt == nullptr) return;  // init failed for this channel

  // Convert ms to RMT ticks (10 ms/tick). Round up so short durations don't become 0.
  uint16_t ticks = (durationMs + PYRO_MS_PER_TICK - 1) / PYRO_MS_PER_TICK;
  if (ticks == 0) ticks = 1;

  // Single-shot symbol: HIGH for ticks, then LOW for 1 tick (end-of-transmission marker)
  rmt_symbol_word_t& sym = pyroSyms[channel - 1];
  sym.level0    = 1;
  sym.duration0 = ticks;
  sym.level1    = 0;
  sym.duration1 = 1;

  if (channel == 1) pyroState.ch1Fired = true;
  else if (channel == 2) pyroState.ch2Fired = true;
  else if (channel == 3) pyroState.ch3Fired = true;

  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count      = 0;  // single shot
  tx_cfg.flags.eot_level = 0;  // hardware drives pin LOW at end of transmission

  rmt_transmit(ch.rmt, ch.encoder, &sym, sizeof(rmt_symbol_word_t), &tx_cfg);

  pyroState.activeChannel  = channel;
  pyroState.fireStartUs    = micros();
  pyroState.fireDurationUs = (unsigned long)durationMs * 1000UL;
}

// ===================== LATCHES =====================

void pyroClearLatches() {
  pyroState.ch1Fired = false;
  pyroState.ch2Fired = false;
  pyroState.ch3Fired = false;
}
