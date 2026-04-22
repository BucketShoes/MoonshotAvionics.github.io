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
#include <driver/gpio.h>

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

// RMT_CLK_SRC_RC_FAST is ~17.5 MHz. ESP32-S3 RMT prescaler is 8-bit (div 1–256),
// so resolution_hz must be >= 17,500,000/256 ≈ 68,360 Hz.
// 1 MHz gives div≈17 (within range). 1 µs per tick.
//
// duration0 is 15 bits → max 32767 ticks = 32 ms per symbol. Too short for igniter
// pulses (default 3000 ms). ESP32-S3 loop_count is also 10 bits → max 1023 repeats.
//
// Solution: symbol duration = ceil(durationMs / 1023) ms; loop_count = ceil(durationMs /
// symbolMs). This keeps both fields in range for any duration up to ~33 s, accurate to
// ±1 ms per loop boundary. Hardware drives pin LOW at EOT — cutoff guaranteed.
#define PYRO_RMT_RESOLUTION_HZ  1000000  // 1 µs per tick; div≈17 (within 8-bit limit)
#define PYRO_TICKS_PER_MS       1000     // 1000 ticks = 1 ms at 1 MHz
#define PYRO_MAX_LOOP_COUNT     1023     // ESP32-S3 RMT hardware loop count register limit

// ===================== INIT =====================

void pyroInit() {
  //NOTE: this uses 3 RMT channels, so is incompatible with upcoming hardware for esp32C3, which only has 2 tx channels.

  // Output pins: drive LOW before handing to RMT so the gate is never left floating
  pinMode(PYRO_CH1_PIN, OUTPUT); digitalWrite(PYRO_CH1_PIN, LOW);
  pinMode(PYRO_CH2_PIN, OUTPUT); digitalWrite(PYRO_CH2_PIN, LOW);
  pinMode(PYRO_CH3_PIN, OUTPUT); digitalWrite(PYRO_CH3_PIN, LOW);

  // Continuity sense: input with internal pulldown; high = wire connected.
  // GPIO 39/40/41 are USB/JTAG-muxed on ESP32-S3; gpio_reset_pin() releases them
  // from the JTAG matrix so digitalRead() works.
  gpio_reset_pin((gpio_num_t)PYRO_SENSE_CH1_PIN);
  gpio_reset_pin((gpio_num_t)PYRO_SENSE_CH2_PIN);
  gpio_reset_pin((gpio_num_t)PYRO_SENSE_CH3_PIN);
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
    ch_cfg.mem_block_symbols = 48;  // ESP32-S3 RMT hardware minimum (SOC requirement)
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

  // HV sense: Arduino ADC read (~20–50 µs max — acceptable per-loop cost).
  // 10k:100k divider → ADC pin sees V_hv / PYRO_HV_DIVIDER_RATIO.
  // EMA with 10% weight smooths noise while tracking slow supply changes.
  uint32_t rawMv = analogReadMilliVolts(PYRO_HV_SENSE_PIN);
  uint32_t hvMv  = (uint32_t)rawMv * PYRO_HV_DIVIDER_RATIO;
  if (pyroState.hvMillivolts == 0) {
    pyroState.hvMillivolts = (uint16_t)hvMv;
  } else {
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

  // Symbol duration scaled so loop_count fits in 10-bit hardware register (max 1023).
  // symbolMs = ceil(durationMs / 1023); loop_count = ceil(durationMs / symbolMs).
  // Accuracy: ±symbolMs per pulse (≤1 ms for durations ≤1023 ms, ≤3 ms for ≤3069 ms).
  uint16_t symbolMs   = (durationMs + PYRO_MAX_LOOP_COUNT - 1) / PYRO_MAX_LOOP_COUNT;
  if (symbolMs == 0) symbolMs = 1;
  uint16_t loopCount  = (durationMs + symbolMs - 1) / symbolMs;

  rmt_symbol_word_t& sym = pyroSyms[channel - 1];
  sym.level0    = 1;
  sym.duration0 = (uint16_t)(symbolMs * PYRO_TICKS_PER_MS);  // HIGH for symbolMs
  sym.level1    = 0;
  sym.duration1 = 1;  // 1-tick gap between loop iterations (1 µs; negligible)

  if (channel == 1) pyroState.ch1Fired = true;
  else if (channel == 2) pyroState.ch2Fired = true;
  else if (channel == 3) pyroState.ch3Fired = true;

  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count      = (int)loopCount;
  tx_cfg.flags.eot_level = 0;  // hardware drives pin LOW at end-of-transmission

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
