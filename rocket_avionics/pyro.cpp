// pyro.cpp — Pyro channel implementation.
//
// Three RMT TX channels, one pre-bound per output GPIO, created at pyroInit().
// Clock: RMT_CLK_SRC_RC_FAST (~17.5 MHz, survives light sleep, immune to APB gating).
// Prescaler: 256 (8-bit max) → tick ≈ 14.6 µs. Each symbol duration0/1 field is 15 bits
// → max 32767 ticks ≈ 479 ms per phase. 48-symbol buffer → max pulse ≈ 23 s.
//
// pyroFire() builds a single contiguous HIGH pulse across as many symbols as needed,
// terminated by a 1-tick LOW in the final symbol, padded to 48 with null words.
// rmt_transmit() is called once with loop_count=0 (single shot) and returns immediately.
// The RMT peripheral drives the pulse entirely in hardware — the CPU is not involved
// after the call. Freezes, logging erases, sleep, and watchdog resets cannot extend it.
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

// 48 symbols per channel — ESP32-S3 RMT hardware minimum for mem_block_symbols.
// pyroFire() fills them with the pulse shape; unused slots are zeroed (null = end marker).
#define PYRO_SYM_COUNT  48
static rmt_symbol_word_t pyroSyms[3][PYRO_SYM_COUNT] = {};

// RC_FAST ~17.5 MHz, prescaler 256 (8-bit max) → tick ≈ 14.629 µs.
// resolution_hz = 17,500,000 / 256 = 68,359 Hz (minimum allowed; keeps div in range).
// Max per-symbol HIGH = 32767 ticks × 14.629 µs ≈ 479 ms.
// 48 symbols × 479 ms = ~23 s max pulse — sufficient for any igniter duration.
#define PYRO_RMT_RESOLUTION_HZ  68359   // div = 256 (8-bit max); tick ≈ 14.629 µs
#define PYRO_TICKS_PER_MS       68      // 1 ms ÷ 14.629 µs ≈ 68 ticks (rounds down; ≈1% short)
#define PYRO_MAX_TICKS_PER_SYM  32767   // 15-bit duration field max

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

  // Build a single contiguous HIGH pulse across multiple symbols, terminated by a
  // 1-tick LOW in the final symbol. Remaining slots zeroed (null word = end marker).
  // No looping — loop_count=0 means single-shot.
  uint32_t totalTicks = (uint32_t)durationMs * PYRO_TICKS_PER_MS;
  rmt_symbol_word_t* syms = pyroSyms[channel - 1];
  memset(syms, 0, sizeof(pyroSyms[0]));

  // Fill symbols with HIGH chunks. All but the last use duration1=0 (zero-duration phase
  // is skipped by RMT hardware — pin stays HIGH, no glitch). The last symbol gets
  // duration1=1 LOW to terminate the pulse.
  int s = 0;
  while (totalTicks > 0 && s < PYRO_SYM_COUNT) {
    uint16_t chunk = (totalTicks > PYRO_MAX_TICKS_PER_SYM) ? PYRO_MAX_TICKS_PER_SYM
                                                            : (uint16_t)totalTicks;
    totalTicks -= chunk;
    syms[s].level0    = 1;
    syms[s].duration0 = chunk;
    if (totalTicks == 0) {
      syms[s].level1    = 0;  // LOW to terminate
      syms[s].duration1 = 1;
    } else {
      syms[s].level1    = 1;  // stay HIGH (zero-duration phase is a no-op)
      syms[s].duration1 = 0;
    }
    s++;
  }
  // Remaining slots are zero (memset above) — null word acts as end-of-data sentinel.

  if (channel == 1) pyroState.ch1Fired = true;
  else if (channel == 2) pyroState.ch2Fired = true;
  else if (channel == 3) pyroState.ch3Fired = true;

  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count      = 0;    // single shot — no looping
  tx_cfg.flags.eot_level = 0;    // pin stays LOW after transmission

  rmt_transmit(ch.rmt, ch.encoder, syms, sizeof(pyroSyms[0]), &tx_cfg);

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
