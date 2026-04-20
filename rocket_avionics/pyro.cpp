// pyro.cpp — Pyro channel implementation.
// Uses ESP-IDF5 new RMT driver. Three RMT TX channels are created at pyroInit(), one per
// output GPIO. At fire time only rmt_transmit() is called — no allocation, no blocking.
// RMT hardware guarantees the pulse ends at the right time even if the CPU is busy.
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
#include <esp_adc/adc_oneshot.h>

// ===================== MODULE STATE =====================

PyroState pyroState = {};

// One RMT TX channel per output GPIO, pre-created at init
struct PyroChannel {
  rmt_channel_handle_t rmt;      // nullptr if init failed
  rmt_encoder_handle_t encoder;  // copy encoder, pre-created
  int                  pin;
};
static PyroChannel pyroChannels[3] = {
  { nullptr, nullptr, PYRO_CH1_PIN },
  { nullptr, nullptr, PYRO_CH2_PIN },
  { nullptr, nullptr, PYRO_CH3_PIN },
};

// Pre-built symbols: [i].sym is the one-shot pulse for each channel.
// Filled in at fire time (duration varies), so we keep one static word per channel.
static rmt_symbol_word_t pyroSyms[3] = {};

// ADC handle for HV sense
static adc_oneshot_unit_handle_t hvAdcHandle = nullptr;
static bool hvAdcReady = false;

// Sense rotation state
static uint8_t senseRotation = 0;
static unsigned long lastSenseUs = 0;
#define SENSE_INTERVAL_US 100000UL  // 10 Hz

// RMT clock: 1 kHz → 1 ms per tick. Max symbol = 32767 ms ≈ 32 s.
#define RMT_RESOLUTION_HZ 1000

// ===================== HELPERS =====================

int pyroChannelPin(uint8_t channel) {
  if (channel < 1 || channel > 3) return -1;
  return pyroChannels[channel - 1].pin;
}

static void driveLow(int pin) {
  if (pin < 0) return;
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)pin, 0);
}

// ===================== INIT =====================

void pyroInit() {
  // Drive all outputs LOW first
  driveLow(PYRO_CH1_PIN);
  driveLow(PYRO_CH2_PIN);
  driveLow(PYRO_CH3_PIN);

  // Continuity sense pins: input with internal pulldown
  gpio_config_t sense_cfg = {};
  sense_cfg.pin_bit_mask = (1ULL << PYRO_SENSE_CH1_PIN) |
                            (1ULL << PYRO_SENSE_CH2_PIN) |
                            (1ULL << PYRO_SENSE_CH3_PIN);
  sense_cfg.mode         = GPIO_MODE_INPUT;
  sense_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  sense_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
  sense_cfg.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&sense_cfg);

  // HV sense pin: ADC input (external divider; no internal pull)
  gpio_config_t hv_cfg = {};
  hv_cfg.pin_bit_mask = (1ULL << PYRO_HV_SENSE_PIN);
  hv_cfg.mode         = GPIO_MODE_INPUT;
  hv_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  hv_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
  hv_cfg.intr_type    = GPIO_INTR_DISABLE;
  gpio_config(&hv_cfg);

  // ADC oneshot for HV sense
  adc_oneshot_unit_init_cfg_t adc_cfg = {};
  adc_cfg.unit_id = ADC_UNIT_1;
  if (adc_oneshot_new_unit(&adc_cfg, &hvAdcHandle) == ESP_OK) {
    adc_oneshot_chan_cfg_t chan_cfg = {};
    chan_cfg.atten    = ADC_ATTEN_DB_12;  // 0–3.3 V range
    chan_cfg.bitwidth = ADC_BITWIDTH_12;
    // GPIO 42 → ADC1 channel 2 on ESP32-S3
    if (adc_oneshot_config_channel(hvAdcHandle, ADC_CHANNEL_2, &chan_cfg) == ESP_OK) {
      hvAdcReady = true;
    }
  }

  // Create one RMT TX channel and copy encoder per output pin
  rmt_copy_encoder_config_t enc_cfg = {};
  for (int i = 0; i < 3; i++) {
    rmt_tx_channel_config_t chan_cfg = {};
    chan_cfg.gpio_num          = (gpio_num_t)pyroChannels[i].pin;
    chan_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    chan_cfg.resolution_hz     = RMT_RESOLUTION_HZ;
    chan_cfg.mem_block_symbols = 2;   // minimum; we use 1 symbol
    chan_cfg.trans_queue_depth = 1;
    chan_cfg.flags.invert_out  = 0;
    chan_cfg.flags.with_dma    = 0;

    if (rmt_new_tx_channel(&chan_cfg, &pyroChannels[i].rmt) != ESP_OK) {
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

  // Update CPU estimate of pulse completion
  if (pyroState.activeChannel != 0) {
    if ((now - pyroState.fireStartUs) >= pyroState.fireDurationUs) {
      pyroState.activeChannel = 0;
    }
  }

  // Rotate sense reads at ~10 Hz
  if ((now - lastSenseUs) < SENSE_INTERVAL_US) return;
  lastSenseUs = now;

  switch (senseRotation) {
    case 0:
      pyroState.ch1Continuity = (gpio_get_level((gpio_num_t)PYRO_SENSE_CH1_PIN) != 0);
      break;
    case 1:
      pyroState.ch2Continuity = (gpio_get_level((gpio_num_t)PYRO_SENSE_CH2_PIN) != 0);
      break;
    case 2:
      pyroState.ch3Continuity = (gpio_get_level((gpio_num_t)PYRO_SENSE_CH3_PIN) != 0);
      break;
    case 3:
      if (hvAdcReady) {
        int raw = 0;
        if (adc_oneshot_read(hvAdcHandle, ADC_CHANNEL_2, &raw) == ESP_OK) {
          // 12-bit ADC, 0–4095 → 0–3300 mV at ADC pin
          uint32_t adcMv = ((uint32_t)raw * 3300UL) / 4095UL;
          pyroState.hvMillivolts = (uint16_t)(adcMv * PYRO_HV_DIVIDER_RATIO);
          pyroState.hvPresent = (pyroState.hvMillivolts >= PYRO_HV_PRESENT_MV);
        }
      }
      break;
  }
  senseRotation = (senseRotation + 1) & 3;
}

// ===================== FIRE =====================

void pyroFire(uint8_t channel, uint16_t durationMs) {
  if (durationMs == 0) return;
  if (channel < 1 || channel > 3) return;

  PyroChannel& ch = pyroChannels[channel - 1];
  if (ch.rmt == nullptr) return;  // init failed for this channel

  // Build the one-shot symbol: HIGH for durationMs ticks (1 tick = 1 ms), then LOW
  pyroSyms[channel - 1].level0    = 1;
  pyroSyms[channel - 1].duration0 = durationMs;
  pyroSyms[channel - 1].level1    = 0;
  pyroSyms[channel - 1].duration1 = 1;  // minimal LOW tail before EOT

  // Set mark latch
  if (channel == 1) pyroState.ch1Fired = true;
  else if (channel == 2) pyroState.ch2Fired = true;
  else if (channel == 3) pyroState.ch3Fired = true;

  rmt_transmit_config_t tx_cfg = {};
  tx_cfg.loop_count      = 0;   // single shot
  tx_cfg.flags.eot_level = 0;   // pin goes LOW after end of transmission

  rmt_transmit(ch.rmt, ch.encoder, &pyroSyms[channel - 1], sizeof(rmt_symbol_word_t), &tx_cfg);

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
