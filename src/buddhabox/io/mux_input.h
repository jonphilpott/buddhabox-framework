/**
 * @file mux_input.h
 * @brief CD74HC4067 16-channel analog multiplexer driver.
 *
 * The CD74HC4067 is a 16:1 analog multiplexer — it connects one of 16
 * input channels to a single common output (SIG/Z) based on a 4-bit
 * binary address written to its select pins (S0–S3). This lets you read
 * up to 16 analog sensors or potentiometers using only one ADC pin and
 * four GPIO pins.
 *
 * USAGE PATTERN — same as AnalogInput:
 * ──────────────────────────────────────
 * analogRead() is too slow (~10–40 µs) to call from the audio ISR.
 * MuxInput follows the same two-phase pattern as AnalogInput:
 *
 *   loop()          — call update() or updateAll() to read the ADC
 *                     and refresh the cached values.
 *   audioCallback() — call read(ch) to get a cached float value.
 *                     This is safe from interrupt context (ARM float
 *                     reads are atomic).
 *
 * UPDATE RATE:
 * ────────────
 * With round-robin update() calls and a loop() running at ~1 kHz,
 * each of the 16 channels refreshes at roughly 62 Hz — more than
 * fast enough for pots and sensors that a human is turning.
 *
 * If you need all channels to feel equally responsive, prefer updateAll()
 * which sweeps all 16 in one blocking call (~400 µs total).
 *
 * WIRING:
 * ───────
 *   CD74HC4067 pin   →   Black Pill
 *   ─────────────────────────────────
 *   S0               →   any GPIO (e.g. PB0)
 *   S1               →   any GPIO (e.g. PB1)
 *   S2               →   any GPIO (e.g. PB3)
 *   S3               →   any GPIO (e.g. PB4)
 *   SIG / Z          →   any ADC pin (e.g. PA0)
 *   EN               →   GND (always enabled) or a GPIO for power saving
 *   VCC              →   3.3V
 *   GND              →   GND
 *
 * NOTE: The EN pin is active LOW. Tie it to GND for permanently-enabled
 * operation, or pass its pin number to the constructor for software
 * control (useful for low-power patches).
 *
 * Example usage — 16 pots controlling parameters:
 *   MuxInput mux(PB0, PB1, PB3, PB4, PA0);  // EN tied to GND
 *
 *   void setup() {
 *       mux.begin();
 *   }
 *
 *   void loop() {
 *       mux.updateAll();  // Sweep all 16 channels
 *       delay(10);
 *   }
 *
 *   // In audio callback (safe — only reads cached values):
 *   float cutoff = mapf(mux.read(0), 0.0f, 1.0f, 200.0f, 5000.0f);
 *   float tempo  = mapf(mux.read(1), 0.0f, 1.0f, 60.0f, 180.0f);
 *
 * Example usage — round-robin update (one channel per loop iteration):
 *   void loop() {
 *       mux.update();  // Advances one channel each call
 *       delay(1);      // ~62 Hz per channel at 1 kHz loop rate
 *   }
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

namespace bb {

/**
 * CD74HC4067 16-channel analog multiplexer driver.
 *
 * See the file header above for how it works and usage examples.
 */
class MuxInput {
public:
  /**
   * Construct a MuxInput driver.
   *
   * @param s0Pin   GPIO pin connected to CD74HC4067 S0 (LSB of address)
   * @param s1Pin   GPIO pin connected to CD74HC4067 S1
   * @param s2Pin   GPIO pin connected to CD74HC4067 S2
   * @param s3Pin   GPIO pin connected to CD74HC4067 S3 (MSB of address)
   * @param sigPin  ADC-capable pin connected to the mux SIG/Z common pin
   * @param enPin   GPIO pin connected to CD74HC4067 EN (active LOW).
   *                Pass -1 (default) if EN is tied directly to GND.
   * @param alpha   EMA smoothing factor per channel, 0.0 to 1.0.
   *                0.05 = smooth (good for sensors)
   *                0.2  = responsive (good for pots)
   *                1.0  = no smoothing (raw readings)
   *
   * Example:
   *   MuxInput mux(PB0, PB1, PB3, PB4, PA0);         // EN to GND
   *   MuxInput mux(PB0, PB1, PB3, PB4, PA0, PB5);    // EN on PB5
   *   MuxInput mux(PB0, PB1, PB3, PB4, PA0, -1, 0.2f); // Faster smoothing
   */
  MuxInput(uint8_t s0Pin, uint8_t s1Pin, uint8_t s2Pin, uint8_t s3Pin,
           uint8_t sigPin, int8_t enPin = -1, float alpha = 0.05f)
      : s0_(s0Pin), s1_(s1Pin), s2_(s2Pin), s3_(s3Pin), sigPin_(sigPin),
        enPin_(enPin), alpha_(alpha), settlingUs_(10), currentCh_(0),
        initMask_(0) {
    // Zero all cached values — they'll be properly initialized on
    // first read of each channel via the initMask_ mechanism.
    for (uint8_t i = 0; i < 16; i++)
      smoothed_[i] = 0.0f;
  }

  /**
   * Initialize GPIO pin modes. Call once from setup().
   *
   * Sets the four select pins and (optionally) the enable pin as outputs,
   * selects channel 0, and enables the mux.
   *
   * Example:
   *   void setup() {
   *       mux.begin();
   *   }
   */
  void begin() {
    // Configure all four address lines as digital outputs
    pinMode(s0_, OUTPUT);
    pinMode(s1_, OUTPUT);
    pinMode(s2_, OUTPUT);
    pinMode(s3_, OUTPUT);

    // Configure and assert the enable pin if provided.
    // EN is active LOW — driving it LOW enables the mux.
    if (enPin_ >= 0) {
      pinMode((uint8_t)enPin_, OUTPUT);
      digitalWrite((uint8_t)enPin_, LOW);
    }

    // Start at channel 0
    selectChannel(0);
  }

  /**
   * Read one channel (round-robin) and update its cached value.
   *
   * Each call advances to the next channel (0→1→...→15→0).
   * Call this once per loop() iteration for a smooth round-robin sweep.
   * At a loop rate of 1 kHz, each channel updates at ~62 Hz.
   *
   * Do NOT call from the audio callback — analogRead() is too slow.
   *
   * Example:
   *   void loop() {
   *       mux.update();
   *       delay(1);
   *   }
   */
  void update() {
    readChannel(currentCh_);
    // Advance to next channel, wrapping at 16 using bitmask
    currentCh_ = (currentCh_ + 1) & 0x0F;
  }

  /**
   * Read all 16 channels in one blocking sweep.
   *
   * Faster to set up than calling update() 16 times across loop
   * iterations. Takes approximately 16 × (settlingUs + analogRead time)
   * — roughly 400 µs with default settings. Fine for loop() context.
   *
   * Do NOT call from the audio callback.
   *
   * Example:
   *   void loop() {
   *       mux.updateAll();
   *       delay(10);
   *   }
   */
  void updateAll() {
    for (uint8_t ch = 0; ch < 16; ch++)
      readChannel(ch);
    // Reset round-robin position to stay consistent with update()
    currentCh_ = 0;
  }

  /**
   * Get the smoothed, normalized value for a channel.
   *
   * Safe to call from the audio callback (ISR context). Only reads a
   * cached float — no ADC access, no blocking.
   *
   * @param ch  Channel number, 0 to 15.
   * @return    Normalized value, 0.0 (0V) to 1.0 (3.3V).
   *
   * Example:
   *   float val = mux.read(3);
   *   float freq = mapf(val, 0.0f, 1.0f, 200.0f, 4000.0f);
   */
  float read(uint8_t ch) const {
    // STM32F4 ADC is 12-bit: raw values are 0–4095.
    // Dividing by 4095 maps to 0.0–1.0.
    return smoothed_[ch & 0x0F] / 4095.0f;
  }

  /**
   * Get the smoothed value as a raw 12-bit integer (0–4095).
   *
   * Useful when you need integer precision or want to do your own
   * range mapping without the float divide.
   *
   * @param ch  Channel number, 0 to 15.
   * @return    Smoothed ADC counts, 0–4095.
   *
   * Example:
   *   uint16_t raw = mux.readRaw(2);
   */
  uint16_t readRaw(uint8_t ch) const { return (uint16_t)smoothed_[ch & 0x0F]; }

  /**
   * Set the EMA smoothing factor for all channels.
   *
   * @param alpha  0.0 to 1.0. Lower = smoother but slower to respond.
   *               0.05 = good for light sensors and slow-moving CVs
   *               0.2  = good for pots turned by hand
   *               1.0  = raw unsmoothed readings
   *
   * Example:
   *   mux.setAlpha(0.15f);  // Switch to pot-friendly response
   */
  void setAlpha(float alpha) { alpha_ = alpha; }

  /**
   * Set the settling time between selecting a channel and reading the ADC.
   *
   * After the address lines change, the mux switch and external RC circuit
   * need a moment to settle before the ADC reading is accurate.
   * The default 10 µs is safe for most sensors and pot values up to 100 kΩ.
   * Reduce to 0 if your sensors are low-impedance and loop speed matters.
   *
   * @param us  Settling delay in microseconds. Default: 10.
   *
   * Example:
   *   mux.setSettlingUs(0);   // Low-impedance sources — no wait needed
   *   mux.setSettlingUs(50);  // High-impedance sources (100kΩ+ pots)
   */
  void setSettlingUs(uint16_t us) { settlingUs_ = us; }

  /**
   * Enable the multiplexer (assert EN pin LOW).
   * Only relevant if an EN pin was provided to the constructor.
   */
  void enable() {
    if (enPin_ >= 0)
      digitalWrite((uint8_t)enPin_, LOW);
  }

  /**
   * Disable the multiplexer (deassert EN pin HIGH).
   *
   * When disabled, all 16 channels are disconnected from SIG.
   * Useful for power saving between update sweeps in battery applications.
   * Only relevant if an EN pin was provided to the constructor.
   */
  void disable() {
    if (enPin_ >= 0)
      digitalWrite((uint8_t)enPin_, HIGH);
  }

private:
  /**
   * Write the 4-bit channel address to the S0–S3 select pins.
   *
   * The CD74HC4067 address mapping:
   *   Channel N → S3=(N>>3)&1, S2=(N>>2)&1, S1=(N>>1)&1, S0=(N>>0)&1
   *
   * For example, channel 9 (binary 1001):
   *   S3=1, S2=0, S1=0, S0=1
   *
   * @param ch  Channel to select, 0 to 15.
   */
  void selectChannel(uint8_t ch) {
    // Extract each bit of the channel number and drive the
    // corresponding address pin. Shifting right by 0,1,2,3
    // isolates the LSB, then bit 1, 2, and MSB respectively.
    digitalWrite(s0_, (ch >> 0) & 0x01);
    digitalWrite(s1_, (ch >> 1) & 0x01);
    digitalWrite(s2_, (ch >> 2) & 0x01);
    digitalWrite(s3_, (ch >> 3) & 0x01);
  }

  /**
   * Select a channel, wait for settling, read the ADC, and update
   * the EMA-smoothed cache for that channel.
   *
   * On the first read of a channel, the cache is initialized directly
   * to the raw value to avoid a slow ramp up from zero.
   *
   * @param ch  Channel to read, 0 to 15.
   */
  void readChannel(uint8_t ch) {
    selectChannel(ch);

    // Wait for the mux switch and external RC to settle.
    // delayMicroseconds(0) is a no-op so this branch-free form is fine.
    if (settlingUs_ > 0)
      delayMicroseconds(settlingUs_);

    uint16_t raw = (uint16_t)analogRead(sigPin_);

    // Check the initialization bitmask. Bit N corresponds to channel N.
    // On first read, skip EMA and copy the raw value directly to avoid
    // an ugly ramp up from zero on startup.
    uint16_t mask = (uint16_t)(1u << ch);
    if (!(initMask_ & mask)) {
      smoothed_[ch] = (float)raw;
      initMask_ |= mask;
    } else {
      // Exponential Moving Average:
      //   smoothed += alpha * (raw - smoothed)
      // alpha close to 0 → slow tracking (smooth)
      // alpha close to 1 → fast tracking (responsive)
      smoothed_[ch] += alpha_ * ((float)raw - smoothed_[ch]);
    }
  }

  // ── Pin assignments ─────────────────────────────────────────────────────
  uint8_t s0_, s1_, s2_, s3_; // Address select pins (GPIO output)
  uint8_t sigPin_;            // Common SIG/Z pin (ADC input)
  int8_t enPin_;              // Enable pin, -1 = not used (tied to GND)

  // ── Configuration ───────────────────────────────────────────────────────
  float alpha_;         // EMA smoothing coefficient (0.0–1.0)
  uint16_t settlingUs_; // µs to wait after channel select

  // ── State ───────────────────────────────────────────────────────────────
  uint8_t currentCh_;  // Next channel for round-robin update()
  float smoothed_[16]; // EMA-smoothed ADC values (in raw counts)
  uint16_t initMask_;  // Bitmask: bit N set = channel N initialized
};

} // namespace bb
