/**
 * @file analog_input.h
 * @brief Smoothed analog input reader for external sensors.
 *
 * Reads analog voltage from an ADC pin and provides a smoothed, normalized
 * value. Designed for connecting external sensors like:
 *
 *   - Light sensors (photoresistors / LDRs) — modulate brightness→parameters
 *   - Potentiometers — manual control of filter cutoff, tempo, etc.
 *   - Touch/pressure sensors — expressive control
 *   - Temperature sensors — very slow ambient modulation
 *
 * WHY SMOOTHING IS NECESSARY:
 * ──────────────────────────
 * Raw ADC readings are noisy — they jitter by several counts even when the
 * voltage is perfectly stable. If you map a jittery ADC reading directly to
 * a filter cutoff, you'll hear buzzing/crackling artifacts.
 *
 * We use an Exponential Moving Average (EMA) filter to smooth the readings:
 *   smoothed = smoothed + alpha * (raw - smoothed)
 *
 * The alpha parameter controls the tradeoff:
 *   alpha = 0.01 → Very smooth, but slow to respond (good for light sensors)
 *   alpha = 0.05 → Default: balanced smoothing (good for most sensors)
 *   alpha = 0.3  → Responsive but less smooth (good for knobs/pots)
 *   alpha = 1.0  → No smoothing (raw readings)
 *
 * Think of alpha as "how much do I trust the latest reading vs the history?"
 *
 * WHY HYSTERESIS AS WELL:
 * ──────────────────────
 * Smoothing shrinks the jitter but never removes it: a knob left alone
 * still wanders by a count or two. That's inaudible on a filter cutoff,
 * but a knob snapped to steps (notes, waveforms) flickers between two
 * of them whenever it sits near a boundary.
 *
 * So the output only moves once the smoothed reading has pulled more
 * than a small "dead band" away from it, and then it follows the reading
 * at that distance, like a slack rope pulled by a hand:
 *
 *   reading > output + band  →  output = reading - band
 *   reading < output - band  →  output = reading + band
 *
 * Turning the knob moves the output smoothly (just lagging by the band);
 * leaving it alone leaves the output perfectly still. The output is
 * rescaled so it still reaches exactly 0.0 and 1.0 at the ends.
 *
 * IMPORTANT — DO NOT CALL FROM AUDIO ISR:
 * analogRead() is slow (~10-40 microseconds) and blocks. Call update() from
 * loop() instead. The smoothed value will be available for the audio callback
 * to read via read() — this is safe because float reads are atomic on ARM.
 *
 * Example usage:
 *   AnalogInput lightSensor(PA0, 0.03f);  // Slow smoothing for light
 *   AnalogInput knob(PA1, 0.2f);          // Faster response for a pot
 *
 *   void loop() {
 *       lightSensor.update();
 *       knob.update();
 *       delay(10);  // ~100 updates/sec is plenty
 *   }
 *
 *   // In audio callback (reading the smoothed value is safe from ISR):
 *   float brightness = lightSensor.read(); // 0.0 (dark) to 1.0 (bright)
 *   float cutoff = mapf(brightness, 0.0f, 1.0f, 200.0f, 4000.0f);
 *   filter.setFrequency(cutoff);
 */

#pragma once

#include <Arduino.h>
#include <cstdint>

namespace bb {

/**
 * Smoothed analog input reader for external sensors.
 *
 * See the file header above for how it works and usage examples.
 */
class AnalogInput {
public:
  /**
   * Construct an AnalogInput reader.
   *
   * @param pin    The analog pin to read (e.g., PA0, PA1, PA4).
   *               On the Black Pill, PA0-PA7 and PB0-PB1 are ADC-capable.
   * @param alpha  Smoothing factor (0.0 to 1.0).
   *               Lower = smoother but slower response.
   *               Higher = noisier but faster response.
   *               Default: 0.05 (good general-purpose value).
   *
   * Example:
   *   AnalogInput sensor(PA0);          // Default smoothing
   *   AnalogInput knob(PA1, 0.15f);     // Faster response for a pot
   */
  AnalogInput(uint8_t pin, float alpha = 0.05f)
      : pin_(pin), raw_(0), alpha_(alpha), smoothed_(0.0f), band_(kDefaultBand),
        held_(0.0f), initialized_(false) {}

  /**
   * Read the ADC and update the smoothed value.
   *
   * Call this from loop() at a regular interval (every 5–20 ms is fine).
   * Do NOT call from the audio callback — analogRead() is too slow and
   * would cause audio glitches.
   *
   * On the first call, the smoothed value is initialized to the current
   * reading (no gradual ramp-up from zero).
   */
  void update() {
    // The Arduino core reads 10 bits (0-1023) unless told otherwise,
    // for compatibility with the old Arduino Uno. The STM32's ADC is
    // 12-bit, and every scaling here assumes 0-4095, so ask for all 12
    // each time: it only sets a variable, and nothing else can then
    // change it behind our back.
    analogReadResolution(12);
    raw_ = analogRead(pin_);

    // 1. Smooth.
    if (!initialized_) {
      // First reading: initialize directly to avoid a slow ramp from 0
      smoothed_ = (float)raw_;
      held_ = smoothed_;
      initialized_ = true;
    } else {
      // Exponential Moving Average (EMA):
      // smoothed += alpha * (raw - smoothed)
      //
      // When raw > smoothed, the difference is positive → smoothed
      // increases. When raw < smoothed, the difference is negative →
      // smoothed decreases. Alpha controls how fast: small alpha = slow
      // tracking, large = fast.
      smoothed_ += alpha_ * ((float)raw_ - smoothed_);
    }

    // 2. Hysteresis: drag the held value only when the reading pulls
    //    further than the band away from it.
    if (smoothed_ > held_ + band_)
      held_ = smoothed_ - band_;
    else if (smoothed_ < held_ - band_)
      held_ = smoothed_ + band_;
  }

  /**
   * Get the smoothed value, after hysteresis, normalized to 0.0–1.0.
   * It stays perfectly still while the knob is left alone.
   *
   * Safe to call from the audio callback (ISR context).
   *
   * @return  Normalized sensor value. 0.0 = 0V, 1.0 = 3.3V (full scale).
   *
   * Example:
   *   float val = sensor.read();
   *   float freq = mapf(val, 0.0f, 1.0f, 100.0f, 2000.0f);
   */
  float read() const {
    // STM32F4 ADC is 12-bit, so raw values range 0–4095. The held value
    // can only get within `band` of either end (the rope stays slack by
    // that much), so stretch band..4095-band out to 0.0..1.0.
    float x = (held_ - band_) / (4095.0f - 2.0f * band_);
    return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
  }

  /*
   * tweaked based on actual read values
   */
  float readLDR() const {
    // Tuned when update() read 10 bits, so read() then topped out at
    // 0.25; the 1.25 (5 x 0.25) keeps the curve as it was now that it
    // reads all 12.
    float r = 1 - (read() * 1.25f);

    float adj = r < 0.0f ? 0.0f : (r * 2.75f);
    adj = adj > 1.0f ? 1.0f : adj;
    return adj;
  }

  /**
   * Get the smoothed value as a raw 12-bit integer (0–4095).
   *
   * Useful when you need integer precision or want to do your own
   * scaling math.
   * Before hysteresis: it still wanders by a count or so.
   *
   * @return  Smoothed ADC value truncated to uint16_t
   */
  uint16_t readRaw() const { return (uint16_t)smoothed_; }

  /**
   * get the last actual read value from the ADC
   *
   * @return really raw adc val
   */
  uint16_t readReallyRaw() const { return raw_; }

  /**
   * Change the smoothing factor at runtime.
   *
   * @param a  New alpha value (0.0–1.0).
   *
   * Example:
   *   // Switch to faster response when reading a knob
   *   sensor.setAlpha(0.2f);
   */
  void setAlpha(float a) { alpha_ = a; }

  /**
   * Set the dead band: how far the reading must move before read()
   * follows it. Wider = steadier, but a little less resolution.
   *
   * @param counts  Band in ADC counts (of 4095). Default 8 (0.2%):
   *                enough to stop a smoothed pot flickering. 0 turns
   *                hysteresis off; try 16-32 for long wires or a noisy
   *                supply.
   *
   * Example:
   *   AnalogInput steps(PA2, 0.3f);
   *   steps.setHysteresis(24.0f); // 8 steps that never flicker
   */
  void setHysteresis(float counts) { band_ = counts < 0.0f ? 0.0f : counts; }

private:
  static constexpr float kDefaultBand = 8.0f; // ADC counts

  uint8_t pin_;      // ADC pin number
  uint16_t raw_;     // last raw value
  float alpha_;      // EMA smoothing coefficient
  float smoothed_;   // Current smoothed value (in raw ADC units, 0-4095)
  float band_;       // Hysteresis dead band, in ADC counts
  float held_;       // Output after hysteresis (ADC counts)
  bool initialized_; // Whether we've taken the first reading yet
};

} // namespace bb
