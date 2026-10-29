/**
 * @file noise.h
 * @brief White and pink noise generators for the BuddhaBox DSP framework.
 *
 * Noise is fundamental to ambient/generative audio. It provides:
 *   - Texture and warmth (filtered pink noise = wind, ocean, breath)
 *   - Excitation for resonant filters (noise → filter = singing bowls, drones)
 *   - The "pluck" energy for Karplus-Strong string synthesis
 *   - Random modulation when fed into control parameters
 *
 * WHITE NOISE vs PINK NOISE:
 * ──────────────────────────
 * White noise has equal energy at every frequency — it sounds harsh and
 * "hissy" like a TV tuned to static. The high frequencies dominate
 * because there are more high-frequency bands per octave.
 *
 * Pink noise has equal energy per *octave* — it rolls off at -3 dB/octave,
 * sounding much more natural and soothing. Rain, waterfalls, and wind are
 * approximately pink noise. It's the go-to for ambient texture.
 *
 * Example usage:
 *   PinkNoise noise(12345);    // Seed determines the random sequence
 *   SVFilter filter;
 *   filter.setFrequency(400.0f);
 *   filter.setResonance(0.6f);
 *
 *   // In audio callback:
 *   float n = noise.process() * 0.3f;   // Pink noise, reduced volume
 *   filter.process(n);
 *   float out = filter.lowpass();        // Warm, filtered noise
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * White noise generator using xorshift32 PRNG.
 *
 * HOW XORSHIFT WORKS:
 * This is a very fast pseudo-random number generator that uses only XOR
 * and bit-shift operations — no multiplication or division. It produces
 * a sequence of 2^32 - 1 values before repeating, which at 44100 Hz
 * would take about 27 hours. Good enough for our purposes!
 *
 * The three shift amounts (13, 17, 5) were chosen by George Marsaglia
 * to produce maximal-length sequences with good statistical properties.
 *
 * Example:
 *   WhiteNoise noise;
 *   float sample = noise.process(); // Returns random value in [-1, +1]
 */
class WhiteNoise {
public:
  /**
   * @param seed  Initial PRNG state. Different seeds = different random
   *              sequences. Use different seeds for each noise source to
   *              avoid correlation (audible as phasing artifacts).
   */
  WhiteNoise(uint32_t seed = 12345) : state_(seed ? seed : 1) {}

  /**
   * Reset the random sequence with a new seed.
   * @param s  New seed value (0 is forced to 1 since xorshift can't have zero
   * state)
   */
  void seed(uint32_t s) { state_ = s ? s : 1; }

  /**
   * Generate the next white noise sample.
   *
   * @return  Random value uniformly distributed in [-1.0, +1.0]
   */
  float process() {
    // xorshift32: three XOR+shift operations produce the next pseudo-random
    // value. This is one of the fastest PRNGs — critical since we call this
    // 44100 times/sec.
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;

    // Convert unsigned 32-bit integer to float in [-1, +1]:
    // Cast to signed int32_t (reinterprets bit pattern) then divide by
    // the max positive value. This gives a uniform distribution.
    return (float)(int32_t)state_ / 2147483648.0f;
  }

private:
  uint32_t state_;
};

/**
 * Pink noise generator using the Voss-McCartney algorithm.
 *
 * HOW VOSS-McCARTNEY WORKS:
 * ─────────────────────────
 * Pink noise needs -3 dB/octave rolloff. The clever trick here is to
 * maintain a set of "rows" (random values), where each row is updated
 * at a different rate:
 *
 *   Row 0: updated every sample     (represents highest octave)
 *   Row 1: updated every 2 samples  (next octave down)
 *   Row 2: updated every 4 samples  (two octaves down)
 *   Row 3: updated every 8 samples  ...and so on
 *
 * The output is the sum of all rows. Because lower rows change less
 * frequently, they contribute lower-frequency content — and since each
 * row contributes equally, we get equal energy per octave = pink noise.
 *
 * Which row to update is determined by the lowest set bit of a counter.
 * This is a very efficient O(1) operation per sample.
 *
 * With 12 rows, we cover about 12 octaves of pink-ness, which spans
 * the full audible range at 44.1 kHz.
 *
 * Example:
 *   PinkNoise pink(54321);
 *   float sample = pink.process(); // Returns pink noise in [-1, +1]
 */
class PinkNoise {
public:
  /** Number of octave rows. 12 rows covers the full audible spectrum. */
  static constexpr int NUM_ROWS = 12;

  /**
   * @param seed  Seed for the internal white noise generator.
   */
  PinkNoise(uint32_t seed = 54321)
      : white_(seed), counter_(0), runningSum_(0.0f) {
    // Initialize all rows to zero (silence at startup)
    for (int i = 0; i < NUM_ROWS; i++)
      rows_[i] = 0.0f;
  }

  /** Re-seed the internal random generator. */
  void seed(uint32_t s) { white_.seed(s); }

  /**
   * Generate the next pink noise sample.
   *
   * @return  Pink noise value, approximately in [-1.0, +1.0]
   *          (clamped to prevent occasional spikes)
   */
  float process() {
    // Step 1: Advance the counter. This drives the octave update pattern.
    counter_++;

    // Step 2: Find which row to update by counting trailing zeros.
    // counter=1 (binary ...001) → row 0 (update every sample)
    // counter=2 (binary ...010) → row 1 (update every 2 samples)
    // counter=4 (binary ...100) → row 2 (update every 4 samples)
    // This naturally creates the octave-spacing we need.
    int numZeros = 0;
    uint32_t n = counter_;
    while ((n & 1) == 0 && numZeros < NUM_ROWS - 1) {
      numZeros++;
      n >>= 1;
    }

    // Step 3: Replace the selected row with a new random value.
    // We subtract the old value and add the new one to maintain a
    // running sum — much faster than re-summing all 12 rows each sample.
    runningSum_ -= rows_[numZeros];
    float newVal = white_.process();
    runningSum_ += newVal;
    rows_[numZeros] = newVal;

    // Step 4: Add one more white noise sample for the highest octave
    // (this row conceptually changes every sample) and normalize.
    float out = (runningSum_ + white_.process()) / (NUM_ROWS + 1);

    // Step 5: Clamp to [-1, +1] to catch rare statistical spikes.
    return clampf(out, -1.0f, 1.0f);
  }

private:
  WhiteNoise white_;     // Internal white noise source
  uint32_t counter_;     // Sample counter drives row update scheduling
  float runningSum_;     // Sum of all rows (maintained incrementally)
  float rows_[NUM_ROWS]; // One random value per octave band
};

} // namespace bb
