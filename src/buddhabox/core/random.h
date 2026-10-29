/**
 * @file random.h
 * @brief LFSR-based random number generator for non-audio randomness.
 *
 * While WhiteNoise in noise.h is optimized for generating audio-rate noise
 * signals, this LFSR (Linear Feedback Shift Register) is designed for
 * "decision-making" randomness:
 *   - Picking random notes from a scale
 *   - Deciding whether to trigger an event (probability gates)
 *   - Generating random parameter values (filter cutoff, delay time, etc.)
 *   - Shuffling sequence patterns
 *
 * HOW AN LFSR WORKS:
 * ──────────────────
 * An LFSR is a shift register where the input bit is computed by XOR-ing
 * selected "tap" positions from the current state. Think of it like a
 * scrambler — each step shifts all bits left by one and feeds back a
 * combination of bits to the input.
 *
 * The tap positions (31, 21, 1, 0 for our 32-bit version) are chosen so
 * that the register cycles through ALL possible non-zero states before
 * repeating — a "maximal length" sequence of 2^32 - 1 values.
 *
 * LFSRs are extremely fast (just shifts and XORs) and produce decent
 * pseudo-randomness for our purposes, though they wouldn't pass rigorous
 * statistical tests. For generative music, that's perfectly fine.
 *
 * Example usage:
 *   LFSR rng(42);
 *
 *   // Pick a random note from a pentatonic scale
 *   uint8_t pentatonic[] = {60, 62, 64, 67, 69};  // C D E G A
 *   uint8_t note = pentatonic[rng.nextRange(0, 4)];
 *
 *   // 30% probability gate
 *   bool trigger = rng.nextFloat() < 0.3f;
 *
 *   // Random filter cutoff between 200 and 2000 Hz
 *   float cutoff = 200.0f + rng.nextFloat() * 1800.0f;
 */

#pragma once

#include <cstdint>

namespace bb {

/**
 * LFSR-based random number generator for non-audio randomness.
 *
 * See the file header above for how it works and usage examples.
 */
class LFSR {
public:
  /**
   * @param seed  Initial state. Must be non-zero (zero would produce
   *              zero forever since XOR of zeros is zero).
   *              Different seeds produce different sequences.
   */
  LFSR(uint32_t seed = 0xACE1u) : state_(seed ? seed : 1) {}

  /**
   * Reset with a new seed value.
   * @param s  New seed (0 is forced to 1 to avoid the stuck-at-zero state)
   */
  void seed(uint32_t s) { state_ = s ? s : 1; }

  /**
   * Advance the LFSR by one step and return the raw 32-bit state.
   *
   * The feedback polynomial uses taps at bits 31, 21, 1, and 0.
   * These tap positions produce a maximal-length sequence — the LFSR
   * visits all 2^32 - 1 non-zero states before repeating.
   *
   * @return  Raw 32-bit pseudo-random value
   *
   * Example:
   *   uint32_t raw = rng.next();  // Full 32-bit random value
   */
  uint32_t next() {
    // XOR the tap bits together to produce the feedback bit.
    // Bit positions 31, 21, 1, 0 form a maximal-length polynomial.
    uint32_t bit =
        ((state_ >> 31) ^ (state_ >> 21) ^ (state_ >> 1) ^ state_) & 1u;
    // Shift the register left and insert the feedback bit at position 0
    state_ = (state_ << 1) | bit;

    // if, for some bizarre reason the state has become 0, set it to this
    // comedy number.
    if (state_ == 0) {
      state_ = 69;
    }

    return state_;
  }

  /*
   * flips a bit in the LFSR based on if val's LSB is 1 or not.  the
   * idea here is that you can pass in the value from one of the
   * analog inputs which will be naturally noisy to flip a bit and
   * change up the sequence
   *
   * @return void
   *
   * Example:
   *   rng.add_entropy(42)
   */
  void add_entropy(uint8_t val) {
    if (val & 1) {
      state_ = state_ ^ 1;
    }
  }

  /**
   * Generate a random float in [0.0, 1.0].
   *
   * Useful for probability checks and generating parameter values.
   *
   * @return  Uniformly distributed float between 0.0 and 1.0
   *
   * Example:
   *   if (rng.nextFloat() < 0.25f) {
   *       // This branch executes ~25% of the time
   *       changeNote();
   *   }
   */
  float nextFloat() {
    // Mask off the sign bit to get a positive value, then normalize
    // to 0.0–1.0 by dividing by the max positive 31-bit integer.
    return (float)(next() & 0x7FFFFFFF) / 2147483647.0f;
  }

  /**
   * Generate a random float in [-1.0, +1.0].
   *
   * Useful for random modulation offsets (e.g., slight pitch drift).
   *
   * @return  Uniformly distributed float between -1.0 and +1.0
   *
   * Example:
   *   // Add subtle random pitch drift to an oscillator
   *   float drift = rng.nextBipolar() * 0.5f;  // ±0.5 Hz
   *   osc.setFrequency(baseFreq + drift);
   */
  float nextBipolar() { return nextFloat() * 2.0f - 1.0f; }

  /**
   * Generate a random integer in [min, max] inclusive.
   *
   * Uses modulo to constrain the range. Note: this has slight bias for
   * ranges that don't evenly divide 2^32, but for musical purposes
   * (picking from 5-12 notes in a scale) the bias is negligible.
   *
   * @param min  Minimum value (inclusive)
   * @param max  Maximum value (inclusive)
   * @return     Random integer in [min, max]
   *
   * Example:
   *   // Pick a random MIDI note between C3 and C5
   *   uint8_t note = rng.nextRange(48, 72);
   *
   *   // Random index into a 5-element array
   *   int idx = rng.nextRange(0, 4);
   */
  uint32_t nextRange(uint32_t min, uint32_t max) {
    if (min >= max)
      return min;
    return min + (next() % (max - min + 1));
  }

  uint32_t getState() { return state_; }

private:
  uint32_t state_;
};

/**
 * Fast pseudo-random numbers for audio-rate noise and randomised DSP.
 *
 * WHY A SECOND GENERATOR?
 * LFSR::next() shifts its register by one bit per call, so each value is
 * roughly double the previous one: fine for picking notes now and then,
 * but strongly correlated if you draw a new value every sample. Random is
 * a "linear congruential generator" (LCG): state = state * a + c, letting
 * the 32-bit multiply overflow wrap around. Every call scrambles all 32
 * bits, it costs two instructions, and it is what Mutable Instruments'
 * modules use for noise. (Not suitable for cryptography.)
 *
 * Each module owns its own Random, so instances don't disturb each other
 * and a seeded instance always repeats the same sequence.
 *
 * Derived from stmlib, Copyright 2012 Emilie Gillet. MIT licence.
 *
 * Example:
 *   Random rng;
 *   rng.seed(42);
 *   float noise = rng.bipolar(); // -1.0 to +1.0
 *   if (rng.uniform() < 0.1f) {  // 10% chance
 *     // ...
 *   }
 */
struct Random {
  uint32_t state = 0x21;

  /**
   * Restart the sequence from a new seed.
   * @param s  Any 32-bit value (0 is fine for an LCG)
   */
  void seed(uint32_t s) { state = s; }

  /**
   * Next raw 32-bit value. The constants are the classic ones from
   * Numerical Recipes, the same as stmlib's.
   * @return  A pseudo-random uint32_t
   */
  uint32_t word() {
    state = state * 1664525u + 1013904223u;
    return state;
  }

  /** @return  A uniform float in [0.0, 1.0) */
  float uniform() { return static_cast<float>(word()) / 4294967296.0f; }

  /** @return  A uniform float in [-1.0, 1.0): white noise */
  float bipolar() { return uniform() * 2.0f - 1.0f; }
};

} // namespace bb
