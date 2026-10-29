/**
 * @file random_sequence.h
 * @brief Random values that can loop: Marbles' "déjà vu" control.
 *
 * At 0 every value is new; turning it up makes the sequence repeat a loop
 * of recent values, more and more faithfully; at 0.5 it locks to the
 * loop; past 0.5 it starts jumping randomly around inside the loop. Great
 * for generative melodies that develop, settle, and wander again.
 *
 * HOW IT WORKS:
 * - A loop buffer holds the last `length` values.
 * - At each step, a "mutation" happens with probability (2*dejaVu - 1)^2:
 *   high at both ends, zero in the middle.
 *   - Below 0.5, a mutation replaces the loop's next value with a fresh
 *     random one, so the loop slowly evolves.
 *   - Above 0.5, a mutation jumps to a random position in the loop.
 *   - Without a mutation, the sequence steps through the loop in order.
 * - next(input) records an external value instead (a looping
 *   sample-and-hold, or a shift register).
 *
 * Derived from Marbles, Copyright 2015 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/random.h"

namespace bb {

/**
 * Looping random sequence generator (call next() once per clock step).
 *
 * Example — an 8-step melody that slowly evolves:
 *   RandomSequence seq;
 *   seq.setLength(8);
 *   seq.setDejaVu(0.35f);
 *
 *   if (clock.process()) {
 *     float v = seq.next(); // 0..1
 *     int note = NOTE_C3 + (int)(v * 24.0f);
 *     osc.setFrequency(midiToFreq(note));
 *   }
 */
class RandomSequence {
public:
  /** Longest loop. */
  static const int kMaxLength = 16;

  RandomSequence() { init(); }

  /**
   * Fill the loop with fresh values.
   * @param seed  Random seed, for repeatable sequences
   */
  void init(uint32_t seed = 0x21) {
    random_.seed(seed);
    for (int i = 0; i < kMaxLength; ++i)
      loop_[i] = random_.uniform();
    writeHead_ = 0;
    length_ = 8;
    step_ = 0;
  }

  /** @param x  0 = always new, 0.5 = locked loop, 1 = random jumps */
  void setDejaVu(float x) { dejaVu_ = x; }

  /** @param n  Loop length in steps, 1 to 16 */
  void setLength(int n) {
    if (n < 1 || n > kMaxLength)
      return;
    length_ = n;
    step_ %= n;
  }

  /** Restart the loop from its first step. */
  void reset() { step_ = length_ - 1; }

  /** @return  The next random value, 0.0 to 1.0 */
  float next() { return advance(false, 0.0f); }

  /**
   * The next value, recording `input` into the loop when it mutates
   * instead of a random value.
   * @param input  Value to record, 0.0 to 1.0
   * @return       The next value, 0.0 to 1.0
   */
  float next(float input) { return advance(true, input); }

private:
  float advance(bool external, float input) {
    const float pSqrt = 2.0f * dejaVu_ - 1.0f;
    const bool mutate = random_.uniform() < pSqrt * pSqrt;
    if (mutate && dejaVu_ <= 0.5f) {
      // New value at the end of the loop.
      loop_[writeHead_] = external ? input : random_.uniform();
      writeHead_ = (writeHead_ + 1) % kMaxLength;
      step_ = length_ - 1;
    } else if (mutate) {
      // Jump somewhere in the loop.
      step_ = static_cast<int>(random_.uniform() * static_cast<float>(length_));
    } else {
      step_ = (step_ + 1) % length_;
    }
    return loop_[(writeHead_ + kMaxLength - length_ + step_) % kMaxLength];
  }

  Random random_;
  float loop_[kMaxLength] = {};
  int writeHead_ = 0, length_ = 8, step_ = 0;
  float dejaVu_ = 0.0f;
};

} // namespace bb
