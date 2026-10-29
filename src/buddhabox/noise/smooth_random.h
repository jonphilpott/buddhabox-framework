/**
 * @file smooth_random.h
 * @brief A smoothly wandering random signal (a random LFO), from Plaits.
 *
 * At each tick of its clock it picks a new random target (±1) and glides
 * there along an S-curve, so there are no corners. Use it for vibrato
 * that never repeats, "living" filter sweeps, slow drift, or random pans.
 *
 * HOW IT WORKS: a phase accumulator at the set rate; between ticks, the
 * output moves from the old value to the new one along
 * smoothStep(phase) = 3t^2 - 2t^3, which starts and ends with zero slope.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"

namespace bb {

/**
 * Smooth random modulation source.
 *
 * Example — a drone whose pitch drifts by a few cents:
 *   SmoothRandom drift;
 *   drift.setFrequency(0.2f); // a new target every 5 seconds
 *
 *   float cents = 8.0f * drift.process();
 *   osc.setFrequency(110.0f * semitonesToRatio(cents / 100.0f));
 */
class SmoothRandom {
public:
  SmoothRandom() { init(); }

  /**
   * Reset to 0.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = from_ = interval_ = 0.0f;
  }

  /** @param hz  New targets per second */
  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }

  /** @param s  Random seed, for repeatable wanderings */
  void seed(uint32_t s) { random_.seed(s); }

  /**
   * Generate the next value.
   * @return  -1.0 to +1.0
   */
  float process() {
    phase_ += frequency_;
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      from_ += interval_;
      interval_ = random_.bipolar() - from_;
    }
    return from_ + interval_ * smoothStep(phase_);
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float frequency_ = 0.0f;
  float phase_ = 0.0f, from_ = 0.0f, interval_ = 0.0f;
  Random random_;
};

} // namespace bb
