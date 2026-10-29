/**
 * @file clocked_noise.h
 * @brief Band-limited sample-and-hold noise, from Plaits.
 *
 * Random values held for one clock period each: stepped "computer" noise
 * at low rates, then a pitched, crunchy buzz, then white noise.
 *
 * HOW IT WORKS: a phase accumulator ticks at the set rate; at each tick a
 * new random value is held. The steps land between samples, so each one
 * gets a PolyBLEP correction at its exact position. That keeps the tone
 * clean and pitched instead of aliased. Near and above a quarter of the
 * sample rate it crossfades into plain white noise. A sync input restarts
 * the clock, so a pattern of steps can be locked to a rhythm.
 *
 * Compare SampleAndHold (mod/sample_and_hold.h), which holds any input
 * you give it at control rate; ClockedNoise is an audio-rate source.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * Anti-aliased stepped noise.
 *
 * Example — a retro "computer thinking" bleep stream:
 *   ClockedNoise steps;
 *   steps.setFrequency(12.0f); // 12 new values per second
 *
 *   float s = steps.process() * 0.3f;
 */
class ClockedNoise {
public:
  ClockedNoise() { init(); }

  /**
   * Reset the generator.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = sample_ = nextSample_ = 0.0f;
  }

  /** @param hz  New values per second, 0 to SAMPLE_RATE */
  void setFrequency(float hz) {
    frequency_ = clampf(hz * sampleTime_, 0.0f, 1.0f);
  }

  /** @param s  Random seed, for repeatable patterns */
  void seed(uint32_t s) { random_.seed(s); }

  /**
   * Generate the next sample.
   * @param sync  true restarts the clock, so a new value is drawn now
   * @return      -1.0 to +1.0, one sample late (BLEP look-ahead)
   */
  float process(bool sync = false) {
    if (sync)
      phase_ = 1.0f;
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    const float raw = random_.bipolar();
    const float rawAmount = clampf(4.0f * (frequency_ - 0.25f), 0.0f, 1.0f);
    phase_ += frequency_;
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      float t = phase_ / frequency_;
      float discontinuity = raw - sample_;
      thisSample += discontinuity * thisBlepSample(t);
      nextSample += discontinuity * nextBlepSample(t);
      sample_ = raw;
    }
    nextSample += sample_;
    nextSample_ = nextSample;
    return thisSample + rawAmount * (raw - thisSample);
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float frequency_ = 0.001f;
  float phase_ = 0.0f, sample_ = 0.0f, nextSample_ = 0.0f;
  Random random_;
};

} // namespace bb
