/**
 * @file decimator.h
 * @brief Anti-aliased sample-rate reducer from Plaits.
 *
 * The lo-fi crunch of an old sampler or game console: the input is
 * sampled-and-held at a lower rate.
 *
 * HOW IT WORKS: a phase accumulator ticks at the target rate. At each
 * tick it grabs a new input value and holds it. Done naively, the steps
 * land on whole samples, which adds harsh, inharmonic aliasing. Here each
 * step is placed at its exact fractional position: the input is
 * interpolated at that instant, and the step's edge is smoothed with a
 * PolyBLEP correction. What remains is the intended stair-step aliasing,
 * without the extra grit. Costs one sample of latency while active.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * Sample-rate reducer.
 *
 * Example:
 *   Decimator crush;
 *   crush.setRate(6000.0f); // 6 kHz "sampler" sound
 *
 *   float s = crush.process(osc.process());
 */
class Decimator {
public:
  Decimator() { init(); }

  /**
   * Reset the hold state.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    phase_ = 0.0f;
    sample_ = nextSample_ = previousSample_ = 0.0f;
    setRate(rateHz_);
  }

  /**
   * @param hz  Target sample rate; at or above SAMPLE_RATE the signal
   *            passes through unchanged
   */
  void setRate(float hz) {
    rateHz_ = hz;
    frequency_ = clampf(hz / sampleRate_, 0.0f, 1.0f);
  }

  /**
   * Process one sample.
   * @param in  Input sample
   * @return    The decimated sample (one sample late while active)
   */
  float process(float in) {
    // At or above the host rate: pass through untouched (no latency).
    if (frequency_ >= 1.0f) {
      sample_ = nextSample_ = previousSample_ = in;
      return in;
    }
    float thisSample = nextSample_;
    nextSample_ = 0.0f;
    phase_ += frequency_;
    if (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      // t: how far back (in host samples, as a fraction) the tick really
      // happened; 0 = exactly now.
      float t = phase_ / frequency_;
      // Interpolate the input at the tick's true position.
      float newSample = previousSample_ + (in - previousSample_) * (1.0f - t);
      // Spread the step over this sample and the next (PolyBLEP).
      float discontinuity = newSample - sample_;
      thisSample += discontinuity * thisBlepSample(t);
      nextSample_ += discontinuity * nextBlepSample(t);
      sample_ = newSample;
    }
    nextSample_ += sample_;
    previousSample_ = in;
    return thisSample;
  }

private:
  float sampleRate_ = SAMPLE_RATE;
  float rateHz_ = 8000.0f;
  float frequency_ = 1.0f;
  float phase_ = 0.0f;
  float sample_ = 0.0f, nextSample_ = 0.0f, previousSample_ = 0.0f;
};

} // namespace bb
