/**
 * @file limiter.h
 * @brief Smooth output limiter from Rings and Warps.
 *
 * Keeps a signal from exceeding full scale, smoothly, with no hard
 * clipping. Put it last in a chain that might get loud: resonators,
 * feedback, stacked voices. Especially useful before the int16_t
 * conversion in your audio callback.
 *
 * HOW IT WORKS:
 * - A peak follower tracks the level. It rises fast (attack about 0.5 ms)
 *   and falls very slowly (release about 1 s), so the gain doesn't pump
 *   on every waveform cycle.
 * - When the peak goes above 1, the gain becomes 1/peak, so loud passages
 *   are turned down to full scale.
 * - A soft limiter (scaled by 0.8 first) rounds off fast transients the
 *   follower is too slow to catch.
 * - In stereo, both channels share one gain, which also watches the
 *   left-right difference, so the stereo image doesn't wobble.
 *
 * Derived from Rings and Warps, Copyright 2014-2015 Emilie Gillet.
 * MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Peak-following soft limiter.
 *
 * Example:
 *   Limiter limiter;
 *   float l = mixL, r = mixR;
 *   limiter.process(l, r); // now safely within ±1
 */
class Limiter {
public:
  Limiter() { init(); }

  /**
   * Reset the peak follower.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    float ratio = 48000.0f / sampleRate;
    attack_ = rescaleCoefficient(0.05f, ratio);
    release_ = rescaleCoefficient(0.00002f, ratio);
    peak_ = 0.5f;
  }

  /**
   * @param gain  Gain before limiting (1 = unity). Raise it to make quiet
   *              material louder; the limiter catches the peaks.
   */
  void setPreGain(float gain) { preGain_ = gain; }

  /**
   * Limit one mono sample.
   * @param in  Input sample, any level
   * @return    Limited sample, within ±1
   */
  float process(float in) {
    float pre = in * preGain_;
    slope(peak_, fabsf(pre), attack_, release_);
    return softLimit(pre * gain() * 0.8f);
  }

  /**
   * Limit one stereo sample in place, with a shared gain.
   * @param left   Left in/out
   * @param right  Right in/out
   */
  void process(float &left, float &right) {
    float l = left * preGain_;
    float r = right * preGain_;
    float peak = maxf(maxf(fabsf(l), fabsf(r)), fabsf(r - l));
    slope(peak_, peak, attack_, release_);
    float g = gain() * 0.8f;
    left = softLimit(l * g);
    right = softLimit(r * g);
  }

private:
  float gain() const { return peak_ <= 1.0f ? 1.0f : 1.0f / peak_; }

  float attack_ = 0.05f, release_ = 0.00002f;
  float peak_ = 0.5f;
  float preGain_ = 1.0f;
};

} // namespace bb
