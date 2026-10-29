/**
 * @file plucker.h
 * @brief Rings' pick/finger exciter for StringModel and ModalResonator.
 *
 * A short burst of noise shaped like a pick or a finger, ready to excite
 * a resonator. Call trigger() on each note.
 *
 * HOW IT WORKS:
 * - Each trigger emits white noise for a fraction of one period of the
 *   note. The fraction depends on `position`.
 * - The burst goes through a feedback comb filter. A pick near the end of
 *   a string excites harmonics differently from one in the middle, and
 *   the comb imitates that.
 * - A low-pass filter sets the burst's brightness: soft finger or hard
 *   pick.
 *
 * MEMORY: the comb holds up to 256 floats (1 KB). Float storage, because
 * the comb's feedback can push the burst above ±1.
 *
 * Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/delay.h"

namespace bb {

/**
 * Noise-burst pluck exciter.
 *
 * Example:
 *   Plucker pick;
 *   StringModel<> string;
 *   pick.setFrequency(196.0f); // match the string's pitch
 *   pick.setCutoff(4000.0f);   // a bright pick
 *   string.setFrequency(196.0f);
 *
 *   if (noteOn) {
 *     pick.trigger();
 *   }
 *   float s = string.process(pick.process());
 */
class Plucker {
public:
  Plucker() { init(); }

  /**
   * Reset the burst and filters.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    // Rings: comb delay up to 255 samples at 48 kHz.
    maxComb_ = minf(255.0f * sampleRate / 48000.0f, kCombSize - 1.0f);
    comb_.clear();
    svf_.init(sampleRate);
    remaining_ = 0;
    combPeriod_ = 0.0f;
    combGain_ = 0.0f;
  }

  /** @param hz  Pitch of the note being plucked (sets burst length) */
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }

  /** @param hz  Burst low-pass cutoff: low = soft, high = bright pick */
  void setCutoff(float hz) { cutoff_ = hz / sampleRate_; }

  /** @param x  0..1: pluck position along the string */
  void setPosition(float x) { position_ = x; }

  /** @param s  Random seed, for repeatable bursts */
  void seed(uint32_t s) { random_.seed(s); }

  /** Start a new burst using the current settings. */
  void trigger() {
    float ratio = position_ * 0.9f + 0.05f;
    float period = 1.0f / frequency_ * ratio;
    remaining_ = static_cast<uint32_t>(period);
    while (period >= maxComb_)
      period *= 0.5f;
    combPeriod_ = period;
    combGain_ = (1.0f - position_) * 0.8f;
    svf_.setCoefficients(cutoff_ < 0.499f ? cutoff_ : 0.499f, 1.0f,
                         TanApprox::Dirty);
  }

  /**
   * Generate the next sample of the burst.
   * @return  The excitation signal (0 between bursts, once rung out)
   */
  float process() {
    float in = 0.0f;
    if (remaining_) {
      in = random_.bipolar();
      --remaining_;
    }
    // combPeriod_ samples before this write = read(combPeriod_ - 1).
    float s = in + combGain_ * comb_.read(combPeriod_ - 1.0f);
    comb_.write(s);
    return svf_.process(s).lp;
  }

private:
  static const uint16_t kCombSize = 256;
  float sampleRate_ = SAMPLE_RATE;
  float maxComb_ = 234.0f;
  float frequency_ = 220.0f / SAMPLE_RATE;
  float cutoff_ = 0.1f;
  float position_ = 0.5f;
  uint32_t remaining_ = 0;
  float combPeriod_ = 0.0f;
  float combGain_ = 0.0f;
  Random random_;
  DelayLine<kCombSize, float> comb_;
  Svf svf_;
};

} // namespace bb
