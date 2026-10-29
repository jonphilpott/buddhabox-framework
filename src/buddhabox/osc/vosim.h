/**
 * @file vosim.h
 * @brief VOSIM ("VOice SIMulation") oscillator from Plaits.
 *
 * A 1970s technique for vowel-like sounds. Two formant sines are
 * restarted at the pitch rate and shaped by a decaying window, giving two
 * spectral peaks, as a voice has.
 *
 * HOW IT WORKS: at each pitch cycle both formant sines restart. A raised
 * cosine window (one per cycle) fades them, so each cycle is a short burst
 * of two sines. Pitch comes from the restart rate; the vowel colour comes
 * from the two formant frequencies. `shape` changes the sines' starting
 * phase, from smooth to buzzy.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Two-formant VOSIM oscillator.
 *
 * Example:
 *   VosimOscillator voice;
 *   voice.setCarrierFrequency(98.0f);            // G2
 *   voice.setFormantFrequencies(700.0f, 1200.0f); // roughly "ah"
 *   voice.setShape(0.3f);
 *
 *   float s = voice.process() * 0.5f;
 */
class VosimOscillator {
public:
  VosimOscillator() { init(); }

  /**
   * Reset the oscillator.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = formant1Phase_ = formant2Phase_ = 0.0f;
  }

  /** @param hz  The pitch in Hz (up to SAMPLE_RATE / 4) */
  void setCarrierFrequency(float hz) {
    carrier_ = minf(hz * sampleTime_, 0.25f);
  }

  /**
   * Set the two formants (spectral peaks). Vowels sit around 300-900 Hz
   * for the first and 900-2500 Hz for the second.
   * @param hz1  First formant in Hz
   * @param hz2  Second formant in Hz
   */
  void setFormantFrequencies(float hz1, float hz2) {
    formant1_ = minf(hz1 * sampleTime_, 0.25f);
    formant2_ = minf(hz2 * sampleTime_, 0.25f);
  }

  /** @param x  Starting phase of the bursts: smooth (0) to buzzy (1) */
  void setShape(float x) { shape_ = x; }

  /**
   * Generate the next sample. (No look-ahead: the restarts are smooth
   * enough without correction.)
   * @return  About -1.0 to +1.0
   */
  float process() {
    carrierPhase_ += carrier_;
    if (carrierPhase_ >= 1.0f) {
      carrierPhase_ -= 1.0f;
      float resetTime = carrierPhase_ / carrier_;
      formant1Phase_ = resetTime * formant1_;
      formant2Phase_ = resetTime * formant2_;
    } else {
      formant1Phase_ += formant1_;
      if (formant1Phase_ >= 1.0f)
        formant1Phase_ -= 1.0f;
      formant2Phase_ += formant2_;
      if (formant2Phase_ >= 1.0f)
        formant2Phase_ -= 1.0f;
    }
    // The window: half a cosine per cycle, 2 at the start to 0 at the end.
    float window = sineFromPhase(carrierPhase_ * 0.5f + 0.25f) + 1.0f;
    float resetPhase = 0.75f - 0.25f * shape_;
    float resetAmplitude = sineFromPhase(resetPhase);
    float f1 = sineFromPhase(formant1Phase_ + resetPhase) - resetAmplitude;
    float f2 = sineFromPhase(formant2Phase_ + resetPhase) - resetAmplitude;
    return window * (f1 + f2) * 0.25f + resetAmplitude;
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float carrier_ = 0.0f, formant1_ = 0.0f, formant2_ = 0.0f, shape_ = 0.0f;
  float carrierPhase_ = 0.0f, formant1Phase_ = 0.0f, formant2Phase_ = 0.0f;
};

} // namespace bb
