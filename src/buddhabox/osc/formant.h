/**
 * @file formant.h
 * @brief Plaits' formant oscillator: a hard-synced sine.
 *
 * A sine at the "formant" frequency, restarted at the "carrier" (pitch)
 * frequency. The result has the pitch of the carrier but a peak in its
 * spectrum at the formant: a vocal or nasal tone. Sweeping the formant
 * sounds like a filter sweep with no filter.
 *
 * HOW IT WORKS: two phase counters. Each time the carrier wraps, the
 * formant sine restarts from zero phase (hard sync). The restart is a
 * jump, so it gets a PolyBLEP correction for an alias-free result.
 * `phaseShift` offsets the formant sine's phase, changing the waveform at
 * the restart.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * Formant (hard-synced sine) oscillator.
 *
 * Example:
 *   FormantOscillator voice;
 *   voice.setCarrierFrequency(110.0f); // the pitch you hear
 *   voice.setFormantFrequency(800.0f); // the "vowel" peak
 *
 *   // Sweep the formant slowly for a talking drone:
 *   voice.setFormantFrequency(mapf(lfo.process(), -1, 1, 400, 1600));
 *   float s = voice.process() * 0.5f;
 */
class FormantOscillator {
public:
  FormantOscillator() { init(); }

  /**
   * Reset the oscillator.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = formantPhase_ = nextSample_ = 0.0f;
  }

  /** @param hz  The pitch you hear, in Hz (up to SAMPLE_RATE / 4) */
  void setCarrierFrequency(float hz) {
    carrier_ = minf(hz * sampleTime_, 0.25f);
  }

  /** @param hz  The spectral peak in Hz, usually above the carrier */
  void setFormantFrequency(float hz) {
    formant_ = minf(hz * sampleTime_, 0.25f);
  }

  /** @param x  Phase offset of the formant sine, in cycles (0..1) */
  void setPhaseShift(float x) { phaseShift_ = x; }

  /**
   * Generate the next sample.
   * @return  About -1.0 to +1.0, one sample late (BLEP look-ahead)
   */
  float process() {
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    carrierPhase_ += carrier_;
    if (carrierPhase_ >= 1.0f) {
      // Restart the formant at the exact moment the carrier wrapped.
      carrierPhase_ -= 1.0f;
      float resetTime = carrierPhase_ / carrier_;
      float before = sineFromPhase(formantPhase_ +
                                   (1.0f - resetTime) * formant_ + phaseShift_);
      float after = sineFromPhase(phaseShift_);
      float discontinuity = after - before;
      thisSample += discontinuity * thisBlepSample(resetTime);
      nextSample += discontinuity * nextBlepSample(resetTime);
      formantPhase_ = resetTime * formant_;
    } else {
      formantPhase_ += formant_;
      if (formantPhase_ >= 1.0f)
        formantPhase_ -= 1.0f;
    }
    nextSample += sineFromPhase(formantPhase_ + phaseShift_);
    nextSample_ = nextSample;
    return thisSample;
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float carrier_ = 0.0f, formant_ = 0.01f, phaseShift_ = 0.0f;
  float carrierPhase_ = 0.0f, formantPhase_ = 0.0f, nextSample_ = 0.0f;
};

} // namespace bb
