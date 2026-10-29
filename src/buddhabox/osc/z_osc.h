/**
 * @file z_osc.h
 * @brief Plaits' Z oscillator: Casio CZ-style "resonance" waveforms.
 *
 * A resonant, filter-sweep-like tone with no filter, in the style of the
 * Casio CZ synths' resonance waveforms.
 *
 * HOW IT WORKS (the CZ trick):
 * - A "formant" sine runs at the resonant frequency and is restarted twice
 *   per carrier cycle.
 * - It is multiplied by a window that fades out over each half cycle
 *   ("ramp down"), and by a contour shaped by `shape`.
 * - The window makes the formant sound like a resonant filter excited
 *   once per cycle, and sweeping the formant sounds like sweeping the
 *   cutoff.
 * - `mode` picks one of three CZ-like window/offset styles.
 * - Restarts are PolyBLEP-corrected.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * CZ-style resonant oscillator.
 *
 * Example:
 *   ZOscillator z;
 *   z.setCarrierFrequency(55.0f);
 *   z.setMode(0.5f);
 *
 *   // "Filter sweep" by moving the resonance:
 *   z.setFormantFrequency(mapf(env.process(), 0, 1, 100, 3000));
 *   float s = z.process() * 0.5f;
 */
class ZOscillator {
public:
  ZOscillator() { init(); }

  /**
   * Reset the oscillator.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = discontinuityPhase_ = formantPhase_ = nextSample_ = 0.0f;
  }

  /** @param hz  Pitch in Hz (limited to SAMPLE_RATE / 8) */
  void setCarrierFrequency(float hz) {
    carrier_ = minf(hz * sampleTime_, 0.125f);
  }

  /** @param hz  The "resonance" frequency in Hz */
  void setFormantFrequency(float hz) {
    formant_ = minf(hz * sampleTime_, 0.25f);
  }

  /**
   * @param x  0..1. Below 0.5 narrows each cycle's window; above 0.5
   *           shifts its phase
   */
  void setShape(float x) { shape_ = x; }

  /** @param x  0..1: three styles at 0..1/3, 1/3..2/3, 2/3..1 */
  void setMode(float x) { mode_ = x; }

  /**
   * Generate the next sample.
   * @return  About -1.0 to +1.0, one sample late (BLEP look-ahead)
   */
  float process() {
    const float f0 = carrier_, f1 = formant_;
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    // The discontinuity phase runs at twice the carrier: two restarts per
    // carrier cycle.
    discontinuityPhase_ += 2.0f * f0;
    carrierPhase_ += f0;
    if (discontinuityPhase_ >= 1.0f) {
      discontinuityPhase_ -= 1.0f;
      float resetTime = discontinuityPhase_ / (2.0f * f0);
      float carrierBefore = carrierPhase_ >= 1.0f ? 1.0f : 0.5f;
      float carrierAfter = carrierPhase_ >= 1.0f ? 0.0f : 0.5f;
      float before = z(carrierBefore, 1.0f,
                       formantPhase_ + (1.0f - resetTime) * f1, shape_, mode_);
      float after = z(carrierAfter, 0.0f, 0.0f, shape_, mode_);
      float discontinuity = after - before;
      thisSample += discontinuity * thisBlepSample(resetTime);
      nextSample += discontinuity * nextBlepSample(resetTime);
      formantPhase_ = resetTime * f1;
      if (carrierPhase_ > 1.0f)
        carrierPhase_ = discontinuityPhase_ * 0.5f;
    } else {
      formantPhase_ += f1;
      if (formantPhase_ >= 1.0f)
        formantPhase_ -= 1.0f;
    }
    if (carrierPhase_ >= 1.0f)
      carrierPhase_ -= 1.0f;
    nextSample +=
        z(carrierPhase_, discontinuityPhase_, formantPhase_, shape_, mode_);
    nextSample_ = nextSample;
    return thisSample;
  }

private:
  /**
   * One sample of the windowed formant.
   * @param c  Carrier phase
   * @param d  Position in the current half cycle
   * @param f  Formant phase
   */
  static float z(float c, float d, float f, float shape, float mode) {
    float rampDown = 0.5f * (1.0f + sineFromPhase(0.5f * d + 0.25f));
    float offset, phaseShift;
    if (mode < 0.333f) {
      offset = 1.0f;
      phaseShift = 0.25f + mode * 1.50f;
    } else if (mode < 0.666f) {
      phaseShift = 0.7495f - (mode - 0.33f) * 0.75f;
      offset = -sineFromPhase(phaseShift);
    } else {
      phaseShift = 0.7495f - (mode - 0.33f) * 0.75f;
      offset = 0.001f;
    }
    float discontinuity = sineFromPhase(f + phaseShift);
    float contour;
    if (shape < 0.5f) {
      shape *= 2.0f;
      if (c >= 0.5f)
        rampDown *= shape;
      contour = 1.0f + (sineFromPhase(c + 0.25f) - 1.0f) * shape;
    } else {
      contour = sineFromPhase(c + shape * 0.5f);
    }
    return (rampDown * (offset + discontinuity) - offset) * contour;
  }

  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float carrier_ = 0.0f, formant_ = 0.0f, shape_ = 0.0f, mode_ = 0.0f;
  float carrierPhase_ = 0.0f, discontinuityPhase_ = 0.0f;
  float formantPhase_ = 0.0f, nextSample_ = 0.0f;
};

} // namespace bb
