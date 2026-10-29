/**
 * @file grainlet.h
 * @brief Plaits' grainlet oscillator: one windowed sine grain per cycle.
 *
 * A stream of tiny grains, one per pitch cycle: a formant sine shaped by a
 * carrier window. Like the formant oscillator, but the window morphs from
 * a short blip to a long smooth bell, for sounds from glassy and
 * percussive to round and vocal.
 *
 * HOW IT WORKS:
 * - Each carrier cycle restarts the formant sine and multiplies it by a
 *   window (a stretched or squeezed half-sine) chosen by `shape`.
 * - `bleed` lets the window itself through as well, adding the carrier's
 *   fundamental and body.
 * - The restart is PolyBLEP-corrected.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * Grainlet oscillator.
 *
 * Example:
 *   GrainletOscillator g;
 *   g.setCarrierFrequency(130.8f); // C3
 *   g.setFormantFrequency(1500.0f);
 *   g.setShape(0.7f);              // long smooth bell
 *   g.setBleed(0.3f);
 *
 *   float s = g.process() * 0.5f;
 */
class GrainletOscillator {
public:
  GrainletOscillator() { init(); }

  /**
   * Reset the oscillator.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    carrierPhase_ = formantPhase_ = nextSample_ = 0.0f;
  }

  /** @param hz  Pitch in Hz (limited to SAMPLE_RATE / 8) */
  void setCarrierFrequency(float hz) {
    carrier_ = minf(hz * sampleTime_, 0.125f);
  }

  /** @param hz  Grain (formant) frequency in Hz */
  void setFormantFrequency(float hz) {
    formant_ = minf(hz * sampleTime_, 0.25f);
  }

  /** @param x  0..1: short blip, then a skewed bell, then a long bell */
  void setShape(float x) { shape_ = x; }

  /** @param x  0..1 (or more): how much of the window itself is heard */
  void setBleed(float x) { bleed_ = x; }

  /**
   * Generate the next sample.
   * @return  About -1.0 to +1.0, one sample late (BLEP look-ahead)
   */
  float process() {
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    carrierPhase_ += carrier_;
    if (carrierPhase_ >= 1.0f) {
      carrierPhase_ -= 1.0f;
      float resetTime = carrierPhase_ / carrier_;
      float before =
          grainlet(1.0f, formantPhase_ + (1.0f - resetTime) * formant_);
      float after = grainlet(0.0f, 0.0f);
      float discontinuity = after - before;
      thisSample += discontinuity * thisBlepSample(resetTime);
      nextSample += discontinuity * nextBlepSample(resetTime);
      formantPhase_ = resetTime * formant_;
    } else {
      formantPhase_ += formant_;
      if (formantPhase_ >= 1.0f)
        formantPhase_ -= 1.0f;
    }
    nextSample += grainlet(carrierPhase_, formantPhase_);
    nextSample_ = nextSample;
    return thisSample;
  }

private:
  /**
   * The window, 0..1, over one carrier cycle. Three regions of `shape`:
   * a squeezed half-sine (short blip), a half-sine with a moving peak,
   * and a stretched one (long bell).
   */
  float carrier(float phase) const {
    float shape = shape_ * 3.0f;
    int region = static_cast<int>(shape);
    float t = 1.0f - (shape - static_cast<float>(region));
    if (region == 0) {
      phase = phase * (1.0f + t * t * t * 15.0f);
      if (phase >= 1.0f)
        phase = 1.0f;
      phase += 0.75f;
    } else if (region == 1) {
      float breakpoint = 0.001f + 0.499f * t * t * t;
      phase = phase < breakpoint
                  ? phase * (0.5f / breakpoint)
                  : 0.5f + (phase - breakpoint) * 0.5f / (1.0f - breakpoint);
      phase += 0.75f;
    } else {
      t = 1.0f - t;
      phase = 0.25f + phase * (0.5f + t * t * t * 14.5f);
      if (phase >= 0.75f)
        phase = 0.75f;
    }
    return (sineFromPhase(phase) + 1.0f) * 0.25f;
  }

  /** One sample of the grain: window times (formant sine + bleed). */
  float grainlet(float carrierPhase, float formantPhase) const {
    return carrier(carrierPhase) * (sineFromPhase(formantPhase) + bleed_) /
           (1.0f + bleed_);
  }

  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float carrier_ = 0.0f, formant_ = 0.0f, shape_ = 0.0f, bleed_ = 0.0f;
  float carrierPhase_ = 0.0f, formantPhase_ = 0.0f, nextSample_ = 0.0f;
};

} // namespace bb
