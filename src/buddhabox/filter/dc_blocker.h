/**
 * @file dc_blocker.h
 * @brief Removes DC offset and sub-audio drift.
 *
 * A one-pole high-pass filter, y[n] = x[n] - x[n-1] + pole * y[n-1].
 *
 * WHY: offsets creep in from asymmetric waveshapers, rectifiers and
 * feedback loops. An offset wastes headroom (the DAC clips sooner) and
 * makes speakers thump when a sound starts or stops.
 *
 * HOW IT WORKS: the difference x[n] - x[n-1] cancels anything constant;
 * the feedback term restores everything above the cutoff. The closer
 * `pole` is to 1, the lower the cutoff.
 *
 * Derived from stmlib dsp/filter.h, Copyright 2014 Emilie Gillet.
 * MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * DC-blocking high-pass filter.
 *
 * Example:
 *   DcBlocker dc;
 *   float out = dc.process(softclip(in * 4.0f));
 */
class DcBlocker {
public:
  DcBlocker() { init(); }

  /**
   * Set the cutoff and clear state.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   * @param cutoffHz    Nominal cutoff; 5..20 Hz is typical
   */
  void init(float sampleRate = SAMPLE_RATE, float cutoffHz = 10.0f) {
    // Gillet writes the pole as 1 - fc / sampleRate (no 2*pi): her cutoff
    // figures are nominal, so we keep the same formula to match.
    pole_ = 1.0f - cutoffHz / sampleRate;
    reset();
  }

  /** Clear the filter's memory. */
  void reset() { x_ = y_ = 0.0f; }

  /**
   * Process one sample.
   * @param in  Input sample
   * @return    The input with DC removed
   */
  float process(float in) {
    y_ = y_ * pole_ + in - x_;
    x_ = in;
    return y_;
  }

private:
  float pole_ = 0.999f;
  float x_ = 0.0f, y_ = 0.0f;
};

} // namespace bb
