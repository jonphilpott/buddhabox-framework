/**
 * @file one_pole.h
 * @brief Zero-delay-feedback one-pole low-pass / high-pass filter.
 *
 * The simplest useful filter, with a gentle 6 dB/octave slope. Low-pass
 * smooths control signals and tames harsh highs; high-pass removes
 * rumble. It uses the same zero-delay-feedback design as Svf, so the
 * cutoff stays accurate up to Nyquist and can change every sample.
 *
 * HOW IT WORKS: one integrator with feedback. Each sample computes the
 * low-pass output from the input and a single state variable; the
 * high-pass is whatever the low-pass removed (in - lp).
 *
 * Derived from stmlib dsp/filter.h, Copyright 2014 Emilie Gillet.
 * MIT licence.
 */

#pragma once

#include "buddhabox/filter/svf.h" // for TanApprox and tanApprox()

namespace bb {

/** The two outputs of OnePole::process(). */
struct OnePoleOut {
  float lp; ///< low-pass
  float hp; ///< high-pass (input minus low-pass)
};

/**
 * One-pole filter with simultaneous low-pass and high-pass outputs.
 *
 * Example — remove rumble below 80 Hz:
 *   OnePole hpf;
 *   hpf.setFrequency(80.0f);
 *
 *   float clean = hpf.process(in).hp;
 */
class OnePole {
public:
  OnePole() { init(); }

  /**
   * Set the sample rate (needed only by setFrequency()) and clear state.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    setCoefficients(0.01f, TanApprox::Dirty);
    reset();
  }

  /** Clear the filter's memory. */
  void reset() { state_ = 0.0f; }

  /**
   * Set the -3 dB point (uses tanf(): control rate).
   * @param hz  Cutoff in Hz
   */
  void setFrequency(float hz) {
    setCoefficients(hz * sampleTime_, TanApprox::Exact);
  }

  /**
   * Lower-level form.
   * @param f       Cutoff / sample rate, 0.0 to 0.5
   * @param approx  tan() approximation
   */
  void setCoefficients(float f, TanApprox approx) {
    g_ = tanApprox(f, approx);
    gi_ = 1.0f / (1.0f + g_);
  }

  /**
   * Process one sample.
   * @param in  Input sample
   * @return    The low-pass and high-pass outputs
   */
  OnePoleOut process(float in) {
    OnePoleOut o;
    o.lp = (g_ * in + state_) * gi_;
    state_ = g_ * (in - o.lp) + o.lp;
    o.hp = in - o.lp;
    return o;
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float g_ = 0.0f, gi_ = 1.0f;
  float state_ = 0.0f;
};

} // namespace bb
