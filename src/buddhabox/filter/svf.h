/**
 * @file svf.h
 * @brief Zero-delay-feedback state-variable filter (stmlib's Svf).
 *
 * One filter giving low-pass, band-pass and high-pass outputs at once.
 *
 * This is the "topology-preserving transform" (TPT, or zero-delay
 * feedback) SVF described by Andrew Simper and Vadim Zavalishin. Unlike
 * the classic Chamberlin SVF (SVFilter, sv_filter.h) it stays stable and
 * in tune right up to Nyquist, and its coefficients can change every
 * sample without blowing up. That is why Gillet uses it everywhere:
 * filters, resonator modes, formants, drums.
 *
 * WHICH SVF SHOULD I USE?
 * - SVFilter: the simpler original BuddhaBox filter. Set a cutoff in Hz
 *   and a 0..1 resonance; cutoff limited to SAMPLE_RATE / 4.
 * - Svf: accurate to Nyquist, takes a Q (0.5 = no peak, 10+ = ringing),
 *   and lets you pick a cheap tan() approximation for per-sample sweeps.
 *
 * Derived from stmlib dsp/filter.h, Copyright 2014 Emilie Gillet.
 * MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * How Svf computes g = tan(pi * f).
 *
 * tan() is slow on the Cortex-M4, so stmlib offers polynomial
 * approximations. They differ audibly only near Nyquist; ported modules
 * use the same one as the original so they sound the same.
 */
enum class TanApprox {
  Exact,    ///< tanf(): exact, slowest (control rate only)
  Accurate, ///< 11th-order polynomial: good across the audio range
  Fast,     ///< 5th-order: tuned for 16 Hz..16 kHz at 48 kHz
  Dirty     ///< 3rd-order: fine below about 8 kHz
};

/**
 * tan(pi * f) for a normalised frequency f (cutoff / sample rate).
 *
 * Each polynomial is a truncated Taylor series of tan(pi*f) with
 * coefficients tweaked to minimise error over its intended range. The
 * pi^n factors fold the "pi *" inside tan() into the coefficients;
 * `constexpr` makes the compiler compute them, so no double-precision
 * maths (slow, software-emulated on the M4) runs on the chip.
 *
 * @param f       Normalised frequency, 0.0 to 0.5
 * @param approx  Which approximation to use
 * @return        tan(pi * f)
 *
 * Example:
 *   float g = tanApprox(1000.0f / SAMPLE_RATE, TanApprox::Fast);
 */
inline float tanApprox(float f, TanApprox approx) {
  constexpr double pi = 3.14159265358979323846;
  constexpr float piF = static_cast<float>(pi);
  float f2 = f * f;
  switch (approx) {
  case TanApprox::Exact:
    f = f < 0.497f ? f : 0.497f; // tan explodes at f = 0.5
    return tanf(piF * f);
  case TanApprox::Dirty: {
    constexpr float a = static_cast<float>(3.736e-01 * pi * pi * pi);
    return f * (piF + a * f2);
  }
  case TanApprox::Fast: {
    constexpr float a = static_cast<float>(3.260e-01 * pi * pi * pi);
    constexpr float b = static_cast<float>(1.823e-01 * pi * pi * pi * pi * pi);
    return f * (piF + f2 * (a + b * f2));
  }
  case TanApprox::Accurate:
  default: {
    constexpr double pi3 = pi * pi * pi;
    constexpr double pi5 = pi3 * pi * pi;
    constexpr double pi7 = pi5 * pi * pi;
    constexpr double pi9 = pi7 * pi * pi;
    constexpr double pi11 = pi9 * pi * pi;
    constexpr float a = static_cast<float>(3.333314036e-01 * pi3);
    constexpr float b = static_cast<float>(1.333923995e-01 * pi5);
    constexpr float c = static_cast<float>(5.33740603e-02 * pi7);
    constexpr float d = static_cast<float>(2.900525e-03 * pi9);
    constexpr float e = static_cast<float>(9.5168091e-03 * pi11);
    return f * (piF + f2 * (a + f2 * (b + f2 * (c + f2 * (d + f2 * e)))));
  }
  }
}

/** The three simultaneous outputs of Svf::process(). */
struct SvfOut {
  float lp; ///< low-pass
  float bp; ///< band-pass
  float hp; ///< high-pass
};

/**
 * Zero-delay-feedback state-variable filter.
 *
 * Example:
 *   Svf filter;
 *   filter.setFrequency(800.0f, 2.0f); // 800 Hz, gentle peak
 *
 *   // In the audio callback:
 *   SvfOut o = filter.process(noise.process());
 *   float dark = o.lp, nasal = o.bp, thin = o.hp;
 */
class Svf {
public:
  Svf() { init(); }

  /**
   * Set the sample rate (needed only by setFrequency()) and clear state.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    setCoefficients(0.01f, 100.0f, TanApprox::Dirty);
    reset();
  }

  /** Silence the filter's memory (its two integrators). */
  void reset() { state1_ = state2_ = 0.0f; }

  /**
   * Set cutoff and resonance (uses tanf(): control rate).
   * @param hz  Cutoff in Hz, up to just below SAMPLE_RATE / 2
   * @param q   Resonance: 0.5 = no peak, 0.707 = Butterworth, 10+ =
   *            ringing
   */
  void setFrequency(float hz, float q) {
    setCoefficients(hz * sampleTime_, q, TanApprox::Exact);
  }

  /**
   * Lower-level form used by components.
   * @param f       Cutoff / sample rate, 0.0 to 0.5
   * @param q       Resonance (see setFrequency())
   * @param approx  tan() approximation; Fast or Dirty for sweeps
   */
  void setCoefficients(float f, float q, TanApprox approx) {
    setGQ(tanApprox(f, approx), q);
  }

  /**
   * Set the pre-warped frequency g directly (e.g. copied from another
   * filter's g()).
   * @param g  tan(pi * f)
   * @param q  Resonance
   */
  void setGQ(float g, float q) {
    g_ = g;
    r_ = 1.0f / q;
    h_ = 1.0f / (1.0f + r_ * g_ + g_ * g_);
  }

  /**
   * Process one sample; all three responses come out together. Unused
   * outputs cost nothing: the compiler drops them.
   *
   * HOW IT WORKS: two integrators in a loop. Solving that loop for the
   * high-pass output first (the `hp` line) lets each integrator use the
   * current sample instead of the previous one, which is what keeps the
   * tuning exact at high frequencies.
   *
   * @param in  Input sample
   * @return    The low-pass, band-pass and high-pass outputs
   */
  SvfOut process(float in) {
    SvfOut o;
    o.hp = (in - r_ * state1_ - g_ * state1_ - state2_) * h_;
    o.bp = g_ * o.hp + state1_;
    state1_ = g_ * o.hp + o.bp;
    o.lp = g_ * o.bp + state2_;
    state2_ = g_ * o.bp + o.lp;
    return o;
  }

  /** @return  The pre-warped frequency coefficient g */
  float g() const { return g_; }

  /**
   * @return  1 / q. Multiply bp by this for a band-pass with unity gain
   *          at the peak whatever the resonance ("normalised band-pass").
   */
  float r() const { return r_; }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float g_ = 0.0f, r_ = 1.0f, h_ = 1.0f;
  float state1_ = 0.0f, state2_ = 0.0f;
};

} // namespace bb
