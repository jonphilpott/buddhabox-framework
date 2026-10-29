/**
 * @file lag.h
 * @brief Clock-synchronised glide between steps: Marbles' "steps" control.
 *
 * Glides between stepped values, from a sharp step to a smooth glide or a
 * sine-like curve, all in time with a phase ramp.
 *
 * HOW THIS DIFFERS FROM SlewLimiter (slew.h): SlewLimiter glides at a
 * fixed time in seconds. Lag's glide time follows the clock: the phase
 * ramp (0..1 per step, e.g. from ClockToRamp) tells it how long a step is,
 * so the same setting glides proportionally at any tempo.
 *
 * HOW IT WORKS:
 * - Smoothness 0..0.6: a low-pass glide, from instant (0) to about half a
 *   step (0.6).
 * - 0.6..1: crossfades into an interpolation along the step, from
 *   S-curved (raised cosine) to linear, so the output traces a smooth
 *   curve through the values instead of lagging behind them.
 *
 * BLACK PILL CHANGE: Partials called exp2() and cos() every sample; this
 * port uses semitonesToRatio() (fastExp2) and the flash sine table.
 *
 * Derived from Marbles, Copyright 2015 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Tempo-relative glide processor.
 *
 * Example — smooth a random sequence in time with a clock:
 *   ClockToRamp ramp;
 *   RandomSequence seq;
 *   Lag lag;
 *   float target = 0.0f, lastPhase = 0.0f;
 *
 *   float phase = ramp.process(clockHigh);
 *   if (phase < lastPhase) { // a new step began
 *     target = seq.next();
 *     lag.resetRamp();
 *   }
 *   lastPhase = phase;
 *   float cv = lag.process(target, 0.8f, phase);
 */
class Lag {
public:
  Lag() { init(); }

  /** Reset the glide state to 0. */
  void init() { rampStart_ = rampValue_ = lpState_ = previousPhase_ = 0.0f; }

  /**
   * Call at each new step (when the target changes), so the
   * interpolation starts from where the output is now.
   */
  void resetRamp() { rampStart_ = rampValue_; }

  /**
   * Advance one sample.
   * @param value       The current step's target value
   * @param smoothness  0.0 (sharp steps) to 1.0 (linear interpolation)
   * @param phase       Position within the step, 0.0 to 1.0, rising and
   *                    wrapping to 0 at the next step
   * @return            The smoothed value
   */
  float process(float value, float smoothness, float phase) {
    // 1. Step rate from the phase ramp.
    float frequency = phase - previousPhase_;
    if (frequency < 0.0f)
      frequency += 1.0f;
    previousPhase_ = phase;
    // 2. Low-pass glide: from half the step rate up to 7 octaves above;
    // near 0 smoothness, no lag at all.
    frequency *= 0.25f;
    frequency *= semitonesToRatio(84.0f * (1.0f - smoothness));
    if (frequency >= 1.0f)
      frequency = 1.0f;
    if (smoothness <= 0.05f)
      frequency += 20.0f * (0.05f - smoothness) * (1.0f - frequency);
    onePole(lpState_, value, frequency);
    // 3. Interpolation along the step: raised-cosine (S-curve) blending
    // to linear. 0.5 - 0.5 cos(pi x) = 0.5 - 0.5 sin(pi x + pi/2).
    float interpAmount = clampf((smoothness - 0.6f) * 5.0f, 0.0f, 1.0f);
    float linearity = clampf((1.0f - smoothness) * 5.0f, 0.0f, 1.0f);
    float warped = 0.5f - 0.5f * sineFromPhase(phase * 0.5f + 0.25f);
    float interpPhase = lerpf(warped, phase, linearity);
    float interp = lerpf(rampStart_, value, interpPhase);
    rampValue_ = interp;
    return lerpf(lpState_, interp, interpAmount);
  }

private:
  float rampStart_ = 0.0f, rampValue_ = 0.0f;
  float lpState_ = 0.0f, previousPhase_ = 0.0f;
};

} // namespace bb
