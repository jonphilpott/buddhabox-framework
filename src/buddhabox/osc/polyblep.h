/**
 * @file polyblep.h
 * @brief PolyBLEP corrections: the standard trick for alias-free jumps.
 *
 * WHY: a jump that happens *between* two samples (a saw resetting, a
 * square flipping, a sample-and-hold step) can't be represented exactly,
 * so it lands on the nearest sample. That rounding error contains
 * frequencies above Nyquist, which fold back down as inharmonic aliasing:
 * the "digital" whine of naive oscillators.
 *
 * HOW: we know where the jump really happened (a fraction t of a sample
 * ago) and how big it was. A band-limited jump differs from a hard one
 * only near the discontinuity, so we add a small two-sample correction:
 * one part to this sample, one to the next. "BLEP" = band-limited step;
 * the polynomial versions here are cheap approximations of it.
 *
 * USAGE PATTERN (t = how far past the jump this sample is, 0 = "exactly
 * now", 1 = "a whole sample ago"):
 *   thisSample += jump * thisBlepSample(t);
 *   nextSample += jump * nextBlepSample(t);
 * Because a correction lands on the *next* sample, an oscillator using
 * this outputs one sample late (a "look-ahead" of one sample).
 *
 * The "integrated" versions correct a jump in *slope* (a corner, as in a
 * triangle wave) rather than in value.
 *
 * Derived from stmlib dsp/polyblep.h, Copyright 2017 Emilie Gillet.
 * MIT licence.
 */

#pragma once

namespace bb {

/**
 * Correction for the sample on which a step lands.
 * @param t  Fraction of a sample since the step, 0.0 to 1.0
 * @return   Multiply by the step size and add to this sample
 */
inline float thisBlepSample(float t) { return 0.5f * t * t; }

/**
 * Correction for the sample after a step.
 * @param t  Fraction of a sample since the step, 0.0 to 1.0
 * @return   Multiply by the step size and add to the next sample
 */
inline float nextBlepSample(float t) {
  t = 1.0f - t;
  return -0.5f * t * t;
}

/**
 * Correction for the sample after a corner (a jump in slope).
 * @param t  Fraction of a sample since the corner, 0.0 to 1.0
 * @return   Multiply by the slope change (per sample) and add to the next
 *           sample
 */
inline float nextIntegratedBlepSample(float t) {
  const float t1 = 0.5f * t;
  const float t2 = t1 * t1;
  const float t4 = t2 * t2;
  return 0.1875f - t1 + 1.5f * t2 - t4;
}

/**
 * Correction for the sample on which a corner lands.
 * @param t  Fraction of a sample since the corner, 0.0 to 1.0
 * @return   Multiply by the slope change and add to this sample
 *
 * Example (triangle corner):
 *   float jump = (slopeUp + slopeDown) * frequency;
 *   thisSample -= thisIntegratedBlepSample(t) * jump;
 *   nextSample -= nextIntegratedBlepSample(t) * jump;
 */
inline float thisIntegratedBlepSample(float t) {
  return nextIntegratedBlepSample(1.0f - t);
}

} // namespace bb
