/**
 * @file dsp_common.h
 * @brief Shared constants, types, and utility functions for the BuddhaBox DSP
 * framework.
 *
 * This header is included by every DSP module. It defines the fundamental
 * audio parameters (sample rate, block size) and small helper functions that
 * show up everywhere in audio code.
 *
 * DESIGN NOTE: We keep everything in this file `inline` or `constexpr` so it
 * can be included from multiple translation units without linker errors.
 * This is the standard C++ approach for header-only utilities.
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "buddhabox/core/lut_sine.h"

namespace bb {

// ─── Core Audio Parameters ──────────────────────────────────────────────────
//
// These two constants define the heartbeat of the entire system:
//
// SAMPLE_RATE: How many audio samples we generate per second. 44100 is "CD
// quality" — the standard for consumer audio. Every DSP module uses this to
// convert between time/frequency and sample counts.
//
// AUDIO_BLOCK_SIZE: How many stereo frames we process in one batch. The DMA
// hardware sends audio in chunks — we fill one half of the buffer while the
// hardware plays the other half. 256 frames = ~5.8 ms of latency, which is
// imperceptible for a generative ambient device.

constexpr uint32_t SAMPLE_RATE = 44100;
constexpr uint16_t AUDIO_BLOCK_SIZE = 256;

// ─── Math Constants ─────────────────────────────────────────────────────────
//
// We define float versions of PI to avoid implicit double→float conversions.
// On the STM32F4, the FPU handles 32-bit floats in hardware but doubles in
// software — so using float everywhere is a significant performance win.

constexpr float PI_F = 3.14159265358979f;
constexpr float TWO_PI_F = 6.28318530717959f;

// ─── Utility Functions ──────────────────────────────────────────────────────

/**
 * Clamp a float to a range [lo, hi].
 *
 * Essential in audio to prevent values from exceeding -1.0 to 1.0 (which
 * causes harsh digital clipping when converted to 16-bit integers).
 *
 * @param x   The value to clamp
 * @param lo  Minimum allowed value
 * @param hi  Maximum allowed value
 * @return    x constrained to [lo, hi]
 *
 * Example:
 *   float safe = clampf(oscillator.process(), -1.0f, 1.0f);
 */
inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

/**
 * The smaller of two floats.
 *
 * WHY NOT std::fmin()? The Cortex-M4's FPU has no min/max instruction,
 * and the C library's fminf() must also handle NaN inputs, so GCC emits
 * a function call. A plain comparison compiles to two instructions.
 *
 * @param a  First value
 * @param b  Second value
 * @return   Whichever is smaller
 *
 * Example:
 *   float f = minf(hz / SAMPLE_RATE, 0.25f); // cap at a quarter of fs
 */
inline float minf(float a, float b) { return a < b ? a : b; }

/**
 * The larger of two floats (see minf() for why this exists).
 *
 * @param a  First value
 * @param b  Second value
 * @return   Whichever is larger
 */
inline float maxf(float a, float b) { return a > b ? a : b; }

/**
 * Linear interpolation between two values.
 *
 * This is one of the most common operations in DSP. It smoothly blends
 * between value `a` (when t=0) and value `b` (when t=1). Used for:
 * - Smoothing parameter changes (avoids clicks/pops)
 * - Reading fractional positions in delay buffers
 * - Crossfading between signals
 *
 * @param a  Start value (returned when t = 0.0)
 * @param b  End value (returned when t = 1.0)
 * @param t  Blend factor, typically 0.0 to 1.0
 * @return   The interpolated value: a + (b - a) * t
 *
 * Example:
 *   // Crossfade between dry and wet signal
 *   float mix = lerpf(drySignal, wetSignal, 0.5f);  // 50/50 mix
 */
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

/**
 * Blend between two audio signals — semantic alias for lerpf.
 *
 * Identical to lerpf(a, b, t). The separate name makes patch code
 * read more naturally when crossfading between two signal sources,
 * rather than interpolating between two scalar values.
 *
 * @param a  First signal (returned when t = 0.0)
 * @param b  Second signal (returned when t = 1.0)
 * @param t  Blend amount, 0.0 to 1.0
 * @return   Crossfaded output: a + (b - a) * t
 *
 * Example:
 *   // Mix between two oscillators with a pot or LFO
 *   float out = blend(osc1.process(), osc2.process(), 0.5f);
 */
inline float blend(float a, float b, float t) { return lerpf(a, b, t); }

/**
 * Map a value from one range to another (like Arduino's map() but for floats).
 *
 * Commonly used to convert sensor readings (0.0–1.0) into musically useful
 * ranges like frequency (200–2000 Hz) or amplitude (0.0–0.8).
 *
 * @param x      The input value
 * @param inLo   Lower bound of the input range
 * @param inHi   Upper bound of the input range
 * @param outLo  Lower bound of the output range
 * @param outHi  Upper bound of the output range
 * @return       x rescaled from [inLo, inHi] to [outLo, outHi]
 *
 * Example:
 *   // Convert a 0-1 sensor reading to a filter cutoff frequency
 *   float cutoff = mapf(sensor.read(), 0.0f, 1.0f, 200.0f, 5000.0f);
 */
inline float mapf(float x, float inLo, float inHi, float outLo, float outHi) {
  return outLo + (x - inLo) * (outHi - outLo) / (inHi - inLo);
}

/**
 * Soft clipping — warm saturation that gently rounds off peaks.
 *
 * Unlike hard clipping (clampf), which abruptly chops the waveform at the
 * limit and creates harsh harmonics, soft clipping gradually compresses
 * the signal as it approaches ±1.0, producing a warm, tube-amp-like
 * distortion.
 *
 * Uses a fast tanh approximation: x / (1 + |x|). This has the properties:
 *   - Output approaches ±1.0 asymptotically (never exceeds it)
 *   - Near zero, output ≈ input (transparent at low levels)
 *   - At high levels, the curve flattens (soft compression)
 *
 * The drive parameter controls how much saturation is applied:
 *   - drive = 1.0: transparent, almost no effect
 *   - drive = 2.0: gentle warmth, subtle harmonics
 *   - drive = 5.0: noticeable saturation, like a warm preamp
 *   - drive = 10.0+: heavy distortion, approaching hard clipping
 *
 * @param x      Input sample (any range, will be compressed to ±1.0)
 * @param drive  Saturation amount, 1.0 = clean, higher = more distorted.
 *               Default: 1.0.
 * @return       Soft-clipped sample in approximately (-1.0, +1.0)
 *
 * Example:
 *   // Add gentle warmth to a mix
 *   float out = softclip(osc.process() + noise.process(), 2.0f);
 *
 *   // Heavy distortion on an oscillator
 *   float distorted = softclip(osc.process(), 8.0f);
 */
inline float softclip(float x, float drive = 1.0f) {
  x *= drive;
  return x / (1.0f + fabsf(x));
}

/**
 * Fast sine approximation using a parabolic curve.
 *
 * HOW THIS WORKS:
 * A sine wave looks like an upside-down parabola near its peaks. We exploit
 * this by computing y = (4/π)x - (4/π²)x|x|, then apply a correction pass
 * to improve peak accuracy. Total error is under 0.1% — imperceptible in
 * audio. On the STM32F4 this runs ~4x faster than sinf().
 *
 * IMPORTANT: Input must be in the range [0, 2π). If you pass phase in [0,1),
 * multiply by TWO_PI_F first:
 *
 *   float sample = fastSin(phase * TWO_PI_F);
 *
 * The function normalizes from [0, 2π) to (-π, π] internally. We skip the
 * fmodf() call because callers that use a phase accumulator always stay in
 * [0, 2π) by construction — saving a relatively expensive modulo operation.
 *
 * @param x  Angle in radians, expected in [0, 2π)
 * @return   Approximation of sin(x), range approximately [-1, +1]
 */
inline float fastSin(float x) {
  // Fold from [0, 2π) into (-π, π] — the range where the parabolic
  // approximation is valid. Values above π get shifted down by 2π.
  if (x > PI_F)
    x -= TWO_PI_F;

  // First pass: parabolic approximation of sin(x)
  // y = (4/π)x - (4/π²)x|x| maps the sine shape onto a parabola
  const float B = 4.0f / PI_F;
  const float C = -4.0f / (PI_F * PI_F);
  float y = B * x + C * x * fabsf(x);

  // Second pass: correction factor improves accuracy from ~88% to ~99.7%.
  // The constant 0.225 was determined empirically to minimize peak error.
  y = 0.225f * (y * fabsf(y) - y) + y;
  return y;
}

// ─── Table Interpolation ────────────────────────────────────────────────────
//
// Many modules read curves from lookup tables (see lut_sine.h and luts.h).
// A table holds `size` steps plus at least one guard entry at the end, so
// reading at index 1.0 never runs off the end. These helpers find the two
// (or four) entries around a fractional position and blend them.

/**
 * Read a lookup table at a fractional position (linear interpolation).
 *
 * HOW IT WORKS: `index` (0..1) is scaled to a table position such as
 * 137.4. The whole part (137) picks an entry, the fractional part (0.4)
 * says how far to blend towards the next entry.
 *
 * @param table  Pointer to `size + 1` (or more) floats
 * @param index  Position, 0.0 to 1.0 (1.0 reads the guard entry)
 * @param size   Number of table steps, e.g. 256.0f for a 257-entry table
 * @return       The interpolated table value
 *
 * Example:
 *   // A four-decade curve: 1 at x = 0, 10000 at x = 1
 *   float q = interpolate(lut4Decades(), knob, 256.0f);
 */
inline float interpolate(const float *table, float index, float size) {
  index *= size;
  int32_t i = static_cast<int32_t>(index);    // whole part picks the entry...
  float frac = index - static_cast<float>(i); // ...fraction blends
  float a = table[i];
  float b = table[i + 1];
  return a + (b - a) * frac;
}

/**
 * As interpolate(), but only the fractional part of `index` counts, so
 * 1.25 reads the same place as 0.25. Use it for periodic tables (one
 * cycle of a waveform) driven by a phase that may exceed 1.
 *
 * @param table  Pointer to a periodic table with a guard entry
 * @param index  Any non-negative position; only its fraction is used
 * @param size   Number of table steps per cycle
 * @return       The interpolated table value
 *
 * Example:
 *   float s = interpolateWrap(lutSine(), phase + 0.3f, kSineTableSize);
 */
inline float interpolateWrap(const float *table, float index, float size) {
  index -= static_cast<float>(static_cast<int32_t>(index));
  return interpolate(table, index, size);
}

/**
 * 4-point Hermite (cubic) interpolation of a table.
 *
 * Smoother than linear: it fits a curve through two neighbours on each
 * side, so it adds fewer high-frequency "corners". The table needs one
 * guard entry before the read position and two after.
 *
 * @param table  Pointer to the entry for index 0 (entry -1 must exist)
 * @param index  Position, 0.0 to 1.0
 * @param size   Number of table steps
 * @return       The interpolated table value
 *
 * Example:
 *   // Wavefolder tables start one entry early, hence the + 1
 *   float y = interpolateHermite(lutFoldSine() + 1, x, 512.0f);
 */
inline float interpolateHermite(const float *table, float index, float size) {
  index *= size;
  int32_t i = static_cast<int32_t>(index);
  float f = index - static_cast<float>(i);
  const float xm1 = table[i - 1];
  const float x0 = table[i];
  const float x1 = table[i + 1];
  const float x2 = table[i + 2];
  // Polynomial coefficients arranged for Horner's method:
  // ((a*f - b)*f + c)*f + x0 costs only three multiplies.
  const float c = (x1 - xm1) * 0.5f;
  const float v = x0 - x1;
  const float w = c + v;
  const float a = w + v + (x2 - x0) * 0.5f;
  const float bNeg = w + a;
  return (((a * f) - bNeg) * f + c) * f + x0;
}

/**
 * Sine of a phase in cycles (0..1 = one full turn), read from the flash
 * sine table. Cheaper than sinf() and accurate to about -100 dB.
 *
 * @param phase  Phase in cycles; any non-negative value (only the
 *               fractional part counts)
 * @return       sin(2*pi*phase), -1.0 to +1.0
 *
 * Example:
 *   float lfo = sineFromPhase(lfoPhase);          // sine
 *   float lfo90 = sineFromPhase(lfoPhase + 0.25f); // cosine
 */
inline float sineFromPhase(float phase) {
  return interpolateWrap(lutSine(), phase, kSineTableSize);
}

// ─── Saturation and Smoothing Helpers ───────────────────────────────────────

/**
 * Equal-power crossfade: the gains follow a quarter sine and cosine, so
 * their squares always sum to 1 and the loudness stays constant. Use it
 * for uncorrelated signals such as dry and wet (a linear crossfade dips
 * by about 3 dB in the middle).
 *
 * @param a     Signal heard at fade = 0
 * @param b     Signal heard at fade = 1
 * @param fade  Blend, 0.0 to 1.0
 * @return      The blended signal
 *
 * Example:
 *   float out = crossfadeEqualPower(dry, wet, mix);
 */
inline float crossfadeEqualPower(float a, float b, float fade) {
  const float t = fade * 0.25f; // a quarter cycle at fade = 1
  return a * sineFromPhase(t + 0.25f) + b * sineFromPhase(t);
}

/**
 * S-shaped ease curve on 0..1 with zero slope at both ends.
 *
 * @param x  Input, 0.0 to 1.0
 * @return   3x^2 - 2x^3, 0.0 to 1.0
 */
inline float smoothStep(float x) { return x * x * (3.0f - 2.0f * x); }

/**
 * Gentle saturation: a rational approximation of tanh(x). Linear near
 * zero and exactly ±1 (with zero slope) at x = ±3. Only valid on ±3: use
 * softLimitClip() when the input might go further.
 *
 * Compare softclip(): x / (1 + |x|) never quite reaches 1, so it squashes
 * more gently; softLimit() keeps more level, then flattens out hard.
 *
 * @param x  Input, -3.0 to +3.0
 * @return   Approximately tanh(x), -1.0 to +1.0
 *
 * Example:
 *   float out = softLimit(mix * 1.5f);
 */
inline float softLimit(float x) {
  return x * (27.0f + x * x) / (27.0f + 9.0f * x * x);
}

/**
 * softLimit() for any input: clamps to ±1 outside ±3.
 *
 * @param x  Input, any range
 * @return   Approximately tanh(x), -1.0 to +1.0
 */
inline float softLimitClip(float x) {
  if (x < -3.0f)
    return -1.0f;
  if (x > 3.0f)
    return 1.0f;
  return softLimit(x);
}

/**
 * One-pole low-pass step: move `state` a fraction `coefficient` of the
 * way towards `target`. With coefficient c applied every sample, the time
 * constant is about 1 / (c * SAMPLE_RATE) seconds.
 *
 * @param state        The filter state, updated in place
 * @param target       The value to move towards
 * @param coefficient  Speed, 0.0 (frozen) to 1.0 (instant)
 *
 * Example:
 *   onePole(smoothedCutoff, targetCutoff, 0.001f); // ~23 ms glide
 */
inline void onePole(float &state, float target, float coefficient) {
  state += coefficient * (target - state);
}

/**
 * One-pole step with separate rise and fall speeds, e.g. an envelope
 * follower with a fast attack and a slow release.
 *
 * @param state   The state, updated in place
 * @param target  The value to move towards
 * @param rise    Coefficient (0..1) used when target > state
 * @param fall    Coefficient (0..1) used when target < state
 *
 * Example:
 *   slope(level, fabsf(in), 0.05f, 0.0001f);
 */
inline void slope(float &state, float target, float rise, float fall) {
  float error = target - state;
  state += (error > 0.0f ? rise : fall) * error;
}

/**
 * Move `state` towards `target` by at most `maxDelta` per call: a
 * fixed-rate slew limiter.
 *
 * @param state     The state, updated in place
 * @param target    The value to move towards
 * @param maxDelta  Largest change allowed per call (> 0)
 */
inline void slew(float &state, float target, float maxDelta) {
  state += clampf(target - state, -maxDelta, maxDelta);
}

/**
 * Convert a one-pole coefficient tuned for one sample rate to another.
 *
 * Gillet's code applies fixed coefficients once per sample at her
 * firmware's rate (often 48 kHz). At 44.1 kHz the same coefficient would
 * make a filter or envelope about 9% slower. After n samples a one-pole
 * has covered 1 - (1 - c)^n of the distance to its target, so matching
 * the time in seconds means raising (1 - c) to the power
 * ratio = originalRate / newRate.
 *
 * Uses powf(): call it from setup or parameter changes, not per sample.
 *
 * @param c      The original coefficient, 0.0 to 1.0
 * @param ratio  originalRate / newRate, e.g. 48000.0f / SAMPLE_RATE
 * @return       The equivalent coefficient at the new rate
 *
 * Example:
 *   attack_ = rescaleCoefficient(0.05f, 48000.0f / SAMPLE_RATE);
 */
inline float rescaleCoefficient(float c, float ratio) {
  if (ratio == 1.0f)
    return c; // exact at the original rate
  return 1.0f - powf(1.0f - c, ratio);
}

// ─── Fast Exponentials and Logarithms ───────────────────────────────────────
//
// exp2f() and log2f() from the C library cost a few hundred cycles on the
// Cortex-M4. These versions take about 20 and are accurate to roughly one
// part in ten million (0.0002 cents of pitch), plenty for audio.

/**
 * Fast 2^x.
 *
 * HOW IT WORKS:
 * 1. Split x into a whole number n and a fraction f in [-0.5, 0.5].
 * 2. 2^n is built directly as a float's bit pattern: a float stores
 *    sign | exponent | mantissa, and the exponent field is n + 127.
 * 3. 2^f is a short polynomial (the Taylor series of e^(f ln 2)); with
 *    |f| <= 0.5, seven terms keep the error near 1e-7.
 * 4. 2^x = 2^n * 2^f.
 *
 * @param x  Exponent, clamped to -126..126
 * @return   2^x
 *
 * Example:
 *   float ratio = fastExp2(semitones / 12.0f); // interval to ratio
 */
inline float fastExp2(float x) {
  x = clampf(x, -126.0f, 126.0f);
  // Step 1: round to nearest integer, leaving a fraction in [-0.5, 0.5].
  int32_t n = static_cast<int32_t>(x + (x >= 0.0f ? 0.5f : -0.5f));
  float f = x - static_cast<float>(n);
  // Step 2: 2^n from the exponent bits. A union reinterprets the same
  // 32 bits as an integer or a float ("type punning"); GCC supports it.
  union {
    int32_t i;
    float f;
  } pow2;
  pow2.i = (n + 127) << 23;
  // Step 3: 2^f = e^(f ln2) by Horner's method (coefficients ln2^k / k!).
  float p =
      1.0f + f * (0.693147181f +
                  f * (0.240226507f +
                       f * (0.0555041087f +
                            f * (0.00961812911f +
                                 f * (0.00133335581f + f * 0.000154035304f)))));
  return pow2.f * p;
}

/**
 * Fast log2(x) for x > 0.
 *
 * HOW IT WORKS:
 * 1. A float is m * 2^e. Read e straight from the exponent bits, and
 *    rebuild m (1 <= m < 2) by forcing the exponent to zero.
 * 2. Move m into [0.707, 1.414) so the series below converges fast.
 * 3. log(m) = 2 atanh(s) with s = (m - 1) / (m + 1); |s| <= 0.172, so
 *    four terms of the atanh series are enough.
 * 4. log2(x) = e + log(m) / ln 2.
 *
 * @param x  A positive number (0 or negatives give nonsense)
 * @return   log2(x)
 *
 * Example:
 *   float db = 6.0206f * fastLog2(amplitude); // 20 log10(a)
 */
inline float fastLog2(float x) {
  union {
    float f;
    int32_t i;
  } bits;
  bits.f = x;
  // Step 1: exponent and mantissa.
  int32_t e = ((bits.i >> 23) & 0xff) - 127;
  bits.i = (bits.i & 0x007fffff) | 0x3f800000; // force exponent to 2^0
  float m = bits.f;
  // Step 2: centre the mantissa around 1.
  if (m > 1.41421356f) {
    m *= 0.5f;
    e += 1;
  }
  // Step 3: atanh series (s + s^3/3 + s^5/5 + s^7/7).
  float s = (m - 1.0f) / (m + 1.0f);
  float s2 = s * s;
  float lnM =
      2.0f * s * (1.0f + s2 * (0.333333333f + s2 * (0.2f + s2 * 0.142857143f)));
  // Step 4: 1 / ln 2 = 1.44269504.
  return static_cast<float>(e) + lnM * 1.44269504f;
}

/**
 * Frequency ratio for a musical interval in semitones:
 * +12 = 2.0 (an octave up), -12 = 0.5, +7 = 1.498 (a fifth).
 *
 * @param semitones  Interval, any value within about ±1500
 * @return           2^(semitones / 12)
 *
 * Example:
 *   osc.setFrequency(baseHz * semitonesToRatio(7.0f)); // up a fifth
 */
inline float semitonesToRatio(float semitones) {
  return fastExp2(semitones * (1.0f / 12.0f));
}

} // namespace bb
