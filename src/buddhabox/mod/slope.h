/**
 * @file slope.h
 * @brief Tides 2 function generator: envelope, LFO or oscillator.
 *
 * The single-channel core of Tides (2018): an envelope, a looping LFO or
 * an audio oscillator with a skewable, shapeable waveform, plus
 * end-of-attack and end-of-release gates.
 *
 * HOW IT WORKS:
 * 1. A ramp (phase 0..1) runs at the set frequency. In AD mode it runs
 *    once per trigger; in LOOPING mode it cycles; in AR mode it climbs
 *    while the gate is high and falls when it is released.
 * 2. `slope` skews the ramp into a rise and a fall of different lengths:
 *    0 = instant rise, slow fall; 0.5 = symmetric; 1 = slow rise,
 *    instant fall.
 * 3. `shape` bends the rise and fall through a family of curves:
 *    logarithmic, sine, linear, exponential... (control rates), or
 *    towards rounder or spikier waveforms (audio rates).
 * 4. `smoothness` below 0.5 low-passes the output (rounding corners);
 *    above 0.5 it wavefolds it (adding wiggles and harmonics).
 * 5. At audio rates the looping waveform is anti-aliased with PolyBLEP.
 *
 * MEMORY: Tides' 14 curve tables (57 KB) live in flash (lut_slope.h).
 * Partials built them in RAM, which the Black Pill could not afford.
 *
 * Not included: Tides' four-output modes. Run several Slopes to build
 * them.
 *
 * Derived from Tides 2, Copyright 2017 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/mod/lut_slope.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * Tides-style function generator.
 *
 * Example — a breathing LFO, slow rise and quick fall:
 *   Slope breath;
 *   breath.setMode(Slope::LOOPING);
 *   breath.setFrequency(0.1f); // one breath every 10 s
 *   breath.setSlope(0.8f);
 *   breath.setShape(0.3f);
 *
 *   float lfo = breath.process(false); // -1..1
 *
 * Example — an AD envelope with an end-of-attack gate:
 *   Slope env;
 *   env.setMode(Slope::AD);
 *   env.setFrequency(2.0f); // attack + decay = 0.5 s
 *   float level = env.process(trigger);
 *   bool peak = env.endOfAttack();
 */
class Slope {
public:
  /** Ramp behaviour. */
  enum Mode { AD, LOOPING, AR };

  /** CONTROL: envelopes and LFOs (sharp corners kept). AUDIO: an
   *  anti-aliased oscillator (looping mode only). */
  enum Range { CONTROL, AUDIO };

  Slope() { init(); }

  /**
   * Reset the ramp and filters.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    frequency_ = 0.0f;
    nextSample_ = 0.0f;
    lp1_ = lp2_ = 0.0f;
    previousGate_ = false;
    previousInput_ = previousOutput_ = breakpoint_ = 0.0f;
    eoa_ = eor_ = false;
  }

  /** @param m  AD, LOOPING or AR */
  void setMode(Mode m) { mode_ = m; }

  /** @param r  CONTROL or AUDIO */
  void setRange(Range r) { range_ = r; }

  /** @param hz  Cycle frequency; for AD, attack + decay lasts 1/hz s */
  void setFrequency(float hz) { hz_ = hz; }

  /** @param x  0..1: rise/fall balance (Tides' SLOPE knob) */
  void setSlope(float x) { slope_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: curve family (Tides' SHAPE knob) */
  void setShape(float x) { shape_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: below 0.5 smoother, above 0.5 folded */
  void setSmoothness(float x) { smoothness_ = clampf(x, 0.0f, 1.0f); }

  /**
   * Advance one sample.
   * @param gate  Triggers AD mode, holds AR mode, resets LOOPING mode on
   *              a rising edge
   * @return      0.0 to 1.0 in AD and AR modes, -1.0 to +1.0 in LOOPING
   */
  float process(bool gate) {
    const bool rising = gate && !previousGate_;
    previousGate_ = gate;
    const bool audio = range_ == AUDIO && mode_ == LOOPING;
    float f0 = minf(hz_ * sampleTime_, 0.25f);

    // 1. Parameters, conditioned as in Tides.
    float pw = slope_;
    if (!audio && pw < 0.5f) {
      // Skew the response so the useful range of short attacks is wider.
      pw = 0.5f + 0.6f * (pw - 0.5f) / (fabsf(pw - 0.5f) + 0.1f);
    }
    // At high frequencies, shape and fold are reduced to limit aliasing.
    const float harmonics = 3.0f + fabsf(pw - 0.5f) * 5.0f;
    const float shapeAmount = fabsf(shape_ - 0.5f) * 2.0f;
    const float shapeAttenuation = tame(f0, harmonics, 16.0f);
    const float shape = 0.5f + (shape_ - 0.5f) * shapeAttenuation;
    float smoothness = smoothness_;
    if (smoothness > 0.5f) {
      smoothness =
          0.5f +
          (smoothness - 0.5f) *
              tame(f0,
                   harmonics * (3.0f + shapeAmount * shapeAttenuation * 5.0f),
                   12.0f);
    }
    const float fold = maxf(2.0f * (smoothness - 0.5f), 0.0f);

    // 2. Advance the ramp.
    stepRamp(f0, pw, gate, rising);

    // 3. Skewed (and in audio range, band-limited) slope, 0..1.
    float raw;
    if (mode_ == AR) {
      raw = phase_;
    } else if (audio) {
      raw = bandLimitedSlope(phase_, frequency_, pw);
    } else {
      raw = skewedRamp(phase_, frequency_, pw);
    }

    // 4. Curve shape. Control-rate tables cover rise-then-fall;
    // audio-rate tables are monotonic shapers.
    float s = audio ? shape * 3.9999f : shape * 5.9999f + 5.0f;
    int si = static_cast<int>(s);
    float shaped = waveshape(raw, si, s - static_cast<float>(si));

    // 5. Fold, then the end-of-attack/release gates.
    float out = foldOutput(shaped, fold);
    if (mode_ == AR) {
      eoa_ = phase_ >= 0.5f;
      eor_ = phase_ >= 1.0f;
    } else if (mode_ == LOOPING) {
      eoa_ = phase_ >= pw;
      eor_ = phase_ < minf(0.5f, 96.0f * frequency_);
    } else {
      eoa_ = phase_ >= pw;
      eor_ = phase_ >= 1.0f;
    }

    // 6. Smoothing below 0.5: a two-pole low-pass that follows the
    // frequency, so the rounding is the same at every rate.
    if (smoothness < 0.5f) {
      float ratio = smoothness * 2.0f;
      ratio *= ratio;
      ratio *= ratio;
      float f = frequency_ * 0.5f;
      f += (1.0f - f) * ratio;
      onePole(lp1_, out, f);
      onePole(lp2_, lp1_, f);
      out = lp2_;
    }
    return out;
  }

  /** @return  true once the attack is over (for chaining) */
  bool endOfAttack() const { return eoa_; }

  /** @return  true once the cycle has ended (for chaining) */
  bool endOfRelease() const { return eor_; }

private:
  void stepRamp(float f0, float pw, bool gate, bool rising) {
    frequency_ = f0;
    if (mode_ == AD) {
      if (rising)
        phase_ = 0.0f;
      phase_ = minf(phase_ + frequency_, 1.0f);
    } else if (mode_ == AR) {
      // Rise to 0.5 while held; fall from 0.5 to 1 when released, each at
      // its own speed.
      if (phase_ < 0.5f && !gate) {
        phase_ = 0.5f;
      } else if (phase_ > 0.5f && gate) {
        phase_ = 0.0f;
      }
      float slope =
          phase_ < 0.5f ? 0.5f / (1.0e-6f + pw) : 0.5f / (1.0f + 1.0e-6f - pw);
      phase_ = minf(phase_ + frequency_ * slope, gate ? 0.5f : 1.0f);
    } else {
      if (rising) {
        phase_ = 0.0f;
      } else {
        phase_ += frequency_;
        if (phase_ >= 1.0f)
          phase_ -= 1.0f;
      }
    }
  }

  /** Rise over [0, pw), fall over [pw, 1); output 0..0.5 then 0.5..1. */
  static float skewedRamp(float phase, float frequency, float pw) {
    pw = clampf(pw, fabsf(frequency) * 2.0f, 1.0f - 2.0f * fabsf(frequency));
    return phase < pw ? phase * (0.5f / pw)
                      : (phase - pw) * (0.5f / (1.0f - pw)) + 0.5f;
  }

  /** Triangle-like slope with PolyBLEP-corrected corners (audio range). */
  float bandLimitedSlope(float phase, float frequency, float pw) {
    pw = clampf(pw, fabsf(frequency) * 2.0f, 1.0f - 2.0f * fabsf(frequency));
    float thisSample = nextSample_;
    float nextSample = 0.0f;
    float wrapPoint = pw;
    if (phase < pw * 0.5f) {
      wrapPoint = 0.0f;
    } else if (phase > 0.5f + pw * 0.5f) {
      wrapPoint = 1.0f;
    }
    const float slopeUp = 1.0f / pw;
    const float slopeDown = 1.0f / (1.0f - pw);
    const float d = phase - wrapPoint;
    if (d >= 0.0f && d < frequency) {
      const float t = d / frequency;
      float discontinuity = -(slopeUp + slopeDown) * frequency;
      if (wrapPoint != pw)
        discontinuity = -discontinuity;
      thisSample += thisIntegratedBlepSample(t) * discontinuity;
      nextSample += nextIntegratedBlepSample(t) * discontinuity;
    }
    nextSample +=
        phase < pw ? phase * slopeUp : 1.0f - (phase - pw) * slopeDown;
    nextSample_ = nextSample;
    return thisSample;
  }

  /**
   * Blend of two neighbouring shape tables; in AR mode, rescaled around
   * the level where the gate changed, so release starts where attack
   * stopped.
   */
  float waveshape(float input, int table, float blend) {
    const float (*shapes)[1025] = lutSlopeShape();
    float index = 1024.0f * input;
    int i = static_cast<int>(index);
    float frac = index - static_cast<float>(i);
    i &= 1023;
    const float *a = shapes[table];
    const float *b = shapes[table + 1 > 11 ? 11 : table + 1];
    float x = a[i] + (a[i + 1] - a[i]) * frac;
    float y = b[i] + (b[i + 1] - b[i]) * frac;
    float output = x + (y - x) * blend;
    if (mode_ != AR)
      return output;
    if (previousInput_ <= 0.5f && input > 0.5f) {
      breakpoint_ = previousOutput_;
    } else if (previousInput_ > 0.5f && input < 0.5f) {
      breakpoint_ = previousOutput_;
    } else if (input == 1.0f) {
      breakpoint_ = 1.0f;
    } else if (input == 0.5f) {
      breakpoint_ = 0.0f;
    }
    output = input <= 0.5f ? breakpoint_ + (1.0f - breakpoint_) * output
                           : breakpoint_ * output;
    previousInput_ = input;
    previousOutput_ = output;
    return output;
  }

  /** Wavefolding: bipolar for looping, unipolar for envelopes. */
  float foldOutput(float unipolar, float amount) {
    if (mode_ == LOOPING) {
      float bipolar = 2.0f * unipolar - 1.0f;
      if (amount <= 0.0f)
        return bipolar;
      float folded =
          interpolate(lutSlopeBipolarFold(),
                      0.5f + bipolar * (0.03f + 0.46f * amount), 1024.0f);
      return bipolar + (folded - bipolar) * amount;
    }
    if (amount <= 0.0f)
      return unipolar;
    float folded =
        interpolate(lutSlopeUnipolarFold(), unipolar * amount, 1024.0f);
    return unipolar + (folded - unipolar) * amount;
  }

  /**
   * 1 at low frequencies, falling to 0 as f0 * harmonics nears Nyquist:
   * scales down settings that would alias.
   */
  static float tame(float f0, float harmonics, float order) {
    f0 *= harmonics;
    float maxF = 0.5f / order;
    float amount = clampf(1.0f - (f0 - maxF) / (0.5f - maxF), 0.0f, 1.0f);
    return amount * amount * amount;
  }

  float sampleTime_ = 1.0f / SAMPLE_RATE;
  Mode mode_ = LOOPING;
  Range range_ = CONTROL;
  float hz_ = 1.0f, slope_ = 0.5f, shape_ = 0.5f, smoothness_ = 0.5f;
  float phase_ = 0.0f, frequency_ = 0.0f, nextSample_ = 0.0f;
  float lp1_ = 0.0f, lp2_ = 0.0f;
  float previousInput_ = 0.0f, previousOutput_ = 0.0f, breakpoint_ = 0.0f;
  bool previousGate_ = false, eoa_ = false, eor_ = false;
};

} // namespace bb
