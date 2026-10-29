/**
 * @file hihat.h
 * @brief TR-808 hi-hat, plus a ring-modulated variant, from Plaits.
 *
 * HOW IT WORKS:
 * 1. Metallic noise, from one of two sources:
 *    - SQUARE_808: six square waves at unrelated ratios, summed, as on the
 *      808 (and its cymbal). The clash of inharmonic partials sounds
 *      metallic.
 *    - RING_MOD: three pairs of oscillators ring-modulated together, for a
 *      harsher, more electronic sound.
 * 2. A band-pass filter colours the noise (`tone`). With some
 *    `noisiness`, it is blended with clocked random noise, for washier,
 *    cymbal-like sounds.
 * 3. A VCA with a decaying envelope. SQUARE_808 uses the 808's lopsided
 *    "swing" VCA; RING_MOD uses a clean VCA and a two-stage decay.
 * 4. A high-pass filter removes the low end.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/osc/basic.h"

namespace bb {

/**
 * 808-style hi-hat with an alternative ring-mod model.
 *
 * Example:
 *   HiHat hat;
 *   hat.setDecay(0.2f); // closed
 *
 *   if (sixteenth) {
 *     hat.trigger();
 *   }
 *   float s = hat.process() * 0.5f;
 */
class HiHat {
public:
  /** Metallic noise source. */
  enum Model { SQUARE_808, RING_MOD };

  HiHat() { init(); }

  /**
   * Silence the hat and choose a model.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   * @param model       SQUARE_808 (default) or RING_MOD
   */
  void init(float sampleRate = SAMPLE_RATE, Model model = SQUARE_808) {
    sampleRate_ = sampleRate;
    rateRatio_ = 48000.0f / sampleRate;
    model_ = model;
    for (int i = 0; i < 6; ++i) {
      squarePhase_[i] = 0;
      ringOsc_[i].init(sampleRate);
      ringOsc_[i].setShape(i % 2 == 0 ? BasicOscillator::SQUARE
                                      : BasicOscillator::SAW);
    }
    coloration_.init(sampleRate);
    hpf_.init(sampleRate);
    envelope_ = noiseClock_ = noiseSample_ = 0.0f;
    update();
  }

  /** @param hz  Base pitch of the metal; around 400 Hz is the 808 */
  void setFrequency(float hz) {
    hz_ = hz;
    update();
  }

  /** @param x  0..1: colour, from dark to sizzling */
  void setTone(float x) {
    tone_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: closed (short) to open (long) */
  void setDecay(float x) {
    decay_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: blend of metallic noise and plain clocked noise */
  void setNoisiness(float x) {
    noisiness_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: how hard the next hit is (and sustain level) */
  void setAccent(float x) { accent_ = clampf(x, 0.0f, 1.0f); }

  /** @param on  Continuous sound instead of triggered hits */
  void setSustain(bool on) { sustain_ = on; }

  /** @param s  Random seed for the clocked noise */
  void seed(uint32_t s) { random_.seed(s); }

  /** Strike the hat. */
  void trigger() {
    envelope_ = (1.5f + 0.5f * (1.0f - decay_)) * (0.3f + 0.7f * accent_);
  }

  /**
   * Generate the next sample.
   * @return  The hat, about ±1
   */
  float process() {
    // 1. Metallic noise.
    float s = model_ == SQUARE_808 ? squareNoise() : ringModNoise();
    // 2. Colour, then blend in clocked noise.
    s = coloration_.process(s).bp;
    noiseClock_ += noiseF_;
    if (noiseClock_ >= 1.0f) {
      noiseClock_ -= 1.0f;
      noiseSample_ = random_.uniform() - 0.5f;
    }
    s += noisiness2_ * (noiseSample_ - s);
    // 3. VCA. The ring-mod model's envelope switches to a faster "cut"
    // decay once it falls below 0.5.
    bool twoStage = model_ == RING_MOD;
    envelope_ *= envelope_ > 0.5f || !twoStage ? envelopeDecay_ : cutDecay_;
    float gain = sustain_ ? accent_ * decay_ : envelope_;
    if (model_ == SQUARE_808) {
      // The 808's VCA: lopsided and saturating.
      s *= s > 0.0f ? 4.0f : 0.1f;
      s = s / (1.0f + fabsf(s));
      s = (s + 0.1f) * gain;
    } else {
      s *= gain;
    }
    // 4. High-pass.
    return hpf_.process(s).hp;
  }

private:
  /**
   * Six square waves at the 808's inharmonic ratios, summed. Phases are
   * 32-bit integers: overflow wraps them for free, and the top bit is
   * the square wave.
   */
  float squareNoise() {
    uint32_t sum = 0;
    for (int i = 0; i < 6; ++i) {
      squarePhase_[i] += squareIncrement_[i];
      sum += squarePhase_[i] >> 31;
    }
    return 0.33f * static_cast<float>(sum) - 1.0f;
  }

  /** Three square x saw pairs, each ring-modulated (multiplied). */
  float ringModNoise() {
    float out = 0.0f;
    for (int i = 0; i < 6; i += 2) {
      out += ringOsc_[i].process(ringF_[i], 0.5f) *
             ringOsc_[i + 1].process(ringF_[i + 1], 0.5f);
    }
    return out;
  }

  void update() {
    const float f0 = hz_ / sampleRate_;
    const float f048 = hz_ / 48000.0f;
    // 1. Metallic oscillators (the 808 ratios). Plaits feeds them twice
    // the note frequency.
    static const float kRatios[6] = {1.0f,   1.304f, 1.466f,
                                     1.787f, 1.932f, 2.536f};
    for (int i = 0; i < 6; ++i) {
      float f = minf(2.0f * f0 * kRatios[i], 0.499f);
      squareIncrement_[i] = static_cast<uint32_t>(f * 4294967296.0f);
    }
    // Ring-mod pairs: fixed frequencies in Hz, scaled by pitch.
    const float ratio = 2.0f * f048 / (0.01f + 2.0f * f048);
    static const float kRingHz[6] = {200.0f,  7530.0f, 510.0f,
                                     8075.0f, 730.0f,  10500.0f};
    for (int i = 0; i < 6; ++i)
      ringF_[i] = kRingHz[i] / sampleRate_ * ratio;
    // 2. Filters and clocked noise.
    float cutoff =
        clampf(150.0f * semitonesToRatio(tone_ * 72.0f), 0.0f, 16000.0f) /
        sampleRate_;
    coloration_.setCoefficients(
        cutoff, model_ == SQUARE_808 ? 3.0f + 3.0f * tone_ : 1.0f,
        TanApprox::Accurate);
    hpf_.setCoefficients(cutoff, 0.5f, TanApprox::Accurate);
    noisiness2_ = noisiness_ * noisiness_;
    noiseF_ = clampf(f0 * (16.0f + 16.0f * (1.0f - noisiness2_)), 0.0f, 0.5f);
    // 3. Envelope rates.
    envelopeDecay_ =
        1.0f - rescaleCoefficient(0.003f * semitonesToRatio(-decay_ * 84.0f),
                                  rateRatio_);
    cutDecay_ =
        1.0f - rescaleCoefficient(0.0025f * semitonesToRatio(-decay_ * 36.0f),
                                  rateRatio_);
  }

  float sampleRate_ = SAMPLE_RATE, rateRatio_ = 1.0f;
  Model model_ = SQUARE_808;
  float hz_ = 400.0f, tone_ = 0.5f, decay_ = 0.5f, noisiness_ = 0.0f;
  float accent_ = 0.8f;
  bool sustain_ = false;
  uint32_t squarePhase_[6] = {0, 0, 0, 0, 0, 0};
  uint32_t squareIncrement_[6] = {0, 0, 0, 0, 0, 0};
  float ringF_[6] = {0, 0, 0, 0, 0, 0};
  float noisiness2_ = 0.0f, noiseF_ = 0.0f;
  float envelopeDecay_ = 0.99f, cutDecay_ = 0.99f;
  float envelope_ = 0.0f, noiseClock_ = 0.0f, noiseSample_ = 0.0f;
  Random random_;
  BasicOscillator ringOsc_[6];
  Svf coloration_, hpf_;
};

} // namespace bb
