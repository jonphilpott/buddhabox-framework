/**
 * @file synth_kick.h
 * @brief TR-909-style digital kick, from Plaits.
 *
 * HOW IT WORKS:
 * - The body is an oscillator, not a filter. Its waveform morphs from a
 *   clean sine to a soft-clipped triangle with jittery phase
 *   (`dirtiness`).
 * - A fast pitch envelope (`fmAmount`, `fmDecay`) sweeps the pitch down
 *   at the start: the classic punchy "doom".
 * - The body goes through a model of a transistor VCA (asymmetric and
 *   saturating), driven by an envelope that holds for 1 ms then decays.
 * - A click and a burst of filtered noise make the transient (`tone` sets
 *   its level), and a low-pass filter (also `tone`) sets the brightness.
 * - Sustain mode plays the body continuously.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/filter/svf.h"

namespace bb {

/**
 * 909-style synthetic kick.
 *
 * Example:
 *   SynthKick kick;
 *   kick.setFrequency(45.0f);
 *   kick.setFmAmount(0.7f);
 *
 *   if (clock.process()) {
 *     kick.trigger();
 *   }
 *   float s = kick.process();
 */
class SynthKick {
public:
  SynthKick() { init(); }

  /**
   * Silence the drum.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    rateRatio_ = 48000.0f / sampleRate;
    // Fixed per-sample coefficients from Plaits, converted to this rate.
    phaseNoiseCoefficient_ = rescaleCoefficient(0.002f, rateRatio_);
    envelopeLpCoefficient_ = rescaleCoefficient(0.1f, rateRatio_);
    clickRise_ = rescaleCoefficient(0.5f, rateRatio_);
    clickFall_ = rescaleCoefficient(0.1f, rateRatio_);
    clickHp_ = rescaleCoefficient(0.04f, rateRatio_);
    noiseLp_ = rescaleCoefficient(0.05f, rateRatio_);
    noiseHp_ = rescaleCoefficient(0.005f, rateRatio_);
    clickFilter_.init(sampleRate);
    clickFilter_.setCoefficients(5000.0f / sampleRate, 2.0f, TanApprox::Fast);
    phase_ = phaseNoise_ = 0.0f;
    fm_ = fmLp_ = 0.0f;
    bodyEnv_ = bodyEnvLp_ = transientEnv_ = transientEnvLp_ = 0.0f;
    bodyEnvPulseWidth_ = fmPulseWidth_ = 0;
    toneLp_ = 0.0f;
    clickLpState_ = clickHpState_ = noiseLpState_ = noiseHpState_ = 0.0f;
    update();
  }

  /** @param hz  Pitch in Hz */
  void setFrequency(float hz) {
    hz_ = hz;
    update();
  }

  /** @param x  0..1: brightness, and the click/noise transient level */
  void setTone(float x) {
    tone_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: body decay */
  void setDecay(float x) {
    decay_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: clean sine to jittery soft-clipped triangle */
  void setDirtiness(float x) {
    dirtinessKnob_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: depth of the initial pitch drop */
  void setFmAmount(float x) { fmAmount_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: length of the pitch drop */
  void setFmDecay(float x) {
    fmDecay_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: how hard the next hit is (and sustain level) */
  void setAccent(float x) { accent_ = clampf(x, 0.0f, 1.0f); }

  /** @param on  Continuous body instead of triggered hits */
  void setSustain(bool on) { sustain_ = on; }

  /** @param s  Random seed for the noise */
  void seed(uint32_t s) { random_.seed(s); }

  /** Strike the drum. */
  void trigger() {
    fm_ = 1.0f;
    bodyEnv_ = transientEnv_ = 0.3f + 0.7f * accent_;
    bodyEnvPulseWidth_ = static_cast<int>(sampleRate_ * 0.001f);
    fmPulseWidth_ = static_cast<int>(sampleRate_ * 0.0013f);
  }

  /**
   * Generate the next sample.
   * @return  The drum, about ±1
   */
  float process() {
    // 1. Slowly wandering phase noise, for the "dirty" waveform.
    onePole(phaseNoise_, random_.uniform() - 0.5f, phaseNoiseCoefficient_);
    float mix = 0.0f;
    if (sustain_) {
      phase_ += f0_;
      if (phase_ >= 1.0f)
        phase_ -= 1.0f;
      float body = distortedSine(phase_, phaseNoise_, dirtiness_);
      mix -= transistorVca(body, accent_ * decaySquared_);
    } else {
      // 2. Pitch: held at a fixed phase for 1.3 ms (a clean start), then
      // swept down by the FM envelope.
      if (fmPulseWidth_) {
        --fmPulseWidth_;
        phase_ = 0.25f;
      } else {
        fm_ *= fmDecayCoefficient_;
        float fm = 1.0f + fmAmount_ * 3.5f * fmLp_;
        phase_ += minf(f0_ * fm, 0.5f);
        if (phase_ >= 1.0f)
          phase_ -= 1.0f;
      }
      // 3. Envelopes: hold for 1 ms, then decay; all lightly smoothed.
      if (bodyEnvPulseWidth_) {
        --bodyEnvPulseWidth_;
      } else {
        bodyEnv_ *= bodyEnvDecay_;
        transientEnv_ *= transientEnvDecay_;
      }
      onePole(bodyEnvLp_, bodyEnv_, envelopeLpCoefficient_);
      onePole(transientEnvLp_, transientEnv_, envelopeLpCoefficient_);
      onePole(fmLp_, fm_, envelopeLpCoefficient_);
      // 4. Body through the VCA, plus the click-and-noise transient.
      float body = distortedSine(phase_, phaseNoise_, dirtiness_);
      float transient = click(bodyEnvPulseWidth_ ? 0.0f : 1.0f) + attackNoise();
      mix -= transistorVca(body, bodyEnvLp_);
      mix -= transient * transientEnvLp_ * tone_;
    }
    // 5. Tone filter.
    onePole(toneLp_, mix, toneCoefficient_);
    return toneLp_;
  }

private:
  /** Sine, morphing into a soft-clipped triangle with a jittery phase. */
  static float distortedSine(float phase, float phaseNoise, float dirtiness) {
    phase += phaseNoise * dirtiness;
    phase -= static_cast<float>(static_cast<int32_t>(phase));
    float triangle = (phase < 0.5f ? phase : 1.0f - phase) * 4.0f - 1.0f;
    float sine = 2.0f * triangle / (1.0f + fabsf(triangle));
    float cleanSine = sineFromPhase(phase + 0.75f);
    return sine + (1.0f - dirtiness) * (cleanSine - sine);
  }

  /** A transistor VCA: offset, saturating, with some control bleed. */
  static float transistorVca(float s, float gain) {
    s = (s - 0.6f) * gain;
    return 3.0f * s / (2.0f + fabsf(s)) + gain * 0.3f;
  }

  /** The click: a step turned into a short pulse and band-limited. */
  float click(float in) {
    slope(clickLpState_, in, clickRise_, clickFall_);
    onePole(clickHpState_, clickLpState_, clickHp_);
    return clickFilter_.process(clickLpState_ - clickHpState_).lp;
  }

  /** Band-passed noise for the attack. */
  float attackNoise() {
    onePole(noiseLpState_, random_.uniform(), noiseLp_);
    onePole(noiseHpState_, noiseLpState_, noiseHp_);
    return noiseLpState_ - noiseHpState_;
  }

  void update() {
    f0_ = hz_ / sampleRate_;
    const float f048 = hz_ / 48000.0f;
    decaySquared_ = decay_ * decay_;
    float fmDecay2 = fmDecay_ * fmDecay_;
    // Dirtiness fades out at high pitches.
    dirtiness_ = dirtinessKnob_ * maxf(1.0f - 8.0f * f048, 0.0f);
    fmDecayCoefficient_ =
        1.0f - 1.0f / (0.008f * (1.0f + fmDecay2 * 4.0f) * sampleRate_);
    bodyEnvDecay_ = 1.0f - 1.0f / (0.02f * sampleRate_) *
                               semitonesToRatio(-decaySquared_ * 60.0f);
    transientEnvDecay_ = 1.0f - 1.0f / (0.005f * sampleRate_);
    toneCoefficient_ = rescaleCoefficient(
        minf(4.0f * f048 * semitonesToRatio(tone_ * 108.0f), 1.0f), rateRatio_);
  }

  float sampleRate_ = SAMPLE_RATE, rateRatio_ = 1.0f;
  float hz_ = 50.0f, tone_ = 0.5f, decay_ = 0.5f;
  float dirtiness_ = 0.0f, dirtinessKnob_ = 0.0f;
  float fmAmount_ = 0.5f, fmDecay_ = 0.5f, accent_ = 0.8f;
  bool sustain_ = false;
  float f0_ = 0.0f, decaySquared_ = 0.25f;
  float fmDecayCoefficient_ = 0.99f, bodyEnvDecay_ = 0.99f;
  float transientEnvDecay_ = 0.99f;
  float toneCoefficient_ = 1.0f;
  float phaseNoiseCoefficient_ = 0.002f, envelopeLpCoefficient_ = 0.1f;
  float clickRise_ = 0.5f, clickFall_ = 0.1f, clickHp_ = 0.04f;
  float noiseLp_ = 0.05f, noiseHp_ = 0.005f;
  float phase_ = 0.0f, phaseNoise_ = 0.0f;
  float fm_ = 0.0f, fmLp_ = 0.0f;
  float bodyEnv_ = 0.0f, bodyEnvLp_ = 0.0f;
  float transientEnv_ = 0.0f, transientEnvLp_ = 0.0f;
  int bodyEnvPulseWidth_ = 0, fmPulseWidth_ = 0;
  float toneLp_ = 0.0f;
  float clickLpState_ = 0.0f, clickHpState_ = 0.0f;
  float noiseLpState_ = 0.0f, noiseHpState_ = 0.0f;
  Random random_;
  Svf clickFilter_;
};

} // namespace bb
