/**
 * @file string_model.h
 * @brief Elements' string: Karplus-Strong with dispersion and a buzzing
 *        bridge.
 *
 * A plucked or struck string you excite with any signal: a noise burst, a
 * click, the Plucker, Dust, or audio.
 *
 * HOW THIS DIFFERS FROM KarplusStrong (karplus.h): KarplusStrong is the
 * classic algorithm — a delay line and an averaging filter, plucked by
 * its own noise burst. StringModel adds tuned loss filters, stiffness
 * (inharmonic, piano-like partials), a sitar-like "curved bridge", a
 * second pickup, and accepts any excitation.
 *
 * HOW IT WORKS:
 * - A delay line one period long holds the travelling wave. Each pass
 *   through the loop the wave is filtered and fed back, so the excitation
 *   repeats at the string's pitch and slowly dies away.
 * - Two filters in the loop set the decay: a short FIR whose gain sets the
 *   overall decay time (`damping`), and an Svf low-pass whose cutoff sets
 *   how fast high harmonics die (`brightness`). The delay is shortened by
 *   the low-pass's own delay so the note stays in tune.
 * - `dispersion` (0..1, centre = none):
 *   - above the centre, part of the delay becomes an all-pass filter,
 *     which delays high frequencies more than low ones, as in a stiff
 *     piano string; near 1, noise also jitters the pitch (rattling);
 *   - below the centre, a nonlinear "curved bridge" bends the waveform
 *     when it swings one way, giving a buzzing, sitar-like tone.
 * - aux() is a second pickup at `position`. Reading the delay line
 *   part-way along is a comb filter: the thinner tone of a pickup near
 *   the bridge.
 *
 * NAME: "String" would clash with Arduino's String class, hence
 * StringModel.
 *
 * MEMORY: the template parameter is the main delay length (float storage,
 * plus half that again for the dispersion all-pass). StringModel<2048>
 * (the default) uses 12 KB and plays notes down to ~22 Hz directly; lower
 * notes still work (the loop then runs slower than the sample rate and is
 * interpolated). StringModel<1024> uses 6 KB.
 *
 * Derived from Elements, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/luts.h"
#include "buddhabox/core/random.h"
#include "buddhabox/filter/dc_blocker.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/delay.h"

namespace bb {

/**
 * Dispersive Karplus-Strong string.
 *
 * @tparam MAX_DELAY  Main delay length in samples (sets the lowest
 *                    directly-played note: SAMPLE_RATE / MAX_DELAY)
 *
 * Example — a sitar-like drone, re-plucked every 2 s:
 *   StringModel<> sitar;
 *   Plucker pick;
 *   sitar.setFrequency(110.0f);
 *   sitar.setDispersion(0.1f); // curved bridge: buzzy
 *   sitar.setDamping(0.9f);
 *   pick.setFrequency(110.0f);
 *
 *   if (clock.process()) {
 *     pick.trigger();
 *   }
 *   float s = sitar.process(pick.process());
 */
template <uint16_t MAX_DELAY = 2048> class StringModel {
public:
  StringModel() { init(); }

  /**
   * Silence the string and reset its filters.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    // Leave room for the Hermite read's guard samples.
    maxDelay_ = static_cast<float>(MAX_DELAY) - 4.0f;
    string_.clear();
    stretch_.clear();
    iirDamping_.init(sampleRate);
    dcBlocker_.init(sampleRate, 20.0f);
    // The dispersion noise filter's coefficient was tuned per sample at
    // Elements' 32 kHz.
    noiseRateRatio_ = 32000.0f / sampleRate;
    controlPeriod_ = static_cast<int>(sampleRate / 2000.0f + 0.5f);
    if (controlPeriod_ < 1)
      controlPeriod_ = 1;
    counter_ = 0;
    frequency_ = 220.0f / sampleRate;
    delay_ = 1.0f / frequency_;
    clampedPosition_ = 0.0f;
    dispersionNow_ = 0.0f;
    dampingCompensation_ = 0.0f;
    dispersionNoise_ = 0.0f;
    curvedBridge_ = 0.0f;
    firX1_ = firX2_ = 0.0f;
    firDamping_ = firBrightness_ = 0.0f;
    srcPhase_ = 0.0f;
    out_[0] = out_[1] = aux_[0] = aux_[1] = 0.0f;
    outValue_ = auxValue_ = 0.0f;
  }

  /** @param hz  Pitch in Hz */
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }

  /**
   * @param x  0..1. 0.5 = plain string (with a small dead zone). Above:
   *           stiff, inharmonic string, rattling near 1. Below: buzzing
   *           "curved bridge", strongest at 0.
   */
  void setDispersion(float x) {
    dispersion_ = x < 0.49f ? (x - 0.49f) / 0.49f
                            : (x > 0.51f ? (x - 0.51f) / 0.49f : 0.0f);
  }

  /** @param x  0..1: low = dull (highs die fast), 1 = bright */
  void setBrightness(float x) { brightness_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: decay time; from 0.95 up, towards endless sustain */
  void setDamping(float x) { damping_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: second pickup position, for aux() */
  void setPosition(float x) { position_ = clampf(x, 0.0f, 1.0f); }

  /** @param s  Seed for the dispersion noise (repeatable output) */
  void seed(uint32_t s) { random_.seed(s); }

  /**
   * Process one sample.
   * @param in  Excitation signal
   * @return    The string output; aux() holds the second pickup
   */
  float process(float in) {
    if (counter_ == 0) {
      counter_ = controlPeriod_;
      updateControl();
    }
    --counter_;
    // Below the lowest note that fits, the loop runs slower than the
    // sample rate (srcRatio < 1) and its output is linearly interpolated
    // up. Otherwise srcRatio = 1 and the loop runs once per sample.
    srcPhase_ += srcRatio_;
    if (srcPhase_ > 1.0f) {
      srcPhase_ -= 1.0f;
      runLoop(in);
    }
    auxValue_ = lerpf(aux_[1], aux_[0], srcPhase_);
    outValue_ = lerpf(out_[1], out_[0], srcPhase_);
    return outValue_;
  }

  /** @return  The second pickup from the last process() call */
  float aux() const { return auxValue_; }

private:
  /** One trip round the string loop. */
  void runLoop(float in) {
    // 1. Advance the per-sample ramps planned in updateControl().
    delay_ += delayIncrement_;
    clampedPosition_ += positionIncrement_;
    dispersionNow_ += dispersionIncrement_;
    dampingCompensation_ += compensationIncrement_;
    float delay = delay_;
    float combDelay = delay * clampedPosition_;
    delay *= dampingCompensation_; // the low-pass filter's own delay
    delay -= 1.0f;                 // the FIR filter's one-sample delay

    // 2. Read the string, with or without dispersion.
    float s;
    float dispersion = dispersionNow_;
    {
      // Filtered noise, used to jitter the delay when dispersion > 0.75.
      float noise = random_.bipolar();
      noise *= 1.0f / (0.2f + noiseFilter_);
      onePole(dispersionNoise_, noise, noiseFilterRescaled_);
      float stretchPoint =
          dispersion <= 0.0f ? 0.0f : dispersion * (2.0f - dispersion) * 0.475f;
      float noiseAmount =
          dispersion > 0.75f ? 4.0f * (dispersion - 0.75f) : 0.0f;
      float bridgeCurving = dispersion < 0.0f ? -dispersion : 0.0f;
      noiseAmount = noiseAmount * noiseAmount * 0.025f;
      float acBlockingAmount = bridgeCurving;
      bridgeCurving = bridgeCurving * bridgeCurving * 0.01f;
      float apGain = -0.618f * dispersion / (0.15f + fabsf(dispersion));
      float delayFm =
          1.0f + dispersionNoise_ * noiseAmount - curvedBridge_ * bridgeCurving;
      delay *= delayFm;
      // Split the delay into a plain part and an all-pass part. The
      // all-pass delays highs more than lows: stiff-string inharmonicity.
      // (Delays are "samples before this write"; DelayLine's read
      // functions count from the most recent sample, hence the - 1.)
      float apDelay = delay * stretchPoint;
      float mainDelay = delay - apDelay;
      if (apDelay >= 4.0f && mainDelay >= 4.0f) {
        s = string_.readHermite(mainDelay - 1.0f);
        s = stretch_.allpass(s, static_cast<uint16_t>(apDelay), apGain);
      } else {
        s = string_.readHermite(delay - 1.0f);
      }
      // Curved bridge: partly remove DC (the bridge clips one way only),
      // then compute the waveform-dependent bend for the next pass.
      float sAc = dcBlocker_.process(s);
      s += acBlockingAmount * (sAc - s);
      float value = fabsf(s) - 0.025f;
      float sign = s > 0.0f ? 1.0f : -1.5f;
      curvedBridge_ = (fabsf(value) + value) * sign;
    }

    // 3. Add the excitation, filter, write back.
    s += in;
    s = firDamping(s);
    s = iirDamping_.process(s).lp;
    string_.write(s);
    out_[1] = out_[0];
    aux_[1] = aux_[0];
    out_[0] = s;
    aux_[0] = string_.read(combDelay - 1.0f);
  }

  /**
   * Three-tap FIR: overall loss (damping) plus a gentle low-pass whose
   * depth depends on brightness. Its gains ramp per sample.
   */
  float firDamping(float x) {
    float h0 = (1.0f + firBrightness_) * 0.5f;
    float h1 = (1.0f - firBrightness_) * 0.25f;
    float y = firDamping_ * (h0 * firX1_ + h1 * (x + firX2_));
    firX2_ = firX1_;
    firX1_ = x;
    firBrightness_ += firBrightnessIncrement_;
    firDamping_ += firDampingIncrement_;
    return y;
  }

  void updateControl() {
    const float step = 1.0f / controlPeriod_;
    // 1. Delay length for this pitch; switch to the slow-loop mode below
    // the lowest note that fits.
    float delay = clampf(1.0f / frequency_, 4.0f, maxDelay_ - 4.0f);
    srcRatio_ = delay * frequency_;
    if (srcRatio_ >= 0.9999f) {
      srcPhase_ = 1.0f;
      srcRatio_ = 1.0f;
    }
    float clampedPosition = 0.5f - 0.98f * fabsf(position_ - 0.5f);
    delayIncrement_ = (delay - delay_) * step;
    positionIncrement_ = (clampedPosition - clampedPosition_) * step;
    dispersionIncrement_ = (dispersion_ - dispersionNow_) * step;

    // 2. Loop gain from the decay time. rt60 is the time (in samples) for
    // the sound to fall by 60 dB; the gain per pass follows from how many
    // passes that takes.
    float lfDamping = damping_ * (2.0f - damping_);
    float rt60 = 0.07f * semitonesToRatio(lfDamping * 96.0f) * sampleRate_;
    float rt60Base212 = -120.0f * delay / srcRatio_ / rt60;
    if (rt60Base212 < -127.0f)
      rt60Base212 = -127.0f;
    float dampingCoefficient = semitonesToRatio(rt60Base212);
    float brightness = brightness_ * brightness_;
    noiseFilter_ = semitonesToRatio((brightness_ - 1.0f) * 48.0f);
    noiseFilterRescaled_ = rescaleCoefficient(noiseFilter_, noiseRateRatio_);

    // 3. Low-pass cutoff, in semitones above the fundamental.
    float dampingCutoff =
        24.0f + damping_ * damping_ * 48.0f + brightness_ * brightness_ * 24.0f;
    if (dampingCutoff > 84.0f)
      dampingCutoff = 84.0f;
    float dampingF = frequency_ * semitonesToRatio(dampingCutoff);
    if (dampingF > 0.499f)
      dampingF = 0.499f;

    // 4. Above damping 0.95, crossfade towards infinite sustain.
    if (damping_ >= 0.95f) {
      float toInfinite = 20.0f * (damping_ - 0.95f);
      dampingCoefficient += toInfinite * (1.0f - dampingCoefficient);
      brightness += toInfinite * (1.0f - brightness);
      dampingF += toInfinite * (0.4999f - dampingF);
      dampingCutoff += toInfinite * (128.0f - dampingCutoff);
    }
    firDampingIncrement_ = (dampingCoefficient - firDamping_) * step;
    firBrightnessIncrement_ = (brightness - firBrightness_) * step;
    iirDamping_.setCoefficients(dampingF, 0.5f, TanApprox::Accurate);
    float compensation = 1.0f - interpolate(lutSvfShift(), dampingCutoff, 1.0f);
    compensationIncrement_ = (compensation - dampingCompensation_) * step;
  }

  float sampleRate_ = SAMPLE_RATE;
  float maxDelay_ = MAX_DELAY - 4.0f;
  float noiseRateRatio_ = 1.0f;
  float frequency_ = 220.0f / SAMPLE_RATE;
  float dispersion_ = 0.0f;
  float brightness_ = 0.5f;
  float damping_ = 0.3f;
  float position_ = 0.8f;
  int controlPeriod_ = 22;
  int counter_ = 0;
  float srcRatio_ = 1.0f, srcPhase_ = 0.0f;
  // Current value and per-sample step of each ramped parameter.
  float delay_ = 0.0f, delayIncrement_ = 0.0f;
  float clampedPosition_ = 0.0f, positionIncrement_ = 0.0f;
  float dispersionNow_ = 0.0f, dispersionIncrement_ = 0.0f;
  float dampingCompensation_ = 0.0f, compensationIncrement_ = 0.0f;
  float firDamping_ = 0.0f, firDampingIncrement_ = 0.0f;
  float firBrightness_ = 0.0f, firBrightnessIncrement_ = 0.0f;
  float firX1_ = 0.0f, firX2_ = 0.0f;
  float noiseFilter_ = 0.0f, noiseFilterRescaled_ = 0.0f;
  float dispersionNoise_ = 0.0f;
  float curvedBridge_ = 0.0f;
  float out_[2], aux_[2];
  float outValue_ = 0.0f, auxValue_ = 0.0f;
  Random random_;
  DelayLine<MAX_DELAY, float> string_;
  DelayLine<MAX_DELAY / 2, float> stretch_;
  Svf iirDamping_;
  DcBlocker dcBlocker_;
};

} // namespace bb
