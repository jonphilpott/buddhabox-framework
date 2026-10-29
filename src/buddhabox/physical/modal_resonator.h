/**
 * @file modal_resonator.h
 * @brief Rings' modal resonator: struck or bowed strings, bars and bells.
 *
 * The sound of a struck or bowed object (string, bar, plate, bell, glass)
 * made by summing resonant band-pass filters, one per vibration "mode".
 * Feed it a short burst (a click, noise, a pluck, Dust) and it rings.
 * Singing bowls, gamelan, wind chimes: this is the ambient workhorse.
 *
 * HOW IT WORKS:
 * - Each mode is an Svf band-pass tuned to one partial. Partial k sits at
 *   k * frequency * stretch, where the stretch grows with `structure`:
 *   none for a harmonic string, more for stiff bars, compressed for bells.
 * - Each mode's Q sets how long it rings. Higher partials lose Q faster
 *   (`brightness` controls how fast), which is how real objects behave.
 * - `position` imitates where the object is struck: mode k is weighted by
 *   0.5 + 0.5*cos(2*pi*k*position). That is a comb filter applied
 *   directly to the mode gains instead of to the audio, which avoids
 *   flanging when position is modulated.
 * - With bowed modes on, the first 8 modes also get a delay-line loop
 *   driven by a nonlinear "bow table" (Elements' banded waveguides), so a
 *   continuous bow pressure gives a sustained, scratchy tone.
 *
 * COST ON THE BLACK PILL (84 MHz, ~1900 cycles per sample in total), an
 * estimate from the instruction count — measure your patch with
 * AudioEngine::cpuLoad(): about 20 cycles per mode per sample, plus
 * filter updates at 2 kHz. The default of 24 modes should take roughly
 * a quarter of the CPU; setModes(16) is
 * plenty for bells and bowls, 32+ for bright metallic sounds. Rings runs
 * up to 64 on a 168 MHz chip.
 *
 * MEMORY: the template parameter sizes the bowed-mode delay lines (8 of
 * them, 4 bytes per sample each). 0 (the default) disables bowing and
 * costs nothing. They hold float, not int16_t: the bowed signal is tiny
 * (around 0.01) and the loop resonates strongly, so 16-bit storage would
 * leave it only ~8 bits of resolution and audibly change the tone.
 * - ModalResonator<512>   16 KB; notes below ~86 Hz fold up an octave
 * - ModalResonator<1024>  32 KB; notes below ~43 Hz fold up an octave
 *
 * Derived from Rings and Elements, Copyright 2015 Emilie Gillet.
 * MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/luts.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/delay.h"
#include "buddhabox/osc/sine.h"

namespace bb {

/**
 * Modal resonator (sum of tuned band-pass modes).
 *
 * @tparam BOW_DELAY  Bowed-mode delay length in samples; 0 = no bowing
 *
 * Example — a singing bowl struck by random dust:
 *   ModalResonator<> bowl;
 *   Dust dust;
 *   bowl.setFrequency(220.0f);
 *   bowl.setStructure(0.15f); // bell-like partials
 *   bowl.setDamping(0.85f);   // long ring
 *   bowl.setModes(16);
 *   dust.setDensity(0.5f);
 *
 *   float s = bowl.process(dust.process());
 *
 * Example — bowed (needs bow memory):
 *   ModalResonator<512> bowed;
 *   bowed.setBowedModes(true);
 *   float s = bowed.process(0.0f, 0.6f); // bow pressure 0.6
 */
template <uint16_t BOW_DELAY = 0> class ModalResonator {
public:
  /** Most modes one resonator can run. */
  static const int kMaxModes = 64;
  /** Modes that get a bowed waveguide when bowing is on. */
  static const int kBowedModes = 8;

  ModalResonator() { init(); }

  /**
   * Reset all modes.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    // 1. Control rate: recompute filters 2000 times a second.
    controlPeriod_ = static_cast<int>(sampleRate / 2000.0f + 0.5f);
    if (controlPeriod_ < 1)
      controlPeriod_ = 1;
    counter_ = 0;
    // 2. Rate scaling for Q: Rings was tuned at 48 kHz, the bowed modes
    // (from Elements) at 32 kHz. Scaling Q keeps decay times in seconds.
    qScale_ = sampleRate / 48000.0f;
    bowQScale_ = sampleRate / 32000.0f;
    for (int i = 0; i < kMaxModes; ++i)
      modes_[i].init(sampleRate);
    for (int i = 0; i < kBowedModes; ++i) {
      bowFilters_[i].init(sampleRate);
      bowDelays_[i].clear();
      bowPeriod_[i] = 1;
    }
    previousPosition_ = 0.0f;
    positionIncrement_ = 0.0f;
    bowSignal_ = 0.0f;
    lfoPhase_ = 0.0f;
    numModes_ = 0;
    odd_ = even_ = side_ = 0.0f;
  }

  /** @param hz  Pitch of the fundamental in Hz */
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }

  /**
   * Inharmonicity.
   * @param x  0..1. Below 0.25: bell-like (partials squeezed); 0.25..0.3:
   *           harmonic string; above: stiffer bars and plates; near 1:
   *           metallic
   */
  void setStructure(float x) { structure_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: how long high partials ring relative to low ones */
  void setBrightness(float x) { brightness_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: overall decay, from a dead thud to ~30 s of ring */
  void setDamping(float x) { damping_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: strike/pickup point; 0.5 = middle (odd modes only) */
  void setPosition(float x) { position_ = clampf(x, 0.0f, 1.0f); }

  /** @param n  Number of modes, 1..64. Fewer is cheaper and simpler. */
  void setModes(int n) {
    resolution_ = n < 1 ? 1 : (n > kMaxModes ? kMaxModes : n);
  }

  /** @param on  Elements' bowed modes (needs BOW_DELAY > 0) */
  void setBowedModes(bool on) { bowed_ = on && BOW_DELAY > 0; }

  /**
   * Side-channel pickup LFO (Elements), for a moving stereo image.
   * @param hz      LFO rate in Hz
   * @param offset  Pickup offset, 0..1
   */
  void setSideLfo(float hz, float offset) {
    lfoFrequency_ = hz / sampleRate_;
    lfoOffset_ = offset;
  }

  /**
   * Process one sample.
   * @param in           Excitation (strikes, noise, audio)
   * @param bowStrength  Bow pressure 0..1 (only with bowed modes on)
   * @return             The sum of all modes; odd(), even() and side()
   *                     give this sample's other outputs
   */
  float process(float in, float bowStrength = 0.0f) {
    // 1. Control-rate update: retune all filters, and plan a linear ramp
    // of the position over the next period (position is very sensitive
    // to zipper noise).
    if (counter_ == 0) {
      counter_ = controlPeriod_;
      computeFilters();
      positionIncrement_ = (position_ - previousPosition_) / controlPeriod_;
    }
    --counter_;
    previousPosition_ += positionIncrement_;

    // 2. Triangle LFO (0..0.5) that sweeps the side output's pickup.
    lfoPhase_ += lfoFrequency_;
    if (lfoPhase_ >= 1.0f)
      lfoPhase_ -= 1.0f;
    float lfo = lfoPhase_ > 0.5f ? 1.0f - lfoPhase_ : lfoPhase_;

    // 3. Per-mode gains from the position: 0.5 + 0.5*cos(2*pi*k*pos).
    CosineOscillator amplitudes, sideAmplitudes;
    amplitudes.initApproximate(previousPosition_);
    sideAmplitudes.initApproximate(lfoOffset_ + lfo);

    // 4. Run the modes, summed in two groups (odd and even mode numbers,
    // Rings' two outputs); the side sum uses the LFO-swept gains.
    float input = in * 0.125f;
    float odd = 0.0f, even = 0.0f, side = 0.0f;
    for (int i = 0; i < numModes_; ++i) {
      float s = modes_[i].process(input).bp;
      float a = amplitudes.next();
      if (i & 1) {
        even += s * a;
      } else {
        odd += s * a;
      }
      side += s * sideAmplitudes.next();
    }
    float center = odd + even;
    side_ = side - center;

    // 5. Bowed modes: each is a delay line one period long, closed
    // through a narrow band-pass, all excited by the bow-table
    // nonlinearity. The bow signal is the sum of the loops, fed back
    // next sample.
    if (bowed_) {
      int numBowed = numModes_ < kBowedModes ? numModes_ : kBowedModes;
      float bowSum = 0.0f;
      input += bowSignal_;
      amplitudes.start();
      for (int i = 0; i < numBowed; ++i) {
        // at(n - 1): the sample written n samples ago (read before write).
        float s = 0.99f * bowDelays_[i].at(bowPeriod_[i] - 1);
        bowSum += s;
        Svf &f = bowFilters_[i];
        s = f.process(input + s).bp * f.r();
        bowDelays_[i].write(s);
        center += s * amplitudes.next() * 8.0f;
      }
      bowSignal_ = bowTable(bowSum, bowStrength);
    }

    odd_ = odd;
    even_ = even;
    return center;
  }

  /** @return  Odd-numbered modes from the last process() call */
  float odd() const { return odd_; }

  /** @return  Even-numbered modes from the last process() call */
  float even() const { return even_; }

  /**
   * @return  Difference between an LFO-swept pickup and the main one; use
   *          (center + side, center - side) for a moving stereo image
   */
  float side() const { return side_; }

private:
  /**
   * Bow friction curve: steep for small bow/string velocity differences
   * (the bow grips), flat for large ones (it slips). The stick-slip
   * switching is what makes a bowed tone.
   */
  static float bowTable(float x, float velocity) {
    x = 0.13f * velocity - x;
    float bow = fabsf(x * 6.0f) + 0.75f;
    bow *= bow;
    bow *= bow;
    bow = 0.25f / bow;
    bow = clampf(bow, 0.0025f, 0.245f);
    return x * bow;
  }

  void computeFilters() {
    // 1. Stretch per partial from the structure knob.
    float stiffness = interpolate(lutStiffness(), structure_, 256.0f);
    float harmonic = frequency_;
    float stretchFactor = 1.0f;
    // 2. Base Q: 4 decades of range, scaled to keep decay times constant
    // across sample rates.
    float q = 500.0f * interpolate(lut4Decades(), damping_, 256.0f) * qScale_;
    // 3. Brightness: each partial's Q is the previous one's times qLoss.
    // At very low structure (bell) the brightness range is reduced to
    // avoid clipping.
    float attenuation = 1.0f - structure_;
    attenuation *= attenuation;
    attenuation *= attenuation;
    attenuation *= attenuation;
    float brightness = brightness_ * (1.0f - 0.2f * attenuation);
    float qLoss = brightness * (2.0f - brightness) * 0.85f + 0.15f;
    float qLossDampingRate = structure_ * (2.0f - structure_) * 0.1f;
    numModes_ = 0;
    for (int i = 0; i < resolution_; ++i) {
      // Partials above Nyquist are pinned there and not counted.
      float partial = harmonic * stretchFactor;
      if (partial >= 0.49f) {
        partial = 0.49f;
      } else {
        numModes_ = i + 1;
      }
      modes_[i].setCoefficients(partial, 1.0f + partial * q, TanApprox::Fast);
      if (i < kBowedModes && BOW_DELAY > 0) {
        uint16_t period = static_cast<uint16_t>(1.0f / partial);
        while (period >= kBowSize)
          period >>= 1; // fold down by octaves until it fits
        bowPeriod_[i] = period;
        bowFilters_[i].setGQ(modes_[i].g(),
                             1.0f + partial * 1500.0f * bowQScale_);
      }
      stretchFactor += stiffness;
      // Negative stiffness shrinks so partials never fold below zero;
      // positive stiffness shrinks slowly, adding a few high partials.
      stiffness *= stiffness < 0.0f ? 0.93f : 0.98f;
      // Stops the highest partials decaying too fast.
      qLoss += qLossDampingRate * (1.0f - qLoss);
      harmonic += frequency_;
      q *= qLoss;
    }
  }

  // A DelayLine needs at least one slot, so "no bowing" still declares a
  // 1-sample line (4 bytes each) rather than a zero-sized array.
  static const uint16_t kBowSize = BOW_DELAY > 0 ? BOW_DELAY : 1;

  float sampleRate_ = SAMPLE_RATE;
  float frequency_ = 220.0f / SAMPLE_RATE;
  float structure_ = 0.25f;
  float brightness_ = 0.5f;
  float damping_ = 0.3f;
  float position_ = 0.999f;
  int resolution_ = 24;
  bool bowed_ = false;
  float lfoFrequency_ = 0.5f / SAMPLE_RATE;
  float lfoOffset_ = 0.1f;
  int controlPeriod_ = 22;
  int counter_ = 0;
  int numModes_ = 0;
  float qScale_ = 1.0f, bowQScale_ = 1.0f;
  float previousPosition_ = 0.0f, positionIncrement_ = 0.0f;
  float lfoPhase_ = 0.0f;
  float bowSignal_ = 0.0f;
  float odd_ = 0.0f, even_ = 0.0f, side_ = 0.0f;
  Svf modes_[kMaxModes];
  Svf bowFilters_[kBowedModes];
  DelayLine<kBowSize, float> bowDelays_[kBowedModes];
  uint16_t bowPeriod_[kBowedModes];
};

} // namespace bb
