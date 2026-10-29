/**
 * @file harmonic.h
 * @brief Additive oscillator: up to 32 harmonics from one sine.
 *
 * A sum of harmonics of one pitch, each with its own amplitude. Set them
 * from pots, envelopes, or computed spectra (organ drawbars, formants, a
 * spectral tilt). Plaits' additive engine uses three of these.
 *
 * HOW IT WORKS: instead of computing 32 sines, it computes one,
 * x = sin(phase), and gets the rest from the Chebyshev recurrence
 *   T(k+1) = 2x T(k) - T(k-1)
 * which costs one multiply and one add per harmonic. Each harmonic's
 * amplitude is reduced as it nears Nyquist and silenced above it, so
 * nothing aliases. Harmonic k comes out with a phase offset compared with
 * sin(k*phase): that changes the waveform's shape but not its sound.
 *
 * COST (an estimate from the instruction count; measure your patch with
 * AudioEngine::cpuLoad()): about 4 cycles per harmonic per sample, so 32
 * harmonics take roughly 130 of the ~1900 cycles available per sample.
 * setAmplitude() recomputes every gain: call it at control rate.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Additive (harmonic series) oscillator.
 *
 * Example — an organ-like drawbar sound:
 *   HarmonicOscillator organ;
 *   const float drawbars[8] = {1.0f, 0.5f, 0.0f, 0.3f,
 *                              0.0f, 0.2f, 0.0f, 0.1f};
 *   organ.setAmplitudes(drawbars, 8);
 *   organ.setFrequency(110.0f);
 *
 *   float s = organ.process() * 0.4f;
 */
class HarmonicOscillator {
public:
  /** The most harmonics one oscillator can sum. */
  static const int kMaxHarmonics = 32;

  HarmonicOscillator() { init(); }

  /**
   * Reset to a pure fundamental.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
    for (float &a : amplitude_)
      a = 0.0f;
    amplitude_[0] = 1.0f;
    numHarmonics_ = 1;
    update();
  }

  /** @param hz  Fundamental frequency in Hz */
  void setFrequency(float hz) {
    frequency_ = minf(hz * sampleTime_, 0.5f);
    update();
  }

  /**
   * Set one harmonic's amplitude. Keep the sum of all amplitudes near 1
   * for a full-scale output.
   * @param k  Harmonic number, 1 (fundamental) to 32
   * @param a  Amplitude, typically 0.0 to 1.0
   */
  void setAmplitude(int k, float a) {
    if (k < 1 || k > kMaxHarmonics)
      return;
    amplitude_[k - 1] = a;
    if (k > numHarmonics_)
      numHarmonics_ = k;
    update();
  }

  /**
   * Set harmonics 1..n at once (the rest are silenced).
   * @param a  Array of n amplitudes
   * @param n  Count, 1 to 32
   */
  void setAmplitudes(const float *a, int n) {
    numHarmonics_ = n < kMaxHarmonics ? n : kMaxHarmonics;
    for (int i = 0; i < numHarmonics_; ++i)
      amplitude_[i] = a[i];
    for (int i = numHarmonics_; i < kMaxHarmonics; ++i)
      amplitude_[i] = 0.0f;
    update();
  }

  /**
   * Generate the next sample.
   * @return  The weighted sum of harmonics (about ±1 if the amplitudes
   *          sum to 1)
   */
  float process() {
    phase_ += frequency_;
    if (phase_ >= 1.0f)
      phase_ -= 1.0f;
    // Chebyshev recurrence from x = sin(phase).
    const float twoX = 2.0f * interpolate(lutSine(), phase_, kSineTableSize);
    float previous = 1.0f;
    float current = twoX * 0.5f;
    float sum = 0.0f;
    for (int i = 0; i < numHarmonics_; ++i) {
      sum += gain_[i] * current;
      float temp = current;
      current = twoX * current - previous;
      previous = temp;
    }
    return sum;
  }

private:
  /** Fade each harmonic out as it approaches Nyquist (0.5). */
  void update() {
    for (int i = 0; i < numHarmonics_; ++i) {
      float f = minf(frequency_ * static_cast<float>(i + 1), 0.5f);
      gain_[i] = amplitude_[i] * (1.0f - f * 2.0f);
    }
  }

  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float frequency_ = 0.0f, phase_ = 0.0f;
  int numHarmonics_ = 1;
  float amplitude_[kMaxHarmonics] = {};
  float gain_[kMaxHarmonics] = {};
};

} // namespace bb
