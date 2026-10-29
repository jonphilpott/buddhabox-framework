/**
 * @file sine.h
 * @brief Table sine oscillator and the recursive cosine oscillator.
 *
 * SineOscillator: a plain sine read from the shared flash table (see
 * sineFromPhase() in dsp_common.h). Several Plaits models use it inside
 * their sustain modes; it also offers a quadrature (sine + cosine) output.
 *
 * CosineOscillator: generates the series cos(0), cos(w), cos(2w), ... one
 * value per next() call, using one multiply and one subtract per step.
 * It is rarely heard directly. Gillet uses it to compute sets of
 * per-harmonic gains cheaply: the modal resonator weights mode k by
 * 0.5 + 0.5 * cos(2*pi*k*position) to imitate where along a string or bar
 * it is struck.
 *
 * HOW THE RECURRENCE WORKS:
 * Cosines obey the identity
 *   cos((n+1)w) = 2cos(w) * cos(nw) - cos((n-1)w)
 * so once 2cos(w) is known, each new value needs only the previous two.
 *
 * Derived from stmlib dsp/cosine_oscillator.h (Copyright 2014) and Plaits
 * sine_oscillator.h (Copyright 2016), Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * A sine oscillator reading the shared flash sine table (Plaits).
 *
 * Use it for test tones, FM carriers, or the quadrature output (sine and
 * cosine together, 90 degrees apart) that frequency shifters and stereo
 * effects need. For the framework's multi-waveform oscillator see
 * Oscillator (oscillator.h).
 *
 * Example:
 *   SineOscillator sine;
 *   sine.setFrequency(220.0f);
 *
 *   // In the audio callback:
 *   float s = sine.process() * 0.5f;
 */
class SineOscillator {
public:
  SineOscillator() { init(); }

  /**
   * Reset the phase and set the sample rate.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.0f;
  }

  /**
   * Set the frequency.
   * @param hz  Frequency in Hz, 0 to SAMPLE_RATE / 2
   */
  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }

  /** Restart the cycle at phase 0. */
  void reset() { phase_ = 0.0f; }

  /**
   * Generate the next sample at the frequency set by setFrequency().
   * @return  -1.0 to +1.0
   */
  float process() { return next(frequency_); }

  /**
   * Generate the next sample at a per-call frequency (lower-level form,
   * used by the drum models to sweep pitch every sample).
   *
   * @param frequency  Cycles per sample (Hz / SAMPLE_RATE), 0 to 0.5
   * @return           -1.0 to +1.0
   */
  float next(float frequency) {
    if (frequency >= 0.5f)
      frequency = 0.5f;
    phase_ += frequency;
    if (phase_ >= 1.0f)
      phase_ -= 1.0f;
    return interpolate(lutSine(), phase_, kSineTableSize);
  }

  /**
   * Generate sine and cosine together (90 degrees apart).
   *
   * The table holds an extra quarter cycle, so the cosine is a second read
   * a quarter-cycle further on, with no wrap check needed.
   *
   * @param frequency  Cycles per sample (Hz / SAMPLE_RATE), 0 to 0.5
   * @param amplitude  Output scale
   * @param sine       Receives amplitude * sin(phase)
   * @param cosine     Receives amplitude * cos(phase)
   *
   * Example:
   *   float s, c;
   *   osc.next(440.0f / SAMPLE_RATE, 1.0f, s, c);
   */
  void next(float frequency, float amplitude, float &sine, float &cosine) {
    if (frequency >= 0.5f)
      frequency = 0.5f;
    phase_ += frequency;
    if (phase_ >= 1.0f)
      phase_ -= 1.0f;
    sine = amplitude * interpolate(lutSine(), phase_, kSineTableSize);
    cosine = amplitude * interpolate(lutSine(), phase_ + 0.25f, kSineTableSize);
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float frequency_ = 0.0f;
  float phase_ = 0.0f;
};

/**
 * Generates 0.5 + 0.5 * cos(n * w) for n = 0, 1, 2, ... (range 0..1), one
 * term per next() call.
 *
 * Mostly plumbing: it computes a whole set of per-mode or per-harmonic
 * gains in a loop for the cost of one multiply each. The FxEngine also
 * uses it as a slow LFO, stepping it once every 32 samples.
 *
 * Example:
 *   // Gains for 16 modes struck a third of the way along a string:
 *   CosineOscillator gains;
 *   gains.initApproximate(0.333f);
 *   for (int k = 0; k < 16; ++k) {
 *     modeGain[k] = gains.next();
 *   }
 */
class CosineOscillator {
public:
  /**
   * Set the step size using an exact cosf() (call from setup code).
   * @param frequency  Cycles per step, 0.0 to 1.0
   */
  void init(float frequency) {
    coefficient_ = 2.0f * cosf(TWO_PI_F * frequency);
    initialAmplitude_ = coefficient_ * 0.25f;
    start();
  }

  /**
   * Set the step size using stmlib's cheap parabolic approximation of
   * 2cos(2*pi*f). Safe to call every sample; matches the originals.
   *
   * @param frequency  Cycles per step, 0.0 to 1.0
   */
  void initApproximate(float frequency) {
    // Fold the frequency into a quarter cycle, then a parabola
    // 16x(1 - 2x) approximates the cosine's shape over that range.
    float sign = 16.0f;
    frequency -= 0.25f;
    if (frequency < 0.0f) {
      frequency = -frequency;
    } else if (frequency > 0.5f) {
      frequency -= 0.5f;
    } else {
      sign = -16.0f;
    }
    coefficient_ = sign * frequency * (1.0f - 2.0f * frequency);
    initialAmplitude_ = coefficient_ * 0.25f;
    start();
  }

  /** Rewind to the first term (n = 0). */
  void start() {
    y1_ = initialAmplitude_;
    y0_ = 0.5f;
  }

  /** @return  The most recent value returned by next(), 0..1 */
  float value() const { return y1_ + 0.5f; }

  /**
   * Advance to the next term.
   * @return  0.5 + 0.5 * cos(n * w), 0.0 to 1.0
   */
  float next() {
    float temp = y0_;
    y0_ = coefficient_ * y0_ - y1_;
    y1_ = temp;
    return temp + 0.5f;
  }

private:
  float y1_ = 0.0f, y0_ = 0.5f;
  float coefficient_ = 0.0f, initialAmplitude_ = 0.0f;
};

} // namespace bb
