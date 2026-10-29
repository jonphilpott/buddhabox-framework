/**
 * @file basic.h
 * @brief Plaits' alias-free saw / square / triangle / slope oscillator.
 *
 * A band-limited partner to Oscillator (oscillator.h). Oscillator's naive
 * waveforms alias at high pitches; BasicOscillator removes the aliasing
 * with PolyBLEP corrections, and adds variable pulse width, a "slope"
 * shape (a triangle whose rise and fall can differ), filtered squares and
 * a band-limited impulse train.
 *
 * HOW IT WORKS: a phase accumulator (0..1 per cycle) drives a naive
 * waveform. Wherever the naive waveform jumps (saw reset, square edges) or
 * bends sharply (triangle corners), a PolyBLEP correction is added at the
 * exact fractional time of the event (see polyblep.h). The corrections
 * need one sample of look-ahead, so the output is one sample late.
 *
 * Changes from Plaits: SQUARE_DARK is scaled to about ±1 like the other
 * shapes (it was ±1.5), and the start-up state is set from the shape,
 * which avoids a spike on the first sample of SLOPE and IMPULSE_TRAIN.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/polyblep.h"

namespace bb {

/**
 * Band-limited classic-waveform oscillator (Plaits).
 *
 * Example:
 *   BasicOscillator osc;
 *   osc.setShape(BasicOscillator::SQUARE);
 *   osc.setFrequency(110.0f);
 *   osc.setPulseWidth(0.3f);
 *
 *   // In the audio callback:
 *   float s = osc.process() * 0.4f;
 */
class BasicOscillator {
public:
  /** Waveform choices. */
  enum Shape {
    SAW,
    TRIANGLE,
    SLOPE,           ///< triangle with variable symmetry (pulse width)
    SQUARE,          ///< variable pulse width
    SQUARE_BRIGHT,   ///< square through a high-pass: thin, nasal
    SQUARE_DARK,     ///< square through a low-pass: soft, hollow
    SQUARE_TRIANGLE, ///< integrated square: a smooth triangle-like wave
    IMPULSE_TRAIN    ///< band-limited pulse train
  };

  BasicOscillator() { init(); }

  /**
   * Reset the oscillator.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    phase_ = 0.5f; // Plaits' starting phase
    nextSample_ = 0.0f;
    lpState_ = 0.0f;
    hpState_ = 0.0f;
    needsSync_ = true;
  }

  /**
   * Choose the waveform.
   * @param s  One of the Shape values
   */
  void setShape(Shape s) {
    if (s != shape_)
      needsSync_ = true;
    shape_ = s;
  }

  /**
   * Set the pitch.
   * @param hz  Frequency in Hz, up to SAMPLE_RATE / 4
   */
  void setFrequency(float hz) { frequency_ = hz * sampleTime_; }

  /**
   * Set the pulse width (SQUARE shapes) or symmetry (SLOPE).
   * @param pw  0.0 to 1.0; 0.5 = square / symmetric triangle
   */
  void setPulseWidth(float pw) { pw_ = pw; }

  /**
   * Generate the next sample with the stored frequency and pulse width.
   * @return  About -1.0 to +1.0, one sample late (BLEP look-ahead)
   */
  float process() { return process(frequency_, pw_); }

  /**
   * Generate the next sample (lower-level form, for per-sample pitch
   * modulation).
   *
   * @param frequency  Cycles per sample (Hz / SAMPLE_RATE), up to 0.25
   * @param pw         Pulse width / symmetry, 0.0 to 1.0
   * @return           About -1.0 to +1.0
   */
  float process(float frequency, float pw) {
    // 1. Limits: above a quarter of the sample rate the BLEP corrections
    // overlap and stop working; the pulse width must leave room for both
    // edges.
    frequency = clampf(frequency, 0.000001f, 0.25f);
    pw = clampf(pw, frequency * 2.0f, 1.0f - 2.0f * frequency);
    if (shape_ == TRIANGLE || shape_ == SQUARE_TRIANGLE)
      pw = 0.5f;
    // After init or a shape change, set the edge tracker to match the
    // current phase, so the first sample doesn't see a false edge.
    if (needsSync_) {
      high_ =
          (shape_ == TRIANGLE || shape_ == SLOPE) ? phase_ < pw : phase_ >= pw;
      needsSync_ = false;
    }

    float thisSample = nextSample_;
    float nextSample = 0.0f;
    float out;
    phase_ += frequency;

    if (shape_ == SAW || shape_ == IMPULSE_TRAIN) {
      // 2a. Saw: one jump of -1 per cycle.
      if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        float t = phase_ / frequency;
        thisSample -= thisBlepSample(t);
        nextSample -= nextBlepSample(t);
      }
      nextSample += phase_;
      if (shape_ == SAW) {
        out = 2.0f * thisSample - 1.0f;
      } else {
        // The impulse train is the saw's derivative, gently smoothed.
        lpState_ += 0.25f * ((hpState_ - thisSample) - lpState_);
        out = 4.0f * lpState_;
        hpState_ = thisSample;
      }
    } else if (shape_ == TRIANGLE || shape_ == SLOPE) {
      // 2b. Triangle/slope: no jumps, but corners (jumps in slope), fixed
      // with the integrated BLEP.
      float slopeUp = 2.0f, slopeDown = 2.0f;
      if (shape_ == SLOPE) {
        slopeUp = 1.0f / pw;
        slopeDown = 1.0f / (1.0f - pw);
      }
      if (high_ ^ (phase_ < pw)) {
        float t = (phase_ - pw) / frequency;
        float discontinuity = (slopeUp + slopeDown) * frequency;
        thisSample -= thisIntegratedBlepSample(t) * discontinuity;
        nextSample -= nextIntegratedBlepSample(t) * discontinuity;
        high_ = phase_ < pw;
      }
      if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        float t = phase_ / frequency;
        float discontinuity = (slopeUp + slopeDown) * frequency;
        thisSample += thisIntegratedBlepSample(t) * discontinuity;
        nextSample += nextIntegratedBlepSample(t) * discontinuity;
        high_ = true;
      }
      nextSample += high_ ? phase_ * slopeUp : 1.0f - (phase_ - pw) * slopeDown;
      out = 2.0f * thisSample - 1.0f;
    } else {
      // 2c. Square: a jump up at the pulse width, a jump down at the reset.
      if (high_ ^ (phase_ >= pw)) {
        float t = (phase_ - pw) / frequency;
        thisSample += thisBlepSample(t);
        nextSample += nextBlepSample(t);
        high_ = phase_ >= pw;
      }
      if (phase_ >= 1.0f) {
        phase_ -= 1.0f;
        float t = phase_ / frequency;
        thisSample -= thisBlepSample(t);
        nextSample -= nextBlepSample(t);
        high_ = false;
      }
      nextSample += phase_ < pw ? 0.0f : 1.0f;
      // Filtered variants: the one-pole coefficient follows the
      // frequency, so the tone is the same at every pitch.
      if (shape_ == SQUARE_TRIANGLE) {
        thisSample = 128.0f * (thisSample - 0.5f);
        lpState_ += frequency * 0.0625f * (thisSample - lpState_);
        out = lpState_;
      } else if (shape_ == SQUARE_DARK) {
        thisSample = 4.0f * (thisSample - 0.5f);
        lpState_ += frequency * 2.0f * (thisSample - lpState_);
        out = lpState_ * 0.67f; // Plaits' level was about ±1.5
      } else if (shape_ == SQUARE_BRIGHT) {
        thisSample = 2.0f * thisSample - 1.0f;
        lpState_ += frequency * 2.0f * (thisSample - lpState_);
        out = (thisSample - lpState_) * 0.5f;
      } else {
        out = 2.0f * thisSample - 1.0f;
      }
    }
    nextSample_ = nextSample;
    return out;
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  Shape shape_ = SAW;
  float frequency_ = 0.001f;
  float pw_ = 0.5f;
  float phase_ = 0.5f;
  float nextSample_ = 0.0f;
  float lpState_ = 0.0f;
  float hpState_ = 0.0f;
  bool high_ = false;
  bool needsSync_ = true;
};

} // namespace bb
