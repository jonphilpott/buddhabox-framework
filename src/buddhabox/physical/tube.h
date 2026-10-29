/**
 * @file tube.h
 * @brief Elements' reed-and-tube wind model.
 *
 * Blow noise (or any "breath" signal) into it and it produces a reedy,
 * clarinet-like tone at the tube's pitch.
 *
 * HOW IT WORKS:
 * - A delay line one period long stands in for the air column: pressure
 *   waves travel down it and come back.
 * - At the mouthpiece, the difference between breath pressure and the
 *   returning wave goes through a "reed" curve (a simple quadratic). The
 *   reed lets more air through as the pressure difference grows, then
 *   chokes it off. That nonlinear switching sustains the oscillation.
 * - A one-pole low-pass on the output sets the tone (`timbre`).
 * - Like a clarinet it produces mostly odd harmonics.
 *
 * SAMPLE RATE: the loop's behaviour depends on the sample rate, so the
 * model always runs at Elements' 32 kHz internally and its output is
 * linearly interpolated to 44.1 kHz. The tone filter hides the
 * interpolation.
 *
 * MEMORY: the template parameter is the bore length in samples at 32 kHz
 * (a power of two; float storage, since the reed drives the loop to about
 * ±2.5). Tube<1024> (the default, 4 KB) plays down to ~16 Hz; lower
 * notes jump up an octave.
 *
 * Derived from Elements, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include <cstring>

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Reed-driven tube (clarinet-like) model.
 *
 * @tparam SIZE  Bore delay in samples at 32 kHz (power of two)
 *
 * Example — a breathy reed drone:
 *   Tube reed;
 *   WhiteNoise breath;
 *   OnePole breathFilter;
 *   reed.setFrequency(146.8f); // D3
 *   reed.setTimbre(0.4f);
 *   breathFilter.setFrequency(2000.0f);
 *
 *   float b = breathFilter.process(breath.process()).lp;
 *   float s = reed.process(b, 0.8f); // 0.8 = blowing pressure
 */
template <uint16_t SIZE = 1024> class Tube {
  static_assert((SIZE & (SIZE - 1)) == 0, "SIZE must be a power of two");

public:
  /** The model's internal rate (Elements'). */
  static constexpr float kInternalRate = 32000.0f;

  Tube() { init(); }

  /**
   * Silence the bore.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    // Internal steps per host sample: 32000 / 44100 = 0.726.
    step_ = kInternalRate / sampleRate;
    phase_ = 0.0f;
    previous_ = current_ = 0.0f;
    memset(delayLine_, 0, sizeof(delayLine_));
    delayPtr_ = 0;
    zeroState_ = 0.0f;
    poleState_ = 0.0f;
    setFrequency(220.0f);
  }

  /**
   * Set the sounding pitch.
   *
   * Like a clarinet, a reed tube sounds an octave below its delay loop:
   * the reflection at the mouthpiece inverts the wave, so one cycle takes
   * two trips round the loop. Elements' tube therefore sounded an octave
   * below the frequency it was given; here the loop is tuned an octave up
   * so `hz` is the pitch you hear.
   *
   * @param hz  Pitch in Hz
   */
  void setFrequency(float hz) {
    frequency_ = 2.0f * hz / kInternalRate;
    // The loop's two-tap averaging filter adds half a sample of delay;
    // take it off so the tube is in tune.
    float delay = 1.0f / frequency_ - 0.5f;
    while (delay >= static_cast<float>(SIZE))
      delay *= 0.5f;
    delayIntegral_ = static_cast<int32_t>(delay);
    delayFractional_ = delay - static_cast<float>(delayIntegral_);
    updateLpf();
  }

  /** @param x  0..1: higher scales the breath down (softer, muted) */
  void setDamping(float x) { damping_ = 3.6f - clampf(x, 0.0f, 1.0f) * 1.8f; }

  /** @param x  0..1: brightness of the output */
  void setTimbre(float x) {
    timbre_ = clampf(x, 0.0f, 1.0f);
    updateLpf();
  }

  /**
   * Process one sample.
   * @param breath    Excitation (Elements used filtered noise, about ±1)
   * @param envelope  Blowing pressure, 0.0 to 1.0
   * @return          The tube output, scaled by the envelope
   */
  float process(float breath, float envelope) {
    // Run the internal 32 kHz steps that fall within this host sample
    // (zero or one at 44.1 kHz), then interpolate between the last two.
    phase_ += step_;
    while (phase_ >= 1.0f) {
      phase_ -= 1.0f;
      previous_ = current_;
      current_ = tick(breath, envelope);
    }
    return previous_ + (current_ - previous_) * phase_;
  }

private:
  /** One step of the model at 32 kHz. */
  float tick(float breath, float envelope) {
    if (envelope > 1.0f)
      envelope = 1.0f;
    // 1. Pressure difference across the reed. The returning wave
    // (interpolated read) is averaged with the previous one (a simple
    // loss filter).
    float b = breath * damping_ + 0.8f;
    float x0 = delayLine_[(delayPtr_ + delayIntegral_) & kMask];
    float x1 = delayLine_[(delayPtr_ + delayIntegral_ + 1) & kMask];
    float in = x0 + (x1 - x0) * delayFractional_;
    float pressureDelta = -0.95f * (in * envelope + zeroState_) - b;
    zeroState_ = in;
    // 2. Reed: the opening shrinks as the pressure difference grows.
    float reed = pressureDelta * -0.2f + 0.8f;
    float out = clampf(pressureDelta * reed + b, -5.0f, 5.0f);
    // 3. Send the new wave down the tube (write pointer runs backwards).
    delayLine_[delayPtr_] = out * 0.5f;
    delayPtr_ = (delayPtr_ - 1) & kMask;
    // 4. Tone filter.
    poleState_ += lpfCoefficient_ * (out - poleState_);
    return envelope * poleState_;
  }

  void updateLpf() {
    lpfCoefficient_ = frequency_ * (1.0f + timbre_ * timbre_ * 256.0f);
    if (lpfCoefficient_ >= 0.995f)
      lpfCoefficient_ = 0.995f;
  }

  static const int32_t kMask = SIZE - 1;
  float step_ = 1.0f;
  float phase_ = 0.0f;
  float previous_ = 0.0f, current_ = 0.0f;
  float frequency_ = 220.0f / kInternalRate;
  float damping_ = 3.6f;
  float timbre_ = 0.5f;
  float lpfCoefficient_ = 0.0f;
  int32_t delayIntegral_ = 0;
  float delayFractional_ = 0.0f;
  float delayLine_[SIZE];
  int32_t delayPtr_ = 0;
  float zeroState_ = 0.0f;
  float poleState_ = 0.0f;
};

} // namespace bb
