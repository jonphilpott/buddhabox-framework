/**
 * @file lorenz.h
 * @brief Lorenz attractor: a chaotic, never-repeating modulation source.
 *
 * Three smooth, endlessly varying outputs that never quite repeat:
 * Streams' "Lorenz generator" mode. Perfect for slowly evolving drones —
 * patch x to a filter, y to a pan and z to a reverb amount, and the
 * sound wanders without ever looping.
 *
 * HOW IT WORKS: the Lorenz system is three linked equations for a point
 * (x, y, z) moving through space:
 *   dx/dt = sigma (y - x),  dy/dt = x (rho - z) - y,  dz/dt = x y - beta z
 * With the classic constants (sigma 10, rho 28, beta 8/3) the point loops
 * round two "wings", switching between them unpredictably: the famous
 * butterfly. Each sample moves the point one small step (Euler's method);
 * the rate sets how big a step, so how fast it moves.
 *
 * Original: Streams, fixed-point at 31.089 kHz. This port is float, so it
 * runs at any sample rate.
 *
 * Derived from Streams, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Chaotic three-output modulator.
 *
 * Example:
 *   Lorenz chaos;
 *   chaos.setRate(0.2f); // slow drift
 *
 *   chaos.process();
 *   filter.setFrequency(800.0f + 600.0f * chaos.x());
 *   float pan = 0.5f + 0.5f * chaos.y();
 */
class Lorenz {
public:
  Lorenz() { init(); }

  /**
   * Restart from the same starting point (repeatable).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    rateScale_ = 31089.0f / sampleRate;
    x_ = 0.1f;
    y_ = z_ = 0.0f;
    setRate(rate_);
  }

  /**
   * Speed, exponential: from a slow drift (many seconds per loop) to
   * audio-rate buzz. Streams' rate knob: the step doubles every 1/16.5 of
   * the range, reaching 0.02 at the top (at 31.089 kHz).
   * @param x  0.0 to 1.0
   */
  void setRate(float x) {
    rate_ = clampf(x, 0.0f, 1.0f);
    dt_ = 0.02f * fastExp2((rate_ - 1.0f) * 16.5f) * rateScale_;
  }

  /**
   * Advance one sample.
   * @return  x(), about -1.0 to +1.0; y() and z() then hold the others
   */
  float process() {
    const float sigma = 10.0f, rho = 28.0f, beta = 8.0f / 3.0f;
    float x = x_ + dt_ * (sigma * (y_ - x_));
    float y = y_ + dt_ * (x_ * (rho - z_) - y_);
    float z = z_ + dt_ * (x_ * y_ - beta * z_);
    x_ = x;
    y_ = y;
    z_ = z;
    return this->x();
  }

  /** @return  x, about ±1: swings between the two wings */
  float x() const { return x_ * (1.0f / 20.0f); }

  /** @return  y, about ±1: like x but leading it */
  float y() const { return y_ * (1.0f / 27.0f); }

  /** @return  z, about ±1: rises and falls with each loop */
  float z() const { return (z_ - 25.0f) * (1.0f / 25.0f); }

private:
  float rateScale_ = 1.0f;
  float rate_ = 0.5f;
  float dt_ = 0.0f;
  float x_ = 0.1f, y_ = 0.0f, z_ = 0.0f;
};

} // namespace bb
