/**
 * @file bouncing_ball.h
 * @brief An envelope that falls and bounces like a dropped ball (Peaks).
 *
 * The output drops, bounces lower and faster each time, and comes to rest.
 * Use it for accelerating ratchets, a lively decaying envelope, or (at
 * high gravity) a buzzing, pitch-rising "boing".
 *
 * HOW IT WORKS: a tiny physics simulation, one step per sample. Gravity
 * pulls the velocity down; the velocity moves the position. When the ball
 * hits the floor (0) or the ceiling (1) its velocity reverses, multiplied
 * by the elasticity, so each bounce loses energy.
 *
 * Original: Peaks, 32-bit fixed point at 48 kHz. This port is float, with
 * gravity and velocity in units per second.
 *
 * Derived from Peaks, Copyright 2013 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Bouncing-ball envelope.
 *
 * Example — a bouncing filter ping:
 *   BouncingBall ball;
 *   ball.setGravity(0.4f);
 *   ball.setBounce(0.8f);
 *
 *   float env = ball.process(gateRising);
 *   float s = filter.process(osc.process()).lp * env;
 */
class BouncingBall {
public:
  BouncingBall() { init(); }

  /**
   * Reset the ball (at rest on the floor).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    position_ = velocity_ = 0.0f;
    previousGate_ = false;
    setGravity(gravityKnob_);
  }

  /**
   * How fast the ball falls: from a slow, floaty drop (about 0.4 s from
   * the top) to audio-rate buzzing. Uses powf: control rate.
   * @param x  0.0 to 1.0
   */
  void setGravity(float x) {
    gravityKnob_ = clampf(x, 0.0f, 1.0f);
    // Peaks' table: a fall-time curve from 1 ms to 10 s (shaped by a 0.3
    // power), scaled so the time to fall the full height is about 4.4% of
    // that. (Partials computed this in double; float is plenty here.)
    const float gamma = 0.3f;
    const float a = powf(0.001f, gamma), b = powf(10.0f, gamma);
    float t = powf(a + (b - a) * gravityKnob_, 1.0f / gamma);
    float fallTime = 0.0442f * t * sampleRate_; // in samples
    // From height 1 at rest: h = g t^2 / 2, so g = 2 / t^2 per sample^2.
    gravity_ = 2.0f / (fallTime * fallTime);
  }

  /**
   * How much speed survives each bounce.
   * @param x  0.0 = stops dead, 1.0 = bounces almost forever
   */
  void setBounce(float x) {
    float b = 1.0f - clampf(x, 0.0f, 1.0f);
    elasticity_ = 1.0f - b * b;
  }

  /** @param x  Height the ball is dropped from, 0.0 to 1.0 */
  void setHeight(float x) { height_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  Initial velocity, -1.0 to 1.0 (positive throws it up) */
  void setVelocity(float x) { initialVelocity_ = clampf(x, -1.0f, 1.0f); }

  /** Drop the ball now. */
  void trigger() {
    position_ = height_;
    // Peaks' full-scale throw: about 23 heights per second.
    velocity_ = initialVelocity_ * 23.4f / sampleRate_;
  }

  /**
   * Advance one sample.
   * @param gate  A rising edge drops the ball (or call trigger())
   * @return      The ball's height, 0.0 to 1.0
   */
  float process(bool gate = false) {
    if (gate && !previousGate_)
      trigger();
    previousGate_ = gate;
    velocity_ -= gravity_;
    position_ += velocity_;
    if (position_ < 0.0f) {
      position_ = 0.0f;
      velocity_ = -velocity_ * elasticity_;
    }
    if (position_ > 1.0f) {
      position_ = 1.0f;
      velocity_ = -velocity_ * elasticity_;
    }
    return position_;
  }

private:
  float sampleRate_ = SAMPLE_RATE;
  float gravityKnob_ = 0.5f, gravity_ = 0.0f;
  float elasticity_ = 0.75f;
  float height_ = 1.0f, initialVelocity_ = 0.0f;
  float position_ = 0.0f, velocity_ = 0.0f;
  bool previousGate_ = false;
};

} // namespace bb
