/**
 * @file hysteresis_quantizer.h
 * @brief Turns a 0..1 knob into N steps without flicker at the edges.
 *
 * Use it for mode selectors, range switches, or picking from a list with
 * a pot. A plain `(int)(x * n)` flickers between two steps when a noisy
 * pot sits near a boundary; this doesn't.
 *
 * HOW IT WORKS: the boundaries move away from the current step by a
 * fraction of a step (the hysteresis). To change step, the value must go
 * a little past the halfway point, so small noise near a boundary has no
 * effect.
 *
 * Derived from stmlib dsp/hysteresis_quantizer.h (HysteresisQuantizer2),
 * Copyright 2015 Emilie Gillet. MIT licence.
 */

#pragma once

namespace bb {

/**
 * Stepped selector with hysteresis.
 *
 * Example — choose one of 6 scales with a pot:
 *   HysteresisQuantizer scaleKnob;
 *   scaleKnob.init(6);
 *
 *   int scale = scaleKnob.process(pot.read()); // 0..5, no flicker
 */
class HysteresisQuantizer {
public:
  HysteresisQuantizer() { init(); }

  /**
   * Configure the steps.
   * @param numSteps    How many steps (at least 1)
   * @param hysteresis  In steps: 0.25 = a quarter-step past halfway
   * @param symmetric   false: each step gets 1/n of the range; true: the
   *                    first step is at exactly 0, the last at exactly 1
   */
  void init(int numSteps = 2, float hysteresis = 0.25f,
            bool symmetric = false) {
    numSteps_ = numSteps < 1 ? 1 : numSteps;
    hysteresis_ = hysteresis;
    scale_ = static_cast<float>(symmetric ? numSteps_ - 1 : numSteps_);
    offset_ = symmetric ? 0.0f : -0.5f;
    value_ = 0;
  }

  /**
   * Quantise a value.
   * @param value  0.0 to 1.0
   * @return       The step, 0 to numSteps - 1
   */
  int process(float value) {
    value = value * scale_ + offset_;
    float sign = value > static_cast<float>(value_) ? -1.0f : 1.0f;
    int q = static_cast<int>(value + sign * hysteresis_ + 0.5f);
    if (q < 0)
      q = 0;
    if (q > numSteps_ - 1)
      q = numSteps_ - 1;
    value_ = q;
    return q;
  }

  /** @return  The current step */
  int value() const { return value_; }

  /** @return  The number of steps */
  int numSteps() const { return numSteps_; }

private:
  int numSteps_ = 2;
  float hysteresis_ = 0.25f;
  float scale_ = 2.0f, offset_ = -0.5f;
  int value_ = 0;
};

} // namespace bb
