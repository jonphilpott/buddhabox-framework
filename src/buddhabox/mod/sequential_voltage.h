/**
 * @file sequential_voltage.h
 * @brief A five-step sequencer, after the Buchla Easel's sequential
 *        voltage source.
 *
 * Holds up to five values and steps through them, one step per call to
 * advance(). On the Easel each step is a slider and the pulser does the
 * stepping; the output usually sets the oscillator's pitch, but it can
 * drive anything: timbre, modulation depth, the pulser's own period.
 *
 * Five steps is short on purpose. Against a pulser running at a different
 * rate, or an envelope whose length doesn't divide evenly, a short
 * pattern drifts in and out of phase with everything else, which is
 * where much of the Easel's character comes from.
 *
 * COST: nothing per sample; it only works when advanced.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Up to five values, stepped through in a loop.
 *
 * Example — a pitch pattern stepped by a pulser:
 *   SequentialVoltage seq;
 *   const float steps[] = {0, 7, 3, 10, 5}; // semitones
 *   for (int i = 0; i < 5; i++)
 *     seq.setStep(i, steps[i]);
 *
 *   // In audio callback:
 *   if (pulser.process())
 *     osc.setFrequency(110.0f * semitonesToRatio(seq.advance()));
 */
class SequentialVoltage {
public:
  /** The Easel's sequencer has five steps. */
  static const int kMaxSteps = 5;

  /**
   * Set one step's value.
   * @param index  Step 0..4 (others are ignored)
   * @param value  Any value: semitones, 0..1 levels, Hz...
   */
  void setStep(int index, float value) {
    if (index >= 0 && index < kMaxSteps)
      steps_[index] = value;
  }

  /**
   * How many steps to loop through, from step 0.
   * @param steps  1..5
   */
  void setLength(int steps) {
    length_ = steps < 1 ? 1 : (steps > kMaxSteps ? kMaxSteps : steps);
    if (step_ >= length_)
      step_ = 0;
  }

  /**
   * Move to the next step, wrapping after the last.
   * @return  The new step's value
   */
  float advance() {
    if (++step_ >= length_)
      step_ = 0;
    return steps_[step_];
  }

  /** Make the next advance() land on step 0. */
  void reset() { step_ = length_ - 1; }

  /** @return  The current step's value, without advancing */
  float value() const { return steps_[step_]; }

  /** @return  The current step number, 0..length - 1 */
  int step() const { return step_; }

private:
  float steps_[kMaxSteps] = {};
  int length_ = kMaxSteps;
  // Starts on the last step, so the first advance() plays step 0.
  int step_ = kMaxSteps - 1;
};

} // namespace bb
