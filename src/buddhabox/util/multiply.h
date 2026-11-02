/**
 * @file multiply.h
 * @brief Multiply: one signal times another (a VCA, depth control or
 *        ring modulator).
 *
 * In a patch, cables into the same input add up. Adding is mixing.
 * Multiplying is different: one signal sets how much of the other gets
 * through. That one operation does three familiar jobs:
 *
 *   Envelope × audio   a VCA: the envelope shapes the note's volume
 *   Knob × LFO         depth: the knob sets how far the LFO swings
 *   Audio × audio      ring modulation: sum and difference tones
 *
 * Input b defaults to 1.0, so with nothing wired to it, Multiply passes
 * a straight through.
 *
 * Example usage: a knob sets vibrato depth
 *   Multiply depth;
 *   float wobble = depth.process(lfo.process(), knob.read());
 *   osc.setFrequency(220.0f + 10.0f * wobble);
 */

#pragma once

namespace bb {

/**
 * Multiplies two signals. Holds no state, so one instance can be reused
 * freely; it exists as a class so a patch can wire it like any module.
 *
 * Example:
 *   Multiply vca;
 *   float out = vca.process(osc.process(), env.process());
 */
class Multiply {
public:
  /**
   * Multiply one sample of each input.
   *
   * @param a  First signal, any range.
   * @param b  Second signal, any range (default 1.0: a passes through).
   * @return   a × b.
   *
   * Example:
   *   float out = vca.process(audio, envelope);
   */
  float process(float a, float b = 1.0f) { return a * b; }
};

} // namespace bb
