/**
 * @file mixer.h
 * @brief Mixer: adds up to four signals into one.
 *
 * In a patch, cables into the same input already add together. A Mixer
 * does that adding once, in one place, so the sum can then be cabled to
 * several destinations. Without it, every destination needs its own copy
 * of every cable:
 *
 *   without:  key1, key2, key3 → env.gate    (3 cables)
 *             key1, key2, key3 → hold.trigger (3 more)
 *   with:     key1, key2, key3 → mix (3 cables)
 *             mix → env.gate, mix → hold.trigger (2 cables)
 *
 * Levels: in a patch, each cable's `amount` sets that channel's level, so
 * the Mixer itself has no level settings. Unused inputs default to 0.
 * For more than four channels, cable one Mixer's output into another.
 *
 * Mixing audio: four full-scale signals can sum to 4.0, well past the
 * -1..1 the DAC accepts. Keep the levels down (0.25 each for four) or
 * follow the mix with a limiter.
 *
 * Example usage: three voices into one reverb
 *   Mixer mix;
 *   float dry = mix.process(0.4f * drone.process(),
 *                           0.3f * pluck.process(),
 *                           0.3f * bowl.process());
 */

#pragma once

namespace bb {

/**
 * Adds up to four signals. Holds no state; it is a class so a patch can
 * wire it like any other module.
 *
 * Example:
 *   Mixer mix;
 *   float out = mix.process(a, b); // c and d default to 0
 */
class Mixer {
public:
  /**
   * Add one sample of each input.
   *
   * @param a  First signal, any range (default 0).
   * @param b  Second signal, any range (default 0).
   * @param c  Third signal, any range (default 0).
   * @param d  Fourth signal, any range (default 0).
   * @return   a + b + c + d.
   *
   * Example:
   *   float sum = mix.process(osc1, osc2, noise);
   */
  float process(float a = 0.0f, float b = 0.0f, float c = 0.0f,
                float d = 0.0f) {
    return a + b + c + d;
  }
};

} // namespace bb
