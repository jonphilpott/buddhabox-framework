/**
 * @file slew.h
 * @brief Slew limiter with independent rise and fall times.
 *
 * A slew limiter is essentially a one-pole lowpass filter applied to a
 * control signal rather than an audio signal. It limits how fast a value
 * can change — smoothing out abrupt jumps into gradual transitions.
 *
 * In hardware modular synthesis a slew limiter (also called a "lag
 * processor" or "portamento circuit") sits between a CV source and its
 * destination, rounding off sharp edges. In west-coast synthesis it's
 * used heavily for:
 *
 *   PORTAMENTO: smooth pitch glides between notes
 *     Quantizer output (stepped) → SlewLimiter → oscillator frequency
 *
 *   CV SMOOTHING: remove noise or digitisation steps from sensor readings
 *     AnalogInput → SlewLimiter → filter cutoff
 *
 *   ENVELOPE SHAPING: turn a gate signal into a simple AR envelope
 *     TempoClock tick → gate (0/1) → SlewLimiter → VCA gain
 *
 *   FOLLOWER-STYLE EFFECTS: different response for rising vs falling
 *     Audio envelope follower output → SlewLimiter with fast rise,
 *     slow fall → ducking or sidechain effects
 *
 * HOW IT WORKS:
 * ─────────────
 * Each sample, the slew limiter moves its output toward the target by a
 * fixed fraction — the classic one-pole IIR formula:
 *
 *   output = coeff * output + (1 - coeff) * target
 *
 * When target > output (rising), we use riseCoeff_.
 * When target < output (falling), we use fallCoeff_.
 *
 * The coefficient is derived from the desired time constant:
 *   coeff = exp(-1.0 / (time_seconds * SAMPLE_RATE))
 *
 * A coefficient close to 1.0 means very slow slewing (nearly frozen).
 * A coefficient near 0.0 means almost instant response (no slewing).
 *
 * Example usage — portamento:
 *   SlewLimiter slew;
 *   slew.setTime(0.05f);   // 50 ms glide time (symmetric)
 *
 *   float targetFreq = midiToFreq(currentNote);
 *
 *   // In audio callback:
 *   float freq = slew.process(targetFreq);
 *   osc.setFrequency(freq);
 *   float out = osc.process();
 *
 * Example usage — asymmetric (fast attack, slow release):
 *   SlewLimiter slew;
 *   slew.setSlewRise(0.002f);  // 2 ms rise (punchy)
 *   slew.setSlewFall(0.2f);    // 200 ms fall (lingering)
 *
 *   // In audio callback:
 *   float gate = clock.process() ? 1.0f : 0.0f;
 *   float envCV = slew.process(gate);
 *   float out = osc.process() * envCV;
 *
 * Example usage — sensor smoothing:
 *   SlewLimiter slew;
 *   slew.setTime(0.01f);  // 10 ms smoothing
 *
 *   // In audio callback:
 *   float smooth = slew.process(analogIn.read());
 *   filter.setFrequency(mapf(smooth, 0.0f, 1.0f, 200.0f, 5000.0f));
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Slew limiter with independent rise and fall times.
 *
 * See the file header above for how it works and usage examples.
 */
class SlewLimiter {
public:
  /**
   * Construct a SlewLimiter with a default 10 ms symmetric slew time.
   */
  SlewLimiter() { setTime(0.01f); }

  /**
   * Set a symmetric slew time for both rise and fall.
   *
   * @param seconds  Time constant in seconds. This is the time to reach
   *                 approximately 63% of the way to the target (one RC
   *                 time constant). For a perceptually "complete" glide,
   *                 the actual settling time is about 5x this value.
   *                 0.001 = 1 ms (almost instant)
   *                 0.05  = 50 ms (short glide)
   *                 0.5   = 500 ms (obvious portamento)
   *
   * Example:
   *   slew.setTime(0.1f);  // 100 ms symmetric slew
   */
  void setTime(float seconds) {
    setSlewRise(seconds);
    setSlewFall(seconds);
  }

  /**
   * Set the rise slew time (how fast the output climbs toward a higher target).
   *
   * @param seconds  Rise time constant in seconds. Shorter = snappier attack.
   *
   * Example:
   *   slew.setSlewRise(0.005f);  // Very fast 5 ms rise
   */
  void setSlewRise(float seconds) {
    float samples =
        clampf(seconds * SAMPLE_RATE, 1.0f, (float)SAMPLE_RATE * 30.0f);
    riseCoeff_ = expf(-1.0f / samples);
  }

  /**
   * Set the fall slew time (how fast the output drops toward a lower target).
   *
   * @param seconds  Fall time constant in seconds. Longer = slower decay.
   *
   * Example:
   *   slew.setSlewFall(0.3f);  // 300 ms fall — slow, lingering release
   */
  void setSlewFall(float seconds) {
    float samples =
        clampf(seconds * SAMPLE_RATE, 1.0f, (float)SAMPLE_RATE * 30.0f);
    fallCoeff_ = expf(-1.0f / samples);
  }

  /**
   * Process one input value and return the slewed output.
   *
   * Call this once per sample in your audio callback, or once per block
   * for control-rate signals. The input can be any value — frequency,
   * gain, CV, sensor reading.
   *
   * @param input  The target value to slew toward.
   * @return       The slewed output, which approaches input at the
   *               configured rise or fall rate.
   *
   * Example:
   *   float glideFreq = slew.process(targetFreq);
   *   osc.setFrequency(glideFreq);
   */
  float process(float input) {
    // Choose the appropriate coefficient based on direction of movement.
    // If the input is above the current value, we're rising; use riseCoeff_.
    // If below, we're falling; use fallCoeff_.
    float coeff = input > value_ ? riseCoeff_ : fallCoeff_;

    // One-pole IIR filter: exponentially approach the target.
    // value_ * coeff preserves the old value (inertia).
    // input * (1 - coeff) pulls toward the target (attraction).
    value_ = coeff * value_ + (1.0f - coeff) * input;
    return value_;
  }

  /**
   * Instantly set the current output value without slewing.
   *
   * Useful at startup to pre-load the slew to avoid a startup
   * transient gliding up from zero on the first note.
   *
   * @param v  Value to load into the slew output.
   *
   * Example:
   *   slew.reset(midiToFreq(startNote));
   */
  void reset(float v = 0.0f) { value_ = v; }

  /** @return  The current slewed output value. */
  float getValue() const { return value_; }

private:
  float value_ = 0.0f;     // Current slewed output
  float riseCoeff_ = 0.0f; // Coefficient for upward transitions
  float fallCoeff_ = 0.0f; // Coefficient for downward transitions
};

} // namespace bb
