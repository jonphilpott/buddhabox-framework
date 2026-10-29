/**
 * @file sample_and_hold.h
 * @brief Sample and Hold — captures and freezes an input on a trigger.
 *
 * A Sample and Hold (S&H) module monitors an input signal and, when a
 * trigger fires, captures ("samples") the current value and holds it
 * steady until the next trigger arrives.
 *
 * The held value can be used as a control voltage for anything: pitch,
 * filter cutoff, LFO rate, reverb depth. The classic west-coast patch is:
 *
 *   WhiteNoise → SampleAndHold (triggered by TempoClock) → pitch
 *
 * This produces random stepped voltages — each tick picks a random note.
 * The result sounds like an analogue sequencer being driven by chaos.
 *
 * HOW IT WORKS:
 * ─────────────
 * Between triggers, process() simply returns the last captured value.
 * On a trigger, it reads the current input and stores it as the new
 * held value. Nothing more complicated than that.
 *
 * TRIGGER CONVENTIONS:
 * ────────────────────
 * Triggers in BuddhaBox are typically bool values — true for one sample,
 * then false until the next event. TempoClock::process() returns true
 * exactly once per tick, making it a natural clock source for S&H.
 *
 * Example usage — random stepped pitch from noise:
 *   WhiteNoise noise;
 *   SampleAndHold sh;
 *   TempoClock clock(80.0f, 2);  // 80 BPM, eighth notes
 *   Oscillator osc;
 *
 *   // In audio callback:
 *   bool tick = clock.process();
 *   float cv = sh.process(noise.process(), tick);
 *   // Map noise (-1 to +1) to a useful frequency range (200–800 Hz)
 *   osc.setFrequency(mapf(cv, -1.0f, 1.0f, 200.0f, 800.0f));
 *   float out = osc.process();
 *
 * Example usage — manual trigger and separate value read:
 *   SampleAndHold sh;
 *
 *   if (clock.process()) {
 *       sh.trigger(lfo.process());  // Capture LFO on each clock tick
 *   }
 *   float held = sh.getValue();  // Use the held value elsewhere
 *   filter.setFrequency(mapf(held, -1.0f, 1.0f, 200.0f, 5000.0f));
 *
 * Example usage — with slew for smooth stepped CVs (portamento-like):
 *   WhiteNoise noise;
 *   SampleAndHold sh;
 *   SlewLimiter slew;
 *   slew.setTime(0.03f);  // 30 ms glide between steps
 *
 *   // In audio callback:
 *   float raw = sh.process(noise.process(), clock.process());
 *   float smoothed = slew.process(raw);
 *   osc.setFrequency(mapf(smoothed, -1.0f, 1.0f, 200.0f, 800.0f));
 */

#pragma once

namespace bb {

/**
 * Sample and Hold — captures and freezes an input on a trigger.
 *
 * See the file header above for how it works and usage examples.
 */
class SampleAndHold {
public:
  /**
   * Process one sample with an inline trigger.
   *
   * The most convenient form for use inside an audio callback. If trigger
   * is true, the input is captured and becomes the new held value. The
   * held value is returned either way.
   *
   * @param input    The signal to potentially capture. Any float range.
   * @param trigger  true = capture input now, false = hold previous value.
   * @return         The currently held value.
   *
   * Example:
   *   float cv = sh.process(noise.process(), clock.process());
   */
  float process(float input, bool trigger) {
    if (trigger)
      held_ = input;
    return held_;
  }

  /**
   * Manually trigger a capture from an external source.
   *
   * Use this when you need to capture from a signal not available inline.
   * Pair with getValue() to read the held output separately.
   *
   * @param input  Value to capture and hold.
   *
   * Example:
   *   if (clock.process())
   *       sh.trigger(lfo.process());
   */
  void trigger(float input) { held_ = input; }

  /**
   * Read the currently held value without triggering.
   *
   * @return  The last captured value. Zero until first trigger.
   *
   * Example:
   *   float held = sh.getValue();
   */
  float getValue() const { return held_; }

  /**
   * Preset the held value without triggering.
   *
   * Useful to avoid a "zero until first clock tick" startup condition.
   *
   * @param v  Initial held value.
   *
   * Example:
   *   sh.reset(0.5f);  // Pre-load a specific CV
   */
  void reset(float v = 0.0f) { held_ = v; }

private:
  // The last captured value. Returned by process() and getValue()
  // until the next trigger fires.
  float held_ = 0.0f;
};

} // namespace bb
