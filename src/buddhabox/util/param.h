/**
 * @file param.h
 * @brief Param: a value your own code sets and a patch's cables read.
 *
 * A generated patch (patch_graph.h) rewrites every setter and event that
 * a cable drives, so main.cpp cannot also call them: the cable wins
 * within a few samples. Param is the meeting point instead. Place it in
 * the patch like a knob without a pin, cable it wherever the value should
 * go (through a Range, mixed with an LFO, into a trigger), and set it from
 * main.cpp:
 *
 *   patch::note.setValue(220.0f);  // a value: pitch, depth, anything
 *   patch::strike.pulse();         // a trigger: one sample of 1.0
 *
 * SAFE FROM loop() AND FROM THE AUDIO CALLBACK
 * ────────────────────────────────────────────
 * loop() runs in the main program; the patch runs in the audio
 * interrupt, which can cut in at any instruction. Param stores only a
 * float and a bool, each written in a single instruction on the
 * Cortex-M4, so the interrupt never sees half a write. Both are
 * `volatile`, which tells the compiler another context changes them, so
 * it must reread memory every time instead of keeping a stale copy in a
 * register.
 *
 * WHY pulse() AND NOT setValue(1) THEN setValue(0)?
 * ─────────────────────────────────────────────────
 * From loop(), the two writes can land between two audio samples, and
 * the patch never sees the 1. pulse() raises a flag that only the audio
 * side clears, after reporting 1.0 for exactly one sample. A trigger
 * cable fires on that rising edge.
 *
 * Example usage: a sequencer in loop() playing a patch
 *   // patch.json: "note" (Param) -> osc.setFrequency,
 *   //             "strike" (Param) -> env.trigger
 *   void loop() {
 *     patch::control();
 *     static uint32_t last = 0;
 *     if (millis() - last >= 250) {
 *       last = millis();
 *       patch::note.setValue(notes[step++ & 3]);
 *       patch::strike.pulse();
 *     }
 *   }
 */

#pragma once

namespace bb {

/**
 * A value set by code and read by a patch. Starts at 0 unless given a
 * starting value with setValue().
 *
 * Example:
 *   Param depth;
 *   depth.setValue(0.3f);         // from loop() or the audio callback
 *   float d = depth.process();    // in the patch: 0.3
 */
class Param {
public:
  /**
   * Read the value for one sample. A pending pulse() reads as 1.0 for
   * this one sample, then the value returns.
   *
   * @return  The value, or 1.0 on a pulse.
   *
   * Example:
   *   float v = param.process();
   */
  float process() {
    if (pulse_) {
      pulse_ = false; // only the audio side clears it
      return 1.0f;
    }
    return value_;
  }

  /**
   * Set the value. Takes effect from the next sample (or the next
   * control update, for a cable into a setter).
   *
   * @param v  Any float: a frequency, a 0..1 level, a scale step.
   *
   * Example:
   *   patch::note.setValue(330.0f);
   */
  void setValue(float v) { value_ = v; }

  /**
   * Send a trigger: process() returns 1.0 for one sample. Safe to call
   * from loop(). Calls closer together than one sample merge into one.
   *
   * Example:
   *   patch::strike.pulse(); // fires anything cabled to its trigger
   */
  void pulse() { pulse_ = true; }

private:
  volatile float value_ = 0.0f;
  volatile bool pulse_ = false;
};

} // namespace bb
