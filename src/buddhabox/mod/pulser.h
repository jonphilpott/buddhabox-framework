/**
 * @file pulser.h
 * @brief A timing pulse generator that fires once or repeats, Buchla
 *        Easel style.
 *
 * The pulser is the heartbeat of a Buchla Music Easel patch. It fires a
 * pulse, either once when triggered or repeatedly at a set period. Each
 * pulse typically strikes a low-pass gate, starts an envelope and steps
 * the sequencer. Between pulses it also outputs a falling ramp, from 1.0
 * at the pulse down to 0.0 just before the next, which makes a handy
 * modulation voltage that is locked to the rhythm.
 *
 * HOW IT DIFFERS FROM TempoClock (clock.h): TempoClock thinks in BPM and
 * note divisions and only ticks. Pulser thinks in seconds, can fire just
 * once, and gives the ramp as well.
 *
 * HOW IT WORKS: a phase runs from 0 to 1 over one period. When it wraps,
 * a pulse fires (LOOP), or the pulser stops and waits for trigger()
 * (ONCE).
 *
 *   phase   1 ┤  ╱│  ╱│  ╱│
 *             │ ╱ │ ╱ │ ╱ │
 *           0 ┼╱  │╱  │╱  │
 *   pulses    ^   ^   ^   ^   process() returns true here
 *   value()   is 1 - phase: a falling ramp
 *
 * COST: a few cycles a sample.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * A pulse generator with a period in seconds and a falling-ramp output.
 *
 * Example — strike a low-pass gate twice a second:
 *   Pulser pulser;
 *   pulser.setPeriod(0.5f);
 *
 *   // In audio callback, every sample:
 *   if (pulser.process())
 *     lpg.trigger();
 *
 * Example — one pulse per key press:
 *   pulser.setMode(Pulser::ONCE);
 *   pulser.trigger(); // from a key handler; process() returns true next
 */
class Pulser {
public:
  /** ONCE fires only when trigger() is called; LOOP repeats. */
  enum Mode { ONCE, LOOP };

  Pulser() { init(); }

  /**
   * Reset to the defaults: LOOP mode, a 1-second period, and a pulse
   * waiting to fire on the first process().
   * @param sampleRate  Calls to process() per second (pass the block
   *                    rate if you call it once per block)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    mode_ = LOOP;
    running_ = pending_ = true;
    phase_ = 0.0f;
    setPeriod(1.0f);
  }

  /**
   * Time between pulses in LOOP mode, and the length of the ramp.
   * One division: fine to call every block (e.g. from a knob).
   * @param seconds  Period in seconds; clamped to at least 1 ms
   */
  void setPeriod(float seconds) {
    increment_ = sampleTime_ / maxf(seconds, 0.001f);
  }

  /**
   * Switch between firing once and repeating. Switching to LOOP starts
   * the pulser running from wherever its phase is.
   * @param mode  Pulser::ONCE or Pulser::LOOP
   */
  void setMode(Mode mode) {
    mode_ = mode;
    if (mode == LOOP)
      running_ = true;
  }

  /**
   * Fire a pulse on the next process() and restart the ramp. Works in
   * both modes: in LOOP it also resets the rhythm to start from now.
   */
  void trigger() {
    phase_ = 0.0f;
    running_ = pending_ = true;
  }

  /**
   * Advance one sample.
   * @return  true on the sample a pulse fires, false otherwise
   */
  bool process() {
    // 1. A pulse asked for by trigger() (or init()) fires now.
    bool fired = pending_;
    pending_ = false;
    if (!running_)
      return fired;

    // 2. Move the ramp on. On wrapping, LOOP fires again and ONCE stops,
    //    holding the ramp at its bottom until the next trigger().
    phase_ += increment_;
    if (phase_ >= 1.0f) {
      if (mode_ == LOOP) {
        phase_ -= 1.0f;
        fired = true;
      } else {
        phase_ = 1.0f;
        running_ = false;
      }
    }
    return fired;
  }

  /** @return  The falling ramp: 1.0 at a pulse, down to 0.0 */
  float value() const { return 1.0f - phase_; }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float increment_ = 0.0f; // phase added each sample: 1 / period
  float phase_ = 0.0f;     // 0..1 through the current period
  Mode mode_ = LOOP;
  bool running_ = true; // false once a ONCE pulse has finished
  bool pending_ = true; // a pulse to fire on the next process()
};

} // namespace bb
