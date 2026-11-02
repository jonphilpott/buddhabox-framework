/**
 * @file range.h
 * @brief Range: rescale a control signal, in a straight line or in octaves.
 *
 * Most sources speak in small numbers: a knob gives 0 to 1, an LFO -1 to
 * 1. Most destinations want something else: a filter wants 80 to 8000 Hz,
 * a delay wants 0.05 to 0.6 seconds. Range sits between them.
 *
 *   Knob (0..1) → Range (80..8000 Hz, exponential) → Svf.setFrequency
 *
 * LINEAR OR EXPONENTIAL?
 * ──────────────────────
 * Linear: equal turns of the knob give equal steps in Hz. Halfway on an
 * 80 to 8000 Hz knob is 4040 Hz, so everything below 1 kHz is squashed
 * into the first eighth of the turn.
 *
 * Exponential: equal turns give equal *ratios*. 80 to 8000 Hz is about
 * 6.6 octaves, so each 15% of the turn is one octave, and halfway is
 * 800 Hz. Ears hear pitch and brightness this way, so use exponential for
 * any frequency. Use linear for levels, mix amounts and times.
 *
 * HOW IT WORKS:
 * 1. Turn the input into a position t from 0 to 1 within its range,
 *    clamped so a stray value cannot overshoot the output range.
 * 2. Linear:      out = lo + t * (hi - lo)
 *    Exponential: out = lo * 2^(t * octaves),  octaves = log2(hi / lo)
 *    The logarithm is slow, so the setters work it out once; process()
 *    pays only for fastExp2() (a few dozen cycles).
 *
 * Swap lo and hi to turn a knob backwards: setOutput(1.0f, 0.0f).
 *
 * Example usage: a cutoff knob
 *   AnalogInput knob(PA1);
 *   Range cutoff;
 *   cutoff.setOutput(80.0f, 8000.0f);
 *   cutoff.setExponential(true);
 *   // In the control loop:
 *   knob.update();
 *   filter.setFrequency(cutoff.process(knob.read()), 0.7f);
 *
 * Example usage: an LFO sweeping 200 to 2000 Hz
 *   Range sweep;
 *   sweep.setInput(-1.0f, 1.0f); // an LFO swings both ways
 *   sweep.setOutput(200.0f, 2000.0f);
 *   sweep.setExponential(true);
 *   float hz = sweep.process(lfo.process());
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Rescales a signal from one range to another, linearly or in octaves.
 *
 * Defaults map 0..1 to 0..1 in a straight line, so a fresh Range passes
 * a knob through unchanged.
 *
 * Example:
 *   Range r;
 *   r.setOutput(110.0f, 880.0f);
 *   r.setExponential(true);
 *   float hz = r.process(0.5f); // 311 Hz: halfway in octaves
 */
class Range {
public:
  /**
   * Rescale one sample.
   *
   * @param in  Input signal; values outside the input range are clamped
   *            to its ends.
   * @return    The value in the output range.
   *
   * Example:
   *   float hz = range.process(knob.read());
   */
  float process(float in) {
    // Step 1: position within the input range, 0 to 1.
    float t = clampf((in - inLo_) * inScale_, 0.0f, 1.0f);
    // Step 2: walk the output range, in octaves or in a straight line.
    if (octaves_ != 0.0f)
      return outLo_ * fastExp2(t * octaves_);
    return outLo_ + t * (outHi_ - outLo_);
  }

  /**
   * Set the range the input is expected to cover.
   *
   * @param lo  Input value that gives the output's lo (default 0.0).
   * @param hi  Input value that gives the output's hi (default 1.0).
   *            Must differ from lo.
   *
   * Example:
   *   range.setInput(-1.0f, 1.0f); // for an LFO or audio-rate source
   */
  void setInput(float lo, float hi) {
    inLo_ = lo;
    // Store 1/(hi - lo) so process() multiplies instead of divides: a
    // divide takes about 14 cycles on the Cortex-M4, a multiply one.
    inScale_ = (hi != lo) ? 1.0f / (hi - lo) : 0.0f;
  }

  /**
   * Set the range the output covers.
   *
   * @param lo  Output at the bottom of the input range (default 0.0).
   * @param hi  Output at the top (default 1.0). hi below lo reverses the
   *            direction.
   *
   * Example:
   *   range.setOutput(80.0f, 8000.0f); // Hz for a filter
   */
  void setOutput(float lo, float hi) {
    outLo_ = lo;
    outHi_ = hi;
    update();
  }

  /**
   * Choose the curve.
   *
   * @param on  true: equal steps of input give equal ratios (octaves),
   *            for frequencies. Needs both output ends above zero;
   *            otherwise Range stays linear. false (default): straight
   *            line, for levels and times.
   *
   * Example:
   *   range.setExponential(true);
   */
  void setExponential(bool on) {
    exponential_ = on;
    update();
  }

private:
  /**
   * Work out the octaves between the output ends, once, here rather than
   * in process(). Zero means "linear": an exponential curve needs both
   * ends positive, since no power of two reaches zero or below.
   */
  void update() {
    bool ok = exponential_ && outLo_ > 0.0f && outHi_ > 0.0f;
    octaves_ = ok ? fastLog2(outHi_ / outLo_) : 0.0f;
  }

  float inLo_ = 0.0f;
  float inScale_ = 1.0f; // 1 / (input hi - input lo)
  float outLo_ = 0.0f;
  float outHi_ = 1.0f;
  float octaves_ = 0.0f; // log2(hi / lo), or 0 for linear
  bool exponential_ = false;
};

} // namespace bb
