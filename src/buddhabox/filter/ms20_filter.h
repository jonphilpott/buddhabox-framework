/**
 * @file ms20_filter.h
 * @brief MS-20 low-pass filter: resonance that screams but keeps the bass.
 *
 * The Korg MS-20's filter (the "Korg35"): 12 dB per octave.
 *
 * WHY NOT THE MOOG LADDER? A ladder's resonance feeds the output back
 * to the input upside down. At the resonant peak that adds up, but at
 * low frequencies it subtracts: turning resonance up thins the bass,
 * down to a quarter of the level at full resonance.
 *
 * The MS-20 takes its feedback through a high-pass filter first. Bass
 * never reaches the feedback path, so it passes at full level however
 * high the resonance goes. Only the band around the cutoff is boosted.
 * A diode clipper in the loop keeps the resonance from running away, so
 * at high settings it growls and screams rather than whistling cleanly.
 *
 * HEADROOM: the clipper only leaves room for the resonance if the input
 * is quiet, so process() turns the input down to a quarter and the
 * output back up 4x. Bass comes out at the level it went in, but a
 * screaming peak reaches about +-2.5 (a full-scale saw at full resonance):
 * tame it after the filter.
 *
 * HOW THIS MODEL WORKS (after Will Pirkle's "Virtual Analog Korg35"
 * application note, built on Vadim Zavalishin's zero-delay-feedback
 * filters):
 *
 *   in ─► low-pass 1 ─► (+) ─► clip ─► low-pass 2 ─┬─► out
 *                        ▲                         │
 *                        └── high-pass ◄── x K ◄───┘
 *
 * Each block is a one-pole filter. Feedback in a digital filter usually
 * needs a one-sample delay, which detunes the cutoff and makes high
 * resonance unstable. The "zero-delay feedback" (ZDF) method solves the
 * loop's equation for the current sample instead: alpha0_ below is that
 * solution. The filter stays in tune and stable while the cutoff sweeps.
 *
 * COST: about 72 cycles a sample on the F411 (3.3% of the budget; the Moog
 * ladder is 44), 40 bytes. setFrequency() calls tanf() and costs 233
 * cycles: call it at control rate, or every few samples for an
 * envelope sweep (examples/acid does every 8th, 29 cycles a sample).
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include <math.h>

namespace bb {

/**
 * MS-20 style 2-pole low-pass filter with resonance that keeps the bass.
 *
 * Example — a resonant sweep on a saw:
 *   Ms20Filter filter;
 *   filter.setResonance(0.8f);
 *
 *   // Every few samples (setFrequency() uses tanf()):
 *   filter.setFrequency(cutoffHz);
 *   // Every sample:
 *   float out = filter.process(saw.process());
 */
class Ms20Filter {
public:
  Ms20Filter() {
    setFrequency(1000.0f);
    setResonance(0.0f);
  }

  /**
   * Set the cutoff. Uses tanf(): control rate.
   *
   * @param hz  Cutoff in Hz, clamped to 20 Hz .. 0.45 x the sample rate.
   *            Repeating the last value costs nothing.
   *
   * Example:
   *   filter.setFrequency(800.0f);
   */
  void setFrequency(float hz) {
    hz = clampf(hz, 20.0f, SAMPLE_RATE * 0.45f);
    if (hz == lastHz_)
      return;
    lastHz_ = hz;
    // "Pre-warping": the bilinear transform squeezes the whole analog
    // frequency range into 0..Nyquist, which would pull a high cutoff
    // flat. tan() stretches it back, so the cutoff lands where asked.
    g_ = tanf(PI_F * hz / SAMPLE_RATE);
    update();
  }

  /**
   * Set the resonance.
   *
   * @param res  0.0 = none, 0.7 = a strong peak, 1.0 = self-oscillation
   *             (a growling tone at the cutoff, held in check by the
   *             clipper)
   *
   * Example:
   *   filter.setResonance(0.85f); // acid territory
   */
  void setResonance(float res) {
    // In theory K = 2 is where the loop starts to oscillate on its own,
    // but the clipper takes a little gain away, so the top of the range
    // goes to 2.1: at 1.0 the filter keeps ringing (measured on the
    // host), and at 0.95 it still doesn't.
    k_ = 2.1f * clampf(res, 0.0f, 1.0f);
    update();
  }

  /**
   * Filter one sample.
   *
   * @param in  Input, about -1.0 to 1.0
   * @return    Low-passed output; bass keeps its level at any resonance,
   *            and resonant peaks reach about +-2.5, so follow it with
   *            something that tames them (softLimitClip(), Overdrive)
   *
   * Example:
   *   float out = filter.process(in);
   */
  float process(float in) {
    // 0. Headroom. The clipper in step 3 limits the loop to about +-1, so
    //    a full-scale input would fill it and leave the resonance no
    //    room: measured with a +-1 saw, full resonance added only 3.5 dB
    //    at the cutoff. A quarter of the level leaves 12 dB for the peak
    //    (28 dB of boost measured), and step 6 turns it back up.
    in *= kHeadroom;

    // 1. First low-pass (a zero-delay one-pole: v is the step towards
    //    the input, state z holds what the integrator remembers).
    float v = (in - z1_) * a_;
    float lp1 = v + z1_;
    z1_ = lp1 + v;

    // 2. Close the loop: what the second low-pass and the high-pass
    //    will feed back, solved for this sample by alpha0_.
    float u = alpha0_ * (lp1 + lp2Beta_ * z2_ + hpBeta_ * z3_);

    // 3. The diode clipper: keeps high resonance bounded and gives the
    //    MS-20 its snarl.
    u = softLimitClip(u);

    // 4. Second low-pass: the output.
    v = (u - z2_) * a_;
    float out = v + z2_;
    z2_ = out + v;

    // 5. The feedback high-pass hears K x the output. Only its state is
    //    needed (it is fed back through hpBeta_ next sample).
    v = (k_ * out - z3_) * a_;
    z3_ = v + z3_ + v;

    // 6. Undo step 0: bass comes out at the level it went in.
    return out * (1.0f / kHeadroom);
  }

  /** Clear the filter's memory (silence, no ringing). */
  void clear() { z1_ = z2_ = z3_ = 0.0f; }

private:
  static constexpr float kHeadroom = 0.25f; // input level inside the loop

  /** Recompute the loop's coefficients from g_ and k_. */
  void update() {
    // a: each one-pole's gain, G = g / (1 + g).
    a_ = g_ / (1.0f + g_);
    lp2Beta_ = (k_ - k_ * a_) / (1.0f + g_);
    hpBeta_ = -1.0f / (1.0f + g_);
    alpha0_ = 1.0f / (1.0f - k_ * a_ + k_ * a_ * a_);
  }

  float g_ = 0.0f, a_ = 0.0f, k_ = 0.0f;
  float lp2Beta_ = 0.0f, hpBeta_ = 0.0f, alpha0_ = 1.0f;
  float z1_ = 0.0f, z2_ = 0.0f, z3_ = 0.0f; // the three one-poles' states
  float lastHz_ = -1.0f;
};

} // namespace bb
