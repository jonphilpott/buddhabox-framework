/**
 * @file vactrol.h
 * @brief Detailed vactrol model from Streams: control in, gain + cutoff out.
 *
 * A vactrol is an LED shining on a light-dependent resistor: the part
 * behind Buchla-style low-pass gates. Feed this model a control signal;
 * it returns the gain and filter cutoff a real vactrol would give, to
 * drive your own VCA and filter.
 *
 * Compared with the envelope inside VactrolLpg (from Plaits), this adds:
 * - Memory. A vactrol that has been lit a lot becomes more sensitive (it
 *   reacts faster), and recovers over about a minute.
 * - Second-order response, with a little overshoot on fast rises.
 * - A saturating ("Gompertz") curve from light to gain, so the gain snaps
 *   open and has a long, soft tail.
 * - A plucked mode: each rising edge "plucks" the vactrol, for repeatable
 *   percussive decays.
 *
 * HOW IT WORKS: the LED's light (the input) charges a first-order lag
 * (state 0) and a second lag (state 1), with attack and decay rates chosen
 * by whether the light is rising or falling and by how sensitised the cell
 * is (state 2). The lag state then goes through the saturating curve to
 * give the gain; the cutoff follows the square of the state.
 *
 * Original: Streams, fixed-point at 31.089 kHz, driving an analog VCA and
 * filter. This port is float, with time constants in seconds. The
 * Gompertz curve (two exp() calls and a pow() in Partials) is read from a
 * flash table (lutGompertz) so it is cheap enough to run every sample.
 *
 * Derived from Streams, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/luts.h"

namespace bb {

/**
 * Vactrol (LED + photocell) response model.
 *
 * Example — a plucked low-pass gate from Svf + Vactrol:
 *   Vactrol vac;
 *   Svf lpf;
 *   vac.setPlucked(true);
 *   vac.setDecay(0.8f);
 *
 *   float gain = vac.process(gate ? 1.0f : 0.0f);
 *   lpf.setCoefficients(0.002f + 0.2f * vac.cutoff(), 0.7f,
 *                       TanApprox::Fast);
 *   float out = lpf.process(osc.process()).lp * gain;
 */
class Vactrol {
public:
  Vactrol() { init(); }

  /**
   * Reset the cell (unlit, unsensitised).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    for (float &s : state_)
      s = 0.0f;
    excite_ = 0.0f;
    gate_ = false;
    gain_ = cutoff_ = 0.0f;
    // Streams' fixed rates, per sample at 31089 Hz, converted.
    const float ratio = 31089.0f / sampleRate;
    overshootCoefficient_ =
        rescaleCoefficient(67976239.0f / 2147483648.0f, ratio);
    sensitiseCoefficient_ =
        rescaleCoefficient(138132.0f / 2147483648.0f, ratio);
    desensitiseCoefficient_ =
        rescaleCoefficient(1151.0f / 2147483648.0f, ratio);
    update();
  }

  /** @param seconds  Attack time; Streams' range is 0.01 to 0.5 s */
  void setAttack(float seconds) {
    attack_ = seconds;
    update();
  }

  /** @param seconds  Decay time; Streams' range is 0.05 to 5 s */
  void setDecay(float seconds) {
    decay_ = seconds;
    update();
  }

  /**
   * @param on  true: each rising edge of the input starts a full decay,
   *            like striking a low-pass gate. false: the vactrol follows
   *            the input level.
   */
  void setPlucked(bool on) {
    plucked_ = on;
    update();
  }

  /**
   * Advance one sample.
   * @param control  The light level, 0.0 to 1.0 (negatives count as 0)
   * @return         The gain, 0.0 to 1.0; cutoff() then holds the matching
   *                 filter cutoff, 0.0 to 1.0
   */
  float process(float control) {
    float excite = control > 0.0f ? control : 0.0f;
    if (plucked_) {
      processPlucked(excite);
    } else {
      processFollow(excite);
    }
    return gain_;
  }

  /** @return  The gain from the last process() call, 0..1 */
  float gain() const { return gain_; }

  /**
   * @return  The cutoff from the last process() call, 0..1. Map it to Hz
   *          however suits your filter, e.g.
   *          20.0f * fastExp2(cutoff * 9.97f) for 20 Hz to 20 kHz.
   */
  float cutoff() const { return cutoff_; }

private:
  void processFollow(float excite) {
    // 1. Smooth falling edges of the input, so a short trigger still
    // lights the LED long enough for the cell to react.
    float error = excite - excite_;
    excite_ += error * (error > 0.0f ? 0.5f : 2.0f * decay_c_);
    float input = excite_ * 0.5f;
    // 2. Overshoot helper: a fast lag of the input.
    state_[3] += (input - state_[3]) * overshootCoefficient_;
    // 3. First- and second-order lags. Rising: slower when the cell is
    // not sensitised; falling: slow tail.
    error = input - state_[0];
    float c;
    if (error > 0.0f) {
      if (state_[1] > 0.0f) {
        c = attack_c_ * (1.0f + (1.0f - state_[2]) * (255.0f / 64.0f));
      } else {
        c = fastAttack_c_;
      }
    } else {
      c = state_[1] < 0.0f ? decay_c_ : fastDecay_c_;
    }
    state_[0] += error * c;
    state_[1] += (error - state_[1]) * c;
    // 4. Memory: brightly lit cells become sensitised within about a
    // second, and recover over a minute.
    float sensitivity = state_[0] > 0.125f ? 1.0f : state_[0] * 8.0f;
    error = sensitivity - state_[2];
    state_[2] += error * (error > 0.0f ? sensitiseCoefficient_
                                       : desensitiseCoefficient_);
    // 5. Saturating response, with a little overshoot.
    float index =
        clampf(state_[0] * 0.5f + state_[3] * state_[1], 0.0f, 0.4999f);
    gain_ = index < 0.25f ? gompertz(index * 4.0f) : 1.0f;
    cutoff_ = minf(index * 4.0f, 1.0f);
    cutoff_ *= cutoff_;
  }

  void processPlucked(float excite) {
    // 1. Schmitt trigger on the input (thresholds as in Streams).
    if (!gate_) {
      if (excite > kThreshold) {
        gate_ = true;
        state_[0] = state_[1] = 1.0f;
      }
    } else if (excite < kThreshold * 0.5f) {
      gate_ = false;
    }
    // 2. Two decaying pulses: a fast one for the filter, a slower one for
    // the VCA.
    state_[0] -= state_[0] * fastDecay_c_;
    state_[1] -= state_[1] * decay_c_;
    float error = state_[0] - state_[2];
    state_[2] += error * (error > 0.0f ? fastAttack_c_ : fastDecay_c_);
    // 3. VCA envelope; the tail lengthens as it gets quieter.
    error = state_[1] - state_[3];
    float c = error > 0.0f ? fastAttack_c_ : decay_c_;
    c = c * 0.5f + c * fabsf(error);
    state_[3] += error * c;
    cutoff_ = state_[2];
    gain_ = gompertz(state_[3] * 0.375f);
  }

  /** Light-to-gain S-curve, 0..1, from the flash table. */
  static float gompertz(float x) {
    return interpolate(lutGompertz(), clampf(x, 0.0f, 1.0f), 256.0f);
  }

  /** One-pole coefficient for a time constant in seconds (uses expf). */
  float coefficient(float seconds) const {
    return 1.0f - expf(-1.0f / (seconds * sampleRate_));
  }

  void update() {
    // As in Streams: the "fast" rates are 10x the normal ones; the decay
    // is halved in follow mode, and the fast attack sharpened when
    // plucked.
    attack_c_ = coefficient(attack_);
    fastAttack_c_ = coefficient(attack_ * 0.1f);
    decay_c_ = coefficient(decay_);
    fastDecay_c_ = coefficient(decay_ * 0.1f);
    if (plucked_) {
      fastAttack_c_ = minf(fastAttack_c_ * 16.0f, 1.0f);
    } else {
      decay_c_ *= 0.5f;
    }
  }

  static constexpr float kThreshold = 0.1f; // Streams: ~0.83 V of 10 V
  float sampleRate_ = SAMPLE_RATE;
  float attack_ = 0.01f, decay_ = 0.5f;
  bool plucked_ = false;
  float attack_c_ = 0.0f, fastAttack_c_ = 0.0f;
  float decay_c_ = 0.0f, fastDecay_c_ = 0.0f;
  float overshootCoefficient_ = 0.0f, sensitiseCoefficient_ = 0.0f;
  float desensitiseCoefficient_ = 0.0f;
  float state_[4] = {0, 0, 0, 0};
  float excite_ = 0.0f;
  bool gate_ = false;
  float gain_ = 0.0f, cutoff_ = 0.0f;
};

} // namespace bb
