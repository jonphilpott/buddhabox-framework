/**
 * @file vactrol_lpg.h
 * @brief Plaits' low-pass gate with its vactrol envelope.
 *
 * A combined VCA and low-pass filter that open and close together: the
 * "bongo" sound of Buchla-style West Coast synthesis. Ping it with
 * trigger() for a natural percussive decay, or drive it with a level CV.
 *
 * HOW THIS DIFFERS FROM LowPassGate (low_pass_gate.h): LowPassGate is the
 * original BuddhaBox gate — you set a 0..1 control and it slews towards
 * it. VactrolLpg has its own built-in envelope that imitates a vactrol's
 * fast-rise, slow-tail response, and a "colour" control from VCA-like to
 * strongly filtered.
 *
 * HOW IT WORKS:
 * - A real LPG uses a vactrol: an LED shining on a light-dependent
 *   resistor. The resistor responds fast to light coming on and slowly,
 *   with a long tail, to light going off. The envelope imitates that: a
 *   fast rise, then a decay that starts short and slows as the level
 *   falls.
 * - The envelope's single state drives three things at once: the gain,
 *   the filter cutoff (which falls faster than the gain, since it follows
 *   the 4th power of the state), and an "HF bleed" that lets some
 *   unfiltered signal through. As a note decays it gets quieter and duller
 *   together, which is the point of an LPG.
 *
 * COST: the envelope and cutoff update 4000 times a second (once per
 * Plaits block); the gain ramps smoothly in between.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/filter/svf.h"

namespace bb {

/**
 * Pingable vactrol low-pass gate (Plaits).
 *
 * Example — West Coast "bongo":
 *   VactrolLpg lpg;
 *   lpg.setDecay(0.4f);
 *   lpg.setColour(0.7f);
 *
 *   if (clock.process()) {
 *     lpg.trigger();
 *   }
 *   float s = lpg.process(osc.process());
 */
class VactrolLpg {
public:
  VactrolLpg() { init(); }

  /**
   * Reset the gate (closed).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    controlPeriod_ = static_cast<int>(sampleRate / 4000.0f + 0.5f);
    if (controlPeriod_ < 1)
      controlPeriod_ = 1;
    counter_ = 0;
    filter_.init(sampleRate);
    vactrol_ = 0.0f;
    rampUp_ = false;
    gain_ = 0.0f;
    gainIncrement_ = 0.0f;
    hfBleed_ = 0.0f;
  }

  /** @param x  0..1: decay time, from a click to several seconds */
  void setDecay(float x) { decay_ = clampf(x, 0.0f, 1.0f); }

  /** @param x  0..1: 0 = mostly VCA; 1 = more filtering and bleed */
  void setColour(float x) { colour_ = clampf(x, 0.0f, 1.0f); }

  /**
   * Attack speed for trigger(). Plaits ties it to the note's pitch
   * (higher notes snap open faster).
   * @param hz  Your oscillator's frequency in Hz
   */
  void setAttackPitch(float hz) { attackHz_ = hz; }

  /** Ping the gate: open quickly, then decay. */
  void trigger() { rampUp_ = true; }

  /**
   * Ping mode: pass audio through; use trigger() to strike.
   * @param in  Audio input
   * @return    Gated, filtered audio
   */
  float process(float in) { return run(in, false, 0.0f); }

  /**
   * Level mode: the gate follows a CV through the vactrol response.
   * @param in     Audio input
   * @param level  Control level, 0.0 to 1.0
   * @return       Gated, filtered audio
   */
  float process(float in, float level) { return run(in, true, level); }

private:
  float run(float in, bool levelMode, float level) {
    // 1. Control-rate update of the vactrol envelope.
    if (counter_ == 0) {
      counter_ = controlPeriod_;
      updateEnvelope(levelMode, level);
    }
    --counter_;
    // 2. Ramp the gain, filter, and mix back some unfiltered signal.
    gain_ += gainIncrement_;
    const float s = in * gain_;
    const float lp = filter_.process(s).lp;
    return lp + (s - lp) * hfBleed_;
  }

  void updateEnvelope(bool levelMode, float level) {
    // 1. Envelope rates, per update as in Plaits (where one update = one
    // 12-sample block at 48 kHz).
    const float updateTime = controlPeriod_ / sampleRate_;
    const float hf = colour_;
    const float shortDecay =
        200.0f * updateTime * semitonesToRatio(-96.0f * decay_);
    const float decayTail =
        20.0f * updateTime * semitonesToRatio(-72.0f * decay_ + 12.0f * hf) -
        shortDecay;
    // 2. Target: the CV in level mode (soft-compressed as Plaits does),
    // or a fast ramp to 1 after a trigger in ping mode.
    float target;
    if (levelMode) {
      target = 1.3f * level / (0.3f + fabsf(level));
    } else {
      if (rampUp_) {
        vactrol_ += attackHz_ * 2.0f * updateTime;
        if (vactrol_ >= 1.0f) {
          vactrol_ = 1.0f;
          rampUp_ = false;
        }
      }
      target = rampUp_ ? vactrol_ : 0.0f;
    }
    // 3. Vactrol response: fast rise, then a decay that slows down as the
    // state falls (the decayTail term grows as state^4 drops).
    float error = target - vactrol_;
    float state2 = vactrol_ * vactrol_;
    float state4 = state2 * state2;
    float tail = 1.0f - vactrol_;
    float tail2 = tail * tail;
    float coefficient =
        error > 0.0f ? 0.6f : shortDecay + (1.0f - state4) * decayTail;
    vactrol_ += coefficient * error;
    // 4. Outputs. The cutoff formula gives a frequency normalised to
    // Plaits' 48 kHz. (It uses the *updated* state, unlike `tail` above.)
    float v2 = vactrol_ * vactrol_;
    float frequency48k = 0.003f + 0.3f * v2 * v2 + hf * 0.04f;
    hfBleed_ = (tail2 + (1.0f - tail2) * hf) * hf * hf;
    gainIncrement_ = (vactrol_ - gain_) / controlPeriod_;
    setCutoff(frequency48k);
  }

  /**
   * Plaits used the cheap "dirty" tan() approximation, which puts the
   * cutoff lower than nominal at high settings. To keep that sound at any
   * rate: g = dirty(f) is the coefficient Plaits had at 48 kHz, atan(g)
   * gives back the true cutoff it produced, and tan() of that at the
   * actual rate gives the coefficient we need.
   */
  void setCutoff(float frequency48k) {
    float g = tanApprox(frequency48k, TanApprox::Dirty);
    if (sampleRate_ != 48000.0f) {
      float angle = atanf(g) * (48000.0f / sampleRate_);
      g = tanf(angle < 1.56f ? angle : 1.56f);
    }
    filter_.setGQ(g, 0.4f);
  }

  float sampleRate_ = SAMPLE_RATE;
  float decay_ = 0.5f;
  float colour_ = 0.5f;
  float attackHz_ = 261.6256f; // C4
  int controlPeriod_ = 11;
  int counter_ = 0;
  float vactrol_ = 0.0f;
  bool rampUp_ = false;
  float gain_ = 0.0f, gainIncrement_ = 0.0f;
  float hfBleed_ = 0.0f;
  Svf filter_;
};

} // namespace bb
