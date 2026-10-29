/**
 * @file low_pass_gate.h
 * @brief Low Pass Gate (LPG) — combined VCA and lowpass filter on one CV.
 *
 * The Low Pass Gate is one of Don Buchla's most iconic inventions. It
 * combines a voltage-controlled amplifier (VCA) and a lowpass filter into
 * a single circuit driven by one control signal. As the gate opens:
 *
 *   - The VCA gain rises (louder)
 *   - The filter cutoff rises (brighter)
 *
 * Both happen simultaneously from the same control voltage, giving the LPG
 * its characteristic sound: notes bloom both in volume AND timbre together,
 * and close together — producing a woody, percussive, organic quality that
 * neither a plain VCA nor a plain filter alone can replicate.
 *
 * VACTROL SIMULATION:
 * ───────────────────
 * The original Buchla LPG used a *vactrol* — an optocoupler (LED +
 * light-dependent resistor) as the control element. Vactrols have a slow
 * physical response: the LDR takes time to react to the LED changing
 * brightness. This asymmetric slew (faster opening, slower closing) is a
 * major part of the LPG's sound — it gives notes a percussive attack and
 * a lingering, natural decay.
 *
 * This implementation models the vactrol with a one-pole lowpass smoother
 * on the control signal. The slew time can be tuned with setSlewTime().
 * Asymmetric rise/fall times can be set using setSlewRise()/setSlewFall().
 *
 * FILTER IMPLEMENTATION:
 * ──────────────────────
 * A one-pole lowpass filter is used internally rather than the SVFilter.
 * This is intentional: the original Buchla LPG has a gentle 6 dB/octave
 * rolloff (one pole), not a steep 12 or 24 dB/octave response. The single
 * pole gives a smooth, musical timbral transition as the gate opens.
 *
 * Example usage — triggered by an envelope:
 *   LowPassGate lpg;
 *   lpg.setSlewTime(0.04f);   // 40 ms vactrol-style lag
 *   Envelope env;
 *   env.setAttack(0.005f);
 *   env.setRelease(0.3f);
 *
 *   // In audio callback:
 *   env.gate(triggered);
 *   lpg.setControl(env.process());
 *   float out = lpg.process(osc.process());
 *
 * Example usage — LFO-driven (tremolo with timbral shift):
 *   LowPassGate lpg;
 *   LFO lfo;
 *   lfo.setFrequency(2.0f);
 *
 *   // In audio callback:
 *   float cv = mapf(lfo.process(), -1.0f, 1.0f, 0.0f, 1.0f);
 *   lpg.setControl(cv);
 *   float out = lpg.process(osc.process());
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Low Pass Gate (LPG) — combined VCA and lowpass filter on one CV.
 *
 * See the file header above for how it works and usage examples.
 */
class LowPassGate {
public:
  /**
   * Construct an LPG with default 10 ms symmetric slew time.
   */
  LowPassGate() { setSlewTime(0.01f); }

  /**
   * Set the control voltage — how open the gate is.
   *
   * This is the equivalent of a CV input on a hardware LPG. Feed it
   * from an envelope, LFO, or any 0.0–1.0 signal. The gate does not
   * respond instantly — the vactrol simulation smooths transitions.
   *
   * @param c  Control level, 0.0 (fully closed) to 1.0 (fully open).
   *
   * Example:
   *   lpg.setControl(envelope.process());
   */
  void setControl(float c) { target_ = clampf(c, 0.0f, 1.0f); }

  /**
   * Set a symmetric slew time for rise and fall (vactrol simulation).
   *
   * Longer slew = slower, more laggy response — more vactrol-like.
   * Shorter slew = snappier response — less character but more precise.
   *
   * @param seconds  Slew time for both rise and fall.
   *                 0.005 = 5 ms (snappy)
   *                 0.02  = 20 ms (typical vactrol)
   *                 0.1   = 100 ms (very slow, blob-like)
   *
   * Example:
   *   lpg.setSlewTime(0.025f);  // 25 ms vactrol lag
   */
  void setSlewTime(float seconds) {
    setSlewRise(seconds);
    setSlewFall(seconds);
  }

  /**
   * Set separate rise slew time (how fast the gate opens).
   *
   * Real vactrols typically open faster than they close — set rise
   * shorter than fall for authentic character.
   *
   * @param seconds  Rise slew time in seconds.
   *
   * Example:
   *   lpg.setSlewRise(0.005f);  // Fast 5 ms open
   *   lpg.setSlewFall(0.05f);   // Slow 50 ms close
   */
  void setSlewRise(float seconds) {
    float samples =
        clampf(seconds * SAMPLE_RATE, 1.0f, (float)SAMPLE_RATE * 10.0f);
    // One-pole coefficient: controls how quickly the slew
    // filter reaches its target. Closer to 1.0 = slower.
    riseCoeff_ = expf(-1.0f / samples);
  }

  /**
   * Set separate fall slew time (how fast the gate closes).
   *
   * @param seconds  Fall slew time in seconds.
   */
  void setSlewFall(float seconds) {
    float samples =
        clampf(seconds * SAMPLE_RATE, 1.0f, (float)SAMPLE_RATE * 10.0f);
    fallCoeff_ = expf(-1.0f / samples);
  }

  /**
   * Process one audio sample through the Low Pass Gate.
   *
   * This updates the vactrol simulation, then applies both VCA
   * gain and a one-pole lowpass filter whose cutoff tracks the
   * smoothed control signal.
   *
   * @param input  Audio sample to process, typically -1.0 to +1.0.
   * @return       Gated and filtered sample.
   *
   * Example:
   *   float out = lpg.process(osc.process());
   */
  float process(float input) {
    // Step 1: Slew-limit the control signal (vactrol simulation).
    // Use a different coefficient for rising vs falling to model
    // the asymmetric vactrol response.
    float coeff = target_ > control_ ? riseCoeff_ : fallCoeff_;
    control_ = coeff * control_ + (1.0f - coeff) * target_;

    // Step 2: Apply VCA — scale input by the smoothed control.
    // At control=0: silence. At control=1: full input level.
    float gated = input * control_;

    // Step 3: Apply one-pole lowpass filter whose cutoff tracks
    // the gate. Map control (0-1) to frequency range (20-8000 Hz)
    // using a quadratic curve for a more musically even response.
    // control=0 → 20 Hz (nearly silent and dark)
    // control=1 → 8000 Hz (open and bright)
    //
    // The bilinear approximation alpha ≈ 2π·fc/fs is accurate for
    // fc << fs, avoiding an expf() call in the audio callback.
    float fc = 20.0f + control_ * control_ * 7980.0f;
    float alpha = clampf(TWO_PI_F * fc / SAMPLE_RATE, 0.0f, 1.0f);

    // One-pole LP update: lp_ approaches gated at a rate set by alpha.
    // High alpha (high fc) = fast response = bright/open.
    // Low alpha (low fc)   = slow response = dark/closed.
    lp_ += alpha * (gated - lp_);

    return lp_;
  }

  /**
   * Reset all internal state to zero (silence).
   * Use when switching patches to avoid bleed from previous state.
   */
  void clear() {
    control_ = 0.0f;
    target_ = 0.0f;
    lp_ = 0.0f;
  }

  /** @return  The current smoothed control value (0.0–1.0). */
  float getControl() const { return control_; }

private:
  float target_ = 0.0f;    // Desired control level (set by setControl)
  float control_ = 0.0f;   // Slew-smoothed control (vactrol simulation)
  float riseCoeff_ = 0.0f; // One-pole coeff for gate opening
  float fallCoeff_ = 0.0f; // One-pole coeff for gate closing
  float lp_ = 0.0f;        // One-pole lowpass filter state
};

} // namespace bb
