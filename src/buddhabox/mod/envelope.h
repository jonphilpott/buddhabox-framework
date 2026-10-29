/**
 * @file envelope.h
 * @brief Attack-Release (AR) envelope generator for shaping amplitude over
 * time.
 *
 * An envelope controls how a sound's volume changes from the moment it starts
 * to when it fades away. Without an envelope, an oscillator just drones at a
 * constant level. With one, you can create:
 *
 *   - Percussive plucks:  fast attack (1ms), short release (100ms)
 *   - Soft pads:          slow attack (2s), long release (5s)
 *   - Swells:             very slow attack (10s), slow release (10s)
 *   - Drones:             instant attack, no release (gate stays open)
 *
 * HOW AN AR ENVELOPE WORKS:
 * ────────────────────────
 * The envelope has three states:
 *
 *   IDLE → gate on → ATTACK → (reaches 1.0) → gate off → RELEASE → IDLE
 *
 *        gate on          gate off
 *          │                 │
 *   1.0    │    ┌────────────┤
 *          │   ╱             │╲
 *          │  ╱              │ ╲
 *          │ ╱               │  ╲
 *   0.0 ───┘╱                │   ╲───────
 *          │                 │
 *        attack            release
 *
 * ATTACK:  Output ramps from 0.0 to 1.0 over the attack time.
 *          While the gate is held open, output stays at 1.0 (sustain).
 * RELEASE: When the gate closes, output ramps from its current level
 *          down to 0.0 over the release time.
 *
 * The envelope output is typically multiplied with an audio signal:
 *   output = oscillator.process() * envelope.process()
 *
 * EXPONENTIAL vs LINEAR CURVES:
 * ────────────────────────────
 * This envelope uses exponential curves, which sound more natural than
 * linear ramps. Here's why:
 *
 * Human hearing is logarithmic — we perceive loudness on a log scale.
 * A linear ramp (constant change per sample) sounds like it speeds up
 * at the end of the attack and lingers too long at the start of release.
 * An exponential curve (constant ratio per sample) sounds smooth and
 * even to our ears.
 *
 * The math: each sample, we multiply the envelope value by a coefficient
 * slightly less than 1.0 (for release) or approach 1.0 from below using
 * a similar exponential formula (for attack).
 *
 * Example usage — Percussive pluck:
 *   Envelope env;
 *   env.setAttack(0.001f);    // 1 ms attack (nearly instant)
 *   env.setRelease(0.15f);    // 150 ms release (short decay)
 *
 *   // Trigger a note:
 *   env.gate(true);
 *   // ... some time later (or immediately for a pluck):
 *   env.gate(false);
 *
 *   // In audio callback:
 *   float sample = osc.process() * env.process();
 *
 * Example usage — Slow ambient swell:
 *   Envelope env;
 *   env.setAttack(3.0f);      // 3 second fade-in
 *   env.setRelease(5.0f);     // 5 second fade-out
 *   env.gate(true);           // Start the swell
 *
 * Example usage — Drone (always on):
 *   Envelope env;
 *   env.setAttack(2.0f);      // Gentle 2-second fade-in
 *   env.setRelease(1.0f);     // Won't be used unless gate goes off
 *   env.gate(true);           // Gate on — stays at 1.0 after attack
 *
 * Example usage — Rhythmic with TempoClock:
 *   TempoClock clock(120.0f, 2);  // 120 BPM, eighth notes
 *   Envelope env;
 *   env.setAttack(0.005f);        // 5 ms
 *   env.setRelease(0.1f);         // 100 ms
 *
 *   // In audio callback:
 *   if (clock.process()) {
 *       env.gate(true);   // Trigger on each tick
 *       env.gate(false);  // Immediately release for a percussive hit
 *   }
 *   float sample = osc.process() * env.process();
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Envelope states.
 *
 * The envelope transitions through these states in order:
 *   IDLE → ATTACK → SUSTAIN → RELEASE → IDLE
 *
 * SUSTAIN isn't a timed phase — it just holds at 1.0 while the gate is open.
 */
enum class EnvState {
  IDLE,    // Output = 0.0, waiting for gate
  ATTACK,  // Ramping up from 0.0 toward 1.0
  SUSTAIN, // Holding at 1.0 while gate is open
  RELEASE  // Ramping down from current level toward 0.0
};

/**
 * Attack-Release (AR) envelope generator for shaping amplitude over time.
 *
 * See the file header above for how it works and usage examples.
 */
class Envelope {
public:
  Envelope() {
    setAttack(0.01f); // 10 ms default attack
    setRelease(0.5f); // 500 ms default release
  }

  /**
   * Set the attack time in seconds.
   *
   * This is how long it takes for the envelope to ramp from 0.0 to ~99%
   * after the gate opens.
   *
   * @param seconds  Attack time in seconds.
   *                 0.001 = 1 ms (percussive, instant)
   *                 0.01  = 10 ms (snappy but not clicky)
   *                 0.5   = 500 ms (gradual fade-in)
   *                 3.0   = 3 seconds (slow swell)
   *                 Minimum clamped to ~1 sample to prevent division by zero.
   *
   * Example:
   *   env.setAttack(0.005f);  // 5 ms — good for plucks
   */
  void setAttack(float seconds) {
    // We use -5.0/samples to ensure the exponential reaches 1 - e^-5 (~99.3%)
    // in exactly the requested time. This matches user expectations better
    // than the standard "time constant" tau which only reaches 63%.
    float samples = seconds * SAMPLE_RATE;
    if (samples < 1.0f)
      samples = 1.0f; // Prevent zero/negative
    attackCoeff_ = expf(-5.0f / samples);
  }

  /**
   * Set the release time in seconds.
   *
   * This is how long it takes for the envelope to fade from its current
   * level down to ~1% after the gate closes.
   *
   * @param seconds  Release time in seconds.
   *                 0.05  = 50 ms (very short, clicky)
   *                 0.15  = 150 ms (percussive)
   *                 1.0   = 1 second (medium fade-out)
   *                 5.0   = 5 seconds (long, ambient tail)
   *                 10.0  = 10 seconds (very slow fade)
   *
   * Example:
   *   env.setRelease(2.0f);  // 2-second fade-out
   */
  void setRelease(float seconds) {
    float samples = seconds * SAMPLE_RATE;
    if (samples < 1.0f)
      samples = 1.0f;
    releaseCoeff_ = expf(-5.0f / samples);
  }

  /**
   * Open or close the gate (trigger/release a note).
   *
   * @param on  true = start attack (trigger note)
   *            false = start release (release note)
   *
   * Calling gate(true) always restarts the attack from zero, even if
   * the envelope is already active. This ensures a proper attack slope
   * rather than jumping immediately to full volume.
   *
   * Example:
   *   env.gate(true);   // Note on — start attack
   *   // ... hold for desired duration ...
   *   env.gate(false);  // Note off — start release
   */
  void gate(bool on) {
    if (on) {
      state_ = EnvState::ATTACK;
      oneShot_ = false; // Disable auto-release if we're gating manually
    } else {
      if (state_ != EnvState::IDLE) {
        state_ = EnvState::RELEASE;
      }
    }
  }

  /**
   * Trigger a one-shot note: start attack from the current level.
   *
   * Always restarts the attack from zero, even if the envelope is
   * already active. This ensures a proper attack slope on every trigger.
   *
   * The envelope will automatically move to RELEASE after reaching 1.0.
   *
   * Example:
   *   if (clock.process()) {
   *       env.trigger();
   *   }
   */
  void trigger() {
    state_ = EnvState::ATTACK;
    oneShot_ = true; // Auto-release after attack completes
  }

  /**
   * Instantly reset the envelope to zero and idle state.
   * Use this with caution as it will cause a "click" if the envelope
   * was currently audible. Useful for global stop/reset functions.
   */
  void reset() {
    value_ = 0.0f;
    state_ = EnvState::IDLE;
    oneShot_ = false;
  }

  /**
   * Generate the next envelope sample.
   *
   * Call this once per sample in your audio callback. Multiply the result
   * with your audio signal to apply the envelope.
   *
   * @return  Envelope value, 0.0 (silence) to 1.0 (full volume)
   *
   * Example:
   *   float audio = osc.process() * env.process();
   */
  float process() {
    switch (state_) {
    case EnvState::IDLE:
      // Output stays at 0.0 — no sound
      value_ = 0.0f;
      break;

    case EnvState::ATTACK:
      // Exponential approach toward 1.0:
      // value = value * coeff + (1.0 - coeff) * target
      value_ = attackCoeff_ * value_ + (1.0f - attackCoeff_) * 1.0f;

      // Check if we've reached the top (within a tiny threshold).
      // 0.99 threshold is snappy and helps the envelope move to
      // release quickly for plucks.
      if (value_ >= 0.99f) {
        value_ = 1.0f;
        if (oneShot_) {
          // trigger() mode: auto-release, no sustain
          state_ = EnvState::RELEASE;
          oneShot_ = false;
        } else {
          // gate() mode: hold at 1.0 until gate closes
          state_ = EnvState::SUSTAIN;
        }
      }
      break;

    case EnvState::SUSTAIN:
      // Hold at 1.0. The gate(false) call will transition us
      // to RELEASE when the note should end.
      value_ = 1.0f;
      break;

    case EnvState::RELEASE:
      // Exponential decay toward 0.0:
      value_ *= releaseCoeff_;

      // Snap to zero when we're quiet enough to be inaudible.
      // 0.0001 = -80 dB, well below the noise floor of 16-bit audio.
      if (value_ < 0.0001f) {
        value_ = 0.0f;
        state_ = EnvState::IDLE;
      }
      break;
    }

    return value_;
  }

  /** @return  true if the envelope is active (not idle). */
  bool isActive() const { return state_ != EnvState::IDLE; }

  /** @return  The current envelope state. */
  EnvState getState() const { return state_; }

  /** @return  The current envelope value (0.0–1.0) without advancing. */
  float getValue() const { return value_; }

private:
  EnvState state_ = EnvState::IDLE;
  float value_ = 0.0f;        // Current envelope output (0.0–1.0)
  float attackCoeff_ = 0.0f;  // Exponential coefficient for attack
  float releaseCoeff_ = 0.0f; // Exponential coefficient for release
  bool oneShot_ = false;      // true when triggered via trigger()
};

} // namespace bb
