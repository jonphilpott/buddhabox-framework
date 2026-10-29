/**
 * @file lfo.h
 * @brief Low Frequency Oscillator for modulating audio parameters over time.
 *
 * An LFO is structurally identical to a regular oscillator, but operates at
 * sub-audio frequencies (typically 0.01 Hz to 20 Hz). Instead of producing
 * sound directly, its output is used to modulate other parameters:
 *
 *   - Filter cutoff: makes the tone "breathe" or "sweep"
 *   - Amplitude: creates tremolo (volume wobble)
 *   - Pitch: creates vibrato (pitch wobble)
 *   - Delay time: creates chorus/flanger effects
 *   - Mix levels: slowly crossfade between textures
 *
 * WHY A SEPARATE CLASS FROM OSCILLATOR?
 * ─────────────────────────────────────
 * While the math is the same, the LFO class differs in intent:
 *   - No amplitude parameter — always outputs full -1 to +1 range.
 *     The user scales the output as needed for the target parameter.
 *   - Includes reset() for syncing to tempo clocks
 *   - Default frequency is 1 Hz (not 440 Hz)
 *
 * Keeping them separate also makes code more readable: when you see `LFO`
 * you immediately know it's a modulation source, not an audio source.
 *
 * Example usage:
 *   LFO filterSweep;
 *   filterSweep.setWaveform(Waveform::TRIANGLE);
 *   filterSweep.setFrequency(0.1f);  // One cycle every 10 seconds
 *
 *   // In audio callback:
 *   float mod = filterSweep.process();                    // -1.0 to +1.0
 *   float cutoff = mapf(mod, -1.0f, 1.0f, 200.0f, 3000.0f);  // Scale to Hz
 *   filter.setFrequency(cutoff);
 *
 * Waveform character for modulation:
 *   SINE     — Smooth, natural-sounding sweep
 *   TRIANGLE — Linear sweep, slightly more "mechanical" than sine
 *   SAW      — Ramp up then snap back. Good for rhythmic filter sweeps.
 *   SQUARE   — Abrupt switch between two values. Useful for alternating
 * states.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/oscillator.h" // For the Waveform enum

namespace bb {

/**
 * Low Frequency Oscillator for modulating audio parameters over time.
 *
 * See the file header above for how it works and usage examples.
 */
class LFO {
public:
  LFO() = default;

  /**
   * Set the LFO frequency in Hz.
   *
   * @param hz  Frequency in Hertz. Typical range: 0.01 to 20 Hz.
   *            0.1 Hz = one cycle every 10 seconds (slow, meditative)
   *            1.0 Hz = one cycle per second (gentle pulse)
   *            5.0 Hz = five cycles per second (vibrato speed)
   *
   * Example:
   *   lfo.setFrequency(0.05f);  // Very slow: 20-second cycle
   */
  void setFrequency(float hz) { phaseInc_ = hz / SAMPLE_RATE; }

  /**
   * Set the control-rate divider — run the LFO at a fraction of audio rate.
   *
   * By default the LFO computes a new output value every audio sample
   * (divider = 1). For typical ambient LFOs (0.01–5 Hz), this is massive
   * overkill — the waveform changes so slowly that holding the same value
   * for 64 or even 256 consecutive samples is completely imperceptible.
   *
   * WHY THIS MATTERS:
   * The LFO output is often fed into SVFilter::setFrequency(), which calls
   * sinf() to recompute a filter coefficient. By downsampling the LFO,
   * you reduce those expensive sinf() calls by the same factor. For the
   * three LFOs in a typical patch, divider=64 cuts sinf() calls from
   * 3×44100/sec to 3×689/sec — a 64× reduction in filter coefficient work.
   *
   * ACCURACY:
   * The LFO's internal phase still advances by the correct amount each
   * time it fires (phaseInc_ × divider_), so the LFO frequency stays
   * exact regardless of divider. The only effect is that the output
   * value is held constant between updates — a form of sample-and-hold
   * that is inaudible for slow modulation sources.
   *
   * PRACTICAL GUIDE:
   *   divider = 1    → per-sample (default, maximum fidelity)
   *   divider = 16   → ~2756 Hz control rate (fine for vibrato up to ~5 Hz)
   *   divider = 64   → ~689 Hz control rate (ideal for filter/delay LFOs)
   *   divider = 256  → ~172 Hz control rate (fine for LFOs below ~0.5 Hz)
   *
   * @param n  Number of audio samples between LFO updates (1 = every sample)
   *
   * Example:
   *   lfo.setFrequency(0.025f);  // Very slow: 40-second cycle
   *   lfo.setDivider(256);       // Compute only 172 times/sec — plenty
   */
  void setDivider(uint32_t n) {
    divider_ = (n < 1) ? 1 : n;
    counter_ = 0; // Recompute immediately on next process() call
  }

  /**
   * Set the LFO waveform shape.
   * @param w  One of: Waveform::SINE, SAW, SQUARE, TRIANGLE
   *
   * See Waveform enum in oscillator.h for harmonic characteristics.
   */
  void setWaveform(Waveform w) { waveform_ = w; }

  /**
   * Generate the next LFO sample.
   *
   * Call this once per audio sample in your callback. The output is
   * always in the range [-1.0, +1.0] — you scale it to whatever
   * parameter range you need using mapf() or simple multiplication.
   *
   * If setDivider(n) has been called, the LFO recomputes only once every
   * n samples and holds the cached value in between. This is transparent
   * to the caller — you still call process() every sample.
   *
   * @return  Current LFO value in [-1.0, +1.0]
   *
   * Example:
   *   float mod = lfo.process();
   *   // Use mod to control delay feedback (0.2 to 0.8 range):
   *   float feedback = mapf(mod, -1.0f, 1.0f, 0.2f, 0.8f);
   */
  float process() {
    // counter_ counts down to zero. When it hits zero we recompute the
    // waveform, advance the phase, and reload the counter. In between,
    // we simply return the cached value from the last computation.
    if (counter_ == 0) {
      // ── Compute waveform value at current phase ──────────────────────
      // Same waveform math as Oscillator — see oscillator.h for detailed
      // explanations of each shape's computation.
      switch (waveform_) {
      case Waveform::SINE:
        // fastSin() is defined in dsp_common.h — shared with Oscillator.
        cached_ = fastSin(phase_ * TWO_PI_F);
        break;
      case Waveform::SAW:
        cached_ = 2.0f * phase_ - 1.0f;
        break;
      case Waveform::SQUARE:
        cached_ = phase_ < 0.5f ? 1.0f : -1.0f;
        break;
      case Waveform::TRIANGLE:
        cached_ = phase_ < 0.5f ? 4.0f * phase_ - 1.0f : 3.0f - 4.0f * phase_;
        break;
      }

      // ── Advance phase by divider_ steps at once ───────────────────────
      // When divider_ = 1 this is identical to the normal per-sample
      // advance. When divider_ = 64, we jump forward 64 steps in one shot
      // — the phase stays perfectly accurate since we're just multiplying
      // the per-sample increment by the number of samples we skipped.
      phase_ += phaseInc_ * (float)divider_;
      // Use while in case divider_ is large enough to advance more than
      // one full cycle in a single step (unusual but safe to handle).
      while (phase_ >= 1.0f)
        phase_ -= 1.0f;

      // Reload the counter for the next divider_ - 1 idle samples
      counter_ = divider_ - 1;
    } else {
      counter_--;
    }

    return cached_;
  }

  /** Get the current phase (0.0–1.0). */
  float getPhase() const { return phase_; }

  /** Set phase directly. Useful for syncing multiple LFOs together. */
  void setPhase(float p) { phase_ = p; }

  /**
   * Reset phase to zero (beginning of cycle).
   * Useful for syncing the LFO to a tempo clock on each beat.
   *
   * Also resets the divider counter so the next process() call
   * immediately recomputes at phase 0, rather than returning a
   * stale cached value from before the reset.
   *
   * Example:
   *   if (clock.process()) {  // On each beat...
   *       lfo.reset();        // Restart the LFO cycle
   *   }
   */
  void reset() {
    phase_ = 0.0f;
    counter_ = 0; // Force recompute on next process() call
  }

private:
  Waveform waveform_ = Waveform::SINE;
  float phase_ = 0.0f;
  float phaseInc_ = 1.0f / SAMPLE_RATE; // Default: 1 Hz

  // ── Control-rate divider state ───────────────────────────────────────────
  // divider_: how many audio samples between waveform recomputations (1 = off)
  // counter_: counts down from divider_-1 to 0; at 0 we recompute
  // cached_:  the last computed waveform value, returned on idle samples
  uint32_t divider_ = 1;
  uint32_t counter_ = 0;
  float cached_ = 0.0f;
};

} // namespace bb
