/**
 * @file sv_filter.h
 * @brief State Variable Filter (SVF) with simultaneous low/high/bandpass
 * outputs.
 *
 * Filters are what transform raw waveforms and noise into musical, sculpted
 * sounds. This SVF implementation gives you three filter types from one
 * process() call:
 *
 *   Lowpass  — Removes high frequencies. Makes things sound "dark", "warm",
 *              "muffled". Like hearing music through a wall.
 *   Highpass — Removes low frequencies. Makes things sound "thin", "bright",
 *              "airy". Like a tinny speaker.
 *   Bandpass — Keeps only frequencies near the cutoff. Creates a focused,
 *              "vocal" or "wah-wah" quality.
 *
 * HOW THE CHAMBERLIN SVF WORKS:
 * ─────────────────────────────
 * The SVF uses two integrators and a feedback path. Each sample, it
 * computes all three filter outputs in sequence:
 *
 *   1. lowpass  += f * bandpass       (integrate bandpass → lowpass)
 *   2. highpass  = input - lowpass - damping * bandpass
 *   3. bandpass += f * highpass       (integrate highpass → bandpass)
 *
 * Where:
 *   f = 2 * sin(π * cutoff / sampleRate)  — controls the cutoff frequency
 *   damping = 2 * (1 - resonance)          — controls the resonance peak
 *
 * The "resonance" parameter creates a peak at the cutoff frequency. At high
 * resonance, the filter starts to ring or "sing" — this is the classic
 * synthesizer filter sweep sound. At maximum resonance, the filter
 * self-oscillates and produces a sine wave at the cutoff frequency.
 *
 * STABILITY NOTE: The Chamberlin SVF becomes unstable when the cutoff
 * frequency approaches the Nyquist frequency (sampleRate/2). We clamp
 * the cutoff to sampleRate/4 to stay safe. For this ambient project,
 * we're typically sweeping in the 100–5000 Hz range anyway.
 *
 * Example usage:
 *   SVFilter filter;
 *   filter.setFrequency(800.0f);   // 800 Hz cutoff
 *   filter.setResonance(0.4f);     // Moderate resonance
 *
 *   // In audio callback:
 *   filter.process(inputSample);
 *   float warm = filter.lowpass();     // Use lowpass output
 *   float bright = filter.highpass();  // Or highpass
 *   float focused = filter.bandpass(); // Or bandpass
 *   // You can even mix them!
 *   float notch = filter.lowpass() + filter.highpass(); // Notch filter
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * State Variable Filter (SVF) with simultaneous low/high/bandpass outputs.
 *
 * See the file header above for how it works and usage examples.
 */
class SVFilter {
public:
  SVFilter() {
    setFrequency(1000.0f);
    setResonance(0.0f);
  }

  /**
   * Set the filter cutoff frequency in Hz.
   *
   * This is the frequency where the filter transitions between passing
   * and rejecting. For a lowpass filter, frequencies below this pass
   * through and frequencies above are attenuated.
   *
   * @param hz  Cutoff frequency in Hertz.
   *            Useful range: 20 Hz (very dark) to ~10000 Hz (barely
   * filtering). Note: clamped to SAMPLE_RATE/4 for stability.
   *
   * Example:
   *   // Sweep cutoff with an LFO for a "breathing" effect
   *   float mod = lfo.process();
   *   float cutoff = mapf(mod, -1.0f, 1.0f, 200.0f, 3000.0f);
   *   filter.setFrequency(cutoff);
   */
  void setFrequency(float hz) {
    // Clamp to prevent instability near Nyquist. The Chamberlin SVF
    // uses a sin() approximation that breaks down above SR/4.
    hz = clampf(hz, 20.0f, SAMPLE_RATE * 0.25f);

    // Early-out if the frequency hasn't changed. sinf() is one of the
    // more expensive operations in the audio callback — this filter is
    // often driven by an LFO, which means setFrequency() gets called
    // once per sample. When the LFO uses a control-rate divider (see
    // lfo.h setDivider()), it returns the exact same float value for N
    // consecutive samples. Identical bits → skip the sinf() entirely.
    //
    // Note: exact float equality is usually a red flag, but here it's
    // intentional and safe — we're comparing against our own cached value
    // that we wrote last time, not comparing two independently-computed
    // results that might round differently.
    if (hz == lastHz_)
      return;
    lastHz_ = hz;

    // The coefficient f = 2 * sin(π * fc / fs) controls how fast the
    // integrators respond. At low frequencies, f is small (sluggish
    // integrators = low cutoff). At high frequencies, f is larger.
    f_ = 2.0f * sinf(PI_F * hz / SAMPLE_RATE);
  }

  /**
   * Set the filter resonance (Q).
   *
   * Resonance creates a peak in the frequency response at the cutoff.
   * Higher resonance = sharper, more pronounced peak.
   *
   * @param q  Resonance amount, 0.0 to 1.0.
   *           0.0 = no resonance (gentle rolloff)
   *           0.5 = noticeable peak (classic synth sound)
   *           0.9 = strong ringing, almost self-oscillating
   *           1.0 = self-oscillation (filter produces a sine wave)
   *           Clamped internally to prevent filter blowup.
   *
   * Example:
   *   filter.setResonance(0.3f);  // Subtle warmth
   *   filter.setResonance(0.7f);  // Aggressive, synthy character
   */
  void setResonance(float q) {
    q_ = clampf(q, 0.0f, 1.0f);
    // Damping is the inverse of resonance. Low damping = high resonance.
    // The formula 2*(1-q) maps q=0 → damp=2 (overdamped, gentle) and
    // q=1 → damp=0 (undamped, self-oscillation).
    damp_ = 2.0f * (1.0f - q_);
    // Floor at 0.5 to prevent complete instability at extreme settings
    if (damp_ < 0.5f)
      damp_ = 0.5f;
  }

  /**
   * Process one input sample through the filter.
   *
   * After calling this, read the desired output(s) via lowpass(),
   * highpass(), and/or bandpass(). All three are updated simultaneously.
   *
   * @param input  The input sample (typically -1.0 to +1.0)
   *
   * Example:
   *   float noise = noiseGen.process();
   *   filter.process(noise);
   *   float output = filter.lowpass();  // Warm, filtered noise
   */
  void process(float input) {
    // Step 1: Update lowpass by integrating the bandpass output.
    // The bandpass represents the "speed" of the signal near the cutoff;
    // integrating it accumulates into the low-frequency content.
    lp_ += f_ * bp_;

    // Step 2: Compute highpass as the "remainder" — everything the
    // lowpass and (damped) bandpass didn't capture.
    hp_ = input - lp_ - damp_ * bp_;

    // Step 3: Update bandpass by integrating the highpass output.
    // This creates the feedback loop that makes the filter resonate.
    bp_ += f_ * hp_;
  }

  /** @return  The lowpass filter output (frequencies below cutoff). */
  float lowpass() const { return lp_; }

  /** @return  The highpass filter output (frequencies above cutoff). */
  float highpass() const { return hp_; }

  /** @return  The bandpass filter output (frequencies near cutoff). */
  float bandpass() const { return bp_; }

  /** Reset all internal state to zero. Use when changing patches to avoid
   * artifacts. */
  void clear() { lp_ = hp_ = bp_ = 0.0f; }

private:
  float f_ = 0.0f;       // Frequency coefficient (controls cutoff)
  float q_ = 0.0f;       // Resonance amount (stored for reference)
  float damp_ = 2.0f;    // Damping factor (inverse of resonance)
  float lp_ = 0.0f;      // Lowpass output state
  float hp_ = 0.0f;      // Highpass output state
  float bp_ = 0.0f;      // Bandpass output state
  float lastHz_ = -1.0f; // Cached cutoff frequency for change detection
};

} // namespace bb
