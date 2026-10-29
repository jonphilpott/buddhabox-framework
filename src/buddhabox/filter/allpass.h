/**
 * @file allpass.h
 * @brief Schroeder allpass filter for phase dispersion and reverb diffusion.
 *
 * An allpass filter has a completely flat magnitude response — it passes all
 * frequencies at equal amplitude. What it *does* change is the phase
 * relationship between frequencies, smearing them in time. This property,
 * called "phase dispersion" or "diffusion", is the first building block of
 * reverb algorithms.
 *
 * In isolation, an allpass filter sounds like a comb of subtle, flanging
 * echoes that cancel out the steady-state pitch information but preserve
 * the overall character. Chain two or three with different delay times and
 * gain values and you get a convincing sense of acoustic space — without
 * the complexity or memory footprint of a full Schroeder/Freeverb reverb.
 *
 * HOW THE ALGORITHM WORKS:
 * ─────────────────────────
 * The Schroeder allpass section uses a feedback + feedforward delay:
 *
 *   d      = delay_buffer[M samples ago]
 *   v      = input + g * d          (feedback: mix input with delayed)
 *   output = d - g * v              (feedforward: subtract g*feedback)
 *   write  v into delay buffer
 *
 * Where M is the delay length and g is the gain coefficient (0.0–0.99).
 *
 * The transfer function is:
 *   H(z) = (z^-M - g) / (1 - g * z^-M)
 *
 * It can be shown that |H(e^jw)| = 1 for all frequencies — a true allpass.
 * The phase response (and therefore the diffusion character) is shaped by
 * the delay length M and gain g.
 *
 * MEMORY:
 * ───────
 * The template parameter MAX_SAMPLES controls the delay buffer size.
 * Internally, DelayLine<N> stores int16_t values (2 bytes each):
 *
 *   AllpassFilter<1024>  →  ~23 ms max delay,  2 KB RAM
 *   AllpassFilter<2048>  →  ~46 ms max delay,  4 KB RAM
 *   AllpassFilter<4096>  →  ~93 ms max delay,  8 KB RAM
 *
 * CHOOSING PARAMETERS:
 * ────────────────────
 * For reverb-like diffusion:
 *   - Use different prime-ish delay times for each filter to avoid
 *     comb filtering: e.g., 556, 441, 341 samples (12.6ms, 10ms, 7.7ms).
 *   - Gain values of 0.5–0.7 give the most diffuse sound.
 *   - Connect in series: out1 = ap1.process(in), out2 = ap2.process(out1)
 *
 * Example usage — single allpass (subtle spatial smear):
 *   AllpassFilter<2048> ap;
 *   ap.setDelay(800.0f);  // ~18 ms
 *   ap.setGain(0.6f);
 *
 *   // In audio callback:
 *   float out = ap.process(osc.process());
 *
 * Example usage — two allpass stages in series (room diffusion):
 *   AllpassFilter<2048> ap1, ap2;
 *   ap1.setDelay(556.0f);  ap1.setGain(0.6f);
 *   ap2.setDelay(441.0f);  ap2.setGain(0.6f);
 *
 *   // In audio callback:
 *   float sig = osc.process();
 *   sig = ap1.process(sig);
 *   sig = ap2.process(sig);
 *   float out = blend(osc.process(), sig, 0.4f);  // 40% wet
 *
 * Example usage — LFO-modulated delay (chorus-like shimmer):
 *   AllpassFilter<2048> ap;
 *   LFO lfo;
 *   lfo.setFrequency(0.5f);
 *
 *   // In audio callback:
 *   float modDelay = 500.0f + lfo.process() * 200.0f;  // 300–700 samples
 *   ap.setDelay(modDelay);
 *   float out = ap.process(osc.process());
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/delay.h"

namespace bb {

/**
 * Schroeder allpass filter for phase dispersion and reverb diffusion.
 *
 * See the file header above for how it works and usage examples.
 */
template <uint16_t MAX_SAMPLES = 2048> class AllpassFilter {
public:
  /**
   * Construct an AllpassFilter with default delay and gain.
   *
   * Default delay is half the buffer length. Default gain is 0.5.
   */
  AllpassFilter() {
    setDelay(MAX_SAMPLES * 0.5f);
    setGain(0.5f);
  }

  /**
   * Set the feedback/feedforward gain coefficient.
   *
   * Higher values = more diffusion, stronger comb filtering character.
   * Values above 0.99 risk instability and are clamped.
   *
   * @param g  Gain coefficient, 0.0 to 0.99.
   *           0.0 = no effect (allpass becomes transparent)
   *           0.5 = typical diffuse reverb setting
   *           0.7 = strong diffusion with longer perceived reverb tail
   *
   * Example:
   *   ap.setGain(0.6f);
   */
  void setGain(float g) { g_ = clampf(g, 0.0f, 0.99f); }

  /**
   * Set the delay length in samples.
   *
   * This controls the "size" of the space being simulated. Longer delays =
   * bigger perceived room. Different values for each filter in a chain
   * prevents comb filtering artefacts.
   *
   * @param samples  Delay length in samples, 1.0 to MAX_SAMPLES-1.
   *                 Fractional values are interpolated smoothly.
   *                 At 44100 Hz: 441 samples ≈ 10 ms.
   *
   * Example:
   *   ap.setDelay(556.0f);  // ~12.6 ms
   */
  void setDelay(float samples) {
    delay_ = clampf(samples, 1.0f, (float)(MAX_SAMPLES - 1));
  }

  /**
   * Process one audio sample through the allpass filter.
   *
   * Reads from the delay buffer, applies feedback and feedforward mixing,
   * writes back, and returns the output. All frequencies pass at equal
   * amplitude — only phase relationships are changed.
   *
   * @param input  Audio sample to process, typically -1.0 to +1.0.
   * @return       Allpass-filtered sample (same amplitude, smeared phase).
   *
   * Example:
   *   float out = ap.process(osc.process());
   */
  float process(float input) {
    // Step 1: Read the delayed sample from the buffer.
    float d = line_.read(delay_);

    // Step 2: Compute the value to write into the delay.
    // This combines the input with g_ * the delayed output (feedback).
    float v = input + g_ * d;

    // Step 3: Compute the output using the feedforward path.
    // Subtracting g_*v cancels the direct contribution and keeps
    // the magnitude response flat at all frequencies.
    float output = d - g_ * v;

    // Step 4: Write the feedback value into the delay buffer.
    line_.write(v);

    return output;
  }

  /**
   * Zero the delay buffer. Use when switching patches to prevent old
   * audio from ringing through in the new context.
   */
  void clear() { line_.clear(); }

  /** @return  Current gain coefficient. */
  float getGain() const { return g_; }

  /** @return  Current delay in samples. */
  float getDelay() const { return delay_; }

private:
  DelayLine<MAX_SAMPLES> line_; // Circular delay buffer (2 bytes/sample)
  float g_ = 0.5f;              // Gain coefficient (feedback/feedforward)
  float delay_ = 0.0f;          // Delay length in samples
};

} // namespace bb
