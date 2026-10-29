/**
 * @file moog_filter.h
 * @brief 4-pole Moog ladder filter approximation with resonance.
 *
 * The Moog ladder filter, designed by Dr. Robert Moog in 1965, is perhaps
 * the most iconic filter in electronic music history. Its warm, musical
 * character comes from two key properties:
 *
 *   1. FOUR POLES: Four one-pole lowpass stages in series give a steep
 *      24 dB/octave rolloff — twice as steep as the SVFilter's 12 dB/oct.
 *      This produces a much more dramatic "closing" effect when the cutoff
 *      is swept down.
 *
 *   2. NONLINEARITY: Each transistor stage in the original circuit softly
 *      saturates at high levels, generating warm even harmonics. We
 *      approximate this with a fast tanh-like function at the input stage.
 *
 * HOW THE LADDER WORKS:
 * ─────────────────────
 * Each one-pole stage is a simple leaky integrator:
 *
 *   s[n] += g * (input_n - s[n])
 *
 * Think of it like a capacitor charging toward its input. The coefficient
 * g controls the charge rate: high g (high cutoff) = fast tracking, low g
 * (low cutoff) = slow, sluggish response.
 *
 * Four stages are chained in series, with the last stage's output fed back
 * (negated) to the input by the resonance factor k:
 *
 *   IN → softclip → [s0] → [s1] → [s2] → [s3] → OUT
 *                     ↑                     │
 *                     └──── k × feedback ───┘
 *
 * At k=0: pure lowpass, no resonance.
 * At k≈4: the feedback exactly replaces what the filter attenuates at the
 *          cutoff frequency — the loop sustains itself and the filter rings.
 *
 * NONLINEARITY (the "Moog sound"):
 * ─────────────────────────────────
 * Before entering stage 0, the combined input+feedback is run through
 * softclip (x / (1 + |x|), a fast tanh approximation). This does two
 * things:
 *   - Bounds the feedback loop at high resonance, preventing runaway.
 *   - Adds warm harmonic saturation — the same "analogue feel" that
 *     makes real Moog filters sound different from clean digital ones.
 *
 * COMPARISON WITH SVFilter:
 * ──────────────────────────
 *   SVFilter   — 12 dB/oct, LP + HP + BP outputs, linear, very stable.
 *   MoogFilter — 24 dB/oct, LP output only, nonlinear character,
 *                self-oscillating resonance, slightly more CPU cost.
 *
 * PERFORMANCE NOTE:
 * ─────────────────
 * The most expensive operation, expf(), is called only in setFrequency()
 * and cached. The per-sample cost is four multiply-add pairs plus one
 * softclip (three operations) — well within budget on the STM32F4 FPU.
 *
 * Example usage:
 *   MoogFilter filter;
 *   filter.setFrequency(800.0f);  // 800 Hz cutoff
 *   filter.setResonance(0.7f);    // Strong resonance
 *
 *   // In audio callback:
 *   float output = filter.process(inputSample);
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * 4-pole Moog ladder filter approximation with resonance.
 *
 * See the file header above for how it works and usage examples.
 */
class MoogFilter {
public:
  MoogFilter() {
    setFrequency(1000.0f);
    setResonance(0.0f);
  }

  /**
   * Set the filter cutoff frequency in Hz.
   *
   * The cutoff is where attenuation begins. Below this frequency, content
   * passes at unity gain. Above it, the rolloff is 24 dB per octave.
   *
   * Internally computes g = 1 - exp(-2π*fc/fs), the one-pole coefficient.
   * expf() is called here — not in the audio loop — and the result is
   * cached. Repeated calls with the same Hz value are detected by exact
   * float comparison and return immediately with no computation.
   *
   * @param hz  Cutoff frequency in Hertz.
   *            Useful range: 20 Hz (almost silent) to ~15000 Hz (open).
   *            Clamped to [20, SAMPLE_RATE * 0.45] for stability.
   *
   * Example:
   *   // Sweep cutoff downward with an LFO for a classic filter sweep:
   *   float mod = lfo.process();
   *   filter.setFrequency(mapf(mod, -1.0f, 1.0f, 80.0f, 8000.0f));
   */
  void setFrequency(float hz) {
    hz = clampf(hz, 20.0f, SAMPLE_RATE * 0.45f);

    // Same frequency-change guard as SVFilter: exact float comparison is
    // intentional here — we are comparing against the value we wrote last
    // call, not two independently-computed results.
    if (hz == lastHz_)
      return;
    lastHz_ = hz;

    // One-pole coefficient from the bilinear transform of a first-order
    // RC lowpass: g = 1 - e^(-2π*fc/fs).
    // When g is small (low fc), each stage tracks slowly → low cutoff.
    // When g approaches 1 (high fc), stages track instantly → filter open.
    // expf() is expensive but runs only here, not sample-by-sample.
    g_ = 1.0f - expf(-TWO_PI_F * hz / SAMPLE_RATE);
  }

  /**
   * Set the filter resonance (Q).
   *
   * Resonance feeds the output of the last stage back to the input,
   * creating a peak at the cutoff frequency. At high resonance the filter
   * rings. At maximum (1.0), the loop sustains itself and the filter
   * self-oscillates, producing a sine wave at the cutoff pitch.
   *
   * The nonlinearity at the input stage keeps self-oscillation bounded —
   * the output stays within roughly ±1.0 even at resonance = 1.0.
   *
   * @param res  Resonance amount, 0.0 to 1.0.
   *             0.0 = no feedback (clean lowpass)
   *             0.5 = noticeable peak
   *             0.8 = strong ringing, musical
   *             1.0 = self-oscillation (pitched sine at cutoff frequency)
   *
   * Example:
   *   filter.setResonance(0.6f);  // Warm Moog character
   *   filter.setResonance(1.0f);  // Self-oscillate as a sine oscillator!
   */
  void setResonance(float res) {
    res = clampf(res, 0.0f, 1.0f);

    // Internally scale to [0, 3.99]. At k=4 the feedback gain perfectly
    // compensates the filter's DC attenuation — the loop oscillates. We
    // stop just short of 4.0 to avoid exact marginal stability while still
    // allowing extreme resonance for self-oscillation effects.
    k_ = res * 3.99f;
  }

  /**
   * Process one input sample through the Moog ladder.
   *
   * Runs the nonlinear input stage, resonance feedback, and all four
   * cascaded one-pole filters. Returns the lowpass output — the final
   * stage's value.
   *
   * Call exactly once per sample in your audio callback.
   *
   * @param input  Input sample, typically -1.0 to +1.0.
   * @return       Lowpass-filtered output sample.
   *
   * Example:
   *   float saw = osc.process();
   *   float out = filter.process(saw);
   */
  float process(float input) {
    // Step 1: Apply resonance feedback.
    // Subtract a scaled portion of the last stage's output from the input.
    // This creates the resonance peak: energy at the cutoff frequency
    // makes it back through the loop largely unchanged; everything else
    // is attenuated more than the feedback restores it.
    float x = input - k_ * s_[3];

    // Step 2: Nonlinear input stage — the signature of the Moog sound.
    // The original Moog's transistor pairs soft-saturate at high levels,
    // generating warm harmonic distortion. We approximate with softclip:
    //   out = x / (1 + |x|)
    // This is a fast tanh approximation with the same shape but cheaper
    // to compute on the Cortex-M4 (no transcendental function required).
    //
    // Crucially, this also bounds the feedback loop: no matter how large
    // the feedback signal gets, softclip keeps x in roughly (-1, +1),
    // preventing runaway even at k = 3.99.
    x = softclip(x);

    // Steps 3–6: Four cascaded one-pole lowpass stages (the "ladder").
    // Each stage integrates toward its input at speed g_. Together they
    // produce 4 × 6 dB/oct = 24 dB/oct rolloff.
    //
    // This is identical to charging four RC circuits in series — each one
    // smooths the output of the previous stage a little more.
    s_[0] += g_ * (x - s_[0]);
    s_[1] += g_ * (s_[0] - s_[1]);
    s_[2] += g_ * (s_[1] - s_[2]);
    s_[3] += g_ * (s_[2] - s_[3]);

    return s_[3];
  }

  /**
   * Reset all internal filter state to zero.
   *
   * Call when switching patches or presets to clear residual energy that
   * might cause clicks or unexpected resonance artifacts.
   */
  void clear() { s_[0] = s_[1] = s_[2] = s_[3] = 0.0f; }

private:
  float g_ = 0.0f;       // One-pole coefficient (function of cutoff)
  float k_ = 0.0f;       // Resonance feedback amount [0.0, 3.99]
  float s_[4] = {};      // State of each of the four ladder stages
  float lastHz_ = -1.0f; // Cached frequency for change detection
};

} // namespace bb
