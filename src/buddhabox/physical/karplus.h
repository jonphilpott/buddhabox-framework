/**
 * @file karplus.h
 * @brief Karplus-Strong plucked string synthesis.
 *
 * Karplus-Strong is one of the most elegant algorithms in computer music:
 * a short burst of noise fed into a delay line with filtered feedback
 * produces a surprisingly realistic plucked string sound. The algorithm
 * was discovered by Alex Strong and refined by Kevin Karplus in 1983.
 *
 * HOW IT WORKS:
 * ─────────────
 *
 *   1. EXCITE: Fill the delay line with a burst of white noise.
 *      This is the "pluck" — the initial energy that sets the string
 * vibrating.
 *
 *   2. FEEDBACK LOOP: Each sample, read the oldest value from the delay,
 *      pass it through a simple lowpass filter, and write it back.
 *
 *      ┌──────────────────────────────────────┐
 *      │                                      │
 *      │  noise burst ──▶ [delay line] ──▶ output
 *      │                    ▲      │
 *      │                    │      ▼
 *      │                    └── lowpass
 *      │                      (averaging)
 *      └──────────────────────────────────────┘
 *
 *   3. The delay length determines the pitch:
 *        delaySamples = SAMPLE_RATE / frequency
 *      For A4 (440 Hz): 44100 / 440 ≈ 100.2 samples
 *
 *   4. The lowpass filter causes high frequencies to decay faster than low
 *      frequencies — just like a real string! The result is a tone that
 *      starts bright (the "attack" of the pluck) and mellows over time.
 *
 * WHY IT SOUNDS SO GOOD:
 * ─────────────────────
 * A real vibrating string works almost identically: energy bounces back
 * and forth along the string (the delay line), losing a tiny bit of high
 * frequency content at each reflection (the lowpass filter). Karplus-Strong
 * is essentially a physical model of a string, not just a synthesis trick.
 *
 * DAMPING:
 * ────────
 * The damping parameter (0.0–1.0) controls how quickly the string decays:
 *   0.0 = maximum sustain (rings for a very long time)
 *   0.5 = natural pluck (like a nylon guitar string)
 *   0.9 = very short, muted pluck (like a palm-muted guitar)
 *   1.0 = completely dead (no resonance)
 *
 * Example usage — Simple plucked string:
 *   KarplusStrong string;
 *   string.pluck(midiToFreq(NOTE_C4));   // Pluck at Middle C
 *
 *   // In audio callback:
 *   float sample = string.process();
 *
 * Example usage — Generative harp with tempo clock:
 *   KarplusStrong string;
 *   TempoClock clock(90.0f, 2);   // 90 BPM, eighth notes
 *   LFSR rng(42);
 *   uint8_t scale[] = {60, 62, 64, 67, 69};  // C pentatonic
 *
 *   // In audio callback:
 *   if (clock.process()) {
 *       uint8_t note = scale[rng.nextRange(0, 4)];
 *       string.pluck(midiToFreq(note));
 *   }
 *   float sample = string.process();
 *
 * Example usage — Multiple strings for polyphony:
 *   KarplusStrong strings[4];
 *   int nextString = 0;
 *
 *   // Round-robin plucking across 4 strings:
 *   if (clock.process()) {
 *       strings[nextString].pluck(midiToFreq(note));
 *       nextString = (nextString + 1) % 4;
 *   }
 *
 *   // Mix all strings together:
 *   float mix = 0.0f;
 *   for (int i = 0; i < 4; i++) mix += strings[i].process();
 *   float out = softclip(mix, 1.5f);  // Gentle saturation to tame peaks
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/delay.h"
#include "buddhabox/noise/noise.h"

namespace bb {

/**
 * Karplus-Strong plucked string synthesizer.
 *
 * Combines a noise exciter, delay line, and lowpass feedback filter into
 * a single module that produces realistic plucked string sounds.
 *
 * @tparam MAX_SAMPLES  Maximum delay line size. Determines the lowest
 *                      playable pitch: minFreq = SAMPLE_RATE / MAX_SAMPLES.
 *                      Default 1024 → lowest pitch ≈ 43 Hz (just below F1).
 *                      Use 2048 for bass notes down to ≈ 21.5 Hz.
 */
template <uint16_t MAX_SAMPLES = 1024> class KarplusStrong {
public:
  /**
   * @param seed  Seed for the internal noise generator. Use different
   *              seeds for each string instance to avoid identical attacks.
   */
  KarplusStrong(uint32_t seed = 12345) : noise_(seed) {}

  /**
   * Set the damping factor (how quickly the string decays).
   *
   * @param d  Damping amount, 0.0 to 1.0.
   *           0.0 = longest sustain (ringing metallic string)
   *           0.5 = natural plucked string
   *           0.9 = very muted, short decay
   *           Clamped internally to [0.0, 1.0].
   *
   * Example:
   *   string.setDamping(0.3f);  // Bright, sustained pluck
   */
  void setDamping(float d) { damping_ = clampf(d, 0.0f, 1.0f); }

  /**
   * Trigger a pluck at the specified frequency.
   *
   * This fills the delay line with a burst of white noise (the "excitation")
   * and sets the delay length to match the requested pitch.
   *
   * @param freq  Desired pitch in Hz (e.g., 440.0 for A4).
   *              Must be at least SAMPLE_RATE / MAX_SAMPLES.
   *              Higher frequencies work better (shorter delay = less RAM).
   *
   * Example:
   *   string.pluck(midiToFreq(NOTE_A4));  // Pluck at 440 Hz
   *   string.pluck(330.0f);               // Pluck at E4
   */
  void pluck(float freq) {
    // Calculate the delay length in samples for this pitch.
    // For 440 Hz at 44100 SR: 44100 / 440 = 100.227 samples.
    // The fractional part is handled by the delay line's interpolation.
    delaySamples_ = (float)SAMPLE_RATE / freq;

    // Clamp to valid range
    if (delaySamples_ < 1.0f)
      delaySamples_ = 1.0f;
    if (delaySamples_ > MAX_SAMPLES - 1)
      delaySamples_ = MAX_SAMPLES - 1;

    // Fill the delay line with a noise burst — this is the "pluck" energy.
    // We only fill as many samples as the delay length, not the whole buffer.
    delay_.clear();
    uint16_t burstLen = (uint16_t)delaySamples_;
    for (uint16_t i = 0; i < burstLen; i++) {
      delay_.write(noise_.process());
    }

    // Reset the lowpass filter state for a clean attack
    prevSample_ = 0.0f;
    active_ = true;
  }

  /**
   * Generate the next audio sample.
   *
   * Call this once per sample in your audio callback. After a pluck(),
   * this outputs the decaying string sound. When the string has decayed
   * to silence, it returns 0.0.
   *
   * @return  Audio sample in approximately [-1.0, +1.0], decaying over time.
   *
   * Example:
   *   float sample = string.process();
   *   int16_t out = (int16_t)(sample * 32000.0f);
   */
  float process() {
    if (!active_)
      return 0.0f;

    // Step 1: Read the delayed sample (the "traveling wave")
    float sample = delay_.read(delaySamples_);

    // Step 2: Apply the lowpass filter (simple two-point average).
    // This is the classic Karplus-Strong filter: average the current
    // sample with the previous one. This attenuates high frequencies
    // by ~6 dB/octave, mimicking how real strings lose brightness
    // as they ring. Damping then scales the loop gain: damping = 0 keeps
    // the full gain (longest sustain), damping = 1 halves it each pass
    // (a quick, muted pluck).
    float feedback = (sample + prevSample_) * 0.5f * (1.0f - damping_ * 0.5f);

    prevSample_ = sample;

    // Step 3: Write the filtered sample back into the delay line.
    // This creates the feedback loop — the sound circulates through
    // the delay, losing a bit of energy each round trip.
    delay_.write(feedback);

    // Step 4: Check if the string has decayed to silence.
    // We use a very small threshold — once the signal is this quiet,
    // it's inaudible and we can stop processing.
    if (fabsf(sample) < 0.0001f && fabsf(feedback) < 0.0001f) {
      active_ = false;
    }

    return sample;
  }

  /** @return  true if the string is still ringing. */
  bool isActive() const { return active_; }

private:
  DelayLine<MAX_SAMPLES> delay_; // The "string" — stores the traveling wave
  WhiteNoise noise_;             // Excitation source for plucks
  float delaySamples_ = 100.0f;  // Current pitch (in samples)
  float prevSample_ = 0.0f;      // Previous sample for the lowpass filter
  float damping_ = 0.0f;         // Damping factor (0 = sustain, 1 = muted)
  bool active_ = false;          // Whether the string is currently ringing
};

} // namespace bb
