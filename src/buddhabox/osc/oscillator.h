/**
 * @file oscillator.h
 * @brief Audio-rate oscillator with sine, saw, square, and triangle waveforms.
 *
 * This is the bread-and-butter of any synthesizer — it generates periodic
 * waveforms at audible frequencies. You set a frequency (in Hz) and a waveform
 * shape, then call process() once per sample to get the next output value.
 *
 * HOW IT WORKS — The Phase Accumulator Pattern:
 * ─────────────────────────────────────────────
 * Instead of tracking time in seconds, we maintain a "phase" variable that
 * ramps from 0.0 to 1.0, then wraps back to 0.0. One full 0→1 cycle = one
 * period of the waveform. The speed of this ramp determines the pitch:
 *
 *   phaseIncrement = frequency / sampleRate
 *
 * For example, at 440 Hz and 44100 sample rate:
 *   phaseIncrement = 440 / 44100 ≈ 0.00998
 *   (phase advances ~1% per sample, completing one cycle in ~100 samples)
 *
 * We then use the phase value to compute the waveform shape. This approach
 * is elegant because:
 *   1. It works for any frequency without needing a lookup table per pitch
 *   2. The phase stays in a clean 0–1 range (no growing numbers over time)
 *   3. It's trivial to reset, sync, or modulate the phase
 *
 * Example usage:
 *   Oscillator osc;
 *   osc.setWaveform(Waveform::SAW);
 *   osc.setFrequency(440.0f);    // Concert A
 *   osc.setAmplitude(0.5f);      // Half volume
 *
 *   // In your audio callback:
 *   float sample = osc.process(); // Returns -0.5 to +0.5
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Waveform type selector.
 *
 * Each shape has a distinct sonic character:
 *   SINE     — Pure tone, no harmonics. Smooth and mellow.
 *   SAW      — All harmonics (1/n amplitude). Bright and buzzy.
 *   SQUARE   — Odd harmonics only (1/n amplitude). Hollow, clarinet-like.
 *   TRIANGLE — Odd harmonics only (1/n² amplitude). Softer than square.
 *
 * HARMONIC CONTENT NOTE: In digital audio, these "naive" waveforms will
 * produce aliasing at higher frequencies because the sharp edges contain
 * harmonics above the Nyquist frequency (sampleRate/2). For this ambient
 * project that's generally fine — we're often using low frequencies and
 * filtering the output anyway. For a production synth you'd want
 * band-limited versions (PolyBLEP, wavetable, etc.).
 */
enum class Waveform { SINE, SAW, SQUARE, TRIANGLE };

/**
 * Audio-rate oscillator with sine, saw, square, and triangle waveforms.
 *
 * See the file header above for how it works and usage examples.
 */
class Oscillator {
public:
  Oscillator() = default;

  /**
   * Set the oscillator frequency in Hz.
   *
   * Internally this precomputes the phase increment so that process()
   * only needs a single addition per sample — no division in the hot path.
   *
   * @param hz  Frequency in Hertz (e.g., 440.0 for concert A).
   *            Typical audible range: 20 Hz – 20000 Hz.
   *            For sub-bass drones, try 30–80 Hz.
   *
   * Example:
   *   osc.setFrequency(midiToFreq(NOTE_C4)); // Middle C ≈ 261.6 Hz
   */
  void setFrequency(float hz) {
    freq_ = hz;
    // Phase increment = how much phase advances per sample.
    // A 440 Hz tone at 44100 Hz sample rate: 440/44100 ≈ 0.00998
    // meaning we complete one full 0→1 cycle every ~100 samples.
    phaseInc_ = hz / SAMPLE_RATE;
  }

  /**
   * Set the waveform shape.
   * @param w  One of: Waveform::SINE, SAW, SQUARE, TRIANGLE
   */
  void setWaveform(Waveform w) { waveform_ = w; }

  /**
   * Set the output amplitude (volume scaling).
   *
   * @param a  Amplitude multiplier. 1.0 = full scale (-1 to +1 output).
   *           0.5 = half volume. 0.0 = silence.
   *           When mixing multiple oscillators, keep each around 0.2–0.4
   *           to avoid clipping when they're summed together.
   */
  void setAmplitude(float a) { amplitude_ = a; }

  /**
   * Generate the next audio sample.
   *
   * Call this exactly once per sample in your audio callback.
   * Each call advances the internal phase by one step.
   *
   * @return  The next sample value, range: [-amplitude, +amplitude]
   *
   * Example:
   *   void audioCallback(int16_t* buf, uint16_t len) {
   *       for (int i = 0; i < len; i += 2) {
   *           float sample = osc.process();
   *           int16_t s = (int16_t)(sample * 32000.0f);
   *           buf[i] = s;      // Left channel
   *           buf[i+1] = s;    // Right channel
   *       }
   *   }
   */
  float process() {
    float out = 0.0f;

    // Step 1: Compute waveform value from current phase (0.0–1.0)
    switch (waveform_) {
    case Waveform::SINE:
      // Use a pre-computed wavetable instead of a runtime sine calculation.
      // The table (lut_sine.h, in flash) stores 512 evenly spaced sine
      // values covering one full cycle. We find where our phase falls in the
      // table and interpolate between the two nearest entries (linear
      // interpolation = lerp).
      //
      // This is faster than even the parabolic fastSin() because it replaces
      // all floating-point math with two table lookups and a lerp, which
      // is just 3 multiply-add operations on the Cortex-M4 FPU.
      out = sineTableLookup(phase_);
      break;

    case Waveform::SAW:
      // Linear ramp from -1 to +1 across the phase cycle.
      // phase=0 → -1, phase=0.5 → 0, phase=1 → +1
      out = 2.0f * phase_ - 1.0f;
      break;

    case Waveform::SQUARE:
      // +1 for the first half of the cycle, -1 for the second half.
      // The abrupt transition creates a rich harmonic spectrum.
      out = phase_ < 0.5f ? 1.0f : -1.0f;
      break;

    case Waveform::TRIANGLE:
      // Rises linearly from -1 to +1 in the first half,
      // then falls from +1 to -1 in the second half.
      // Mathematically: 1 - |2 * (2*phase - 1)| but split into
      // two linear segments for clarity.
      out = phase_ < 0.5f ? 4.0f * phase_ - 1.0f  // 0→0.5: ramps -1 to +1
                          : 3.0f - 4.0f * phase_; // 0.5→1: ramps +1 to -1
      break;
    }

    // Step 2: Advance phase and wrap around at 1.0
    // This keeps phase bounded — without wrapping, it would grow forever
    // and eventually lose floating-point precision.
    phase_ += phaseInc_;
    if (phase_ >= 1.0f)
      phase_ -= 1.0f;

    // Step 3: Apply amplitude scaling
    return out * amplitude_;
  }

  /** Get the current phase position (0.0–1.0). Useful for syncing oscillators.
   */
  float getPhase() const { return phase_; }

  /** Manually set the phase. Use to sync one oscillator to another. */
  void setPhase(float p) { phase_ = p; }

private:
  /**
   * Look up a sine value from the shared flash wavetable (lut_sine.h).
   *
   * A wavetable stores one full cycle of a waveform as a pre-computed
   * array: 512 evenly spaced sine values. We find where our phase falls in
   * the table and blend the two nearest entries (linear interpolation),
   * which costs two loads and one multiply-add. The error is below
   * -96 dB, the noise floor of 16-bit audio.
   *
   * The table is `static const` data, so it lives in flash rather than
   * RAM, and every module in the framework shares the same copy.
   *
   * @param phase  Current oscillator phase in [0.0, 1.0)
   * @return       Interpolated sine value in [-1.0, +1.0]
   */
  static float sineTableLookup(float phase) {
    return interpolate(lutSine(), phase, kSineTableSize);
  }

  Waveform waveform_ = Waveform::SINE;
  float freq_ = 440.0f;
  float phase_ = 0.0f;
  float phaseInc_ = 440.0f / SAMPLE_RATE;
  float amplitude_ = 1.0f;
};

} // namespace bb
