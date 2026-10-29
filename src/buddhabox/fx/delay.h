/**
 * @file delay.h
 * @brief Delay line with interpolated reads for echo, chorus, and
 * Karplus-Strong.
 *
 * A delay line is simply a buffer that stores past audio samples and lets you
 * read them back after a specified time. It's one of the most versatile DSP
 * building blocks:
 *
 *   ECHO/DELAY:     write(input + read(time) * feedback)
 *   CHORUS/FLANGER: write(input), mix input with read(lfo_modulated_time)
 *   KARPLUS-STRONG: write(noise_burst + read(pitch_period) * decay)
 *                   → produces surprisingly realistic plucked string sounds!
 *
 * HOW THE CIRCULAR BUFFER WORKS:
 * ──────────────────────────────
 * Instead of shifting all samples forward each step (very slow), we use a
 * "write pointer" that moves through the buffer in a circle:
 *
 *   [sample5][sample6][sample7][sample0][sample1][sample2][sample3][sample4]
 *                               ^writePos
 *
 * When writePos reaches the end, it wraps back to 0. To read a sample from
 * N steps ago, we simply look at (writePos - N), wrapping as needed.
 * This gives us O(1) writes and reads regardless of buffer size.
 *
 * FRACTIONAL DELAY & INTERPOLATION:
 * ─────────────────────────────────
 * When the delay time isn't a whole number of samples (e.g., 100.7 samples),
 * we use linear interpolation between the two nearest samples. This is
 * essential for:
 *   - Smooth delay time modulation (LFO-controlled chorus/flanger)
 *   - Accurate pitch in Karplus-Strong (e.g., 440 Hz needs exactly
 *     44100/440 = 100.227 samples of delay)
 *
 * INT16 STORAGE FORMAT:
 * ────────────────────
 * The buffer stores samples as int16_t (2 bytes each) instead of float
 * (4 bytes each), halving the RAM cost. The API still uses floats — the
 * conversion happens internally at write() and read() boundaries.
 *
 * The tradeoff is quantization noise at -96 dB (the noise floor of 16-bit
 * audio). This is completely inaudible, especially in an ambient context
 * where the signal is already noise/texture-based. CD audio is 16-bit
 * and nobody complains about its noise floor.
 *
 * MEMORY BUDGET (int16_t storage):
 * ───────────────────────────────
 * Each sample = 2 bytes. Compare to 4 bytes with float storage.
 *
 *   DelayLine<1024>   →  ~23 ms,   2 KB  (Karplus-Strong, comb filtering)
 *   DelayLine<4096>   →  ~93 ms,   8 KB  (short echo, slapback delay)
 *   DelayLine<8192>   → ~186 ms,  16 KB  (medium echo)
 *   DelayLine<16384>  → ~371 ms,  32 KB  (longer echo — feasible now!)
 *   DelayLine<22050>  → ~500 ms,  43 KB  (half-second delay, tight but
 * possible)
 *
 * Example usage — Simple echo with feedback:
 *   DelayLine<8192> delay;
 *
 *   // In audio callback:
 *   float delayed = delay.read(6000.0f);            // Read 6000 samples back
 * (~136ms) float output = input * 0.7f + delayed * 0.3f;   // Mix dry + wet
 *   delay.write(input + delayed * 0.5f);             // Write with 50%
 * feedback
 *
 * Example usage — Karplus-Strong plucked string:
 *   DelayLine<1024> string;
 *   float delaySamples = SAMPLE_RATE / midiToFreq(NOTE_C4);  // ~168.7 samples
 *
 *   // Initial excitation: fill delay with noise burst
 *   // Then in audio callback:
 *   float sample = string.read(delaySamples);
 *   float filtered = sample * 0.5f + prevSample * 0.5f; // Simple lowpass =
 * decay string.write(filtered); prevSample = sample;
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include <cstring> // for memset

namespace bb {

/**
 * Circular buffer delay line with int16_t storage and linear interpolation.
 *
 * The API accepts and returns floats in [-1.0, +1.0]. Internally, samples
 * are stored as int16_t to halve memory usage. The float↔int16 conversion
 * adds negligible quantization noise (-96 dB).
 *
 * @tparam MAX_SAMPLES  Maximum delay length in samples. Determines buffer
 *                      size: MAX_SAMPLES * sizeof(T) bytes.
 *                      Default: 4096 (~93 ms at 44.1 kHz, uses 8 KB RAM)
 * @tparam T            Storage type: int16_t (default, 2 bytes/sample,
 *                      values clamped to ±1) or float (4 bytes/sample,
 *                      any range). Physical models whose loops briefly
 *                      exceed ±1 (the Tube's reed) use float.
 *
 * Example — float storage for a signal with headroom:
 *   DelayLine<1024, float> bore; // 4 KB, no clamping
 */
template <uint16_t MAX_SAMPLES = 4096, typename T = int16_t> class DelayLine {
public:
  DelayLine() { clear(); }

  /**
   * Zero out the entire delay buffer.
   * Call this when starting a new sound to prevent stale audio from the
   * previous patch bleeding through.
   */
  void clear() {
    memset(buffer_, 0, sizeof(buffer_));
    writePos_ = 0;
  }

  /**
   * Write one sample into the delay buffer.
   *
   * The float input is converted to int16_t for storage. Values outside
   * [-1.0, +1.0] are clamped to prevent integer overflow (which would
   * wrap around and sound terrible).
   *
   * @param sample  The sample to store, expected range [-1.0, +1.0]
   *
   * Example — echo with feedback:
   *   delay.write(input + delay.read(time) * 0.5f);
   */
  void write(float sample) {
    // Clamp to [-1, +1] then scale to int16 range [-32767, +32767].
    // We use 32767 (not 32768) to keep the range symmetric — this
    // avoids a subtle asymmetry where -1.0 could map to -32768 but
    // +1.0 maps to only +32767.
    store(buffer_[writePos_], sample);

    // Advance write position, wrapping at buffer end.
    writePos_ = (writePos_ + 1) % MAX_SAMPLES;
  }

  /**
   * Read a sample from the delay buffer at a fractional delay time.
   *
   * The int16_t stored values are converted back to float, and linear
   * interpolation is applied between adjacent samples for smooth
   * modulation of the delay time.
   *
   * @param delaySamples  How far back to read, in samples.
   *                      0.0 = the most recently written sample
   *                      MAX_SAMPLES-1 = the oldest sample in the buffer
   *                      Fractional values are interpolated.
   * @return  The delayed sample as a float in [-1.0, +1.0]
   *
   * Example:
   *   // Fixed delay: 2000 samples ≈ 45 ms
   *   float echo = delay.read(2000.0f);
   *
   *   // LFO-modulated delay for chorus effect:
   *   float time = 200.0f + lfo.process() * 100.0f;  // 100-300 samples
   *   float chorus = delay.read(time);
   */
  float read(float delaySamples) const {
    // Step 1: Clamp delay time to valid range
    if (delaySamples < 0.0f)
      delaySamples = 0.0f;
    if (delaySamples > MAX_SAMPLES - 1)
      delaySamples = MAX_SAMPLES - 1;

    // Step 2: Calculate the read position in the circular buffer.
    // We subtract from writePos because the most recent sample is at
    // (writePos - 1) and older samples are further back.
    float pos = (float)writePos_ - delaySamples - 1.0f;
    if (pos < 0.0f)
      pos += MAX_SAMPLES; // Wrap around if we go negative

    // Step 3: Read the two nearest integer positions and convert to float.
    // The division by 32767.0f undoes the scaling applied in write().
    uint16_t idx0 = (uint16_t)pos % MAX_SAMPLES;
    uint16_t idx1 = (idx0 + 1) % MAX_SAMPLES;
    float s0 = load(buffer_[idx0]);
    float s1 = load(buffer_[idx1]);

    // Step 4: Linear interpolation between the two samples.
    // If pos = 100.7, we blend 30% of s0 with 70% of s1.
    // This prevents zipper noise when the delay time changes smoothly.
    // pos is never negative here, so truncating to an integer is the same
    // as floorf(), without the library call.
    float frac = pos - (float)(uint32_t)pos;
    return lerpf(s0, s1, frac);
  }

  /**
   * Read from the delay at a specific tap point (alias for read).
   *
   * Functionally identical to read() — the separate name makes code
   * more readable when using multiple taps from the same delay line.
   *
   * @param delaySamples  Delay time in samples
   * @return  The delayed sample as a float in [-1.0, +1.0]
   *
   * Example:
   *   // Multi-tap delay with three echoes
   *   float tap1 = delay.tap(1000.0f) * 0.6f;
   *   float tap2 = delay.tap(2000.0f) * 0.4f;
   *   float tap3 = delay.tap(3500.0f) * 0.2f;
   *   float output = input + tap1 + tap2 + tap3;
   */
  float tap(float delaySamples) const { return read(delaySamples); }

  /** @return  The maximum delay in samples (the template parameter). */
  uint16_t maxDelay() const { return MAX_SAMPLES; }

  /**
   * Read the sample written exactly `age` samples ago (no interpolation).
   *
   * @param age  0 = the most recent sample, up to MAX_SAMPLES - 1
   * @return     That sample as a float
   *
   * Example:
   *   float last = delay.at(0);
   */
  float at(uint16_t age) const {
    // The most recent sample sits just behind the write position.
    // Adding MAX_SAMPLES before subtracting keeps the index positive.
    uint32_t i = (uint32_t)writePos_ + MAX_SAMPLES - 1u - age;
    return load(buffer_[i % MAX_SAMPLES]);
  }

  /**
   * Read at a fractional delay with 4-point Hermite (cubic) interpolation.
   *
   * Linear interpolation (read()) slightly dulls high frequencies, and
   * the dulling changes as the fraction changes. For tuned feedback loops
   * such as strings, that would make the tone and decay vary with pitch.
   * Hermite interpolation fits a smooth curve through four neighbouring
   * samples, which is much flatter across the spectrum.
   *
   * @param delaySamples  Delay in samples, same meaning as read();
   *                      clamped to 1 .. MAX_SAMPLES - 3
   * @return  The interpolated delayed sample
   *
   * Example:
   *   float s = string.readHermite(100.227f); // 440 Hz at 44.1 kHz
   */
  float readHermite(float delaySamples) const {
    delaySamples = clampf(delaySamples, 1.0f, MAX_SAMPLES - 3.0f);
    uint16_t k = (uint16_t)delaySamples;
    float f = delaySamples - (float)k;
    // Four samples around the read point: one newer, three older.
    const float xm1 = at(k - 1);
    const float x0 = at(k);
    const float x1 = at(k + 1);
    const float x2 = at(k + 2);
    // Same Horner-form cubic as interpolateHermite() in dsp_common.h.
    const float c = (x1 - xm1) * 0.5f;
    const float v = x0 - x1;
    const float w = c + v;
    const float a = w + v + (x2 - x0) * 0.5f;
    const float bNeg = w + a;
    return (((a * f) - bNeg) * f + c) * f + x0;
  }

  /**
   * Run one sample through an all-pass filter built on this delay line.
   *
   * An all-pass passes every frequency at the same level but delays them
   * by different amounts. Physical models use it for dispersion (a stiff
   * piano string, where high partials travel faster) and reverbs use it
   * to smear echoes. This call reads, writes and advances the line.
   *
   *   delayed = the sample from `delay` samples ago
   *   w       = in + g * delayed      (stored)
   *   out     = delayed - g * w
   *
   * @param in     Input sample
   * @param delay  Loop length in samples, 1 .. MAX_SAMPLES
   * @param g      Coefficient, -1.0 to 1.0 (|g| < 1 for stability)
   * @return       The all-passed sample
   *
   * Example:
   *   float s = stretch.allpass(s, 37, -0.4f);
   */
  float allpass(float in, uint16_t delay, float g) {
    float delayed = at(delay - 1); // age 0 is one sample ago, before write
    float w = in + g * delayed;
    write(w);
    return -w * g + delayed;
  }

private:
  // ── Storage conversion ────────────────────────────────────────────────
  // C++ picks the overload that matches the slot's type, so the int16_t
  // and float versions of this class share every other line of code.

  /** Store into a 16-bit slot: clamp to ±1, scale to ±32767. */
  static void store(int16_t &slot, float x) {
    // Clamping first prevents integer overflow, which would wrap around
    // (a loud click) instead of clipping.
    slot = (int16_t)(clampf(x, -1.0f, 1.0f) * 32767.0f);
  }
  /** Store into a float slot unchanged. */
  static void store(float &slot, float x) { slot = x; }
  /** Read a 16-bit slot back to ±1.0 (multiply: cheaper than divide). */
  static float load(int16_t s) { return (float)s * (1.0f / 32767.0f); }
  /** Read a float slot unchanged. */
  static float load(float s) { return s; }

  T buffer_[MAX_SAMPLES]; // Circular sample buffer
  uint16_t writePos_;     // Current write position (0 to MAX_SAMPLES-1)
};

} // namespace bb
