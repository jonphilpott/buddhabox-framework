/**
 * @file fx_engine.h
 * @brief Toolkit for delay-based effects (Clouds/Rings/Elements FxEngine).
 *
 * FxEngine builds reverbs, choruses, diffusers and pitch shifters from
 * many short delay lines sharing one buffer. Reverb, Diffuser, Chorus,
 * Ensemble and PitchShifter are all written with it, and you can write
 * your own the same way.
 *
 * HOW IT WORKS:
 * - All delay lines live in one circular buffer. A single write pointer
 *   moves backwards one step per sample, and each line is a fixed region
 *   ahead of it, so moving one pointer advances every line at once.
 * - Processing works like a tiny programmable DSP chip: an "accumulator"
 *   holds the signal being built. You load or read values into it, scale
 *   and filter it, and write it into delay lines. One effect is a short
 *   program run once per sample.
 * - Two slow cosine LFOs are built in, for modulating delay reads.
 *
 * BLACK PILL CHANGES (from Partials):
 * - The buffer size is a template parameter, so the memory is reserved at
 *   compile time (no heap) and shows up in PlatformIO's RAM report.
 * - Samples are stored as int16_t (2 bytes) instead of float (4 bytes),
 *   as Gillet's originals did. Reverbs and diffusers build up internal
 *   values well above ±1 (about ±2.3 with a full-scale input), so the
 *   16 bits cover ±4 rather than ±1 — like Clouds' "12-bit" format. That
 *   costs two bits of resolution: the noise floor is about -89 dBFS,
 *   still below what the 16-bit DAC reproduces.
 *
 * TYPICAL USE:
 *   setup:    line = engine.addDelay(n); ...; engine.allocate();
 *   process:  engine.start(); engine.read(in); ...; engine.write(line, 0);
 *
 * Derived from Clouds/Rings/Elements fx_engine.h, Copyright 2014 Emilie
 * Gillet. MIT licence.
 */

#pragma once

#include <cstring>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/osc/sine.h"

namespace bb {

/**
 * Shared-buffer delay network with an accumulator.
 *
 * @tparam SIZE  Buffer length in samples; must be a power of two.
 *               RAM used: SIZE * 2 bytes (e.g. 16384 -> 32 KB).
 *
 * Example — a single feedback echo of 0.1 s:
 *   FxEngine<8192> fx;
 *   FxEngine<8192>::Delay echo = fx.addDelay(4410);
 *   fx.allocate();
 *
 *   // Per sample:
 *   float out;
 *   fx.start();
 *   fx.read(in, 1.0f);           // acc = in
 *   fx.readTail(echo, 0.5f);     // acc += 0.5 * (oldest sample of echo)
 *   fx.write(out, 1.0f);         // out = acc
 *   fx.write(echo, 0.0f);        // echo <- acc, then acc = 0
 */
template <uint32_t SIZE> class FxEngine {
  // A power of two lets the index wrap with a bitwise AND (one cycle)
  // instead of a division. static_assert stops the build otherwise.
  static_assert((SIZE & (SIZE - 1)) == 0, "SIZE must be a power of two");

public:
  /** A delay line: a region of the shared buffer. */
  struct Delay {
    int32_t base = 0;
    int32_t length = 0;
  };

  /** Pass as an offset to address the far end of a line (oldest). */
  static const int32_t kTail = -1;

  /** @return  The buffer length in samples */
  static constexpr int32_t capacity() { return SIZE; }

  FxEngine() { reset(); }

  /** Step 1: forget all delay lines before declaring a new layout. */
  void reset() { total_ = 0; }

  /**
   * Step 2: declare a delay line. Lines are laid out one after another
   * with a one-sample gap, as in the original.
   * @param length  Length in samples
   * @return        A handle to use with read()/write()
   */
  Delay addDelay(int32_t length) {
    Delay d;
    d.base = total_;
    d.length = length;
    total_ += length + 1;
    return d;
  }

  /** @return  Samples used by the lines declared so far */
  int32_t used() const { return total_; }

  /** Step 3: silence every line and rewind. */
  void allocate() { clear(); }

  /** Silence every line. */
  void clear() {
    memset(buffer_, 0, sizeof(buffer_));
    writePtr_ = 0;
  }

  /**
   * Set an LFO's rate. The LFOs only step every 32 samples, which is
   * plenty for slow modulation.
   * @param index      0 or 1
   * @param frequency  Cycles per sample (Hz / SAMPLE_RATE)
   */
  void setLfoFrequency(int index, float frequency) {
    lfo_[index].initApproximate(frequency * 32.0f);
  }

  /** Begin one sample: advance all lines, clear the accumulator. */
  void start() {
    writePtr_ = (writePtr_ - 1) & kMask;
    accumulator_ = 0.0f;
    previousRead_ = 0.0f;
    // Step the LFOs once every 32 samples; in between, hold their value.
    if ((writePtr_ & 31) == 0) {
      lfoValue_[0] = lfo_[0].next();
      lfoValue_[1] = lfo_[1].next();
    } else {
      lfoValue_[0] = lfo_[0].value();
      lfoValue_[1] = lfo_[1].value();
    }
  }

  // ── Accumulator operations (all apply to the current sample) ─────────

  /** accumulator = value */
  void load(float value) { accumulator_ = value; }

  /** accumulator += value * scale */
  void read(float value, float scale = 1.0f) { accumulator_ += value * scale; }

  /** value = accumulator, then accumulator *= scale */
  void write(float &value, float scale = 1.0f) {
    value = accumulator_;
    accumulator_ *= scale;
  }

  /**
   * accumulator += line[offset] * scale.
   * @param d       The line
   * @param offset  Samples into the line; kTail reads its oldest sample
   * @param scale   Gain
   */
  void read(const Delay &d, int32_t offset, float scale) {
    float r = load16(buffer_[index(d, offset)]);
    previousRead_ = r;
    accumulator_ += r * scale;
  }

  /** accumulator += (oldest sample of d) * scale */
  void readTail(const Delay &d, float scale) { read(d, kTail, scale); }

  /** line[offset] = accumulator, then accumulator *= scale */
  void write(const Delay &d, int32_t offset, float scale) {
    buffer_[index(d, offset)] = store16(accumulator_);
    accumulator_ *= scale;
  }

  /** line[0] = accumulator, then accumulator *= scale */
  void write(const Delay &d, float scale) { write(d, 0, scale); }

  /**
   * The write half of an all-pass filter. Read the line's tail with gain
   * g first, then writeAllPass(line, -g): together they form
   *   y = -g*x + delayed,  stored = x + g*delayed.
   */
  void writeAllPass(const Delay &d, float scale) {
    write(d, 0, scale);
    accumulator_ += previousRead_;
  }

  /** One-pole low-pass of the accumulator through `state`. */
  void lp(float &state, float coefficient) {
    state += coefficient * (accumulator_ - state);
    accumulator_ = state;
  }

  /** One-pole high-pass of the accumulator through `state`. */
  void hp(float &state, float coefficient) {
    state += coefficient * (accumulator_ - state);
    accumulator_ -= state;
  }

  /**
   * Fractional read with linear interpolation:
   * accumulator += line[offset] * scale, where offset may be a fraction.
   */
  void interpolate(const Delay &d, float offset, float scale) {
    int32_t i = static_cast<int32_t>(offset);
    float frac = offset - static_cast<float>(i);
    float a = load16(buffer_[(writePtr_ + i + d.base) & kMask]);
    float b = load16(buffer_[(writePtr_ + i + d.base + 1) & kMask]);
    float x = a + (b - a) * frac;
    previousRead_ = x;
    accumulator_ += x * scale;
  }

  /**
   * As above, with the offset swept by an LFO.
   * @param lfo        0 or 1
   * @param amplitude  Sweep depth in samples
   */
  void interpolate(const Delay &d, float offset, int lfo, float amplitude,
                   float scale) {
    interpolate(d, offset + amplitude * lfoValue_[lfo], scale);
  }

private:
  static const int32_t kMask = static_cast<int32_t>(SIZE) - 1;

  int32_t index(const Delay &d, int32_t offset) const {
    int32_t o = offset == kTail ? d.length - 1 : offset;
    return (writePtr_ + d.base + o) & kMask;
  }

  /** Largest magnitude a stored sample can hold (see the file header). */
  static constexpr float kHeadroom = 4.0f;

  /** float -> int16, clamped so overflow clips instead of wrapping. */
  static int16_t store16(float x) {
    x = clampf(x, -kHeadroom, kHeadroom);
    return static_cast<int16_t>(x * (32767.0f / kHeadroom));
  }

  /** int16 -> float (multiply by a constant: cheaper than divide). */
  static float load16(int16_t s) {
    return static_cast<float>(s) * (kHeadroom / 32767.0f);
  }

  int16_t buffer_[SIZE];
  int32_t total_ = 0;
  int32_t writePtr_ = 0;
  float accumulator_ = 0.0f;
  float previousRead_ = 0.0f;
  float lfoValue_[2] = {0.0f, 0.0f};
  CosineOscillator lfo_[2];
};

/**
 * Pick a delay-length scale that keeps an effect's timing in seconds
 * (rateScale = SAMPLE_RATE / original rate) but shrinks it if the lines
 * would not fit in the buffer.
 *
 * @param rateScale     sampleRate / the effect's original rate
 * @param totalLengths  Sum of the effect's line lengths at scale 1
 * @param numLines      How many lines (each costs one gap sample)
 * @param capacity      The FxEngine's SIZE
 * @return              The scale to multiply every length by
 *
 * Example:
 *   scale_ = fitScale(SAMPLE_RATE / 32000.0f, 16375, 10, SIZE);
 */
inline float fitScale(float rateScale, float totalLengths, int numLines,
                      int32_t capacity) {
  // Leave a couple of samples per line for rounding up each length.
  float fit = static_cast<float>(capacity - 2 * numLines - 2) / totalLengths;
  return rateScale < fit ? rateScale : fit;
}

} // namespace bb
