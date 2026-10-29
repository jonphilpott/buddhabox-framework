/**
 * @file chorus.h
 * @brief Rings' stereo chorus: gentle, slow and wide.
 *
 * HOW IT WORKS: the input (left + right) goes into one delay line. Four
 * read heads, each swept slowly by a sine or cosine LFO, read it back at
 * 15-30 ms. A delayed copy whose delay keeps changing is slightly
 * detuned, so mixing it with the dry signal gives the "several players at
 * once" thickening. Left and right use different heads and LFO phases,
 * which spreads the sound across the stereo field.
 *
 * BLACK PILL CHANGE: Partials called sin() and cos() four times per
 * sample; this port reads the flash sine table instead.
 *
 * MEMORY: Chorus<2048> (the default) uses 4 KB and holds Rings' delays at
 * 44.1 kHz.
 *
 * Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/fx_engine.h"

namespace bb {

/**
 * Stereo chorus.
 *
 * @tparam SIZE  Delay buffer in samples (power of two); RAM = SIZE * 2
 *
 * Example:
 *   Chorus<> chorus;
 *   chorus.setAmount(0.5f);
 *   chorus.setDepth(0.4f);
 *
 *   float l = osc.process(), r = l;
 *   chorus.process(l, r);
 */
template <uint32_t SIZE = 2048> class Chorus {
public:
  Chorus() { init(); }

  /**
   * Lay out the delay and reset the LFOs.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    float rateScale = sampleRate / 48000.0f;
    scale_ = fitScale(rateScale, 2047.0f, 1, SIZE);
    engine_.reset();
    line_ = engine_.addDelay(static_cast<int32_t>(2047 * scale_ + 0.5f));
    engine_.allocate();
    // Rings' per-sample LFO increments at 48 kHz: 0.2 Hz and 0.26 Hz.
    increment1_ = 4.17e-06f / rateScale;
    increment2_ = 5.417e-06f / rateScale;
    phase1_ = phase2_ = 0.0f;
  }

  /** @param x  Wet amount, 0..1 (the dry is turned down by half of it) */
  void setAmount(float x) { amount_ = x; }

  /** @param x  Sweep depth, 0..1 (up to ±8 ms) */
  void setDepth(float x) { depth_ = x * 384.0f; }

  /** Silence the delay line. */
  void clear() { engine_.clear(); }

  /**
   * Process one stereo sample in place.
   * @param left   Left in/out
   * @param right  Right in/out
   */
  void process(float &left, float &right) {
    FxEngine<SIZE> &c = engine_;
    c.start();
    const float dryAmount = 1.0f - amount_ * 0.5f;

    // 1. Two slow LFOs, each with a sine and a cosine (90 degrees apart).
    phase1_ += increment1_;
    if (phase1_ >= 1.0f)
      phase1_ -= 1.0f;
    phase2_ += increment2_;
    if (phase2_ >= 1.0f)
      phase2_ -= 1.0f;
    float sin1 = sineFromPhase(phase1_);
    float cos1 = sineFromPhase(phase1_ + 0.25f);
    float sin2 = sineFromPhase(phase2_);
    float cos2 = sineFromPhase(phase2_ + 0.25f);

    // 2. Mono sum into the delay line.
    c.read(left, 0.5f);
    c.read(right, 0.5f);
    c.write(line_, 0.0f);

    // 3. Two swept heads per side (25 ms and 16.7 ms centres).
    float depth = depth_ * scale_;
    float wet;
    c.interpolate(line_, sin1 * depth + 1200.0f * scale_, 0.5f);
    c.interpolate(line_, sin2 * depth + 800.0f * scale_, 0.5f);
    c.write(wet, 0.0f);
    left = wet * amount_ + left * dryAmount;
    c.interpolate(line_, cos1 * depth + 800.0f * scale_, 0.5f);
    c.interpolate(line_, cos2 * depth + 1200.0f * scale_, 0.5f);
    c.write(wet, 0.0f);
    right = wet * amount_ + right * dryAmount;
  }

private:
  FxEngine<SIZE> engine_;
  typename FxEngine<SIZE>::Delay line_;
  float scale_ = 1.0f;
  float amount_ = 0.5f;
  float depth_ = 0.5f * 384.0f;
  float phase1_ = 0.0f, phase2_ = 0.0f;
  float increment1_ = 0.0f, increment2_ = 0.0f;
};

} // namespace bb
