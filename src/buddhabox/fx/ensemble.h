/**
 * @file ensemble.h
 * @brief Rings' string-machine ensemble: a lush, shimmering chorus.
 *
 * The sound of 1970s string machines (Solina, ARP String Ensemble).
 *
 * HOW IT WORKS: like a chorus, but with three delay taps per side whose
 * modulation is spread 120 degrees apart, and each tap swept by two LFOs
 * at once: a slow one (0.75 Hz) for the broad "swirl" and a fast one
 * (6.6 Hz) for vibrato-like shimmer. With three taps at three phases,
 * some are always going sharp while others go flat, so the average pitch
 * stays steady while the sound thickens.
 *
 * BLACK PILL CHANGE: Partials computed six double-precision sin() calls
 * per sample (thousands of cycles on the M4, which has no double FPU);
 * this port reads the same quantised phases from the flash sine table.
 *
 * MEMORY: Ensemble<4096> (the default) uses 8 KB and holds Rings' delays
 * at 44.1 kHz.
 *
 * Derived from Rings, Copyright 2015 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/fx_engine.h"

namespace bb {

/**
 * Stereo string-ensemble chorus.
 *
 * @tparam SIZE  Delay buffer in samples (power of two); RAM = SIZE * 2
 *
 * Example:
 *   StringSynthOscillator strings;
 *   Ensemble<> ensemble;
 *   ensemble.setAmount(0.8f);
 *
 *   float l = strings.process() * 0.3f, r = l;
 *   ensemble.process(l, r);
 */
template <uint32_t SIZE = 4096> class Ensemble {
public:
  Ensemble() { init(); }

  /**
   * Lay out the delays and reset the LFOs.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    float rateScale = sampleRate / 48000.0f;
    scale_ = fitScale(rateScale, 2.0f * 2047.0f, 2, SIZE);
    engine_.reset();
    int32_t length = static_cast<int32_t>(2047 * scale_ + 0.5f);
    lineL_ = engine_.addDelay(length);
    lineR_ = engine_.addDelay(length);
    engine_.allocate();
    increment1_ = 1.57e-05f / rateScale; // 0.75 Hz
    increment2_ = 1.37e-04f / rateScale; // 6.6 Hz
    phase1_ = phase2_ = 0.0f;
  }

  /** @param x  Wet amount, 0..1 (the dry is turned down by half of it) */
  void setAmount(float x) { amount_ = x; }

  /** @param x  Modulation depth, 0..1 (up to about ±3 ms) */
  void setDepth(float x) { depth_ = x * 128.0f; }

  /** Silence the delay lines. */
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

    // 1. Slow and fast LFOs, each at 0, 120 and 240 degrees. Rings read a
    // 4096-step sine table without interpolation; quantising the phase
    // the same way keeps its exact values.
    phase1_ += increment1_;
    if (phase1_ >= 1.0f)
      phase1_ -= 1.0f;
    phase2_ += increment2_;
    if (phase2_ >= 1.0f)
      phase2_ -= 1.0f;
    int32_t phi1 = static_cast<int32_t>(phase1_ * 4096.0f);
    int32_t phi2 = static_cast<int32_t>(phase2_ * 4096.0f);
    float slow0 = sine4096(phi1), slow120 = sine4096(phi1 + 1365);
    float slow240 = sine4096(phi1 + 2730);
    float fast0 = sine4096(phi2), fast120 = sine4096(phi2 + 1365);
    float fast240 = sine4096(phi2 + 2730);

    // 2. Each tap's sweep: mostly slow, a little fast.
    float a = depth_ * scale_;
    float b = depth_ * 0.1f * scale_;
    float mod1 = slow0 * a + fast0 * b;
    float mod2 = slow120 * a + fast120 * b;
    float mod3 = slow240 * a + fast240 * b;
    float centre = 1024.0f * scale_; // about 21 ms

    // 3. Write both channels, then read three taps per side: two from its
    // own line, one from the other side's.
    float wet;
    c.read(left, 1.0f);
    c.write(lineL_, 0.0f);
    c.read(right, 1.0f);
    c.write(lineR_, 0.0f);
    c.interpolate(lineL_, mod1 + centre, 0.33f);
    c.interpolate(lineL_, mod2 + centre, 0.33f);
    c.interpolate(lineR_, mod3 + centre, 0.33f);
    c.write(wet, 0.0f);
    left = wet * amount_ + left * dryAmount;
    c.interpolate(lineR_, mod1 + centre, 0.33f);
    c.interpolate(lineR_, mod2 + centre, 0.33f);
    c.interpolate(lineL_, mod3 + centre, 0.33f);
    c.write(wet, 0.0f);
    right = wet * amount_ + right * dryAmount;
  }

private:
  /** sin(2*pi*i/4096) for an integer step i, from the flash table. */
  static float sine4096(int32_t i) {
    return sineFromPhase(static_cast<float>(i & 4095) * (1.0f / 4096.0f));
  }

  FxEngine<SIZE> engine_;
  typename FxEngine<SIZE>::Delay lineL_, lineR_;
  float scale_ = 1.0f;
  float amount_ = 0.5f;
  float depth_ = 0.5f * 128.0f;
  float phase1_ = 0.0f, phase2_ = 0.0f;
  float increment1_ = 0.0f, increment2_ = 0.0f;
};

} // namespace bb
