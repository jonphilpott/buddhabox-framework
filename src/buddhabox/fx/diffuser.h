/**
 * @file diffuser.h
 * @brief Clouds' stereo diffuser: four all-passes per channel.
 *
 * Smears a sound in time without changing its spectrum. Each click turns
 * into a short, dense wash, like the very start of a reverb with no tail.
 * Clouds uses it to blur grains together; it also softens harsh
 * transients and thickens sparse sounds (try it after Particle or Dust).
 *
 * HOW IT WORKS: an all-pass filter delays different frequencies by
 * different amounts while passing all of them at equal level. Four in a
 * row, with unrelated lengths, scatter the energy of each transient over
 * a few tens of milliseconds. Left and right chains use different lengths,
 * so a mono input comes out with a wide stereo image.
 *
 * MEMORY: Diffuser<4096> (the default) uses 8 KB and holds Clouds'
 * lengths at 44.1 kHz.
 *
 * Derived from Clouds, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/fx_engine.h"

namespace bb {

/**
 * Stereo all-pass diffuser.
 *
 * @tparam SIZE  Delay buffer in samples (power of two); RAM = SIZE * 2
 *
 * Example:
 *   Diffuser<> blur;
 *   blur.setAmount(0.7f);
 *
 *   float l = particle.process(), r = l;
 *   blur.process(l, r);
 */
template <uint32_t SIZE = 4096> class Diffuser {
public:
  Diffuser() { init(); }

  /**
   * Lay out the delay lines and clear them.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    static const int32_t kLengths[8] = {126, 180, 269, 444, 151, 205, 245, 405};
    float total = 0.0f;
    for (int i = 0; i < 8; ++i)
      total += kLengths[i];
    float scale = fitScale(sampleRate / 32000.0f, total, 8, SIZE);
    engine_.reset();
    for (int i = 0; i < 8; ++i)
      lines_[i] =
          engine_.addDelay(static_cast<int32_t>(kLengths[i] * scale + 0.5f));
    engine_.allocate();
  }

  /** @param x  Wet/dry mix, 0..1 */
  void setAmount(float x) { amount_ = x; }

  /** @param x  All-pass coefficient, 0..1; higher = more smearing */
  void setDiffusion(float x) { diffusion_ = x; }

  /** Silence the delay lines. */
  void clear() { engine_.clear(); }

  /**
   * Process one stereo sample in place.
   * @param left   Left in/out
   * @param right  Right in/out
   */
  void process(float &left, float &right) {
    const float kap = diffusion_;
    FxEngine<SIZE> &c = engine_;
    float wet;
    c.start();
    // Left: four all-passes in series.
    c.read(left);
    for (int i = 0; i < 4; ++i) {
      c.readTail(lines_[i], kap);
      c.writeAllPass(lines_[i], -kap);
    }
    c.write(wet, 0.0f);
    left += amount_ * (wet - left);
    // Right: four more, with different lengths.
    c.read(right);
    for (int i = 4; i < 8; ++i) {
      c.readTail(lines_[i], kap);
      c.writeAllPass(lines_[i], -kap);
    }
    c.write(wet, 0.0f);
    right += amount_ * (wet - right);
  }

private:
  FxEngine<SIZE> engine_;
  typename FxEngine<SIZE>::Delay lines_[8];
  float amount_ = 1.0f;
  float diffusion_ = 0.625f;
};

} // namespace bb
