/**
 * @file reverb.h
 * @brief Stereo plate-style reverb from Rings, Elements and Clouds.
 *
 * HOW IT WORKS (the "Griesinger" topology from Jon Dattorro's 1997 paper
 * on reverb design):
 * 1. The input (left + right summed) goes through four all-pass filters
 *    in series. All-passes don't colour the sound, but they smear each
 *    click into a dense spray of echoes ("diffusion").
 * 2. That feeds a figure-of-eight loop of two halves. Each half has two
 *    more all-passes, a long delay, and a low-pass filter (high
 *    frequencies die first, as in a real room). Each half feeds the
 *    other.
 * 3. The left output is tapped from one half and the right from the
 *    other, which gives a wide, decorrelated stereo image.
 * 4. Slow LFOs wobble some delay taps, which blurs metallic resonances
 *    and adds a gentle shimmer.
 *
 * PRESETS differ in delay lengths, which delays are modulated, and the
 * rate they were tuned at:
 * - RINGS: brighter and shorter.
 * - ELEMENTS: larger, with modulation inside the input diffuser.
 * - CLOUDS: smaller and denser (delays 75% of Elements').
 *
 * MEMORY: the template parameter sets the buffer (2 bytes per sample).
 * At 44.1 kHz the Clouds preset needs about 22,600 samples to sound
 * exactly as designed; with a smaller buffer every delay is shrunk in
 * proportion, which gives a smaller, denser room (decay time is still set
 * by setTime()). Suggested sizes:
 * - Reverb<16384>  32 KB — Clouds at its 32 kHz size; the default
 * - Reverb<8192>   16 KB — a small, tight room
 *
 * Derived from Rings, Elements and Clouds, Copyright 2014-2015 Emilie
 * Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/fx_engine.h"

namespace bb {

/**
 * Stereo Griesinger/Dattorro reverb.
 *
 * @tparam SIZE  Delay buffer in samples (power of two); RAM = SIZE * 2
 *
 * Example:
 *   Reverb<> reverb; // 32 KB
 *   reverb.setTime(0.85f);
 *   reverb.setAmount(0.4f);
 *
 *   float l = voice.process(), r = l;
 *   reverb.process(l, r); // in place
 */
template <uint32_t SIZE = 16384> class Reverb {
public:
  /** Tunings of the three original modules. */
  enum Preset { RINGS, ELEMENTS, CLOUDS };

  Reverb() { init(); }

  /**
   * Lay out the delay lines for a preset and clear them.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   * @param preset      RINGS, ELEMENTS or CLOUDS (default)
   */
  void init(float sampleRate = SAMPLE_RATE, Preset preset = CLOUDS) {
    preset_ = preset;
    // 1. The preset's original rate and delay lengths.
    static const int32_t kElementsLengths[10] = {150,  214,  319,  527,  2182,
                                                 2690, 4501, 2525, 2197, 6312};
    static const int32_t kCloudsLengths[10] = {113,  162,  241,  399,  1653,
                                               2038, 3411, 1913, 1663, 4782};
    const int32_t *lengths =
        preset == CLOUDS ? kCloudsLengths : kElementsLengths;
    float originalRate = preset == RINGS ? 48000.0f : 32000.0f;
    float total = 0.0f;
    for (int i = 0; i < 10; ++i)
      total += lengths[i];
    // Keep the timing in seconds, unless that would overflow the buffer.
    scale_ = fitScale(sampleRate / originalRate, total, 10, SIZE);
    lpRateRatio_ = originalRate / sampleRate;

    // 2. Lay out the ten delay lines.
    engine_.reset();
    for (int i = 0; i < 10; ++i)
      lines_[i] =
          engine_.addDelay(static_cast<int32_t>(lengths[i] * scale_ + 0.5f));
    engine_.allocate();
    engine_.setLfoFrequency(0, 0.5f / sampleRate);
    engine_.setLfoFrequency(1, 0.3f / sampleRate);

    // 3. Preset-specific modulation: the loop tap on del2 (read at a
    // fixed point, swept by LFO 2), plus optional modulation of del1 or of
    // the first input all-pass.
    switch (preset) {
    case RINGS:
      del2Tap_ = 6261.0f;
      del2Depth_ = 50.0f;
      del1Tap_ = 4460.0f;
      del1Depth_ = 40.0f;
      ap1Depth_ = 0.0f;
      break;
    case ELEMENTS:
      del2Tap_ = 6211.0f;
      del2Depth_ = 100.0f;
      del1Tap_ = 0.0f;
      del1Depth_ = 0.0f;
      ap1Depth_ = 80.0f;
      break;
    case CLOUDS:
      del2Tap_ = 4680.0f;
      del2Depth_ = 100.0f;
      del1Tap_ = 0.0f;
      del1Depth_ = 0.0f;
      ap1Depth_ = 60.0f;
      break;
    }
    lpDecay1_ = lpDecay2_ = 0.0f;
    setLp(lp_);
  }

  /** @param x  Wet/dry mix: 0 = dry only, 1 = wet only */
  void setAmount(float x) { amount_ = x; }

  /** @param x  Input level into the reverb; the modules used 0.2 */
  void setInputGain(float x) { inputGain_ = x; }

  /** @param x  Decay, 0..1: 0.5 small room, 0.9 hall, 0.98 near-endless */
  void setTime(float x) { time_ = x; }

  /** @param x  All-pass coefficient, 0..1: echo smearing (0.625) */
  void setDiffusion(float x) { diffusion_ = x; }

  /** @param x  Damping, 0..1: 1 = bright, lower = darker */
  void setLp(float x) {
    lp_ = x;
    lpCoefficient_ = rescaleCoefficient(clampf(x, 0.0f, 1.0f), lpRateRatio_);
  }

  /** Silence the tail. */
  void clear() { engine_.clear(); }

  /**
   * Process one stereo sample in place.
   * @param left   Left in/out
   * @param right  Right in/out
   */
  void process(float &left, float &right) {
    FxEngine<SIZE> &c = engine_;
    const Line &ap1 = lines_[0];
    const Line &ap2 = lines_[1];
    const Line &ap3 = lines_[2];
    const Line &ap4 = lines_[3];
    const Line &dap1a = lines_[4];
    const Line &dap1b = lines_[5];
    const Line &del1 = lines_[6];
    const Line &dap2a = lines_[7];
    const Line &dap2b = lines_[8];
    const Line &del2 = lines_[9];
    const float kap = diffusion_;
    const float klp = lpCoefficient_;
    const float krt = time_;
    float wet;
    float apout = 0.0f;
    c.start();

    // 1. Optionally smear the first all-pass by rewriting part of it from
    // an LFO-swept position (Elements, Clouds).
    if (ap1Depth_ > 0.0f) {
      c.interpolate(ap1, 10.0f * scale_, 0, ap1Depth_ * scale_, 1.0f);
      c.write(ap1, static_cast<int32_t>(100 * scale_), 0.0f);
    }

    // 2. Input diffusion: four all-passes in series.
    c.read(left + right, inputGain_);
    c.readTail(ap1, kap);
    c.writeAllPass(ap1, -kap);
    c.readTail(ap2, kap);
    c.writeAllPass(ap2, -kap);
    c.readTail(ap3, kap);
    c.writeAllPass(ap3, -kap);
    c.readTail(ap4, kap);
    c.writeAllPass(ap4, -kap);
    c.write(apout);

    // 3. First half of the loop, fed by the end of the second half.
    c.load(apout);
    c.interpolate(del2, del2Tap_ * scale_, 1, del2Depth_ * scale_, krt);
    c.lp(lpDecay1_, klp);
    c.readTail(dap1a, -kap);
    c.writeAllPass(dap1a, kap);
    c.readTail(dap1b, kap);
    c.writeAllPass(dap1b, -kap);
    c.write(del1, 2.0f);
    c.write(wet, 0.0f);
    left += (wet - left) * amount_;

    // 4. Second half, fed by the end of the first.
    c.load(apout);
    if (del1Depth_ > 0.0f) {
      c.interpolate(del1, del1Tap_ * scale_, 0, del1Depth_ * scale_, krt);
    } else {
      c.readTail(del1, krt);
    }
    c.lp(lpDecay2_, klp);
    c.readTail(dap2a, kap);
    c.writeAllPass(dap2a, -kap);
    c.readTail(dap2b, -kap);
    c.writeAllPass(dap2b, kap);
    c.write(del2, 2.0f);
    c.write(wet, 0.0f);
    right += (wet - right) * amount_;
  }

private:
  typedef typename FxEngine<SIZE>::Delay Line;

  FxEngine<SIZE> engine_;
  Line lines_[10];
  Preset preset_ = CLOUDS;
  float scale_ = 1.0f;
  float lpRateRatio_ = 1.0f;
  float del2Tap_ = 4680.0f, del2Depth_ = 100.0f;
  float del1Tap_ = 0.0f, del1Depth_ = 0.0f;
  float ap1Depth_ = 60.0f;

  float amount_ = 0.5f;
  float inputGain_ = 0.2f;
  float time_ = 0.7f;
  float diffusion_ = 0.625f;
  float lp_ = 0.7f;
  float lpCoefficient_ = 0.7f;
  float lpDecay1_ = 0.0f, lpDecay2_ = 0.0f;
};

} // namespace bb
