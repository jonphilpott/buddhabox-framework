/**
 * @file pitch_shifter.h
 * @brief Clouds' delay-line pitch shifter.
 *
 * Shifts audio up or down by any ratio in real time, with a slightly
 * grainy, "tape-splice" character. An octave-up shifter in a reverb's
 * feedback path gives the classic "shimmer".
 *
 * HOW IT WORKS (a "rotating tape head" shifter):
 * - The input is written into a short delay line.
 * - A read head moves along it at a different speed from the write head:
 *   reading faster than writing raises the pitch, slower lowers it. That
 *   is how varispeed tape works.
 * - The read head soon runs into the write head, so there are two heads
 *   half a window apart, each faded in and out by a triangle window. One
 *   always plays while the other jumps back, so the jumps are hidden.
 * - `size` sets the window length. Short windows track transients well
 *   but sound buzzy; long windows are smoother but smear and echo.
 *
 * MEMORY: PitchShifter<4096> (the default) uses 8 KB.
 *
 * Derived from Clouds, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/fx_engine.h"

namespace bb {

/**
 * Two-head delay-line pitch shifter.
 *
 * @tparam SIZE  Delay buffer in samples (power of two); RAM = SIZE * 2
 *
 * Example — a fifth up:
 *   PitchShifter<> shifter;
 *   shifter.setRatio(semitonesToRatio(7.0f));
 *   shifter.setSize(0.5f);
 *
 *   float s = in + 0.5f * shifter.process(in);
 */
template <uint32_t SIZE = 4096> class PitchShifter {
public:
  PitchShifter() { init(); }

  /**
   * Lay out the delay and clear it.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    scale_ = fitScale(sampleRate / 32000.0f, 2047.0f, 1, SIZE);
    maxSize_ = 2047.0f * scale_;
    minSize_ = 128.0f * scale_;
    engine_.reset();
    line_ = engine_.addDelay(static_cast<int32_t>(maxSize_ + 0.5f));
    engine_.allocate();
    // Clouds smoothed size changes once per 1 kHz block with coefficient
    // 0.05 (about 20 ms); convert that to a per-sample coefficient.
    sizeCoefficient_ = rescaleCoefficient(0.05f, 1000.0f / sampleRate);
    phase_ = 0.0f;
    size_ = targetSize_ = maxSize_;
  }

  /**
   * @param ratio  Frequency ratio: 2 = up an octave, 0.5 = down an
   *               octave. Use semitonesToRatio() for intervals.
   */
  void setRatio(float ratio) { ratio_ = ratio; }

  /**
   * Window length, about 4 ms to 64 ms (curved, finer at the short end).
   * Changes are smoothed.
   * @param x  0.0 to 1.0
   */
  void setSize(float x) {
    x = clampf(x, 0.0f, 1.0f);
    targetSize_ = minSize_ + (maxSize_ - minSize_) * x * x * x;
  }

  /** Silence the delay line. */
  void clear() { engine_.clear(); }

  /**
   * Shift one sample.
   * @param in  Input sample
   * @return    The pitch-shifted sample (wet only)
   */
  float process(float in) {
    FxEngine<SIZE> &c = engine_;
    c.start();
    onePole(size_, targetSize_, sizeCoefficient_);
    // 1. Move the read heads relative to the write head. The phase runs
    // 0..1 through the window; its speed sets the pitch change.
    phase_ += (1.0f - ratio_) / size_;
    if (phase_ >= 1.0f)
      phase_ -= 1.0f;
    if (phase_ <= 0.0f)
      phase_ += 1.0f;
    // 2. Triangle crossfade between two heads half a window apart: each
    // is silent when it jumps.
    float tri = 2.0f * (phase_ >= 0.5f ? 1.0f - phase_ : phase_);
    float phase = phase_ * size_;
    float half = phase + size_ * 0.5f;
    if (half >= size_)
      half -= size_;
    float out;
    c.read(in, 1.0f);
    c.write(line_, 0.0f);
    c.interpolate(line_, phase, tri);
    c.interpolate(line_, half, 1.0f - tri);
    c.write(out, 0.0f);
    return out;
  }

private:
  FxEngine<SIZE> engine_;
  typename FxEngine<SIZE>::Delay line_;
  float scale_ = 1.0f;
  float maxSize_ = 2047.0f, minSize_ = 128.0f;
  float sizeCoefficient_ = 0.05f;
  float phase_ = 0.0f;
  float ratio_ = 1.0f;
  float size_ = 2047.0f, targetSize_ = 2047.0f;
};

} // namespace bb
