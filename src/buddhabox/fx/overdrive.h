/**
 * @file overdrive.h
 * @brief Level-compensated soft-clipping overdrive from Plaits.
 *
 * Turning up the drive adds grit and sustain without the output getting
 * much louder.
 *
 * HOW IT WORKS: the input is amplified (pre-gain) into a soft clipper,
 * then scaled back down (post-gain). The pre-gain curve stays gentle over
 * most of the knob, then climbs steeply (up to about 24x) near the top.
 * The post-gain is the inverse of how loud a full-scale signal comes out
 * of the clipper, so perceived level stays roughly constant.
 *
 * COMPARE softclip() in dsp_common.h: a fixed curve with no level
 * compensation.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Overdrive with automatic level compensation.
 *
 * Example:
 *   Overdrive drive;
 *   drive.setDrive(0.6f);
 *
 *   float s = drive.process(bass.process());
 */
class Overdrive {
public:
  Overdrive() { init(); }

  /**
   * Set the drive-smoothing time (about 1 ms).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    // Drive changes are smoothed over about 1 ms to avoid zipper noise.
    // The drive itself is smoothed, not the two gains separately: those
    // must stay matched, or a fast change would spike in level.
    smoothing_ = minf(1.0f / (0.001f * sampleRate), 1.0f);
    smoothedDrive_ = drive_;
  }

  /**
   * @param x  0.0 = gentle warmth at about unity gain, 1.0 = heavily
   *           clipped. (Internally 0.5..1, the range Plaits uses.)
   */
  void setDrive(float x) { drive_ = 0.5f + 0.5f * clampf(x, 0.0f, 1.0f); }

  /**
   * Drive one sample.
   * @param in  Input sample, about ±1
   * @return    Overdriven sample, about ±1
   */
  float process(float in) {
    onePole(smoothedDrive_, drive_, smoothing_);
    // Plaits' gain curves: pre-gain gentle at first, steep near the top;
    // post-gain the inverse of the clipper's output for a full-scale
    // input.
    const float drive = smoothedDrive_;
    const float drive2 = drive * drive;
    const float preGainA = drive * 0.5f;
    const float preGainB = drive2 * drive2 * drive * 24.0f;
    const float preGain = preGainA + (preGainB - preGainA) * drive2;
    const float squashed = drive * (2.0f - drive);
    const float postGain =
        1.0f / softLimitClip(0.33f + squashed * (preGain - 0.33f));
    return softLimitClip(preGain * in) * postGain;
  }

private:
  float smoothing_ = 0.02f;
  float drive_ = 0.5f;
  float smoothedDrive_ = 0.5f;
};

} // namespace bb
