/**
 * @file compressor.h
 * @brief RMS feed-forward compressor with sidechain, from Streams.
 *
 * Evens out loudness: quiet passages pass unchanged, loud ones are turned
 * down. With a sidechain it "ducks" one sound under another (a pad that
 * dips on every kick).
 *
 * HOW IT WORKS:
 * 1. Detector: the input (or the sidechain) is squared and smoothed with
 *    separate attack and release rates. That tracks the signal's power
 *    (RMS level) rather than its peaks, which sounds more natural.
 * 2. Gain computer, in decibels: above the threshold, each dB of input
 *    rise gives only 1/ratio dB of output rise. The soft knee rounds the
 *    corner at the threshold.
 * 3. The gain (with makeup) is applied to the input.
 *
 * BLACK PILL CHANGE: Partials converted to and from decibels with
 * log10() and pow() on every sample, a few hundred cycles each. This port
 * uses fastLog2() and fastExp2() instead (roughly 20 cycles each, accurate
 * to well under 0.01 dB).
 *
 * Derived from Streams, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * RMS compressor.
 *
 * Example — duck a pad under a kick:
 *   Compressor duck;
 *   duck.setThreshold(-30.0f);
 *   duck.setRatio(8.0f);
 *   duck.setRelease(0.2f);
 *
 *   float k = kick.process();
 *   float out = k + duck.process(pad.process(), k);
 */
class Compressor {
public:
  Compressor() { init(); }

  /**
   * Reset the detector.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    detector_ = 0.0f;
    gainReductionDb_ = 0.0f;
    update();
  }

  /** @param db  Level (0 = full scale) where compression starts; -30..0 */
  void setThreshold(float db) { thresholdDb_ = db; }

  /** @param ratio  1 (none) to 25 or more; very high = limiting */
  void setRatio(float ratio) { ratio_ = ratio < 1.0f ? 1.0f : ratio; }

  /** @param seconds  Attack time (Streams: 0.0002 hard / 0.002 soft) */
  void setAttack(float seconds) {
    attack_ = seconds;
    update();
  }

  /** @param seconds  Release time (Streams: 0.15 hard / 0.07 soft) */
  void setRelease(float seconds) {
    release_ = seconds;
    update();
  }

  /** @param db  Gain added after compression */
  void setMakeup(float db) { makeupDb_ = db; }

  /** @param on  Round the corner at the threshold over about 6 dB */
  void setSoftKnee(bool on) { softKnee_ = on; }

  /**
   * Compress, detecting the level from the input itself.
   * @param in  Audio input
   * @return    Compressed audio
   */
  float process(float in) { return process(in, in); }

  /**
   * Compress `in`, detecting the level from `sidechain`.
   * @param in         Audio to compress
   * @param sidechain  Signal whose level controls the gain
   * @return           Compressed audio
   */
  float process(float in, float sidechain) {
    // 1. RMS detector.
    float energy = sidechain * sidechain;
    float error = energy - detector_;
    detector_ +=
        error * (error > 0.0f ? attackCoefficient_ : releaseCoefficient_);
    // 2. Gain computer. Level in dB of RMS = 10 log10(power), and
    // 10 log10(x) = 10 log10(2) * log2(x) = 3.0103 * log2(x).
    float levelDb = 3.01029996f * fastLog2(detector_ + 1e-20f);
    float over = levelDb - thresholdDb_;
    float attenuation = 0.0f;
    if (over > 0.0f) {
      attenuation = over * (1.0f - 1.0f / ratio_);
      // Soft knee (Streams' cubic curve over the first ~6 dB).
      if (softKnee_ && attenuation < 6.02f) {
        float t = attenuation / 6.02f;
        float knee = t * t * t * 6.02f;
        attenuation += (knee - attenuation) * (1.0f - t);
      }
    }
    gainReductionDb_ = attenuation;
    // 3. Apply. 10^(dB / 20) = 2^(dB * log2(10) / 20) = 2^(dB * 0.1661).
    float gain = fastExp2((makeupDb_ - attenuation) * 0.166096405f);
    return in * gain;
  }

  /** @return  Current gain reduction in dB (positive = turned down) */
  float gainReduction() const { return gainReductionDb_; }

private:
  void update() {
    attackCoefficient_ = 1.0f - expf(-1.0f / (attack_ * sampleRate_));
    releaseCoefficient_ = 1.0f - expf(-1.0f / (release_ * sampleRate_));
  }

  float sampleRate_ = SAMPLE_RATE;
  float thresholdDb_ = -12.0f, ratio_ = 4.0f, makeupDb_ = 0.0f;
  float attack_ = 0.0002f, release_ = 0.15f;
  bool softKnee_ = false;
  float attackCoefficient_ = 1.0f, releaseCoefficient_ = 0.001f;
  float detector_ = 0.0f, gainReductionDb_ = 0.0f;
};

} // namespace bb
