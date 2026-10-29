/**
 * @file particle.h
 * @brief Randomly pitched resonant pings, from Plaits' particle engine.
 *
 * Random impulses ("dust"), each ringing a resonant filter tuned to a
 * slightly different, random pitch around a centre frequency. At low
 * densities: droplets, pings, a Geiger counter. At high densities: a
 * shimmering cloud. Plaits sums several of these and adds a diffuser.
 *
 * HOW IT WORKS: each sample, a random number decides whether an impulse
 * fires (probability = density / sample rate). When one does, the
 * band-pass filter is retuned to the centre frequency shifted randomly by
 * up to ±spread semitones, and the impulse rings it. The level is
 * compensated for the filter's Q and the density, so the output stays at
 * a steady loudness.
 *
 * Difference from Plaits: Plaits retunes the filter at most once per
 * 12-sample block; this version retunes on every impulse.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/filter/svf.h"

namespace bb {

/**
 * Pitched random pings.
 *
 * Example — glassy droplets around A5:
 *   Particle drops;
 *   drops.setFrequency(880.0f);
 *   drops.setDensity(6.0f);
 *   drops.setSpread(12.0f); // up to an octave either way
 *   drops.setQ(200.0f);     // long, ringing pings
 *
 *   float s = drops.process();
 */
class Particle {
public:
  Particle() { init(); }

  /**
   * Reset the filter.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    filter_.init(sampleRate);
    preGain_ = 0.0f;
    impulse_ = 0.0f;
  }

  /** @param hz  Average impulses per second */
  void setDensity(float hz) { density_ = hz / sampleRate_; }

  /** @param hz  Centre frequency of the pings in Hz */
  void setFrequency(float hz) { frequency_ = hz / sampleRate_; }

  /**
   * @param semitones  Random detuning range: 0 = all the same pitch,
   *                   24 = up to two octaves either way
   */
  void setSpread(float semitones) { spread_ = semitones; }

  /** @param q  Filter resonance: 1 = soft clicks, 100+ = long pings */
  void setQ(float q) { q_ = q; }

  /** @param s  Random seed, for repeatable patterns */
  void seed(uint32_t s) { random_.seed(s); }

  /** Fire an impulse on the next sample (e.g. from a trigger input). */
  void trigger() { forced_ = true; }

  /**
   * Generate the next sample.
   * @return  The ringing output (about ±1); impulse() holds the raw
   *          impulse of this sample
   */
  float process() {
    float u = random_.uniform();
    if (forced_) {
      u = density_ < 1.0f ? density_ : 0.999f;
      forced_ = false;
    }
    float s = 0.0f;
    if (u <= density_) {
      s = u;
      // A new random pitch for this particle. The gain compensates the
      // filter's Q and the density (Plaits' formula, written against
      // 48 kHz, hence the f48 / density48 terms).
      float detune = spread_ * (2.0f * random_.uniform() - 1.0f);
      float f = minf(semitonesToRatio(detune) * frequency_, 0.25f);
      float density48 = maxf(density_ * sampleRate_ / 48000.0f, 1e-6f);
      float f48 = f * sampleRate_ / 48000.0f;
      preGain_ = 0.5f / sqrtf(q_ * f48 * sqrtf(density48));
      filter_.setCoefficients(f, q_, TanApprox::Dirty);
    }
    impulse_ = s;
    return filter_.process(preGain_ * s).bp;
  }

  /** @return  The raw impulse from the last process() call (0..1) */
  float impulse() const { return impulse_; }

private:
  float sampleRate_ = SAMPLE_RATE;
  float density_ = 0.001f, frequency_ = 0.02f, spread_ = 12.0f, q_ = 10.0f;
  float preGain_ = 0.0f, impulse_ = 0.0f;
  bool forced_ = false;
  Random random_;
  Svf filter_;
};

} // namespace bb
