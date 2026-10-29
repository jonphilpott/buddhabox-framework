/**
 * @file dust.h
 * @brief Sparse random impulses ("crackle"), from Plaits.
 *
 * Random impulses, each with a random height, arriving at a set average
 * rate. At low densities: vinyl crackle, rain, a Geiger counter. At high
 * densities: a hiss. Also an excellent exciter for resonators — feed Dust
 * into a ModalResonator for wind chimes or rain on a bell.
 *
 * HOW IT WORKS: each sample, a random number u (0..1) is drawn. If u is
 * below the probability of an impulse in this sample (density / sample
 * rate), an impulse of height u / probability (0..1) is emitted;
 * otherwise silence. Reusing u for the height costs no extra random
 * number.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"

namespace bb {

/**
 * Random impulse generator.
 *
 * Example — rain on a bell:
 *   Dust dust;
 *   ModalResonator bell;
 *   dust.setDensity(3.0f); // about three drops per second
 *
 *   float s = bell.process(dust.process());
 */
class Dust {
public:
  Dust() { init(); }

  /**
   * Set the sample rate (keeps the density).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    setDensity(density_);
  }

  /**
   * @param hz  Average impulses per second, 0 to SAMPLE_RATE
   */
  void setDensity(float hz) {
    density_ = hz;
    probability_ = minf(hz * sampleTime_, 1.0f);
    inverse_ = probability_ > 0.0f ? 1.0f / probability_ : 0.0f;
  }

  /** @param s  Random seed, for repeatable patterns */
  void seed(uint32_t s) { random_.seed(s); }

  /**
   * Generate the next sample.
   * @return  An impulse (0.0 to 1.0) or 0.0
   */
  float process() {
    float u = random_.uniform();
    return u < probability_ ? u * inverse_ : 0.0f;
  }

private:
  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float density_ = 100.0f, probability_ = 0.0f, inverse_ = 0.0f;
  Random random_;
};

} // namespace bb
