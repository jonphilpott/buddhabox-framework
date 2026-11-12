/**
 * @file spring_reverb.h
 * @brief A two-spring tank: the drippy, boingy reverb of guitar amps and
 *        the Buchla Easel.
 *
 * A spring reverb sends sound down a coiled spring and picks it up at the
 * other end. A spring is dispersive: different frequencies travel along
 * it at different speeds. So a click goes in and comes out smeared into
 * a "chirp", a quick whistling sweep. The sound bounces back and forth
 * along the spring, and every trip smears it further, which turns each
 * echo into a longer chirp: the "drip" and "boing" a hall reverb never
 * makes.
 *
 * HOW THIS MODEL WORKS (after Parker and Valimaki's spring models):
 *
 *   in ─► high-pass ─► (+) ─► 12 all-passes ─► delay ─► low-pass ─┬─► out
 *                       ▲     (the chirp)    (one trip)  (losses) │
 *                       └──────────── x feedback ◄────────────────┘
 *
 * 1. High-pass: real tanks pass little bass, which keeps them from
 *    booming.
 * 2. Dispersion: a chain of all-pass filters. An all-pass passes every
 *    frequency at the same level but delays some more than others, so
 *    in a long enough chain a click spreads out into a chirp. Each one is
 *    "stretched" (it uses the sample 4 back, not 1 back), which moves
 *    the chirp into the spring's range, below about 5.5 kHz. At half
 *    rate, 2 back is the same time (see STRETCH below).
 * 3. Delay: the time for one trip along the spring, 35 to 42 ms.
 * 4. Low-pass and feedback: each trip loses some treble and some level,
 *    so echoes get darker and quieter. The low-pass also removes the
 *    stretched all-passes' copies of the chirp above 5.5 kHz.
 *
 * Two springs of different lengths, one per output, give a wide stereo
 * image from a mono input, as real two- and three-spring tanks do.
 *
 * COST: 466 cycles a sample on the F411 (21% of the budget), nearly all
 * of it the 24 all-pass sections; 2 x SIZE x 4 bytes of RAM, 16 KB at
 * the default size.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/delay.h"

namespace bb {

/**
 * A stereo spring reverb tank.
 *
 * @tparam SIZE     Delay per spring in samples; must hold the longer
 *                  trip (1871 samples at 44.1 kHz), so 2048 is the
 *                  smallest that works at the full rate
 * @tparam STRETCH  How many samples back each all-pass reaches (a power
 *                  of two). 4 at 44.1 kHz; 2 at half rate, which delays
 *                  by the same time and so keeps the same chirp
 *
 * Example — on a voice's output:
 *   SpringReverb<> spring;
 *   spring.setTime(0.8f);   // long, splashy tail
 *   spring.setAmount(0.35f);
 *
 *   // In audio callback:
 *   float left = voice, right = voice;
 *   spring.process(left, right);
 *
 * Example — at half rate, for half the cost (examples/noisebox):
 *   SpringReverb<1024, 2> spring; // 8 KB
 *   spring.init(SAMPLE_RATE / 2);
 *   // call process() on every second sample, fill the gaps in between
 */
template <uint16_t SIZE = 2048, int STRETCH = 4> class SpringReverb {
public:
  SpringReverb() { init(); }

  /**
   * Clear the springs and set the defaults (time 0.75, tone 0.5, chirp
   * 0.5, amount 0.3).
   * @param sampleRate  Sample rate in Hz
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    // Fixed input high-pass at about 150 Hz: one-pole coefficient
    // 2 * pi * f / sample rate (accurate enough this far below Nyquist).
    hpCoefficient_ = TWO_PI_F * 150.0f / sampleRate;
    hpState_ = 0.0f;
    // Trip times in samples: 35.3 and 42.4 ms at 44.1 kHz. Different,
    // and with no common factor, so the two springs' echoes never line
    // up into a flutter.
    // Whole samples: the trips never change, so there is nothing to
    // interpolate.
    springs_[0].trip = (uint16_t)(1557.0f * sampleRate / 44100.0f);
    springs_[1].trip = (uint16_t)(1871.0f * sampleRate / 44100.0f);
    for (Spring &s : springs_) {
      s.delay.clear();
      s.lp = 0.0f;
      for (int k = 0; k < kStretch; k++)
        for (int i = 0; i < kSections; i++)
          s.w[k][i] = 0.0f;
    }
    tap_ = 0;
    setTime(0.75f);
    setTone(0.5f);
    setChirp(0.5f);
    setAmount(0.3f);
  }

  /**
   * How long the tail rings: the feedback around each spring.
   * @param x  0.0 = one echo, 1.0 = very long (about 5 seconds)
   */
  void setTime(float x) {
    // An upside-down parabola, so the long tails sit in the top half of
    // the range instead of being crammed into its last few percent.
    float inverse = 1.0f - clampf(x, 0.0f, 1.0f);
    feedback_ = 0.97f * (1.0f - inverse * inverse);
  }

  /**
   * Brightness of the tank: the loop's low-pass cutoff, 1.5 to 6 kHz.
   * Uses expf(): control rate.
   * @param x  0.0 = dark and dull, 1.0 = bright and splashy
   */
  void setTone(float x) {
    float hz = 1500.0f + 4500.0f * clampf(x, 0.0f, 1.0f);
    lpCoefficient_ = 1.0f - expf(-TWO_PI_F * hz / sampleRate_);
  }

  /**
   * How strongly each echo is smeared into a chirp: the all-passes'
   * coefficient.
   * @param x  0.0 = a light chirp, 1.0 = a deep "pew" on every echo
   */
  void setChirp(float x) { chirp_ = 0.3f + 0.45f * clampf(x, 0.0f, 1.0f); }

  /**
   * Dry/wet mix, as Reverb's.
   * @param x  0.0 = dry only, 1.0 = reverb only
   */
  void setAmount(float x) { amount_ = clampf(x, 0.0f, 1.0f); }

  /**
   * Process one stereo sample in place. Both springs hear the mono sum;
   * the left output comes from one spring and the right from the other.
   * @param left   Left sample, replaced with the mix
   * @param right  Right sample, replaced with the mix
   */
  void process(float &left, float &right) {
    // 1. Mono input, high-passed (input minus its own low-pass).
    float in = 0.5f * (left + right);
    hpState_ += hpCoefficient_ * (in - hpState_);
    in -= hpState_;

    // 2. Run both springs. The stretched all-passes all use the sample
    //    from kStretch steps ago, so one shared position in their little
    //    ring buffers serves every section of both springs.
    float wetLeft = runSpring(springs_[0], in);
    float wetRight = runSpring(springs_[1], in);
    tap_ = (tap_ + 1) & (kStretch - 1);

    // 3. Crossfade dry to wet, as Reverb does.
    left += (wetLeft - left) * amount_;
    right += (wetRight - right) * amount_;
  }

private:
  static const int kSections = 12;     // all-passes per spring
  static const int kStretch = STRETCH; // a power of two: wrap with &

  struct Spring {
    DelayLine<SIZE, float> delay;
    uint16_t trip; // samples for one trip along the spring
    float lp;      // the loop low-pass's state, which is also the output
    // Each all-pass's state for the last kStretch samples. Stored by time
    // first, then section, so one sample's states sit side by side: the
    // loop walks one row with a single pointer.
    float w[kStretch][kSections];
  };

  /**
   * One spring, one sample.
   * @return  The spring's output: the low-passed end of one trip
   */
  float runSpring(Spring &s, float in) {
    // Feed the input in, plus what came back from the last trip.
    float v = in + feedback_ * s.lp;

    // Dispersion. Each section is a stretched first-order all-pass,
    //   H(z) = (a + z^-4) / (1 + a z^-4)
    // a = 0 is a plain 4-sample delay; nearer 1, low frequencies are
    // delayed much more than high ones, which makes the chirp.
    //
    // WHY THIS FORM: the obvious one, y[n] = a(x[n] - y[n-4]) + x[n-4],
    // keeps both old inputs and old outputs. This one (direct form II)
    // keeps a single internal value w per sample, so each section is one
    // load, one store and two multiply-adds:
    //   w[n] = x[n] - a w[n-4]       y[n] = a w[n] + w[n-4]
    // On the F411 that halved the cost.
    const float a = chirp_;
    float *w = s.w[tap_]; // the row written 4 samples ago: read, replace
    for (int i = 0; i < kSections; i++) {
      float old = w[i];
      float now = v - a * old;
      w[i] = now;
      v = a * now + old;
    }

    // One trip along the spring, then the treble lost on the way.
    s.delay.write(v);
    float back = s.delay.at(s.trip);
    s.lp += lpCoefficient_ * (back - s.lp);
    return s.lp;
  }

  Spring springs_[2];
  int tap_ = 0;
  float sampleRate_ = SAMPLE_RATE;
  float hpCoefficient_ = 0.0f, hpState_ = 0.0f;
  float lpCoefficient_ = 0.5f;
  float feedback_ = 0.7f;
  float chirp_ = 0.5f;
  float amount_ = 0.3f;
};

} // namespace bb
