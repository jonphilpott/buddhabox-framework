/**
 * @file vowel.h
 * @brief A "talking" filter: five formant band-passes, from Plaits.
 *
 * Five resonant band-pass filters tuned to the formants (resonances) of a
 * human voice, morphing between five vowels and five voice types. Put any
 * bright source through it (a pulse train, a saw, noise) and it speaks.
 *
 * HOW IT WORKS: a vowel is mostly defined by where the throat and mouth
 * resonate, at a handful of frequencies called formants. A table gives
 * the five formants (frequency and level) for each vowel and voice type;
 * `vowel` and `voice` blend between table entries, and the five filters
 * are retuned to the blend.
 *
 * COST: each set*() call retunes five filters (five fastExp2() calls);
 * call them at control rate, not every sample. Plaits used its cheap tan()
 * approximation, which slightly lowers the highest formants; this does
 * too.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/filter/svf.h"

namespace bb {

/**
 * Five-formant vowel filter.
 *
 * Example — a choir-like drone:
 *   BasicOscillator buzz;
 *   VowelFilter vowel;
 *   buzz.setShape(BasicOscillator::IMPULSE_TRAIN);
 *   buzz.setFrequency(110.0f);
 *   vowel.setVoice(0.3f);
 *
 *   // Every few ms, from a slow LFO:
 *   vowel.setVowel(0.5f + 0.5f * lfo.process());
 *   float s = vowel.process(buzz.process()) * 0.5f;
 */
class VowelFilter {
public:
  /** Number of formant filters. */
  static const int kNumFormants = 5;

  VowelFilter() { init(); }

  /**
   * Reset the filters.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleRate_ = sampleRate;
    for (Svf &f : filter_)
      f.init(sampleRate);
    update();
  }

  /** @param x  0..1: morphs through roughly "a", "e", "i", "o", "u" */
  void setVowel(float x) {
    vowel_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /** @param x  0..1: voice type, from bass (0) to soprano/child (1) */
  void setVoice(float x) {
    voice_ = clampf(x, 0.0f, 1.0f);
    update();
  }

  /**
   * Shift all formants, e.g. for a smaller or larger "head".
   * @param semitones  Shift in semitones (positive = smaller head)
   */
  void setShift(float semitones) {
    shift_ = semitones;
    update();
  }

  /**
   * Filter one sample. The resonances are strong: Plaits drives it with
   * a pulse train at about ±1 and gets roughly ±1 out.
   * @param in  Input sample
   * @return    The sum of the five formant band-passes
   */
  float process(float in) {
    float out = 0.0f;
    for (int i = 0; i < kNumFormants; ++i)
      out += gain_[i] * filter_[i].process(in).bp;
    return out;
  }

private:
  /** One formant: pitch as a MIDI note number, level 0..255. */
  struct Formant {
    uint8_t note;
    uint8_t level;
  };

  void update() {
    // [vowel][voice][formant]
    static const Formant kTable[5][5][kNumFormants] = {
        {{{74, 255}, {83, 114}, {97, 90}, {98, 90}, {100, 25}},
         {{75, 255}, {84, 128}, {100, 114}, {101, 101}, {103, 20}},
         {{76, 255}, {85, 128}, {100, 18}, {102, 16}, {104, 3}},
         {{79, 255}, {85, 161}, {101, 25}, {104, 4}, {110, 0}},
         {{79, 255}, {85, 128}, {101, 6}, {106, 25}, {110, 0}}},
        {{{67, 255}, {91, 64}, {98, 90}, {101, 64}, {102, 32}},
         {{67, 255}, {92, 51}, {99, 64}, {103, 51}, {105, 25}},
         {{69, 255}, {93, 51}, {100, 32}, {102, 25}, {103, 25}},
         {{67, 255}, {91, 16}, {100, 8}, {103, 4}, {110, 0}},
         {{65, 255}, {95, 25}, {101, 45}, {105, 2}, {110, 0}}},
        {{{59, 255}, {92, 8}, {99, 40}, {102, 20}, {104, 10}},
         {{61, 255}, {94, 45}, {101, 32}, {103, 25}, {105, 8}},
         {{60, 255}, {93, 16}, {101, 16}, {104, 4}, {105, 4}},
         {{65, 255}, {92, 25}, {100, 8}, {105, 4}, {110, 0}},
         {{60, 255}, {96, 64}, {101, 12}, {106, 12}, {110, 1}}},
        {{{67, 255}, {78, 72}, {98, 22}, {99, 25}, {101, 2}},
         {{67, 255}, {79, 80}, {99, 64}, {101, 64}, {102, 12}},
         {{68, 255}, {79, 80}, {100, 12}, {102, 20}, {103, 5}},
         {{69, 255}, {79, 90}, {101, 40}, {104, 10}, {110, 0}},
         {{69, 255}, {79, 72}, {101, 20}, {106, 20}, {110, 0}}},
        {{{65, 255}, {74, 25}, {98, 6}, {100, 10}, {101, 4}},
         {{65, 255}, {74, 25}, {100, 36}, {101, 51}, {103, 12}},
         {{66, 255}, {75, 25}, {100, 18}, {102, 8}, {104, 5}},
         {{63, 255}, {77, 64}, {99, 8}, {104, 2}, {110, 0}},
         {{63, 255}, {77, 40}, {100, 4}, {106, 2}, {110, 0}}},
    };
    // 1. Position in the 5 x 5 table, blended bilinearly.
    float p = vowel_ * (5 - 1.001f);
    float r = voice_ * (5 - 1.001f);
    int pi = static_cast<int>(p), ri = static_cast<int>(r);
    float pf = p - pi, rf = r - ri;
    for (int i = 0; i < kNumFormants; ++i) {
      const Formant &a = kTable[pi][ri][i];
      const Formant &b = kTable[pi][ri + 1][i];
      const Formant &c = kTable[pi + 1][ri][i];
      const Formant &d = kTable[pi + 1][ri + 1][i];
      float f0 = a.note + (b.note - a.note) * rf;
      float f1 = c.note + (d.note - c.note) * rf;
      float note = minf(f0 + (f1 - f0) * pf, 160.0f);
      float l0 = a.level + (b.level - a.level) * rf;
      float l1 = c.level + (d.level - c.level) * rf;
      gain_[i] = (l0 + (l1 - l0) * pf) / 256.0f;
      // 2. The note number is semitones above 55 Hz (MIDI 33).
      float hz = 55.0f * semitonesToRatio(note - 33.0f + shift_);
      filter_[i].setCoefficients(minf(hz / sampleRate_, 0.49f), 20.0f,
                                 TanApprox::Dirty);
    }
  }

  float sampleRate_ = SAMPLE_RATE;
  float vowel_ = 0.0f, voice_ = 0.5f, shift_ = 0.0f;
  float gain_[kNumFormants] = {};
  Svf filter_[kNumFormants];
};

} // namespace bb
