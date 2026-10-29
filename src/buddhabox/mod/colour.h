/**
 * @file colour.h
 * @brief Five-note just-intonation scales ("colours") plus a melody maker.
 *
 * WHAT A COLOUR IS: every scale has five notes. Three are fixed — Sa (the
 * root), ma (a fourth up) and Pa (a fifth up) — and two vary:
 * - the lower variable note is Re komal, Re, Ga komal or Ga (a flat
 *   second, second, minor third or major third);
 * - the upper variable note is Dha komal, Dha, Ni komal or Ni (a flat
 *   sixth, sixth, flat seventh or major seventh).
 * Four choices times four choices gives 16 colours. Colour c uses lower
 * note c / 4 and upper note c % 4, so colour 0 is the darkest pair and
 * colour 15 the brightest. COLOUR_BY_BRIGHTNESS orders all 16 from night
 * to day, ready to map from a light sensor or a knob.
 *
 * WHY JUST INTONATION: the ratios are small whole-number fractions (3/2
 * for Pa, 4/3 for ma) rather than equal-tempered semitones, so long
 * drones beat less against each other.
 *
 * HOW THIS DIFFERS FROM Quantizer (quantizer.h): Quantizer snaps a
 * control voltage to an equal-tempered scale. Colour hands out the
 * frequencies of a five-note just scale by index, and can generate a
 * 16-step melody of indices to play through it.
 */

#pragma once

#include "buddhabox/core/random.h"

namespace bb {

/**
 * Just-intonation ratios for the 13 notes of the octave, Sa to Sa'.
 * Index: 0 Sa, 1 Re komal, 2 Re, 3 Ga komal, 4 Ga, 5 ma, 6 tivra Ma,
 * 7 Pa, 8 Dha komal, 9 Dha, 10 Ni komal, 11 Ni, 12 Sa'.
 *
 * A namespace-scope constexpr array has internal linkage, so every file
 * that includes this header gets its own 52-byte copy in flash and the
 * linker never sees a duplicate definition.
 */
constexpr float JUST_OFFSETS[13] = {1.0f,  1.067f, 1.125f, 1.2f, 1.25f,
                                    1.33f, 1.406f, 1.5f,   1.6f, 1.667f,
                                    1.8f,  1.875f, 2.0f};

/**
 * Colour indices ordered from darkest/night-time (index 0) to
 * brightest/day-time (index 15).
 *
 * Darkness is driven primarily by the lower variable scale degree
 * (Re komal < Ga komal < Re < Ga) and secondarily by the upper
 * variable degree (Dha komal < Ni komal < Dha < Ni).
 *
 * Example:
 *   int pos = map(lightValue, 0, 1023, 0, 15);
 *   colour.setColour(COLOUR_BY_BRIGHTNESS[pos]);
 */
constexpr uint8_t COLOUR_BY_BRIGHTNESS[16] = {
    0,  // Re komal + Dha komal — Phrygian flat-6, darkest
    2,  // Re komal + Ni komal  — Phrygian flat-7, haunting
    1,  // Re komal + Dha       — Phrygian, dark with major 6th
    3,  // Re komal + Ni        — Phrygian with major 7th
    8,  // Ga komal + Dha komal — minor flat-6, ominous
    10, // Ga komal + Ni komal  — minor blues, brooding
    9,  // Ga komal + Dha       — natural minor (Aeolian)
    11, // Ga komal + Ni        — harmonic minor
    4,  // Re + Dha komal       — minor pentatonic feel
    6,  // Re + Ni komal        — Mixolydian, floating
    5,  // Re + Dha             — major pentatonic
    12, // Ga + Dha komal       — major with dark 6th, tense
    7,  // Re + Ni              — major with leading tone
    14, // Ga + Ni komal        — bright, slightly unresolved
    13, // Ga + Dha             — bright, Lydian-ish
    15, // Ga + Ni              — full major, brightest
};

/**
 * A five-note just scale with a selectable colour, and a melody of
 * scale indices to play through it.
 *
 * Note numbers count up the scale across octaves: 0-4 are the first
 * octave (Sa, lower note, ma, Pa, upper note), 5-9 the next, and so on.
 *
 * Example:
 *   Colour colour;
 *   LFSR rng;
 *   colour.setRoot(65.4f);                  // C2
 *   colour.setColour(COLOUR_BY_BRIGHTNESS[10]);
 *   colour.newMelody(&rng, 7);              // indices 0..7
 *
 *   // On each clock tick, play the next melody note two octaves up.
 *   osc.setFrequency(colour.getNote(2, colour.getMelody(step++)));
 */
class Colour {
public:
  /** Steps in a melody from newMelody(). */
  static const int kMelodyLength = 16;

  /**
   * Frequency of a scale note.
   * @param octave  Octaves above the root, 0 to about 6
   * @param num     Scale index: 0-4 first octave, 5-9 the next, ...
   * @return        Frequency in Hz
   *
   * Example:
   *   float pa = colour.getNote(1, 3); // Pa, one octave above the root
   */
  float getNote(uint8_t octave, uint8_t num) {
    uint8_t n = num % 5; // which of the five notes
    uint8_t o = num / 5; // how many octaves the index itself climbs
    // 1 << k is 2^k: each octave doubles the frequency.
    return (root_ * offsets_[n]) * (1 << (o + octave));
  }

  /**
   * Choose the scale's two variable notes.
   * @param colour  0 to 15 (wraps); see COLOUR_BY_BRIGHTNESS for an
   *                order from dark to bright
   *
   * Example:
   *   colour.setColour(15); // Ga + Ni: the major pentatonic plus ma
   */
  void setColour(uint8_t colour) {
    uint8_t c = colour % 16;
    uint8_t bottom = c / 4; // 0..3: Re komal, Re, Ga komal, Ga
    uint8_t top = c % 4;    // 0..3: Dha komal, Dha, Ni komal, Ni

    offsets_[0] = JUST_OFFSETS[0];          // Sa
    offsets_[1] = JUST_OFFSETS[bottom + 1]; // lower variable note
    offsets_[2] = JUST_OFFSETS[5];          // ma
    offsets_[3] = JUST_OFFSETS[7];          // Pa
    offsets_[4] = JUST_OFFSETS[top + 8];    // upper variable note
    colour_ = c;
  }

  /** @return  The current colour, 0 to 15 */
  uint8_t getColour() const { return colour_; }

  /**
   * Set the root (Sa).
   * @param hz  Root frequency in Hz. This is a frequency, not a MIDI
   *            note: use midiToFreq() to convert one.
   *
   * Example:
   *   colour.setRoot(midiToFreq(NOTE_C2));
   */
  void setRoot(float hz) { root_ = hz; }

  /** @return  The root frequency in Hz */
  float getRoot() const { return root_; }

  /**
   * Generate a new 16-step melody of scale indices.
   *
   * Step 0 is always Sa and the last step is Sa or Pa, so phrases
   * start and end at rest. The steps between are random, with a
   * second roll whenever a note would repeat the one before.
   *
   * @param rng   Random source
   * @param rmax  Highest scale index to use, 0 to 255 (e.g. 7 reaches
   *              ma in the second octave)
   *
   * Example:
   *   colour.newMelody(&rng, 9); // two octaves of the scale
   */
  void newMelody(LFSR *rng, uint8_t rmax) {
    melody_[0] = 0;
    for (int i = 1; i < kMelodyLength - 1; i++) {
      uint8_t n = rng->nextRange(0, rmax);
      // Naive, best-effort: one re-roll discourages repeated notes
      // without ever looping forever when rmax is 0.
      if (n == melody_[i - 1])
        n = rng->nextRange(0, rmax);
      melody_[i] = n;
    }
    // End on Sa (index 0) or Pa (index 3), half the time each.
    melody_[kMelodyLength - 1] = rng->nextFloat() < 0.5f ? 0 : 3;
  }

  /**
   * Read one melody step.
   * @param idx  Step number (wraps every 16)
   * @return     A scale index to pass to getNote()
   */
  uint8_t getMelody(uint8_t idx) const { return melody_[idx % kMelodyLength]; }

private:
  uint8_t colour_ = 0;
  float offsets_[5] = {JUST_OFFSETS[0], JUST_OFFSETS[1], JUST_OFFSETS[5],
                       JUST_OFFSETS[7], JUST_OFFSETS[8]};
  float root_ = 65.41f; // C2
  uint8_t melody_[kMelodyLength] = {};
};

} // namespace bb
