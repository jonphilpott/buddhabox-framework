/**
 * @file midi_utils.h
 * @brief MIDI note number to frequency conversion and note constants.
 *
 * MIDI (Musical Instrument Digital Interface) represents musical notes as
 * numbers from 0 to 127. This file provides the math to convert those
 * numbers into frequencies (Hz) that oscillators understand.
 *
 * THE MIDI NOTE NUMBER SYSTEM:
 * ───────────────────────────
 * MIDI note numbers are arranged chromatically (every semitone = +1):
 *
 *   Note  | MIDI | Frequency
 *   ──────┼──────┼──────────
 *   C2    |  36  |   65.4 Hz
 *   C3    |  48  |  130.8 Hz
 *   A3    |  57  |  220.0 Hz
 *   C4    |  60  |  261.6 Hz  (Middle C)
 *   A4    |  69  |  440.0 Hz  (Concert A — the tuning reference)
 *   C5    |  72  |  523.3 Hz
 *
 * The formula: freq = 440 * 2^((note - 69) / 12)
 *
 * This works because:
 *   - An octave = doubling the frequency = +12 semitones
 *   - Each semitone multiplies frequency by 2^(1/12) ≈ 1.0595
 *   - Note 69 (A4) is defined as exactly 440 Hz
 *
 * BUILDING SCALES:
 * ────────────────
 * To create a scale, just list the MIDI note numbers. For example:
 *   C major:      {60, 62, 64, 65, 67, 69, 71}  (whole, whole, half, whole,
 * whole, whole, half) C pentatonic: {60, 62, 64, 67, 69}           (the "can't
 * go wrong" scale) C minor:      {60, 62, 63, 65, 67, 68, 70}
 *
 * To transpose to a different key, add an offset to all notes.
 * E.g., D major = C major with +2 added to each note.
 *
 * Example usage:
 *   // Play a random note from the A minor pentatonic scale
 *   uint8_t scale[] = {57, 60, 62, 64, 67};  // A C D E G
 *   uint8_t note = scale[rng.nextRange(0, 4)];
 *   osc.setFrequency(midiToFreq(note));
 *
 *   // Play Middle C
 *   osc.setFrequency(midiToFreq(NOTE_C4));  // 261.6 Hz
 *
 *   // Transpose up one octave
 *   osc.setFrequency(midiToFreq(NOTE_C4 + 12));  // 523.3 Hz
 */

#pragma once

#include <cmath>
#include <cstdint>

namespace bb {

/**
 * Convert a MIDI note number to frequency in Hz.
 *
 * Uses the standard equal temperament tuning formula with A4 = 440 Hz.
 *
 * @param noteNumber  MIDI note (0–127). Middle C = 60, Concert A = 69.
 * @return            Frequency in Hz.
 *
 * Examples:
 *   midiToFreq(69)  → 440.0   (A4)
 *   midiToFreq(60)  → 261.6   (C4, Middle C)
 *   midiToFreq(36)  → 65.4    (C2, deep bass)
 *   midiToFreq(84)  → 1046.5  (C6, high)
 */
inline float midiToFreq(uint8_t noteNumber) {
  // 440 * 2^((note - 69) / 12)
  // We use powf() here since this is called infrequently (on note changes,
  // not per-sample), so the cost of a power function is negligible.
  return 440.0f * powf(2.0f, ((float)noteNumber - 69.0f) / 12.0f);
}

// ─── Note Name Constants ────────────────────────────────────────────────────
//
// These provide readable names for common reference pitches.
// For sharps/flats, just add or subtract from the base note:
//   C#4 = NOTE_C4 + 1 = 61
//   Eb4 = NOTE_C4 + 3 = 63
//   G4  = NOTE_C4 + 7 = 67
//
// INTERVAL REFERENCE (semitones above root):
//   +0  = Unison      +5  = Perfect 4th   +10 = Minor 7th
//   +1  = Minor 2nd   +7  = Perfect 5th   +11 = Major 7th
//   +2  = Major 2nd   +8  = Minor 6th     +12 = Octave
//   +3  = Minor 3rd   +9  = Major 6th
//   +4  = Major 3rd

constexpr uint8_t NOTE_C0 = 12;
constexpr uint8_t NOTE_C1 = 24;
constexpr uint8_t NOTE_C2 = 36;
constexpr uint8_t NOTE_C3 = 48;
constexpr uint8_t NOTE_C4 = 60; // Middle C
constexpr uint8_t NOTE_C5 = 72;
constexpr uint8_t NOTE_C6 = 84;
constexpr uint8_t NOTE_C7 = 96;
constexpr uint8_t NOTE_A4 = 69; // Concert A = 440 Hz

} // namespace bb
