/**
 * @file clock.h
 * @brief Sample-accurate tempo clock for synchronizing events and sequencing.
 *
 * The TempoClock generates regular "tick" events at a specified BPM. You call
 * process() once per audio sample, and it returns true when a tick occurs.
 * Use these ticks to drive:
 *
 *   - Note changes in a generative sequence
 *   - LFO resets (sync LFO phase to the beat)
 *   - Probability-based events ("30% chance of changing note each beat")
 *   - Step sequencer advances
 *   - Rhythmic volume gates (tremolo synced to tempo)
 *
 * WHY SAMPLE-ACCURATE TIMING?
 * ──────────────────────────
 * You might think: "why not just use millis() in loop()?" Two reasons:
 *
 *   1. PRECISION: millis() has ~1ms resolution. At 120 BPM, a quarter note
 *      is 500ms — a 1ms error is 0.2%. But for faster divisions (sixteenth
 *      notes = 125ms), the error becomes audible as "jitter" or "sloppiness".
 *
 *   2. SYNCHRONIZATION: The audio callback runs at exact sample intervals.
 *      By counting samples instead of wall-clock time, our tick events are
 *      perfectly aligned with the audio stream. A note that starts on a tick
 *      will start at the exact right sample — no latency or drift.
 *
 * DIVISIONS:
 * ──────────
 * The division parameter controls how many ticks per beat:
 *   1 = quarter notes (one tick per beat)
 *   2 = eighth notes (two ticks per beat)
 *   4 = sixteenth notes (four ticks per beat)
 *   3 = triplets (three ticks per beat)
 *   8 = thirty-second notes (eight ticks per beat)
 *
 * Example usage:
 *   TempoClock clock(72.0f, 1);  // 72 BPM, quarter notes
 *   LFSR rng(42);
 *   uint8_t scale[] = {60, 62, 64, 67, 69};  // C pentatonic
 *
 *   // In audio callback:
 *   if (clock.process()) {
 *       // This runs once per beat — pick a new note
 *       uint8_t note = scale[rng.nextRange(0, 4)];
 *       osc.setFrequency(midiToFreq(note));
 *   }
 *   float sample = osc.process();
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Sample-accurate tempo clock for synchronizing events and sequencing.
 *
 * See the file header above for how it works and usage examples.
 */
class TempoClock {
public:
  /**
   * @param bpm       Tempo in beats per minute. Typical range: 40–200.
   *                  For ambient music, try 60–90 BPM.
   * @param division  Ticks per beat. 1=quarter, 2=eighth, 4=sixteenth.
   */
  TempoClock(float bpm = 120.0f, uint8_t division = 1) {
    setBPM(bpm);
    setDivision(division);
  }

  /**
   * Set the tempo in beats per minute.
   *
   * @param bpm  Tempo value. Can be fractional (e.g., 72.5 BPM).
   *             The internal counter uses floats so we don't accumulate
   *             rounding errors over time.
   *
   * Example:
   *   clock.setBPM(80.0f);  // Slow, meditative tempo
   */
  void setBPM(float bpm) {
    bpm_ = bpm;
    recalc();
  }

  /**
   * Set the clock division (ticks per beat).
   *
   * @param div  1 = quarter notes, 2 = eighth notes, 4 = sixteenth notes, etc.
   *             At 120 BPM with div=4, you get 8 ticks per second.
   *
   * Example:
   *   clock.setDivision(2);  // Eighth notes
   */
  void setDivision(uint8_t div) {
    division_ = div > 0 ? div : 1; // Prevent divide-by-zero
    recalc();
  }

  /**
   * Process one sample step. Call this once per sample in your audio callback.
   *
   * @return  true if a clock tick occurred on this sample, false otherwise.
   *          At 120 BPM with division=1, this returns true once every
   *          22050 samples (~500 ms).
   *
   * Example:
   *   // In audio callback:
   *   if (clock.process()) {
   *       // A tick just happened — do something musical!
   *       advanceSequencer();
   *   }
   */
  bool process() {
    sampleCounter_++;
    if (sampleCounter_ >= samplesPerTick_) {
      // Using subtraction instead of reset-to-zero preserves any
      // fractional overshoot, preventing long-term timing drift.
      // For example, if samplesPerTick_ = 22050.5 and we've counted
      // 22051, the remainder (0.5) carries over to the next cycle.
      sampleCounter_ -= samplesPerTick_;
      tickCount_++;
      return true;
    }
    return false;
  }

  /**
   * Reset the clock to the beginning (tick 0, counter 0).
   * Use this to re-sync the clock, e.g., when restarting a sequence.
   */
  void reset() {
    sampleCounter_ = 0.0f;
    tickCount_ = 0;
  }

  /**
   * Get the current tick count (how many ticks have occurred since reset).
   * Useful for sequencer step tracking: step = getTick() % numSteps.
   *
   * @return  Total ticks since last reset (wraps at 2^32)
   *
   * Example:
   *   // 4-step sequence
   *   int step = clock.getTick() % 4;
   */
  uint32_t getTick() const { return tickCount_; }

  /** @return  The current BPM setting. */
  float getBPM() const { return bpm_; }

private:
  /**
   * Recompute the samples-per-tick value from current BPM and division.
   *
   * The math:
   *   beats_per_second = bpm / 60
   *   ticks_per_second = beats_per_second * division
   *   samples_per_tick = sample_rate / ticks_per_second
   *
   * Simplified: samples_per_tick = (sample_rate * 60) / (bpm * division)
   */
  void recalc() {
    samplesPerTick_ = (float)SAMPLE_RATE * 60.0f / (bpm_ * (float)division_);
  }

  float bpm_ = 120.0f;
  uint8_t division_ = 1;
  float samplesPerTick_ = 22050.0f; // Precomputed for efficiency
  float sampleCounter_ = 0.0f;      // Float counter to avoid drift
  uint32_t tickCount_ = 0;          // Total ticks since reset
};

} // namespace bb
