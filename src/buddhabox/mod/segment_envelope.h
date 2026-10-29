/**
 * @file segment_envelope.h
 * @brief Peaks' multi-segment envelope: ADSR, AD, AR, looping, or custom.
 *
 * HOW THIS DIFFERS FROM Envelope (envelope.h): Envelope is the original
 * BuddhaBox attack-sustain-release envelope with exponential RC-style
 * curves (no decay stage). SegmentEnvelope is a
 * general list of up to six segments, each with its own level, time and
 * curve, plus a sustain point and an optional loop — so it can be an
 * ADSR, a looping LFO of any shape, or a custom contour.
 *
 * HOW IT WORKS:
 * - Segment i ramps from level[i] to level[i+1] over time[i] seconds,
 *   following a curve: linear, exponential (fast then slow) or quartic
 *   (slow then fast).
 * - A rising gate restarts at segment 0, from the current level (so a
 *   retrigger doesn't click) unless hard reset is on.
 * - The sustain point is a segment the envelope holds at while the gate
 *   stays high; a falling gate jumps to it ("release").
 * - A loop (start..end) makes the envelope cycle through those segments.
 *
 * BLACK PILL CHANGE: the exponential and quartic curves came from expf()
 * and powf() every sample in Partials; here they are read from flash
 * tables (lutEnvExpo, lutEnvQuartic), as Peaks itself did.
 *
 * Original: Peaks, 16-bit fixed point at 48 kHz. This port is float with
 * times in seconds.
 *
 * Derived from Peaks, Copyright 2013 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/luts.h"

namespace bb {

/**
 * Multi-segment envelope generator.
 *
 * Example — ADSR on a gate:
 *   SegmentEnvelope env;
 *   env.setAdsr(0.01f, 0.3f, 0.6f, 1.5f); // seconds and sustain level
 *
 *   float level = env.process(keyHeld);
 *   float s = osc.process() * level;
 *
 * Example — a looping, skewed LFO (fast rise, slow fall):
 *   env.setAdLoop(0.1f, 0.9f);
 *   float lfo = env.process(true);
 */
class SegmentEnvelope {
public:
  /** Segment curve shapes. */
  enum Shape { LINEAR, EXPONENTIAL, QUARTIC };

  /** Most segments in one envelope. */
  static const int kMaxSegments = 6;

  SegmentEnvelope() { init(); }

  /**
   * Reset to an idle ADSR (2 ms, 100 ms, 0.5, 300 ms).
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    sampleTime_ = 1.0f / sampleRate;
    setAdsr(0.002f, 0.1f, 0.5f, 0.3f);
    segment_ = numSegments_; // idle
    phase_ = 0.0f;
    increment_ = 0.0f;
    startValue_ = value_ = 0.0f;
    previousGate_ = false;
  }

  // ── Presets (times in seconds, levels 0..1) ─────────────────────────
  // Peaks uses a quartic attack and exponential decay/release for ADSR,
  // exponential AD, and linear shapes for the others.

  /**
   * Attack-decay-sustain-release.
   * @param attack   Seconds
   * @param decay    Seconds
   * @param sustain  Level, 0.0 to 1.0
   * @param release  Seconds
   */
  void setAdsr(float attack, float decay, float sustain, float release) {
    set3(attack, decay, sustain, release, 2, QUARTIC, EXPONENTIAL, EXPONENTIAL,
         0, 0);
  }

  /** Attack-decay, ignoring the gate length. @param attack, decay  s */
  void setAd(float attack, float decay) {
    set2(attack, decay, 0, EXPONENTIAL, 0);
  }

  /** Attack-release: holds at full while the gate is high. Seconds. */
  void setAr(float attack, float release) {
    set2(attack, release, 1, LINEAR, 0);
  }

  /** Attack, decay to sustain, then release, ignoring the gate length. */
  void setAdr(float attack, float decay, float sustain, float release) {
    set3(attack, decay, sustain, release, 0, LINEAR, LINEAR, LINEAR, 0, 0);
  }

  /** Looping attack-decay: cycles while running, like an LFO. Seconds. */
  void setAdLoop(float attack, float decay) {
    set2(attack, decay, 0, LINEAR, 2);
  }

  /** Looping attack-decay-release. Seconds and sustain level. */
  void setAdrLoop(float attack, float decay, float sustain, float release) {
    set3(attack, decay, sustain, release, 0, LINEAR, LINEAR, LINEAR, 0, 3);
  }

  // ── Custom segments ─────────────────────────────────────────────────
  // Segment i starts at level(i) and ends at level(i + 1), so a
  // 3-segment envelope uses levels 0..3.

  /** @param i  Level index 0..6  @param level  0.0 to 1.0 */
  void setLevel(int i, float level) { level_[i] = level; }

  /** @param i  Segment 0..5  @param seconds  Duration */
  void setTime(int i, float seconds) { time_[i] = seconds; }

  /** @param i  Segment 0..5  @param shape  Its curve */
  void setShape(int i, Shape shape) { shape_[i] = shape; }

  /** @param n  Number of segments in use, 1..6 */
  void setNumSegments(int n) { numSegments_ = n; }

  /** @param segment  Hold at the start of this segment while the gate
   *                  is high (0 = no sustain) */
  void setSustainPoint(int segment) { sustainPoint_ = segment; }

  /**
   * Loop from segment `start` when segment `end` is reached.
   * @param start  First looped segment
   * @param end    Segment that jumps back (0 = no loop)
   */
  void setLoop(int start, int end) {
    loopStart_ = start;
    loopEnd_ = end;
  }

  /** @param on  true: every trigger restarts from level 0 */
  void setHardReset(bool on) { hardReset_ = on; }

  /**
   * Advance one sample.
   * @param gate  true while the key/gate is held
   * @return      The envelope level, 0.0 to 1.0
   */
  float process(bool gate) {
    bool rising = gate && !previousGate_;
    bool falling = !gate && previousGate_;
    previousGate_ = gate;
    // 1. Events: trigger, release, or the end of a segment.
    if (rising) {
      startValue_ =
          (segment_ == numSegments_ || hardReset_) ? level_[0] : value_;
      segment_ = 0;
      phase_ = 0.0f;
    } else if (falling && sustainPoint_) {
      startValue_ = value_;
      segment_ = sustainPoint_;
      phase_ = 0.0f;
    } else if (phase_ >= 1.0f) {
      startValue_ = level_[segment_ + 1];
      ++segment_;
      phase_ = 0.0f;
      if (segment_ == loopEnd_)
        segment_ = loopStart_;
    }
    // 2. Speed: zero when finished or sustaining.
    bool done = segment_ == numSegments_;
    bool sustained = sustainPoint_ && segment_ == sustainPoint_ && gate;
    increment_ =
        sustained || done ? 0.0f : sampleTime_ / maxf(time_[segment_], 1e-5f);
    // 3. Value along the segment's curve, then advance.
    float t = curve(shape_[segment_], minf(phase_, 1.0f));
    value_ = startValue_ + (level_[segment_ + 1] - startValue_) * t;
    phase_ += increment_;
    return value_;
  }

  /** @return  The level from the last process() call */
  float value() const { return value_; }

  /** @return  true while the envelope is running (not finished) */
  bool active() const { return segment_ < numSegments_; }

private:
  /**
   * Peaks' curves on 0..1: linear; exponential (fast then slow); quartic
   * x^3.32 (slow then fast). The two curved ones come from flash tables.
   */
  static float curve(Shape shape, float x) {
    switch (shape) {
    case EXPONENTIAL:
      return interpolate(lutEnvExpo(), x, 256.0f);
    case QUARTIC:
      return interpolate(lutEnvQuartic(), x, 256.0f);
    case LINEAR:
    default:
      return x;
    }
  }

  void set2(float a, float d, int sustainPoint, Shape shape, int loopEnd) {
    numSegments_ = 2;
    sustainPoint_ = sustainPoint;
    level_[0] = 0.0f;
    level_[1] = 1.0f;
    level_[2] = 0.0f;
    time_[0] = a;
    time_[1] = d;
    shape_[0] = shape_[1] = shape;
    loopStart_ = 0;
    loopEnd_ = loopEnd;
  }

  void set3(float a, float d, float s, float r, int sustainPoint, Shape sa,
            Shape sd, Shape sr, int loopStart, int loopEnd) {
    numSegments_ = 3;
    sustainPoint_ = sustainPoint;
    level_[0] = 0.0f;
    level_[1] = 1.0f;
    level_[2] = s;
    level_[3] = 0.0f;
    time_[0] = a;
    time_[1] = d;
    time_[2] = r;
    shape_[0] = sa;
    shape_[1] = sd;
    shape_[2] = sr;
    loopStart_ = loopStart;
    loopEnd_ = loopEnd;
  }

  float sampleTime_ = 1.0f / SAMPLE_RATE;
  float level_[kMaxSegments + 1] = {0, 0, 0, 0, 0, 0, 0};
  float time_[kMaxSegments] = {0.01f, 0.01f, 0.01f, 0.01f, 0.01f, 0.01f};
  Shape shape_[kMaxSegments] = {LINEAR, LINEAR, LINEAR, LINEAR, LINEAR, LINEAR};
  int numSegments_ = 3, sustainPoint_ = 2, loopStart_ = 0, loopEnd_ = 0;
  bool hardReset_ = false;
  int segment_ = 3;
  float phase_ = 0.0f, increment_ = 0.0f;
  float startValue_ = 0.0f, value_ = 0.0f;
  bool previousGate_ = false;
};

} // namespace bb
