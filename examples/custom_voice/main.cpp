/**
 * @file main.cpp
 * @brief Composition example: build your own voice and your own effect.
 *
 * Two patterns for extending the framework without touching it:
 *
 * 1. A voice class (BellVoice) that owns existing modules and exposes a
 *    small, musical interface: setPitch(), strike(), process().
 * 2. An effect class (PingPongEcho) written on FxEngine, the same toolkit
 *    the Reverb and Chorus are built with.
 *
 * The patch: two bells play notes chosen by a RandomSequence and snapped
 * to a pentatonic scale by a Quantizer, into a ping-pong echo, then a
 * limiter.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/dynamics/vactrol_lpg.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/fx/fx_engine.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/mod/clock.h"
#include "buddhabox/mod/quantizer.h"
#include "buddhabox/mod/random_sequence.h"
#include "buddhabox/physical/modal_resonator.h"

using namespace bb;

/**
 * A struck bell: a modal resonator through a vactrol low-pass gate.
 *
 * The resonator rings; the gate adds the natural "darkening" of a real
 * decay (quieter and duller together). Each strike is a one-sample
 * impulse into the resonator plus a ping of the gate.
 *
 * Example:
 *   BellVoice bell;
 *   bell.setPitch(440.0f);
 *   bell.strike(0.8f);
 *   float s = bell.process();
 */
class BellVoice {
public:
  BellVoice() {
    // 8 modes: about 510 cycles per bell, measured (12 would be 700).
    resonator_.setModes(8);
    resonator_.setStructure(0.2f); // bell-like, slightly inharmonic
    resonator_.setDamping(0.75f);
    resonator_.setPosition(0.3f);
    lpg_.setDecay(0.8f);
    lpg_.setColour(0.6f);
  }

  /**
   * Tune the bell.
   * @param hz  Fundamental in Hz
   */
  void setPitch(float hz) {
    resonator_.setFrequency(hz);
    lpg_.setAttackPitch(hz); // higher bells open faster, as in Plaits
  }

  /**
   * Strike the bell.
   * @param velocity  0.0 to 1.0
   */
  void strike(float velocity) {
    impulse_ = velocity;
    lpg_.trigger();
  }

  /**
   * Generate the next sample.
   * @return  The bell, about ±1
   */
  float process() {
    float excitation = impulse_;
    impulse_ = 0.0f; // the strike lasts exactly one sample
    return lpg_.process(resonator_.process(excitation));
  }

private:
  ModalResonator<> resonator_;
  VactrolLpg lpg_;
  float impulse_ = 0.0f;
};

/**
 * Stereo ping-pong echo written on FxEngine.
 *
 * Two delay lines cross-feed: the left echo feeds the right line and
 * vice versa, so each repeat bounces to the other side.
 *
 * @tparam SIZE  FxEngine buffer (power of two); RAM = SIZE * 2 bytes
 *
 * Example:
 *   PingPongEcho<8192> echo;
 *   float l = in, r = in;
 *   echo.process(l, r);
 */
template <uint32_t SIZE = 8192> class PingPongEcho {
public:
  PingPongEcho() {
    // Declare the lines once, then allocate: each line is just a region
    // of the shared buffer. Half the buffer each, minus the gap samples.
    left_ = engine_.addDelay(SIZE / 2 - 2);
    right_ = engine_.addDelay(SIZE / 2 - 2);
    engine_.allocate();
  }

  /** @param x  Feedback per bounce, 0.0 to 0.9 */
  void setFeedback(float x) { feedback_ = clampf(x, 0.0f, 0.9f); }

  /** @param x  Wet amount, 0.0 to 1.0 */
  void setAmount(float x) { amount_ = x; }

  /**
   * Process one stereo sample in place.
   * @param left   Left in/out
   * @param right  Right in/out
   */
  void process(float &left, float &right) {
    FxEngine<SIZE> &c = engine_;
    float echoL, echoR;
    c.start(); // advance every line by one sample
    // Left line: the dry left plus feedback from the right line's tail.
    c.read(left, 0.5f);
    c.readTail(right_, feedback_);
    c.write(left_, 0.0f);
    // Right line: the dry right plus feedback from the left line's tail.
    c.read(right, 0.5f);
    c.readTail(left_, feedback_);
    c.write(right_, 0.0f);
    // Outputs: each side hears the *other* line's tail.
    c.readTail(right_, 1.0f);
    c.write(echoL, 0.0f);
    c.readTail(left_, 1.0f);
    c.write(echoR, 0.0f);
    left += amount_ * echoL;
    right += amount_ * echoR;
  }

private:
  FxEngine<SIZE> engine_;
  typename FxEngine<SIZE>::Delay left_, right_;
  float feedback_ = 0.5f, amount_ = 0.4f;
};

// ── The patch ───────────────────────────────────────────────────────────
const int kNumBells = 2; // about 54% of the CPU for both
BellVoice bells[kNumBells];
PingPongEcho<8192> echo; // 16 KB: about 93 ms per side
Limiter limiter;
TempoClock clock8(84.0f, 2); // eighth notes at 84 BPM
RandomSequence melody;
Quantizer scale;
Random rng;
int nextBell = 0;

/**
 * On each eighth note, maybe strike the next bell at a new pitch.
 */
void onTick() {
  if (rng.uniform() > 0.6f)
    return;                                       // leave space
  float pitch = 1.5f * melody.next();             // 0 to 1.5 octaves
  float q = scale.process(pitch, 0.7f);           // snap to the scale
  bells[nextBell].setPitch(220.0f * fastExp2(q)); // from A3
  bells[nextBell].strike(0.5f + 0.5f * rng.uniform());
  nextBell = (nextBell + 1) % kNumBells; // round-robin voices
}

/**
 * Fill one block of audio.
 * @param buffer  Interleaved stereo int16 samples
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    if (clock8.process())
      onTick();
    float mix = 0.0f;
    // Gain 2.0, set from a host simulation: a one-sample strike puts
    // little energy into a resonator, so the bells peak near 0.42,
    // leaving room for the echo's repeats on top.
    for (int b = 0; b < kNumBells; ++b)
      mix += bells[b].process() * 2.0f;
    float left = mix, right = mix;
    echo.process(left, right);
    limiter.process(left, right);
    buffer[i] = (int16_t)(left * 32000.0f);
    buffer[i + 1] = (int16_t)(right * 32000.0f);
  }
}

void setup() {
  scale.init(Quantizer::preset(Quantizer::PENTATONIC));
  melody.setLength(8);
  melody.setDejaVu(0.3f);
  echo.setFeedback(0.55f);
  Serial.begin(115200);
  AudioEngine::begin();
}

/** Print the measured CPU load once a second (pio device monitor). */
void loop() {
  Serial.print("CPU: ");
  Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
  Serial.print("%  peak: ");
  Serial.print(AudioEngine::cpuPeak() * 100.0f, 1);
  Serial.println("%");
  delay(1000);
}
