/**
 * @file main.cpp
 * @brief A generative drum machine from the Plaits and Peaks drum models.
 *
 * TempoClock ─► step counter ─► RandomSequence (variation)
 *                   │
 *     ┌─────────────┼─────────────┬───────────────┐
 *     ▼             ▼             ▼               ▼
 * AnalogKick   AnalogSnare     HiHat          FmDrum (tom)
 *     └─────────────┴──────┬──────┴───────────────┘
 *                          ▼
 *                  Overdrive ─► Limiter ─► DAC
 *
 * A fixed four-on-the-floor kick and backbeat, with hats and toms chosen
 * by a RandomSequence whose "déjà vu" setting keeps the variations
 * looping, so the groove repeats with small surprises.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/drums/analog_kick.h"
#include "buddhabox/drums/analog_snare.h"
#include "buddhabox/drums/fm_drum.h"
#include "buddhabox/drums/hihat.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/overdrive.h"
#include "buddhabox/mod/clock.h"
#include "buddhabox/mod/random_sequence.h"

using namespace bb;

AnalogKick kick;
AnalogSnare snare;
HiHat hat;
FmDrum tom;
TempoClock clock16(118.0f, 4); // 118 BPM, ticks on sixteenth notes
RandomSequence variation;
Overdrive drive;
Limiter limiter;
int step = 0;

/**
 * Decide which drums play on this sixteenth note.
 * @param s  Step within the bar, 0..15
 */
void onStep(int s) {
  if (s % 4 == 0)
    kick.trigger();
  if (s == 4 || s == 12)
    snare.trigger();
  // Hats on most steps; accent and length vary with the sequence.
  float v = variation.next();
  if (v > 0.25f) {
    hat.setAccent(s % 2 ? 0.4f : 0.9f);
    hat.setDecay(v > 0.85f ? 0.6f : 0.15f); // the odd open hat
    hat.trigger();
  }
  if (v > 0.92f)
    tom.trigger();
}

/**
 * Fill one block of audio.
 * @param buffer  Interleaved stereo int16 samples
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    if (clock16.process()) {
      onStep(step);
      step = (step + 1) & 15;
    }
    float mix = kick.process() * 0.8f + snare.process() * 0.4f +
                hat.process() * 0.25f + tom.process() * 0.3f;
    mix = limiter.process(drive.process(mix));
    int16_t out = (int16_t)(mix * 32000.0f);
    buffer[i] = out;
    buffer[i + 1] = out;
  }
}

void setup() {
  kick.setFrequency(48.0f);
  kick.setDecay(0.55f);
  snare.setFrequency(190.0f);
  snare.setSnappy(0.65f);
  tom.morph(0.6f, 0.3f, false);
  variation.setLength(16);
  variation.setDejaVu(0.4f); // mostly repeating, slowly mutating
  drive.setDrive(0.3f);
  Serial.begin(115200); // USB serial, for the CPU meter below
  AudioEngine::begin();
}

/**
 * Report how much of the CPU the audio uses, once a second. Open the
 * monitor with `pio device monitor`.
 */
void loop() {
  Serial.print("CPU: ");
  Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
  Serial.println("%");
  delay(1000);
}
