/**
 * @file main.cpp
 * @brief The smallest complete BuddhaBox program: one sine tone.
 *
 * Every BuddhaBox program has the same three parts:
 * 1. DSP objects as globals (allocated at compile time, never with new).
 * 2. audioCallback(): called by the DMA interrupt whenever the DAC needs
 *    another block of samples. Fill the buffer one stereo frame at a time.
 * 3. setup(): configure the objects, then start the audio engine.
 *
 * Wiring: PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN on the UDA1334A.
 */

#include <Arduino.h>

#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/osc/oscillator.h"

using namespace bb;

// 1. DSP objects.
Oscillator osc;

/**
 * Fill one block of audio. Runs in interrupt context: no Serial, no
 * analogRead(), no allocation.
 *
 * @param buffer  Interleaved stereo int16 samples [L, R, L, R, ...]
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    // DSP works in floats from -1.0 to 1.0; convert to 16-bit only here,
    // with a little headroom (32000 rather than 32767).
    float sample = osc.process();
    int16_t s = (int16_t)(clampf(sample, -1.0f, 1.0f) * 32000.0f);
    buffer[i] = s;     // left
    buffer[i + 1] = s; // right
  }
}

void setup() {
  // 3. Configure, then start the DMA. Audio begins immediately.
  osc.setWaveform(Waveform::SINE);
  osc.setFrequency(220.0f);
  osc.setAmplitude(0.5f);
  AudioEngine::begin();
}

void loop() {
  // Nothing to do: the interrupt keeps the audio flowing. Read pots or
  // buttons here and pass the values to the DSP objects.
}
