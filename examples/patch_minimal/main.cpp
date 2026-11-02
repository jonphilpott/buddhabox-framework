/**
 * @file main.cpp
 * @brief Runs the patch in patch_graph.h. This file is yours: edit it
 *        freely. gen_patch.py wrote it once and will not overwrite it.
 *
 * patch_graph.h is generated from patch.json: change the patch there and
 * rerun tools/gen_patch.py. Reach any module from here as patch::<id>,
 * e.g. patch::osc.setFrequency(440.0f), to add your own logic.
 */

#include <Arduino.h>

#include "buddhabox/engine/audio_engine.h"
#include "patch_graph.h"

using namespace bb;

/**
 * Fill one block of audio. Runs in interrupt context: no Serial, no
 * analogRead(), no allocation.
 *
 * @param buffer  Interleaved stereo int16 samples [L, R, L, R, ...]
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    float left, right;
    patch::process(left, right);
    buffer[i] = (int16_t)(clampf(left, -1.0f, 1.0f) * 32000.0f);
    buffer[i + 1] = (int16_t)(clampf(right, -1.0f, 1.0f) * 32000.0f);
  }
}

void setup() {
  patch::setup();
  AudioEngine::begin();
}

void loop() {
  // Read pots or buttons here and pass the values to patch::<id>.
}
