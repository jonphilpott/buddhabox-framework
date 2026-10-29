/**
 * @file main.cpp
 * @brief Singing bowls: struck resonators, chaotic drift and reverb.
 *
 * A generative ambient patch built from the Mutable Instruments ports:
 *
 *   Dust ──► ModalResonator (bowl) ──┐
 *   Particle (droplets) ─────────────┼─► mix ─► Hilbert frequency shift
 *   StringSynthOscillator (drone) ───┘            │
 *                                                 ▼
 *          Lorenz ──► modulates bowl structure, drone filter, shift
 *                                                 │
 *                                   Reverb ◄──────┘ ─► Limiter ─► DAC
 *
 * - Dust strikes the bowl at random, a few times a minute.
 * - Particle adds sparse, glassy pings around the bowl's pitch.
 * - A string-machine drone sits underneath, through a slowly opening
 *   low-pass.
 * - A Lorenz attractor wanders the bowl's inharmonicity and the drone's
 *   brightness, so the patch never quite repeats.
 * - A Hilbert frequency shifter detunes everything by a fraction of a Hz,
 *   which makes the reverb tail beat and swirl.
 *
 * RAM: about 41 KB (32 KB of it is the reverb). The serial monitor prints
 * the measured CPU load once a second.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/filter/hilbert.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/reverb.h"
#include "buddhabox/mod/lorenz.h"
#include "buddhabox/noise/dust.h"
#include "buddhabox/noise/particle.h"
#include "buddhabox/osc/sine.h"
#include "buddhabox/osc/string_synth.h"
#include "buddhabox/physical/modal_resonator.h"

using namespace bb;

// ── Voices ──────────────────────────────────────────────────────────────
ModalResonator<> bowl; // no bowing: saves RAM
Dust striker;
Particle droplets;
StringSynthOscillator drone;
Svf droneFilter;

// ── Modulation and effects ──────────────────────────────────────────────
Lorenz chaos;
Hilbert hilbert;
SineOscillator shifter; // quadrature carrier for the frequency shift
Reverb<> reverb;        // 32 KB
Limiter limiter;

// Control-rate work runs every kControlPeriod samples (about 1.4 kHz),
// which keeps per-sample cost down.
const int kControlPeriod = 32;
int controlCounter = 0;

/**
 * Update slow-moving parameters from the Lorenz attractor.
 */
void controlUpdate() {
  // Lorenz outputs wander around ±1; map them to musical ranges.
  bowl.setStructure(0.15f + 0.1f * chaos.x()); // around bell-like
  bowl.setBrightness(0.5f + 0.2f * chaos.y());
  float cutoff = 300.0f + 250.0f * (chaos.z() + 1.0f);
  droneFilter.setCoefficients(cutoff / SAMPLE_RATE, 0.9f, TanApprox::Fast);
}

/**
 * Fill one block of audio (see the minimal example for the basics).
 * @param buffer  Interleaved stereo int16 samples
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    // 1. Slow modulation.
    chaos.process();
    if (++controlCounter >= kControlPeriod) {
      controlCounter = 0;
      controlUpdate();
    }

    // 2. Sources: the struck bowl, the droplets, the filtered drone.
    float bowlOut = bowl.process(striker.process());
    float drops = droplets.process() * 0.3f;
    float droneOut = droneFilter.process(drone.process()).lp * 0.15f;
    float mix = bowlOut * 0.5f + drops + droneOut;

    // 3. Frequency shift by ~0.3 Hz: I*cos - Q*sin moves every partial up.
    float s, c;
    hilbert.process(mix);
    shifter.next((0.3f + 0.2f * chaos.y()) / SAMPLE_RATE, 1.0f, s, c);
    float shifted = hilbert.i() * c - hilbert.q() * s;

    // 4. Stereo: dry mix left, shifted copy right, then reverb.
    float left = mix, right = shifted;
    reverb.process(left, right);
    limiter.process(left, right);

    buffer[i] = (int16_t)(left * 32000.0f);
    buffer[i + 1] = (int16_t)(right * 32000.0f);
  }
}

void setup() {
  bowl.setFrequency(146.8f); // D3
  bowl.setModes(16);
  bowl.setDamping(0.85f);
  bowl.setPosition(0.3f);
  striker.setDensity(0.08f); // about five strikes a minute

  droplets.setFrequency(1174.7f); // D6, two octaves above the bowl
  droplets.setDensity(0.6f);
  droplets.setSpread(7.0f);
  droplets.setQ(300.0f);

  drone.setFrequency(73.4f); // D2
  drone.setRegistration(0.25f);

  chaos.setRate(0.05f); // very slow drift

  reverb.setTime(0.9f);
  reverb.setAmount(0.45f);
  reverb.setLp(0.6f);

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
