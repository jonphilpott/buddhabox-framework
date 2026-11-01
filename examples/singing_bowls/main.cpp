/**
 * @file main.cpp
 * @brief Singing bowls: struck resonators, chaotic drift and reverb.
 *
 * A generative ambient patch built from the Mutable Instruments ports:
 *
 *   Dust ──► ModalResonator (bowl) ──┐   (stereo from the bowl's side)
 *   Particle (droplets) ─────────────┼─► mix ─► Reverb ─► Limiter ─► DAC
 *   StringSynthOscillator (drone) ───┘
 *
 *          Lorenz ──► modulates bowl structure, brightness, drone filter
 *
 * - Dust strikes the bowl at random, a few times a minute.
 * - Particle adds sparse, glassy pings around the bowl's pitch.
 * - A string-machine drone sits underneath, through a slowly opening
 *   low-pass.
 * - A Lorenz attractor wanders the bowl's inharmonicity and the drone's
 *   brightness, so the patch never quite repeats.
 * - The bowl's side() output, a second pickup swept by a slow LFO, is
 *   added to one channel and subtracted from the other, so the bowl
 *   moves across the stereo field.
 *
 * RAM: about 41 KB (32 KB of it is the reverb). The serial monitor prints
 * the measured CPU load once a second; the engine lights the blue LED
 * if a block ever runs late.
 *
 * CPU: about 75%, measured with the profile example: the 12-mode bowl
 * is about 700 cycles per sample and the reverb about 480, of 1905.
 * Each extra mode costs about 48 cycles.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/reverb.h"
#include "buddhabox/mod/lorenz.h"
#include "buddhabox/noise/dust.h"
#include "buddhabox/noise/particle.h"
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
Reverb<> reverb; // 32 KB
Limiter limiter;

// Control-rate work runs every kControlPeriod samples (about 1.4 kHz),
// which keeps per-sample cost down.
const int kControlPeriod = 32;
int controlCounter = 0;

// The bowl's level in the mix (its centre and side outputs alike).
const float kBowlGain = 3.5f;

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
    //    Gains set from a host simulation of two minutes of the patch: a
    //    full strike peaks near 0.5, droplets near 0.17, and the drone
    //    sits about 8 dB below the bowl's average level. The bowl needs
    //    a big gain because a one-sample strike puts little energy in.
    float bowlOut = bowl.process(striker.process()) * kBowlGain;
    float drops = droplets.process() * 0.3f;
    float droneOut = droneFilter.process(drone.process()).lp * 0.06f;
    float mix = bowlOut + drops + droneOut;

    // 3. Stereo: the bowl's side signal (a moving second pickup minus the
    // main one) widens it: (centre + side, centre - side). It comes free
    // with the resonator, which computes it anyway.
    float width = bowl.side() * kBowlGain;
    float left = mix + width, right = mix - width;
    reverb.process(left, right);
    limiter.process(left, right);

    buffer[i] = (int16_t)(left * 32000.0f);
    buffer[i + 1] = (int16_t)(right * 32000.0f);
  }
}

void setup() {
  bowl.setFrequency(146.8f); // D3
  bowl.setModes(12);         // 16 modes would push the patch to about 85%
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

  reverb.setTime(0.99f);
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
  Serial.print("%  peak: ");
  Serial.print(AudioEngine::cpuPeak() * 100.0f, 1);
  Serial.println("%");
  delay(1000);
}
