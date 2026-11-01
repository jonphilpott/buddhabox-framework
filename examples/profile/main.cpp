/**
 * @file main.cpp
 * @brief CPU profiler: measures each module's cost per sample on the
 *        Black Pill and prints it over USB serial.
 *
 * WHY THIS EXISTS: AudioEngine::cpuLoad() measures a running patch, but
 * a patch over 100% starves loop() (the audio interrupt runs back to
 * back), so it can never print the figure that would explain why. This
 * program never starts the audio engine. loop() runs each module for
 * kSamples samples between two reads of the cycle counter, so the
 * figures arrive however heavy the modules are.
 *
 * The modules are set up as in the singing_bowls example, so the total
 * at the bottom predicts that patch's load.
 *
 * Usage: pio run -e profile -t upload, then pio device monitor. The
 * table repeats every few seconds.
 *
 * Budget: 84 MHz / 44.1 kHz = 1905 cycles per sample for everything,
 * of which roughly 80% (about 1500) is safe to spend on audio.
 */

#include <Arduino.h>

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

ModalResonator<> bowl8, bowl12, bowl16, bowl24;
Dust striker;
Particle droplets;
StringSynthOscillator drone;
Svf droneFilter;
Lorenz chaos;
Hilbert hilbert;
SineOscillator shifter;
Reverb<> reverb;
Limiter limiter;

// Samples per measurement: long enough to include many of the
// resonator's control-rate filter updates (one every 22 samples).
const int kSamples = 4096;

// Results go here so the compiler can't remove the work as unused:
// `volatile` means "something outside the program reads this".
volatile float sink;

/** Cycles available per sample: CPU clock / sample rate. */
float budget() { return (float)SystemCoreClock / SAMPLE_RATE; }

/**
 * Print one row: average cycles per sample and percent of the budget.
 * @param name    Module label
 * @param cycles  Total cycles for kSamples samples
 */
void report(const char *name, uint32_t cycles) {
  float perSample = (float)cycles / kSamples;
  Serial.print(name);
  Serial.print(": ");
  Serial.print(perSample, 0);
  Serial.print(" cycles/sample, ");
  Serial.print(100.0f * perSample / budget(), 1);
  Serial.println("%");
}

/**
 * Time some code run kSamples times, print the row, and add the cycles
 * to `sum`. A macro rather than a function so the code is pasted inline
 * and timed exactly as a patch would run it. `...` and __VA_ARGS__ let
 * the code contain commas, which would otherwise split it into several
 * macro arguments. DWT->CYCCNT is the Cortex-M4's cycle counter.
 */
#define PROFILE(sum, name, ...)                                                \
  do {                                                                         \
    float acc = 0.0f;                                                          \
    uint32_t start = DWT->CYCCNT;                                              \
    for (int n = 0; n < kSamples; ++n) {                                       \
      __VA_ARGS__;                                                             \
    }                                                                          \
    uint32_t cycles = DWT->CYCCNT - start;                                     \
    sink = acc;                                                                \
    report(name, cycles);                                                      \
    sum += cycles;                                                             \
  } while (0)

void setup() {
  // Enable the cycle counter (AudioEngine::begin() normally does this).
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // The same settings as singing_bowls.
  bowl8.setModes(8);
  bowl12.setModes(12);
  bowl16.setModes(16);
  bowl24.setModes(24);
  ModalResonator<> *bowls[] = {&bowl8, &bowl12, &bowl16, &bowl24};
  for (ModalResonator<> *b : bowls) {
    b->setFrequency(146.8f);
    b->setDamping(0.85f);
    b->setPosition(0.3f);
  }
  striker.setDensity(0.08f);
  droplets.setFrequency(1174.7f);
  droplets.setDensity(0.6f);
  droplets.setSpread(7.0f);
  droplets.setQ(300.0f);
  drone.setFrequency(73.4f);
  drone.setRegistration(0.25f);
  droneFilter.setCoefficients(500.0f / SAMPLE_RATE, 0.9f, TanApprox::Fast);
  chaos.setRate(0.05f);
  reverb.setTime(0.9f);
  reverb.setAmount(0.45f);
  reverb.setLp(0.6f);

  Serial.begin(115200);
}

void loop() {
  uint32_t total = 0;
  float in = 0.1f;

  Serial.print("\nCPU clock: ");
  Serial.print(SystemCoreClock / 1000000);
  Serial.print(" MHz, budget ");
  Serial.print(budget(), 0);
  Serial.println(" cycles/sample");

  // The resonator at four sizes; only the 12-mode one is in
  // singing_bowls, so the others add to `other`, not `total`.
  uint32_t other = 0;
  PROFILE(other, "ModalResonator, 8 modes ", acc += bowl8.process(in));
  PROFILE(total, "ModalResonator, 12 modes", acc += bowl12.process(in));
  PROFILE(other, "ModalResonator, 16 modes", acc += bowl16.process(in));
  PROFILE(other, "ModalResonator, 24 modes", acc += bowl24.process(in));

  PROFILE(total, "Dust                    ", acc += striker.process());
  PROFILE(total, "Particle                ", acc += droplets.process());
  PROFILE(total, "StringSynthOscillator   ", acc += drone.process());
  PROFILE(total, "Svf                     ", acc += droneFilter.process(in).lp);
  PROFILE(total, "Lorenz                  ", acc += chaos.process());
  // Not in singing_bowls any more, but still worth knowing the cost.
  PROFILE(other, "Hilbert                 ", acc += hilbert.process(in));
  PROFILE(other, "SineOscillator (quad)   ", {
    float s, c;
    shifter.next(0.3f / SAMPLE_RATE, 1.0f, s, c);
    acc += s + c;
  });
  PROFILE(total, "Reverb<16384>           ", {
    float l = in, r = in;
    reverb.process(l, r);
    acc += l + r;
  });
  PROFILE(total, "Limiter                 ", {
    float l = in, r = in;
    limiter.process(l, r);
    acc += l + r;
  });

  report("singing_bowls total     ", total);
  delay(3000);
}
