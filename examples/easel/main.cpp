/**
 * @file main.cpp
 * @brief A Buchla Music Easel-style patch: a slowly evolving, folded and
 *        FM'd tone that swells from note to note, into a spring tank.
 *
 * The Easel's sound source (the 208) is a small, pre-wired West Coast
 * synth. This patch rebuilds its signal flow from BuddhaBox modules and
 * plays on its own. Nothing is struck: the sound never stops, it swells
 * into each new note and changes colour slowly, because an imaginary hand
 * is easing the timbre and modulation sliders and the two oscillators
 * slide in and out of tune with each other.
 *
 *   Pulser ─┬─► SequentialVoltage ─► pitch ─┬─► Modulation osc ──┐
 *           │                               │      │ FM          │
 *           │                               └─► Complex osc      │
 *           └─► Envelope ─► FM depth, timbre ─►    (2x, folded)  │
 *                  │                                  │          │
 *                  └──────────────► level ─► LPG A ◄──┘          │
 *                         hand ─► level ─► LPG B ◄───────────────┘
 *                                           │
 *                              SpringReverb ─► Limiter ─► DAC
 *
 * 1. Pulser: a new note about every 7 s, a little early or late each
 *    time. Each pulse steps the sequencer and restarts the envelope.
 * 2. Sequential voltage source: five pitches in D minor. Now and then
 *    the "hand" moves one step's slider to a new note.
 * 3. Envelope: 5 s up, 12 s down, squared so each swell starts gently
 *    (see the audio callback). Notes overlap, so it never reaches
 *    silence; it swells with each note and sags between them.
 * 4. Modulation oscillator: a triangle a fifth above the pitch, but
 *    never exactly. Its ratio wanders a few hundredths of a percent, so
 *    the FM sidebands beat and drift: the slow churning in the tone.
 *    It is also the second voice, B.
 * 5. Complex oscillator: a sine, frequency-modulated, then wave-folded.
 *    The hand sweeps the folding and FM depth across most of their
 *    range over tens of seconds, and the envelope adds more of both as
 *    each note swells. Runs at twice the sample rate (see OVERSAMPLING).
 * 6. Low-pass gates, both in level mode: A follows the envelope, never
 *    fully closing. B, the drone a fifth up, follows the hand. A gate
 *    darkens as it closes, so the swells change colour as well as level.
 * 7. A long spring, then a limiter.
 * 8. Drift: a slow random wobble of a few cents on the pitch, as an
 *    analogue oscillator wanders.
 *
 * OVERSAMPLING: folding and FM create harmonics above 22 kHz, which
 * "fold back" below it as off-key whine (aliasing). Running the complex
 * oscillator at 88.2 kHz and filtering off everything above 18 kHz before
 * keeping every second sample pushes that whine further down. A host
 * test, with the folder at 0.9 and FM at depth 0.9, measured the tone
 * against the aliasing:
 * - 147 to 294 Hz: about 95 dB either way; no audible aliasing.
 * - 587 Hz: 31 dB without, 45 dB with.
 * - 1175 Hz, folded without FM: 11 dB without, 27 dB with.
 * The sequence here sits low, so it mostly matters when you play higher.
 * It costs about 130 cycles.
 *
 * BONGO: for the Easel's quick, struck sound, set the pulser to 0.36 s
 * and the envelope to 0.003 s up and 0.6 s down; call lpgA.trigger() in
 * onPulse() and use lpgA.process(voiceA) with no level; raise the timbre
 * and FM to about 0.45 and 0.35.
 *
 * Cost: 74% of the F411's CPU, measured; the spring is about a quarter.
 * Generative as it is; set kKnobs to true and wire four pots to play it.
 *
 * Wiring:
 *   Audio  PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *          VIN -> 5V, GND -> GND (5 V, not 3V3)
 *   Knobs  (kKnobs only) 10k linear pots, outer legs to 3V3 and GND,
 *          wipers to PA0 timbre, PA1 FM depth, PA2 pulser rate,
 *          PA3 swell length
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/dynamics/vactrol_lpg.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/spring_reverb.h"
#include "buddhabox/fx/table_wavefolder.h"
#include "buddhabox/io/analog_input.h"
#include "buddhabox/mod/envelope.h"
#include "buddhabox/mod/pulser.h"
#include "buddhabox/mod/sequential_voltage.h"
#include "buddhabox/noise/smooth_random.h"
#include "buddhabox/osc/basic.h"
#include "buddhabox/osc/sine.h"

using namespace bb;

// ── Front panel ──────────────────────────────────────────────────────

// true: read the four pots wired as above. false: play on its own. Leave
// it false with no pots wired, or the floating pins read noise.
constexpr bool kKnobs = false;

constexpr float kRoot = 146.83f;     // D3
constexpr float kPulsePeriod = 7.0f; // seconds between notes, on average
constexpr float kModRatio = 1.5f;    // modulation osc: about a fifth up

// How far the modulation ratio wanders from kModRatio: 0.003 is 0.3%,
// about 5 cents. Enough for the FM sidebands to beat slowly against the
// harmonics; much more and the tone turns clangorous.
constexpr float kRatioWander = 0.003f;

// Notes the sequencer starts with, and the notes the "hand" picks from
// when it moves a slider: D minor pentatonic, in semitones above D3.
constexpr float kStartSteps[5] = {0.0f, 7.0f, 3.0f, 10.0f, 5.0f};
constexpr float kScale[] = {0.0f,  3.0f,  5.0f,  7.0f,  10.0f,
                            12.0f, 15.0f, 17.0f, 19.0f, 22.0f};
constexpr int kScaleSize = sizeof(kScale) / sizeof(kScale[0]);

// ── Modules ──────────────────────────────────────────────────────────

Pulser pulser;
SequentialVoltage sequencer;
Envelope envelope;
BasicOscillator modOsc;
SineOscillator complexOsc; // runs at 2x: see setup()
TableWavefolder folder;
Svf antiAlias1, antiAlias2; // 18 kHz low-pass at 2x, before decimating
VactrolLpg lpgA, lpgB;
SpringReverb<> spring; // 16 KB
Limiter limiter;
SmoothRandom drift, handTimbre, handFm, handDrone, wander, breath;
LFSR hand(2024); // decides when and how the hand moves a slider

AnalogInput knobTimbre(PA0), knobFm(PA1), knobRate(PA2), knobSwell(PA3);

// Knob readings, written by loop() and read by the audio interrupt.
volatile float potTimbre = 0.5f, potFm = 0.4f, potRate = 0.5f, potSwell = 0.5f;

// Control-rate state, set once per block or once per pulse.
float pitchHz = kRoot; // from the sequencer
float driftRatio = 1.0f, modRatio = kModRatio;
float timbre = 0.4f, fmDepth = 0.3f, droneLevel = 0.3f;

/**
 * Control rate, once per block: read the panel (pots or the imaginary
 * hand), and move the drift and the ratio.
 */
void updateControls() {
  // 1. The sliders: the pots, or slow random movements, as if someone
  //    were easing them back and forth. Wide ranges, so the tone travels
  //    from soft and round to bright and buzzy and back.
  if (kKnobs) {
    timbre = potTimbre;
    fmDepth = potFm;
    // Pulser: 15 s to 1.5 s, on a log scale so the knob feels even.
    pulser.setPeriod(15.0f * fastExp2(-3.32f * potRate));
    // Swell: release from 4 to 24 s (about 2 to 12 s once squared;
    // expf inside: fine once a block).
    envelope.setRelease(4.0f + 20.0f * potSwell);
  } else {
    timbre = 0.4f + 0.35f * handTimbre.process();
    fmDepth = 0.3f + 0.25f * handFm.process();
    // Each gap up to 30% shorter or longer than the average.
    pulser.setPeriod(kPulsePeriod * (1.0f + 0.3f * breath.process()));
  }
  droneLevel = 0.3f + 0.2f * handDrone.process();

  // 2. Drift: up to 6 cents either way. The ratio wanders separately,
  //    so the two oscillators drift against each other.
  driftRatio = semitonesToRatio(0.06f * drift.process());
  modRatio = kModRatio * (1.0f + kRatioWander * wander.process());
}

/**
 * One pulse: step the sequence and start a swell. Sometimes the hand
 * first moves one slider to a new note.
 */
void onPulse() {
  if (sequencer.step() == SequentialVoltage::kMaxSteps - 1 &&
      hand.nextFloat() < 0.5f) {
    int step = (int)hand.nextRange(0, SequentialVoltage::kMaxSteps - 1);
    sequencer.setStep(step, kScale[hand.nextRange(0, kScaleSize - 1)]);
  }
  pitchHz = kRoot * semitonesToRatio(sequencer.advance());
  // trigger() rises from wherever the envelope is now, so overlapping
  // swells join up without a dip.
  envelope.trigger();
}

/**
 * One sample of the complex oscillator at twice the sample rate: two
 * folded sines, low-passed at 18 kHz, keeping only the second. The
 * low-pass is what makes dropping every other sample safe: without it,
 * everything between 22 and 44 kHz would fold back down.
 *
 * @param frequency  Cycles per sample at the 2x rate (Hz / 88200)
 */
float complexOscillator(float frequency) {
  float out = 0.0f;
  for (int k = 0; k < 2; k++) {
    float folded = folder.process(complexOsc.next(frequency));
    out = antiAlias2.process(antiAlias1.process(folded).lp).lp;
  }
  return out;
}

void audioCallback(int16_t *buffer, uint16_t length) {
  updateControls();
  const float oversampledTime = 1.0f / (2.0f * SAMPLE_RATE);

  for (uint16_t i = 0; i < length; i += 2) {
    // 1. Timing.
    if (pulser.process())
      onPulse();
    // Squared: the envelope's exponential attack is front-loaded (it
    // covers 63% of the rise in the first fifth of the attack time),
    // which feels abrupt however long the attack is. Squaring makes it
    // ease in instead: now halfway takes a quarter of the attack time,
    // 1.2 s, where before it took a seventh, 0.4 s. It also makes the
    // fall twice as steep in dB, hence the long release in setup().
    float env = envelope.process();
    env *= env;

    // 2. Modulation oscillator, about a fifth above the pitch.
    float hz = pitchHz * driftRatio;
    modOsc.setFrequency(hz * modRatio);
    float mod = modOsc.process();

    // 3. Complex oscillator. FM: the pitch swings by up to 90% either
    //    way (kept under 100%, so it never runs backwards). The envelope
    //    adds FM and folding as each note swells.
    float depth = minf(fmDepth * (0.5f + 0.5f * env), 0.9f);
    folder.setAmount(timbre + 0.25f * env);
    float voiceA =
        complexOscillator(hz * (1.0f + depth * mod) * oversampledTime);

    // 4. Low-pass gates in level mode: A swells with the envelope but
    //    never quite closes; B, the drone, follows the hand.
    float a = lpgA.process(voiceA, 0.25f + 0.75f * env);
    float b = lpgB.process(mod, droneLevel);

    // 5. Mix, A a little left and B a little right, then the spring.
    float left = 0.6f * a + 0.25f * b;
    float right = 0.45f * a + 0.35f * b;
    spring.process(left, right);
    limiter.process(left, right);
    buffer[i] = (int16_t)(clampf(left, -1.0f, 1.0f) * 32000.0f);
    buffer[i + 1] = (int16_t)(clampf(right, -1.0f, 1.0f) * 32000.0f);
  }
}

void setup() {
  // Timing and sequence.
  pulser.setPeriod(kPulsePeriod);
  for (int n = 0; n < SequentialVoltage::kMaxSteps; n++)
    sequencer.setStep(n, kStartSteps[n]);
  envelope.setAttack(5.0f);
  envelope.setRelease(12.0f); // squared, it sounds like about 6 s

  // Oscillators. The complex oscillator and its anti-alias filters run
  // at 88.2 kHz. Two 2-pole low-passes with these Qs make a 4-pole
  // Butterworth: flat up to the cutoff, then falling 24 dB per octave.
  modOsc.setShape(BasicOscillator::TRIANGLE);
  complexOsc.init(2.0f * SAMPLE_RATE);
  antiAlias1.init(2.0f * SAMPLE_RATE);
  antiAlias2.init(2.0f * SAMPLE_RATE);
  antiAlias1.setFrequency(18000.0f, 0.5412f);
  antiAlias2.setFrequency(18000.0f, 1.3066f);

  // Gates: A between VCA and filter, B darker.
  lpgA.setColour(0.5f);
  lpgB.setColour(0.25f);

  // A long spring with a moderate chirp.
  spring.setTime(0.8f);
  spring.setTone(0.4f);
  spring.setChirp(0.5f);
  spring.setAmount(0.35f);

  // The hand, drift and wander are read once per block, so they run at
  // the block rate (172 Hz), with different seeds so they move
  // separately.
  const float blockRate = (float)SAMPLE_RATE / AUDIO_BLOCK_SIZE;
  SmoothRandom *slow[] = {&drift,     &handTimbre, &handFm,
                          &handDrone, &wander,     &breath};
  for (int n = 0; n < 6; n++) {
    slow[n]->init(blockRate);
    slow[n]->seed(n + 1);
  }
  drift.setFrequency(0.15f);      // a new pitch error every 7 s or so
  handTimbre.setFrequency(0.05f); // timbre slider: every 20 s
  handFm.setFrequency(0.033f);    // FM slider: every 30 s
  handDrone.setFrequency(0.025f); // drone level: every 40 s
  wander.setFrequency(0.04f);     // FM ratio: every 25 s
  breath.setFrequency(0.15f);     // pulse timing: every 7 s

  Serial.begin(115200); // USB serial, for the CPU meter below
  AudioEngine::begin();
}

void loop() {
  if (kKnobs) {
    knobTimbre.update();
    knobFm.update();
    knobRate.update();
    knobSwell.update();
    potTimbre = knobTimbre.read();
    potFm = knobFm.read();
    potRate = knobRate.read();
    potSwell = knobSwell.read();
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU: ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.println("%");
  }
}
