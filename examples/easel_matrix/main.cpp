/**
 * @file main.cpp
 * @brief A patchable West Coast voice: noisebox's modules wired by a
 *        modulation matrix, with presets made in a web page.
 *
 * The Buchla Music Easel is a fixed set of modules (pulser, envelope,
 * sequencer, random voltage, two oscillators, low-pass gates) with
 * patch points between them. This patch is the same shape: the modules
 * always run, and a preset says which outputs are patched to which
 * sliders, plus where every slider sits and which slider each pot
 * moves. Presets switch from the menu, with no reflash.
 *
 * MAKING PRESETS: open docs/easel.html, draw patch cables on the panel,
 * save presets.json into this folder, then
 *   python3 tools/gen_presets.py examples/easel_matrix/presets.json
 * which writes presets.h (never edit that by hand).
 *
 * THE MATRIX: a cable joins a source to a slider and adds
 * amount x source to the slider's position (0 to 1, as on the menu).
 *   Sources (each about 0 to 1, or -1 to 1):
 *   - PULSER  a falling ramp, 1 at each pulse down to 0
 *   - ENV     the envelope, struck on every pulse: 0 to 1
 *   - SEQ     the sequencer's note in the Colour's scale, 0 to 1 over
 *             five octaves. Patched to Pitch at amount 1.0 it plays in
 *             tune; other amounts stretch it out of the scale
 *   - RANDOM  a new random value on every pulse, -1 to 1
 *   - MOD     the modulation oscillator, -1 to 1
 *   - CHAOS   a Lorenz attractor, about -1 to 1
 *   Nothing is patched unless a preset patches it: the sequencer only
 *   plays a tune when SEQ goes to Pitch, as on the Easel.
 *
 * TWO SPEEDS: summing every cable for every sample would cost too much,
 * so the matrix runs at the control rate, every 64 samples (689 times
 * a second), which is plenty for envelopes and wobbles. (Every 32 cost
 * another 2% of the CPU.) The exceptions
 * are MOD into FM and MOD into Timbre: those run every sample, because
 * at audio rate they change the tone itself. MOD into anything else is
 * sampled at the control rate, so a fast mod osc there turns into a
 * random-ish stepped value.
 *
 * THE VOICE (as examples/noisebox, plus a second gate):
 *   Mod osc ─── FM ──► Complex osc (sine + noise, folded, 2x)
 *      │                         │
 *      ▼                         ▼
 *   Gate B (level/env)     Gate A (level/ping)
 *      └────────────┬────────────┘
 *                   ▼
 *     crush ─► echo ─► spring (half rate) ─► volume ─► out
 *
 * PRESETS AND POTS: loading a preset sets every item except Volume and
 * the pot assignments. A pot then takes over its item only once it
 * moves, jumping the item to the pot's position, so a preset sounds as
 * saved until you touch something.
 *
 * Wiring: as examples/noisebox (OLED, encoder, 8 pots on PA1-PA7, PB0).
 * Set kKnobs = false without the pots.
 *
 * Cost: see docs/index.html (measured on the board).
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/midi_utils.h"
#include "buddhabox/core/random.h"
#include "buddhabox/dynamics/low_pass_gate.h"
#include "buddhabox/dynamics/vactrol_lpg.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/decimator.h"
#include "buddhabox/fx/delay.h"
#include "buddhabox/fx/spring_reverb.h"
#include "buddhabox/fx/table_wavefolder.h"
#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/encoder_input.h"
#include "buddhabox/io/menu.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/io/switch_input.h"
#include "buddhabox/mod/colour.h"
#include "buddhabox/mod/envelope.h"
#include "buddhabox/mod/lorenz.h"
#include "buddhabox/mod/pulser.h"
#include "buddhabox/noise/clocked_noise.h"
#include "buddhabox/osc/basic.h"
#include "buddhabox/osc/sine.h"

using namespace bb;

// ── Settings ─────────────────────────────────────────────────────────
const bool kKnobs = true;     // pots on PA1-PA7 and PB0
const int kMaxSteps = 8;      // the longest sequence
const int kControlEvery = 64; // samples between matrix updates
const float kControlRate = (float)SAMPLE_RATE / kControlEvery;

// ── Sources and menu items ───────────────────────────────────────────
// tools/gen_presets.py reads both enums from this file to check a
// preset's names, so keep each on its own lines as they are.
enum Source {
  SRC_PULSER,
  SRC_ENV,
  SRC_SEQ,
  SRC_RANDOM,
  SRC_MOD,
  SRC_CHAOS,
  kSources
};

// Every item a cable can reach is a destination: a cable moves the
// item's position. Switches (Mod Wave and below) can be set by a preset
// but not patched.
enum Item {
  PRESET,
  VOLUME,
  PITCH,
  COLOUR,
  NOISE,
  TIMBRE,
  FM,
  MOD_RATE,
  RATE,
  ATTACK,
  DECAY,
  MUTATE,
  SPACE,
  LEVEL_A,
  LEVEL_B,
  CHAOS_RATE,
  CRUSH,
  ECHO_TIME,
  ECHO_MIX,
  FEEDBACK,
  MOD_WAVE,
  MOD_TRACK,
  FOLD,
  GATE_A,
  GATE_B,
  STEPS,
  RANDOMIZE, // an action, not a value
  kItems
};

// ── Presets ──────────────────────────────────────────────────────────
const int kKnobCount = 8;
const int kMaxCables = 16;

/** One patch cable: amount x source added to an item's position. */
struct Cable {
  uint8_t from; // a Source
  uint8_t to;   // an Item
  float amount; // -1 to 1
};

/** One saved setting: an item's menu value. */
struct Setting {
  uint8_t item;
  int16_t value;
};

/** A preset, as tools/gen_presets.py writes it into presets.h. */
struct Preset {
  const char *name;
  Setting settings[kItems];
  uint8_t settingCount;
  Cable cables[kMaxCables];
  uint8_t cableCount;
  uint8_t knobs[kKnobCount]; // the item each pot moves
};

#include "presets.h" // kPresets, kPresetCount, kPresetNames

// ── The menu ─────────────────────────────────────────────────────────
// Pitch is a MIDI note, C1 to C6, shown by name (setup() writes them).
const int kLowNote = NOTE_C1, kHighNote = NOTE_C6;
const int kNotes = kHighNote - kLowNote + 1;
char noteText[kNotes][4];
const char *noteNames[kNotes];

const char *const kModWaves[] = {"Triangle", "Saw", "Square", "Noise"};
const char *const kTracks[] = {"Free", "Ratio"};
const char *const kFolds[] = {"Sine", "Analog"};
const char *const kGates[] = {"Level", "Ping"};
const char *const kGatesB[] = {"Level", "Env"};

MenuItem items[kItems] = {
    {"Preset", 0, 0, kPresetCount - 1, 1, kPresetNames},
    {"Volume", 70, 0, 100, 2},
    {"Pitch", NOTE_C3, kLowNote, kHighNote, 1, noteNames},
    {"Colour", 6, 0, 15, 1},
    {"Noise", 0, 0, 100, 2},
    {"Timbre", 30, 0, 100, 2},
    {"FM", 20, 0, 100, 2},
    {"Mod Rate", 50, 0, 100, 1},
    {"Rate", 50, 0, 100, 2},
    {"Attack", 0, 0, 100, 2},
    {"Decay", 40, 0, 100, 2},
    {"Mutate", 20, 0, 100, 2},
    {"Space", 30, 0, 100, 2},
    {"Level A", 70, 0, 100, 2},
    {"Level B", 0, 0, 100, 2},
    {"Chaos Rate", 40, 0, 100, 2},
    {"Crush", 0, 0, 100, 2},
    {"Echo Time", 25, 1, 37, 1, nullptr, "0ms"}, // tens of ms
    {"Echo Mix", 0, 0, 100, 2},
    {"Feedback", 40, 0, 95, 2},
    {"Mod Wave", 0, 0, 3, 1, kModWaves},
    {"Mod Track", 1, 0, 1, 1, kTracks},
    {"Fold", 0, 0, 1, 1, kFolds},
    {"Gate A", 1, 0, 1, 1, kGates},
    {"Gate B", 0, 0, 1, 1, kGatesB},
    {"Steps", 5, 1, kMaxSteps, 1},
    {"Randomize"},
};
Menu menu(items, kItems);
int defaults[kItems]; // each item's starting value, for loadPreset()

// Where a pot left an item, 0 to 1: finer than the menu's whole-number
// steps. -1 when the menu or a preset set it last.
float fine[kItems];

/** @return  Item `i`'s position, 0 to 1: the pot's, or the menu's. */
float position(int i) {
  if (fine[i] >= 0.0f)
    return fine[i];
  const MenuItem &m = items[i];
  return (float)(m.value - m.min) / (float)(m.max - m.min);
}

// The patch in use. loadPreset() (in loop()) copies a preset's cables
// here; the audio interrupt reads them. cableCount is zeroed first and
// set last, so the interrupt never reads a half-copied cable.
Cable cables[kMaxCables];
volatile int cableCount = 0;

// ── The knobs ────────────────────────────────────────────────────────
AnalogInput knobs[kKnobCount] = {
    AnalogInput(PA1, 0.7f), AnalogInput(PA2, 0.7f), AnalogInput(PA3, 0.7f),
    AnalogInput(PA4, 0.7f), AnalogInput(PA5, 0.7f), AnalogInput(PA6, 0.7f),
    AnalogInput(PA7, 0.7f), AnalogInput(PB0, 0.7f)};
uint8_t knobItems[kKnobCount]; // which item each pot moves: the preset's
float knobLast[kKnobCount];    // each pot's last reading

// ── Modules ──────────────────────────────────────────────────────────
Pulser pulser;
Envelope envelope;
BasicOscillator modOsc;
ClockedNoise modNoise;     // the mod osc's Noise wave
SineOscillator complexOsc; // runs at 2x: see setup()
TableWavefolder folder;
Svf antiAlias; // 18 kHz low-pass at 2x, before decimating
Random white;
Lorenz chaos;    // runs at the control rate
VactrolLpg lpgA; // Gate A: the pinged vactrol, for the bongo
// Gate B: the simpler one-pole gate. A second VactrolLpg cost 6% of the
// CPU, which put the patch over 80%; this one opens on a level, or
// follows the envelope.
LowPassGate lpgB;
Decimator crush;
DelayLine<16384> echo; // 0.37 s, 32 KB
// Half rate, as in examples/noisebox (crunchier than full rate). 8 KB.
SpringReverb<1024, 2> spring;
Colour colour;
LFSR rng(208);

EncoderInput encoder;
SwitchInput push(PB5);
Oled oled;
bool haveScreen = false;

// ── State for the audio interrupt ────────────────────────────────────
// Written by control() every 64 samples, read every sample.
float baseHz = 130.8f, noiseAmount = 0.0f, timbre = 0.3f, fmDepth = 0.2f;
float modRatio = 1.0f, modFreeHz = 1.0f;
float mutate = 0.2f, space = 0.3f, levelA = 0.7f, levelB = 0.0f;
float echoSamples = 11025.0f, echoMix = 0.0f, echoFeedback = 0.4f;
float volume = 15000.0f;
float fmAudio = 0.0f, timbreAudio = 0.0f; // MOD cables at audio rate
int modWave = 0, steps = 5;
bool modTrack = true, analogFold = false, crushOn = false;
bool pingA = true, envB = false;

// Set on each pulse, read as sources.
float seqValue = 0.0f, randomValue = 0.0f;
float lastMod = 0.0f; // the mod osc's latest sample, as a source

// The sequence: scale indices, 0-4 the first octave, 5-9 the second.
uint8_t sequence[kMaxSteps];
volatile int step = 0;
volatile bool pulsed = false; // for the display
int controlCounter = 0;

// The echo and half-rate spring's state.
float echoTone = 0.0f;
float springPrevL = 0.0f, springPrevR = 0.0f;
float springNowL = 0.0f, springNowR = 0.0f;
float springLastIn = 0.0f;
bool oddSample = false;

/** Roll a new sequence over two octaves of the scale. */
void randomize() {
  for (int i = 0; i < kMaxSteps; i++)
    sequence[i] = (uint8_t)rng.nextRange(0, 9);
}

/**
 * Load preset `n` (from loop()): every item back to its default, then
 * the preset's settings, cables and pot assignments. Volume stays.
 */
void loadPreset(int n) {
  const Preset &p = kPresets[n];
  // 1. Settings. The interrupt may read a mix of old and new values for
  //    one control tick: harmless, and over in under a millisecond.
  for (int i = 0; i < kItems; i++) {
    if (i == PRESET || i == VOLUME)
      continue;
    items[i].value = defaults[i];
    fine[i] = -1.0f;
  }
  for (int s = 0; s < p.settingCount; s++) {
    MenuItem &m = items[p.settings[s].item];
    int v = p.settings[s].value;
    m.value = v < m.min ? m.min : (v > m.max ? m.max : v);
  }

  // 2. Cables: count to zero, copy, then set the count.
  cableCount = 0;
  for (int c = 0; c < p.cableCount; c++)
    cables[c] = p.cables[c];
  cableCount = p.cableCount;

  // 3. Pots. Remember where each one is now, so it takes over only
  //    when it moves.
  for (int k = 0; k < kKnobCount; k++) {
    knobItems[k] = p.knobs[k];
    knobLast[k] = knobs[k].read();
  }
}

/** @return  x clamped to 0..1 */
float unit(float x) { return clampf(x, 0.0f, 1.0f); }

/**
 * The matrix and the settings (audio interrupt, every 64 samples):
 * read the sources, add up the cables, and turn every item's position
 * into the voice's settings. Uses library maths, at the control rate.
 */
void control() {
  // 1. Sources.
  chaos.process();
  float source[kSources];
  source[SRC_PULSER] = pulser.value();
  source[SRC_ENV] = envelope.getValue();
  source[SRC_SEQ] = seqValue;
  source[SRC_RANDOM] = randomValue;
  source[SRC_MOD] = lastMod;
  source[SRC_CHAOS] = chaos.x();

  // 2. Cables. MOD into FM or Timbre is kept apart, for the per-sample
  //    path; everything else adds up here.
  float mod[kItems] = {0.0f};
  float fmA = 0.0f, timbreA = 0.0f;
  int count = cableCount;
  for (int c = 0; c < count; c++) {
    const Cable &k = cables[c];
    if (k.from == SRC_MOD && k.to == FM)
      fmA += k.amount;
    else if (k.from == SRC_MOD && k.to == TIMBRE)
      timbreA += k.amount;
    else
      mod[k.to] += k.amount * source[k.from];
  }
  fmAudio = fmA;
  timbreAudio = timbreA;
  auto p = [&](int i) { return position(i) + mod[i]; };

  // 3. Pitch: five octaves over the item's range, so SEQ (also scaled
  //    over five octaves) lands on its notes at amount 1.0.
  baseHz = 32.703f * semitonesToRatio(60.0f * clampf(p(PITCH), -0.2f, 1.2f));
  // Colour: 16 scales, dark to bright. Patchable like any slider: a
  // cable can sweep the scale while the sequence plays.
  int c16 = (int)(15.0f * unit(p(COLOUR)) + 0.5f);
  colour.setColour(COLOUR_BY_BRIGHTNESS[c16]);

  // 4. The voice. Exponential where the ear hears ratios (rates, times),
  //    linear where it hears amounts.
  noiseAmount = unit(p(NOISE));
  timbre = p(TIMBRE); // the folder clamps it
  fmDepth = 0.95f * unit(p(FM));
  float modRate = unit(p(MOD_RATE));
  modRatio = 0.25f * fastExp2(5.0f * modRate);   // 0.25x - 8x the pitch
  modFreeHz = 0.05f * fastExp2(15.3f * modRate); // 0.05 Hz - 2 kHz
  pulser.setPeriod(4.0f * fastExp2(-7.06f * unit(p(RATE)))); // 4 s - 30 ms
  float a = unit(p(ATTACK)), d = unit(p(DECAY));
  // The envelope's setters use expf(), about 100 cycles each: run them
  // only when the value moves. Squared, so short times get more of the
  // range.
  static float lastA = -1.0f, lastD = -1.0f;
  if (a != lastA) {
    lastA = a;
    envelope.setAttack(0.002f + 4.0f * a * a);
  }
  if (d != lastD) {
    lastD = d;
    envelope.setRelease(0.02f + 6.0f * d * d);
  }
  lpgA.setDecay(d);
  mutate = unit(p(MUTATE));
  space = unit(p(SPACE));
  levelA = unit(p(LEVEL_A));
  levelB = unit(p(LEVEL_B));
  // Lorenz at the control rate: each step is 45x the size it would
  // be in Streams (31 kHz), so the rate tops out at 0.66, where the
  // step is Streams' largest (0.02). As fast as the full range at the
  // audio rate; any higher and it would step too far and blow up.
  chaos.setRate(0.2f + 0.46f * unit(p(CHAOS_RATE)));
  float c = unit(p(CRUSH));
  crushOn = c > 0.005f;
  crush.setRate(SAMPLE_RATE * fastExp2(-6.0f * c)); // to about 700 Hz
  echoSamples =
      minf((0.01f + 0.36f * unit(p(ECHO_TIME))) * SAMPLE_RATE, 16382.0f);
  echoMix = unit(p(ECHO_MIX));
  echoFeedback = 0.95f * unit(p(FEEDBACK));
  float v = unit(p(VOLUME));
  volume = 30000.0f * v * v; // squared: closer to how loudness is heard

  // 5. Switches: menu values, not patchable.
  if (items[MOD_WAVE].value != modWave) {
    modWave = items[MOD_WAVE].value;
    const BasicOscillator::Shape shapes[] = {BasicOscillator::TRIANGLE,
                                             BasicOscillator::SAW,
                                             BasicOscillator::SQUARE};
    if (modWave < 3)
      modOsc.setShape(shapes[modWave]);
  }
  modTrack = items[MOD_TRACK].value;
  analogFold = items[FOLD].value;
  pingA = items[GATE_A].value;
  envB = items[GATE_B].value;
  steps = items[STEPS].value;
}

/**
 * One pulse (audio interrupt): step the sequence (maybe rewriting the
 * step first), draw a new random value, strike the envelope and any
 * Gate A in Ping mode.
 */
void onPulse() {
  int s = (step + 1) % steps;
  if (rng.nextFloat() < mutate)
    sequence[s] = (uint8_t)rng.nextRange(0, 9);
  step = s;
  // The Colour's root is 1 Hz, so getNote() is a ratio; in semitones
  // that's 12 log2(ratio), and SEQ spans five octaves (60 semitones).
  seqValue = 12.0f * fastLog2(colour.getNote(0, sequence[s])) / 60.0f;
  randomValue = rng.nextBipolar();
  envelope.trigger();
  if (pingA)
    lpgA.trigger();
  pulsed = true;
}

/**
 * The complex oscillator at twice the sample rate (as examples/easel):
 * two folded samples, low-passed at 18 kHz, keeping only the second.
 *
 * WHY 2-POLE, NOT easel's 4-POLE: the second filter cost 4% of the CPU,
 * which this patch couldn't spare. A 2-pole falls off half as steeply,
 * so a little more of the folding's top end comes back down as
 * aliasing on high, bright notes.
 *
 * @param frequency  Cycles per sample at the 2x rate (Hz / 88200)
 * @param noise      Added to the sine before folding
 */
float complexOscillator(float frequency, float noise) {
  float out = 0.0f;
  for (int k = 0; k < 2; k++) {
    float folded = folder.process(complexOsc.next(frequency) + noise);
    if (analogFold)
      folded = folder.analog();
    out = antiAlias.process(folded).lp;
  }
  return out;
}

/** Fill one block of audio (interrupt context: no I2C, no Serial). */
void audioCallback(int16_t *buffer, uint16_t length) {
  const float oversampledTime = 1.0f / (2.0f * SAMPLE_RATE);

  for (uint16_t i = 0; i < length; i += 2) {
    // 1. The matrix, every 64 samples; timing and the envelope, every
    //    sample.
    if (++controlCounter >= kControlEvery) {
      controlCounter = 0;
      control();
    }
    if (pulser.process())
      onPulse();
    envelope.process();

    // 2. Modulation oscillator: following the pitch (Ratio) or on its
    //    own (Free), capped at a quarter of the sample rate.
    float hz = clampf(baseHz, 20.0f, 8000.0f);
    float modHz = modTrack ? hz * modRatio : modFreeHz;
    float mod;
    if (modWave == 3) {
      modNoise.setFrequency(modHz);
      mod = modNoise.process();
    } else {
      mod = modOsc.process(minf(modHz / SAMPLE_RATE, 0.25f), 0.5f);
    }
    lastMod = mod;

    // 3. Complex oscillator. FM swings the pitch by up to 95% either
    //    way; MOD cables into FM and Timbre act here, every sample.
    float depth = clampf(fmDepth * (1.0f + fmAudio * mod), 0.0f, 0.95f);
    folder.setAmount(timbre + timbreAudio * mod);
    float x = complexOscillator(hz * (1.0f + depth * mod) * oversampledTime,
                                noiseAmount * white.bipolar());

    // 4. The gates: A on the complex osc (pinged, or held at Level
    //    A), B on the mod osc (held at Level B, or Level B times the
    //    envelope). B is skipped once it has shut.
    x = pingA ? lpgA.process(x) : lpgA.process(x, levelA);
    float b = envB ? levelB * envelope.getValue() : levelB;
    if (b > 0.0f || lpgB.getControl() > 0.0001f) {
      lpgB.setControl(b);
      x += 0.6f * lpgB.process(mod);
    }

    // 5. Crush (skipped at 0), then the echo with darkening repeats.
    if (crushOn)
      x = crush.process(x);
    float wet = echo.read(echoSamples);
    echoTone += 0.35f * (wet - echoTone);
    echo.write(0.7f * x + echoFeedback * echoTone);
    x += echoMix * wet;

    // 6. The spring at half rate (see examples/noisebox): fed the
    //    average of two inputs on odd samples, its output drawn as a
    //    straight line between the last two values.
    x *= 0.7f;
    if (oddSample) {
      springPrevL = springNowL;
      springPrevR = springNowR;
      springNowL = springNowR = 0.5f * (x + springLastIn);
      spring.process(springNowL, springNowR); // fully wet: setup()
    }
    springLastIn = x;
    float wetL = oddSample ? springNowL : 0.5f * (springPrevL + springNowL);
    float wetR = oddSample ? springNowR : 0.5f * (springPrevR + springNowR);
    oddSample = !oddSample;

    // 7. Dry/wet, a soft limit, then Volume (after the limit, so it
    //    changes only the level).
    float left = x + space * (wetL - x);
    float right = x + space * (wetR - x);
    buffer[i] = (int16_t)(softLimitClip(left) * volume);
    buffer[i + 1] = (int16_t)(softLimitClip(right) * volume);
  }
}

/**
 * Draw one frame: the preset's name and a pulse light in the yellow
 * band, the sequence under it, then the menu.
 */
void drawScreen() {
  // 1. Title bar, dark on lit.
  oled.rect(0, 0, 128, 8, true);
  oled.text(2, 0, kPresetNames[items[PRESET].value], false);
  oled.rect(118, 1, 6, 6, true, !pulsed);
  pulsed = false;

  // 2. The sequence: a bar per step, as tall as its note.
  oled.rect(0, 8, 128, 8, true, false);
  int playing = step;
  for (int i = 0; i < steps; i++) {
    int h = 1 + sequence[i] / 2;
    oled.rect(i * 16 + 2, 14 - h, 12, h, true);
    if (i == playing)
      oled.rect(i * 16 + 2, 15, 12, 1, true);
  }

  // 3. The menu.
  menu.draw(oled, 16, 6);
}

void setup() {
  Serial.begin(115200);

  // 1. The complex oscillator and its anti-alias filter at 88.2 kHz:
  //    Q 0.707 is a 2-pole Butterworth, flat right up to the cutoff.
  complexOsc.init(2.0f * SAMPLE_RATE);
  antiAlias.init(2.0f * SAMPLE_RATE);
  antiAlias.setFrequency(18000.0f, 0.7071f);

  // 2. The rest of the voice.
  chaos.init(kControlRate);
  lpgA.setColour(0.6f);
  lpgB.setSlewTime(0.02f); // a little vactrol lag
  spring.init(SAMPLE_RATE / 2);
  spring.setAmount(1.0f); // wet only: the mix is in the callback
  spring.setTime(0.7f);
  spring.setTone(0.45f);
  colour.setRoot(1.0f); // so getNote() returns a ratio
  modOsc.setShape(BasicOscillator::TRIANGLE);

  // 3. Note names for Pitch: MIDI 24 is C1, so octave = note / 12 - 1.
  static const char *const kNames[12] = {"C",  "C#", "D",  "D#", "E",  "F",
                                         "F#", "G",  "G#", "A",  "A#", "B"};
  for (int n = 0; n < kNotes; n++) {
    int midi = kLowNote + n;
    snprintf(noteText[n], sizeof(noteText[n]), "%s%d", kNames[midi % 12],
             midi / 12 - 1);
    noteNames[n] = noteText[n];
  }

  // 4. The first preset. Each pot is read once first, so loadPreset()
  //    knows where it sits and the preset plays until a pot moves.
  // Every item starts at its menu value (fine = -1). loadPreset()
  // resets the rest, but deliberately not Volume, which would
  // otherwise stay at fine = 0.0: silence.
  for (int i = 0; i < kItems; i++) {
    defaults[i] = items[i].value;
    fine[i] = -1.0f;
  }
  for (int k = 0; k < kKnobCount; k++)
    if (kKnobs)
      knobs[k].update();
  randomize();
  loadPreset(0);
  control();

  encoder.begin();
  push.begin();
  haveScreen = oled.begin();

  AudioEngine::begin();
}

void loop() {
  // 1. Pots: a real move sets the item the preset gave the pot.
  for (int k = 0; kKnobs && k < kKnobCount; k++) {
    knobs[k].update();
    float v = knobs[k].read();
    if (v == knobLast[k])
      continue;
    knobLast[k] = v;
    MenuItem &m = items[knobItems[k]];
    m.value = m.min + (int)(v * (m.max - m.min) + 0.5f);
    fine[knobItems[k]] = v;
  }

  // 2. Menu. control() reads the items itself, so a change needs no
  //    more than this, except a new preset.
  push.update();
  Menu::Event event = menu.update(encoder.delta(), push.triggered());
  if (event == Menu::CHANGED) {
    fine[menu.selected()] = -1.0f;
    if (menu.selected() == PRESET)
      loadPreset(items[PRESET].value);
  } else if (event == Menu::ACTION && menu.selected() == RANDOMIZE) {
    randomize();
  }

  // 3. CPU load over USB serial once a second.
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.println("%");
  }

  // 4. Display: redraw at 20 frames a second, one page per loop().
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    drawScreen();
  }
  if (haveScreen)
    oled.display(1);
}
