/**
 * @file main.cpp
 * @brief A West Coast noise box: a folded, FM'd complex oscillator,
 *        a pulser-driven random sequence and a low-pass gate, played
 *        from 8 knobs and an OLED menu.
 *
 * Where examples/easel plays itself, this one is an instrument: the
 * Buchla Music Easel's layout, with the knobs on the panel and the
 * routing in the menu. It goes from a pinging bongo through a slowly
 * churning drone to a screaming, crushed mess.
 *
 *   Pulser ─┬─► sequence (1-8 steps, Random rewrites them) ─► pitch
 *           └─► LPG ping                                       │
 *                                                              ▼
 *   Mod osc (tri, saw, square or noise) ──FM, timbre──► Complex osc
 *        │                                  (sine + noise, folded, 2x)
 *        └── Mod Mix ──────────────────────────────┐       │
 *                                                  ▼       ▼
 *   Lorenz chaos ─► timbre, FM               Low-pass gate (ping/drone)
 *                                                   │
 *                                 crush ─► echo ─► spring ─► out
 *
 * THE PANEL (one pot each):
 * - Noise: white noise into the folder: grit at low settings, a roar
 *   at high ones.
 * - Timbre: how hard the wavefolder folds: round sine to buzzy.
 * - FM: how far the mod osc swings the complex osc's pitch.
 * - Mod Rate: the mod osc. In Ratio mode, 0.25x to 8x the pitch (a
 *   tone); in Free mode, 0.05 Hz (a slow wobble) to 2 kHz.
 * - Rate: the pulser, 4 s to 30 ms between pulses. At the fast end
 *   the gate pings at audio rate and buzzes.
 * - Decay: how long each ping rings. In Drone mode, the gate's level.
 * - Random: each pulse, the chance of rewriting the step about to play
 *   with a random note. 0 locks the loop, full is all random (a
 *   "Turing machine").
 * - Space: the spring.
 *
 * THE MENU (encoder; turn, press to edit, press to finish) also holds:
 * - Volume: the main level, applied after everything else.
 * - Pitch: chromatic, C1 to C6 (32.7 Hz to 1 kHz), shown as a note
 *   name; the sequence plays on top of it, in the Colour's scale.
 * - Colour: that scale, one of 16 five-note scales from dark to bright.
 * - Mod Wave: Triangle, Saw, Square or Noise (random steps at the mod
 *   rate: Free and fast is static, slow is a random wobble).
 * - Mod Track: Ratio or Free (see Mod Rate).
 * - Mod>Timbre: the mod osc moves the folding too (the Easel's
 *   "timbre modulation").
 * - Mod Mix: the mod osc's own signal, added before the gate.
 * - Fold: Sine (smooth, bell-like) or Analog (rougher, vocal).
 * - Chaos, Chaos Rate: a Lorenz attractor wobbling Timbre and FM,
 *   from a slow drift to an audio-rate snarl.
 * - Crush: sample-rate reduction, 44 kHz down to about 700 Hz.
 * - Gate: Ping (each pulse strikes the gate) or Drone (always open).
 * - Steps: the sequence's length.
 * - Echo Time, Echo Mix, Feedback: a delay with darkening repeats.
 * - Randomize: a new sequence.
 * Turning a pot sets its menu item, so the screen shows where it is; the
 * menu can change it too, until the pot is next moved.
 *
 * OVERSAMPLING: folding and FM make harmonics far above 22 kHz, which
 * fold back down as off-key whine. The complex oscillator runs at
 * 88.2 kHz with an 18 kHz low-pass before every second sample is kept,
 * as in examples/easel, where it bought 14-16 dB on high notes.
 *
 * WHY THE SPRING RUNS AT HALF RATE: at the full rate it measured 31%
 * of the CPU in this patch. Its all-passes reach 2 samples back
 * instead of 4, the same time, so the chirp falls in the same place.
 * But it does not sound the same: it is crunchier. At 22 kHz nothing
 * above 11 kHz fits, and the bright fold and noise above that, only
 * partly removed by the averaging in the callback, folds back down to
 * the wrong pitches inside the spring. Here that suits the patch. For
 * a clean spring, low-pass the input properly first (two Svfs at
 * 9 kHz, as the complex oscillator does) or run it at the full rate.
 *
 * Wiring:
 *   Audio    PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *   OLED     VCC -> 3V3, GND -> GND, SCL -> PB8, SDA -> PB9
 *   Encoder  A -> PB6, B -> PB7, C (middle pin) -> GND
 *   Push     one leg -> PB5, the other -> GND
 *   Knobs    8 pots, outer legs to 3V3 and GND, wipers to:
 *              PA1 Noise   PA2 Timbre   PA3 FM      PA4 Mod Rate
 *              PA5 Rate    PA6 Decay    PA7 Random  PB0 Space
 *            Set kKnobs = false without them, or the unconnected pins'
 *            noise will move the settings.
 *
 * Cost: see docs/index.html (measured on the board). About 50 KB of
 * RAM: 32 KB echo, 8 KB spring.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/midi_utils.h"
#include "buddhabox/core/random.h"
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
#include "buddhabox/mod/lorenz.h"
#include "buddhabox/mod/pulser.h"
#include "buddhabox/noise/clocked_noise.h"
#include "buddhabox/osc/basic.h"
#include "buddhabox/osc/sine.h"

using namespace bb;

// ── Settings ─────────────────────────────────────────────────────────
const bool kKnobs = true; // pots on PA1-PA7 and PB0 (see Wiring)
const int kMaxSteps = 8;  // the longest sequence

// ── The menu ─────────────────────────────────────────────────────────
// Volume, Pitch and Colour first, since they're changed most from the
// menu; then the eight pot items, in panel order; then the rest.
enum Item {
  VOLUME,
  PITCH,
  COLOUR,
  NOISE,
  TIMBRE,
  FM,
  MOD_RATE,
  RATE,
  DECAY,
  RANDOM,
  SPACE,
  MOD_WAVE,
  MOD_TRACK,
  MOD_TIMBRE,
  MOD_MIX,
  FOLD,
  CHAOS,
  CHAOS_RATE,
  CRUSH,
  GATE,
  STEPS,
  ECHO_TIME,
  ECHO_MIX,
  FEEDBACK,
  RANDOMIZE, // an action, not a value
  kItems
};

// Pitch is a MIDI note, C1 to C6, shown by name. setup() writes the
// names ("C1", "C#1" ... "C6") into noteText and points noteNames at
// them, so the menu can show labels[value - min].
const int kLowNote = NOTE_C1, kHighNote = NOTE_C6;
const int kNotes = kHighNote - kLowNote + 1; // 61: five octaves
char noteText[kNotes][4];
const char *noteNames[kNotes];

const char *const kModWaves[] = {"Triangle", "Saw", "Square", "Noise"};
const char *const kTracks[] = {"Free", "Ratio"};
const char *const kFolds[] = {"Sine", "Analog"};
const char *const kGates[] = {"Drone", "Ping"};

MenuItem items[kItems] = {
    {"Volume", 70, 0, 100, 2},
    {"Pitch", NOTE_C3, kLowNote, kHighNote, 1, noteNames},
    {"Colour", 6, 0, 15, 1},
    {"Noise", 0, 0, 10, 2},
    {"Timbre", 30, 0, 100, 2},
    {"FM", 20, 0, 100, 2},
    {"Mod Rate", 50, 0, 100, 1},
    {"Rate", 50, 0, 100, 2},
    {"Decay", 40, 0, 100, 2},
    {"Random", 20, 0, 100, 2},
    {"Space", 30, 0, 100, 2},
    {"Mod Wave", 0, 0, 3, 1, kModWaves},
    {"Mod Track", 1, 0, 1, 1, kTracks},
    {"Mod>Timbre", 0, 0, 100, 2},
    {"Mod Mix", 0, 0, 100, 2},
    {"Fold", 0, 0, 1, 1, kFolds},
    {"Chaos", 0, 0, 100, 2},
    {"Chaos Rate", 40, 0, 100, 2},
    {"Crush", 0, 0, 100, 2},
    {"Gate", 1, 0, 1, 1, kGates},
    {"Steps", 5, 1, kMaxSteps, 1},
    {"Echo Time", 25, 1, 37, 1, nullptr, "0ms"}, // tens of ms
    {"Echo Mix", 0, 0, 100, 2},
    {"Feedback", 40, 0, 95, 2},
    {"Randomize"},
};
Menu menu(items, kItems);

// Where each item really is, 0 to 1, when a pot set it last: finer than
// the menu's whole-number steps, so a slow turn sweeps smoothly. -1 when
// the menu set it last.
float fine[kItems];

/** @return  Item `i`'s position, 0 to 1: the pot's, or the menu's. */
float position(int i) {
  if (fine[i] >= 0.0f)
    return fine[i];
  const MenuItem &m = items[i];
  return (float)(m.value - m.min) / (float)(m.max - m.min);
}

// ── The knobs ────────────────────────────────────────────────────────
// Pot n sets menu item kKnobItems[n]. Light smoothing: the hysteresis
// already stops the jitter.
AnalogInput knobs[] = {AnalogInput(PA1, 0.7f), AnalogInput(PA2, 0.7f),
                       AnalogInput(PA3, 0.7f), AnalogInput(PA4, 0.7f),
                       AnalogInput(PA5, 0.7f), AnalogInput(PA6, 0.7f),
                       AnalogInput(PA7, 0.7f), AnalogInput(PB0, 0.7f)};
const int kKnobCount = sizeof(knobs) / sizeof(knobs[0]);
const Item kKnobItems[kKnobCount] = {NOISE, TIMBRE, FM,       MOD_RATE,
                                     RATE,  DECAY,  ECHO_MIX, SPACE};

// ── Modules ──────────────────────────────────────────────────────────
Pulser pulser;
BasicOscillator modOsc;
ClockedNoise modNoise;     // the mod osc's Noise wave
SineOscillator complexOsc; // runs at 2x: see setup()
TableWavefolder folder;
Svf antiAlias1, antiAlias2; // 18 kHz low-pass at 2x, before decimating
Random white;               // the Noise item
Lorenz chaos;
VactrolLpg lpg;
Decimator crush;
// 16384 samples (0.37 s) of 16-bit storage: 32 KB.
DelayLine<16384> echo;
// The spring runs at half rate (see the file header): 1024 samples
// hold its longer trip there, and a stretch of 2 keeps its chirp in
// place. Crunchier than the full-rate spring: see the header. 8 KB.
SpringReverb<1024, 2> spring;
float springPrevL = 0.0f, springPrevR = 0.0f; // its last two outputs
float springNowL = 0.0f, springNowR = 0.0f;
float springLastIn = 0.0f; // the previous sample's input
bool oddSample = false;
float space = 0.3f; // Space: the spring's dry/wet
Colour colour;
LFSR rng(208); // the Easel's sound source is the 208

EncoderInput encoder;
SwitchInput push(PB5);
Oled oled;
bool haveScreen = false;

// ── Settings for the audio interrupt, from applyMenu() ──────────────
// loop() writes these and the audio interrupt reads them. A float is
// written in one instruction, so the interrupt never sees half of one.
float volume = 0.5f;      // Volume, as a gain
float baseHz = 100.0f;    // Pitch
float timbre = 0.3f;      // Timbre
float fmDepth = 0.2f;     // FM
float modRatio = 1.0f;    // Mod Rate, Ratio mode
float modFreeHz = 1.0f;   // Mod Rate, Free mode
bool modTrack = true;     // Mod Track: true = Ratio
int modWave = 0;          // Mod Wave: 3 = Noise
float modTimbre = 0.0f;   // Mod>Timbre
float modMix = 0.0f;      // Mod Mix
float noiseAmount = 0.0f; // Noise
bool analogFold = false;  // Fold
float chaosAmount = 0.0f; // Chaos
bool crushOn = false;     // Crush above 0
bool ping = true;         // Gate
float droneLevel = 0.4f;  // Decay, in Drone mode
float randomChance = 0.2f;
int steps = 5;
float echoSamples = 11025.0f, echoMix = 0.0f, echoFeedback = 0.4f;

// ── The sequence ─────────────────────────────────────────────────────
// Scale indices into the Colour: 0-4 the first octave, 5-9 the second.
// The audio interrupt rewrites them (Random); loop() rolls them all
// (Randomize). Single bytes: each write is atomic.
uint8_t sequence[kMaxSteps];
volatile int step = 0;        // the step playing, for the display
volatile bool pulsed = false; // set on each pulse, for the display
float noteRatio = 1.0f;       // the step's interval above Pitch
float echoTone = 0.0f;        // the echo feedback low-pass's state

/** Roll a new sequence over two octaves of the scale. */
void randomize() {
  for (int i = 0; i < kMaxSteps; i++)
    sequence[i] = (uint8_t)rng.nextRange(0, 9);
}

/**
 * Turn the menu's numbers into the voice's settings. Called from loop()
 * after any change; uses library maths, so control rate only.
 */
void applyMenu() {
  // 1. The panel. Exponential where the ear hears ratios (pitch, rates),
  //    linear where it hears amounts.
  // Volume, squared: the ear hears level roughly on a log scale, and
  // a square is close enough that the bottom half of the range isn't
  // all "nearly full". 70 is about -6 dB.
  float v = position(VOLUME);
  volume = 30000.0f * v * v;
  // Pitch (menu only): a whole note, so it always lands on a semitone.
  baseHz = midiToFreq((uint8_t)items[PITCH].value); // powf: control rate
  noiseAmount = position(NOISE);
  timbre = position(TIMBRE);
  fmDepth = 0.95f * position(FM); // under 1: the pitch never goes negative
  modRatio = 0.25f * fastExp2(5.0f * position(MOD_RATE));     // 0.25x - 8x
  modFreeHz = 0.05f * fastExp2(15.3f * position(MOD_RATE));   // to 2 kHz
  pulser.setPeriod(4.0f * fastExp2(-7.06f * position(RATE))); // 4 s - 30 ms
  lpg.setDecay(position(DECAY));
  droneLevel = position(DECAY);
  randomChance = position(RANDOM);
  space = position(SPACE);

  // 2. The menu-only items.
  modWave = items[MOD_WAVE].value;
  const BasicOscillator::Shape shapes[] = {
      BasicOscillator::TRIANGLE, BasicOscillator::SAW, BasicOscillator::SQUARE};
  if (modWave < 3)
    modOsc.setShape(shapes[modWave]);
  modTrack = items[MOD_TRACK].value;
  modTimbre = 0.5f * position(MOD_TIMBRE);
  modMix = 0.6f * position(MOD_MIX);
  analogFold = items[FOLD].value;
  chaosAmount = position(CHAOS);
  chaos.setRate(0.3f + 0.7f * position(CHAOS_RATE)); // drift to snarl
  // Crush: off at 0, then 44 kHz down to about 700 Hz (six octaves).
  float c = position(CRUSH);
  crushOn = c > 0.0f;
  crush.setRate(SAMPLE_RATE * fastExp2(-6.0f * c));
  ping = items[GATE].value;
  steps = items[STEPS].value;
  colour.setColour(COLOUR_BY_BRIGHTNESS[items[COLOUR].value]);
  echoSamples = minf(items[ECHO_TIME].value * 0.01f * SAMPLE_RATE, 16382.0f);
  echoMix = position(ECHO_MIX);
  echoFeedback = items[FEEDBACK].value / 100.0f;
}

/**
 * One pulse (audio interrupt): maybe rewrite the next step, play it,
 * and strike the gate.
 */
void onPulse() {
  int s = (step + 1) % steps;
  // Random: the Turing machine's lock. At 0 the loop repeats forever;
  // turning it up lets new notes in.
  if (rng.nextFloat() < randomChance)
    sequence[s] = (uint8_t)rng.nextRange(0, 9);
  step = s;
  // The Colour's root is 1 Hz (setup()), so getNote() is the ratio.
  noteRatio = colour.getNote(0, sequence[s]);
  if (ping)
    lpg.trigger();
  pulsed = true;
}

/**
 * The complex oscillator at twice the sample rate: two folded samples
 * (sine plus noise), low-passed at 18 kHz, keeping only the second. The
 * low-pass is what makes dropping every other sample safe.
 *
 * @param frequency  Cycles per sample at the 2x rate (Hz / 88200)
 * @param noise      Added to the sine before folding
 */
float complexOscillator(float frequency, float noise) {
  float out = 0.0f;
  for (int k = 0; k < 2; k++) {
    float folded = folder.process(complexOsc.next(frequency) + noise);
    if (analogFold)
      folded = folder.analog(); // the same sample, the rougher curve
    out = antiAlias2.process(antiAlias1.process(folded).lp).lp;
  }
  return out;
}

/** Fill one block of audio (interrupt context: no I2C, no Serial). */
void audioCallback(int16_t *buffer, uint16_t length) {
  const float oversampledTime = 1.0f / (2.0f * SAMPLE_RATE);

  for (uint16_t i = 0; i < length; i += 2) {
    // 1. Timing.
    if (pulser.process())
      onPulse();

    // 2. Chaos: two of the Lorenz outputs, scaled by the Chaos item.
    chaos.process();
    float chaosTimbre = 0.5f * chaosAmount * chaos.x();
    float chaosFm = 0.5f * chaosAmount * chaos.y();

    // 3. Modulation oscillator: following the pitch (Ratio) or on its
    //    own (Free). The wave's frequency is capped at 0.25 of the
    //    sample rate, as BasicOscillator asks.
    float hz = baseHz * noteRatio;
    float modHz = modTrack ? hz * modRatio : modFreeHz;
    float mod;
    if (modWave == 3) {
      modNoise.setFrequency(modHz);
      mod = modNoise.process();
    } else {
      mod = modOsc.process(minf(modHz / SAMPLE_RATE, 0.25f), 0.5f);
    }

    // 4. Complex oscillator. FM swings the pitch by up to 95% either
    //    way, so it never runs backwards; chaos and the mod osc move
    //    the folding.
    float depth = clampf(fmDepth + chaosFm, 0.0f, 0.95f);
    folder.setAmount(timbre + modTimbre * mod + chaosTimbre);
    float noise = noiseAmount * white.bipolar();
    float x =
        complexOscillator(hz * (1.0f + depth * mod) * oversampledTime, noise);
    x += modMix * mod;

    // 5. Low-pass gate: struck by the pulser, or held open by Decay.
    x = ping ? lpg.process(x) : lpg.process(x, droneLevel);

    // 6. Crush, then the echo. Its repeats go back in through a
    //    low-pass, so each is darker than the last.
    if (crushOn) // skipped at 0: it costs about 5% even passing through
      x = crush.process(x);
    float wet = echo.read(echoSamples);
    echoTone += 0.35f * (wet - echoTone);
    echo.write(0.7f * x + echoFeedback * echoTone);
    x += echoMix * wet;

    // 7. The spring, at half rate. Each odd sample feeds it the
    //    average of the last two inputs (a weak low-pass: some of the
    //    treble still folds back, hence the crunch) and takes a new pair out.
    //    The output between is a straight line through the last two: the
    //    midpoint on even samples, the newest on odd ones.
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

    // 8. Dry/wet, then a soft limit to round off what goes past full
    //    scale. The spring spreads the mono voice into stereo.
    float left = x + space * (wetL - x);
    float right = x + space * (wetR - x);
    //    Volume comes after the limit, so turning it down only makes
    //    it quieter, never cleaner or dirtier.
    buffer[i] = (int16_t)(softLimitClip(left) * volume);
    buffer[i + 1] = (int16_t)(softLimitClip(right) * volume);
  }
}

/**
 * Draw one frame. The yellow top band (rows 0-15 on two-colour panels)
 * holds the title and the sequence; the blue rest holds the menu.
 */
void drawScreen() {
  // 1. Title bar, dark on lit, with a box that flashes on each pulse.
  oled.rect(0, 0, 128, 8, true);
  oled.text(2, 0, "NOISEBOX", false);
  oled.text(64, 0, kGates[items[GATE].value], false);
  oled.rect(118, 1, 6, 6, true, !pulsed);
  pulsed = false;

  // 2. The sequence: a bar per step, as tall as its note, with a line
  //    under the step playing.
  oled.rect(0, 8, 128, 8, true, false);
  int playing = step;
  for (int i = 0; i < steps; i++) {
    int h = 1 + sequence[i] / 2; // 1 to 5 pixels
    oled.rect(i * 16 + 2, 14 - h, 12, h, true);
    if (i == playing)
      oled.rect(i * 16 + 2, 15, 12, 1, true);
  }

  // 3. The menu: six rows under the yellow band.
  menu.draw(oled, 16, 6);
}

void setup() {
  Serial.begin(115200);

  // 1. The complex oscillator and its anti-alias filters run at
  //    88.2 kHz. Two 2-pole low-passes with these Qs make a 4-pole
  //    Butterworth: flat to the cutoff, then 24 dB per octave down.
  complexOsc.init(2.0f * SAMPLE_RATE);
  antiAlias1.init(2.0f * SAMPLE_RATE);
  antiAlias2.init(2.0f * SAMPLE_RATE);
  antiAlias1.setFrequency(18000.0f, 0.5412f);
  antiAlias2.setFrequency(18000.0f, 1.3066f);

  // 2. The rest of the voice.
  lpg.setColour(0.6f);          // between VCA and filter: the bongo
  spring.init(SAMPLE_RATE / 2); // half rate: see the file header
  spring.setAmount(1.0f);       // wet only: the mix is in the callback
  spring.setTime(0.7f);
  spring.setTone(0.45f);
  colour.setRoot(1.0f); // so getNote() returns a ratio over Pitch
  static const char *const kNames[12] = {"C",  "C#", "D",  "D#", "E",  "F",
                                         "F#", "G",  "G#", "A",  "A#", "B"};
  for (int n = 0; n < kNotes; n++) {
    int midi = kLowNote + n;
    // MIDI 24 is C1: octave = note / 12 - 1.
    snprintf(noteText[n], sizeof(noteText[n]), "%s%d", kNames[midi % 12],
             midi / 12 - 1);
    noteNames[n] = noteText[n];
  }
  for (int i = 0; i < kItems; i++)
    fine[i] = -1.0f;
  randomize();
  applyMenu();

  encoder.begin();
  push.begin();
  haveScreen = oled.begin();

  AudioEngine::begin();
}

void loop() {
  // 1. Knobs. Only a real move gets through a knob's hysteresis; then
  //    it sets its menu item, so the screen shows where it is. At
  //    power-up every knob counts as moved: the panel wins.
  static float last[kKnobCount] = {-1, -1, -1, -1, -1, -1, -1, -1};
  bool moved = false;
  for (int k = 0; kKnobs && k < kKnobCount; k++) {
    knobs[k].update();
    float v = knobs[k].read();
    if (v == last[k])
      continue;
    last[k] = v;
    int item = kKnobItems[k];
    MenuItem &m = items[item];
    m.value = m.min + (int)(v * (m.max - m.min) + 0.5f);
    fine[item] = v;
    moved = true;
  }
  if (moved)
    applyMenu();

  // 2. Menu: hand it the encoder and the switch.
  push.update();
  Menu::Event event = menu.update(encoder.delta(), push.triggered());
  if (event == Menu::CHANGED) {
    fine[menu.selected()] = -1.0f; // the menu has this item now
    applyMenu();
  } else if (event == Menu::ACTION && menu.selected() == RANDOMIZE) {
    randomize();
  }

  // 3. CPU load over USB serial once a second (pio device monitor).
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.println("%");
  }

  // 4. Display: redraw at 20 frames a second, send one page per loop()
  //    so the knobs stay quick (see Oled::display()).
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    drawScreen();
  }
  if (haveScreen)
    oled.display(1);
}
