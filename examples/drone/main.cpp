/**
 * @file main.cpp
 * @brief A five-voice drone that chaos keeps drifting: chord palette,
 *        Moog or MS-20 filter, echo and reverb, from 8 pots and a menu.
 *
 * Five oscillators tuned to a chord over a root note, pink noise, a Moog
 * or MS-20 low-pass, and a reverb. A Lorenz attractor (chaos) drifts each
 * oscillator's pitch and level, the noise and the cutoff, so the drone
 * never quite repeats.
 *
 * CHORDS: each oscillator plays the root times a ratio. The palette:
 *   Fifths     1, 3/2, 2, 1/2, 3      root, fifth, octave, sub, 12th
 *   Octaves    1, 2, 4, 1/2, 1/4
 *   Fourths    1, 4/3, 2, 1/2, 8/3
 *   Major      1, 5/4, 3/2, 1/2, 2
 *   Minor      1, 6/5, 3/2, 1/2, 2
 *   Harmonics  1/2, 1, 3/2, 2, 5/2    the first five harmonics of the sub
 *   Colour     1, 3/2, 2 x low, 1/2, 2 x high
 * Colour keeps a frame of root, fifth and sub, and takes the other two
 * notes from a Colour (mod/colour.h): its lower variable note (flat 2nd
 * to major 3rd) and upper one (flat 6th to major 7th), an octave up. The
 * Colour item then picks one of 16 just-intonation chords, dark to
 * bright.
 *
 * SPREAD: at 0 every oscillator is in the middle (mono, as before);
 * turning it up moves them apart, the root and sub staying centred.
 *
 * DETUNE: chaos adds up to Detune Hz to each pitch, the same number of
 * Hz whatever the note, so every oscillator beats at about the same rate.
 *
 * THE MENU (encoder; Menu in io/menu.h does the work):
 *   turn   move between items
 *   press  start editing the item (its value shows as <value>)
 *   turn   change the value; press again to finish
 *
 * SCREENSAVER: after 30 seconds without the encoder, the screen shows
 * the output waveform; turn or press the encoder to get the menu back
 * (that first turn or press only wakes it). Knobs don't count: they
 * keep playing the sound while the scope runs.
 *
 * Wiring:
 *   Audio    PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *            VIN -> 5V, GND -> GND (5 V, not 3V3)
 *   OLED     VCC -> 3V3, GND -> GND, SCL -> PB8, SDA -> PB9
 *   Encoder  A -> PB6, B -> PB7, C (middle pin) -> GND
 *   Push     one leg -> PB5, the other -> GND
 *   Knobs    8 pots, outer legs to 3V3 and GND, wipers to:
 *              PA1 Volume     PA2 Cutoff     PA3 Resonance  PA4 Detune
 *              PA5 Motion     PA6 Reverb     PA7 Chaos      PB0 Chaos Rate
 *            Set kKnobs = false without them. (PA0 is skipped: the
 *            board's KEY button is on it.)
 *
 * Cost: 74-75% of the F411's CPU with five voices and the echo, measured
 * on the board (the reverb is about 29%, the echo about 8%; each voice
 * fewer saves about 4%). 109 KB of RAM, 64 KB of it the echo's buffer.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/midi_utils.h"
#include "buddhabox/engine/audio_engine.h"

#include "buddhabox/osc/basic.h"

#include "buddhabox/filter/moog_filter.h"
#include "buddhabox/filter/ms20_filter.h"

#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/encoder_input.h"
#include "buddhabox/io/menu.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/io/switch_input.h"

#include "buddhabox/fx/delay.h"
#include "buddhabox/fx/reverb.h"

#include "buddhabox/mod/colour.h"
#include "buddhabox/mod/lorenz.h"

#include "buddhabox/noise/noise.h"

using namespace bb;

// ── Settings ─────────────────────────────────────────────────────────
const bool kKnobs = true; // pots on PA1-PA7 and PB0 (see Wiring)
// Samples between control updates (chaos, pitches, cutoff). The filters'
// setFrequency() costs 183-233 cycles; every 16th sample that's 11-15 a
// sample instead. 2,756 updates a second is still far faster than the
// chaos ever moves.
const int kControlEvery = 16;
const float kFilterMax = 8000.0f;
// Make-up gain after each filter, so Volume at 100 peaks at about 0.8.
// The two filters differ a lot in level: the MS-20 boosts its input 4x
// inside, and the Moog thins the bass as resonance rises. Measured on
// the board (Fifths, resonance about 36%), before these gains the Moog
// peaked at about 0.4 and the MS-20 at about 1.3. The gains below bring
// both to about 0.75-0.85 after the soft clip.
// After the filter, so they change the level, not the filter's tone.
const float kMoogGain = 3.4f;
const float kMs20Gain = 1.0f;
// Screensaver: after this long without the encoder (turn or press), the
// screen shows the output waveform; the encoder brings the menu back.
const uint32_t kScreensaverMs = 30000;

// ── The menu ─────────────────────────────────────────────────────────
enum Item {
  VOLUME,
  ROOT,
  FINE,
  CHORD,
  VOICES,
  COLOUR,
  WAVE,
  FILTER,
  CUTOFF,
  RESONANCE,
  DETUNE,
  MOTION,
  CHAOS,
  CHAOS_RATE,
  WOBBLE,
  NOISE,
  SPACE,
  TONE,
  REVERB,
  SPREAD,
  ECHO_TIME,
  FEEDBACK,
  ECHO_MIX,
  kItems
};

const char *const kChords[] = {"Fifths", "Octaves",   "Fourths", "Major",
                               "Minor",  "Harmonics", "Colour"};
const int kColourChord = 6; // the last entry: ratios from the Colour
const char *const kWaves[] = {"Saw", "Square", "Dark", "Triangle", "Slope"};
const BasicOscillator::Shape kShapes[] = {
    BasicOscillator::SAW, BasicOscillator::SQUARE, BasicOscillator::SQUARE_DARK,
    BasicOscillator::TRIANGLE, BasicOscillator::SLOPE};
const char *const kFilters[] = {"Moog", "MS-20"};

// Root: a MIDI note, C1 to C5, shown by name. setup() fills noteText.
const int kLowNote = NOTE_C1, kHighNote = NOTE_C5;
const int kNotes = kHighNote - kLowNote + 1; // 49
char noteText[kNotes][4];
const char *noteNames[kNotes];

MenuItem items[kItems] = {
    {"Volume", 80, 0, 100, 2},
    {"Root", NOTE_C1 + 26, kLowNote, kHighNote, 1, noteNames}, // D3
    // Fine tune in cents (hundredths of a semitone), to match another
    // instrument. +-50 covers any tuning: further than that, the next
    // Root note is closer. A = 432 Hz, for example, is about -32.
    {"Fine", 0, -50, 50, 1, nullptr, " ct"},
    {"Chord", 0, 0, 6, 1, kChords},
    {"Voices", 5, 1, 5, 1},
    {"Colour", 6, 0, 15, 1},
    {"Wave", 0, 0, 4, 1, kWaves},
    {"Filter", 0, 0, 1, 1, kFilters},
    // Cutoff 0-255 so a slow knob turn moves in small steps (about a
    // third of a semitone); the encoder moves 4 at a time.
    {"Cutoff", 40, 0, 255, 4},
    {"Resonance", 0, 0, 100, 2},
    {"Detune", 12, 0, 100, 2}, // x 0.08 Hz: 12 = about 1 Hz, as before
    {"Motion", 50, 0, 100, 2}, // x 20 Hz of chaos on the cutoff
    {"Chaos", 0, 0, 100, 2},
    {"Chaos Rate", 50, 0, 100, 2},
    {"Wobble", 100, 0, 100, 2}, // chaos on each oscillator's level
    {"Noise", 10, 0, 100, 2},   // x 0.01: 10 = 0.1
    {"Space", 70, 0, 100, 2},   // reverb decay
    {"Tone", 70, 0, 100, 2},    // reverb damping: 100 = brightest
    {"Reverb", 66, 0, 100, 2},  // wet/dry mix
    {"Spread", 50, 0, 100, 2},  // oscillators across the stereo field
    {"Echo Time", 500, 50, 740, 10, nullptr, " ms"},
    {"Feedback", 40, 0, 90, 2},
    {"Echo Mix", 0, 0, 100, 2}, // 0: no echo until you ask for one
};
Menu menu(items, kItems);

// ── The knobs ────────────────────────────────────────────────────────
// Each pot drives one menu item; change the table to move a setting.
struct Knob {
  AnalogInput input;
  int item;           // which menu item it sets
  float last = -1.0f; // its last reading; -1 = never read
};

Knob knobs[] = {
    {AnalogInput(PA1, 0.7f), VOLUME},    {AnalogInput(PA2, 0.7f), CUTOFF},
    {AnalogInput(PA3, 0.7f), RESONANCE}, {AnalogInput(PA4, 0.7f), DETUNE},
    {AnalogInput(PA5, 0.7f), MOTION},    {AnalogInput(PA6, 0.7f), REVERB},
    {AnalogInput(PA7, 0.7f), CHAOS},     {AnalogInput(PB0, 0.7f), CHAOS_RATE},
};

// ── Modules ──────────────────────────────────────────────────────────
#define NUM_OSC (5)
BasicOscillator oscillators[NUM_OSC];
PinkNoise noise(42);
// One filter per channel, since spread makes left and right differ.
MoogFilter moog[2];
Ms20Filter ms20[2];
Lorenz chaos; // runs once per control update, not every sample
Reverb<> spring;
Colour colour; // only for its ratios: the Colour chord
// Mono echo: 32768 samples (0.74 s) of 16-bit storage, 64 KB, half the
// F411's RAM. ponytail: 0.74 s is the ceiling; storing every other
// sample would double it, at the cost of a duller, aliased echo.
DelayLine<32768> echo;

EncoderInput encoder;
SwitchInput push(PB5);
Oled oled;
bool haveScreen = false;

// ── Patch settings, from the menu ────────────────────────────────────
// loop() writes these and the audio interrupt reads them. A float is
// written in one instruction, so the interrupt never sees half of one;
// at worst a chord change lands a few oscillators one block apart.
float base_freq = 146.8f; // the root, Hz
float ratios[NUM_OSC] = {1.0f, 1.5f, 2.0f, 0.5f, 3.0f};
bool useMs20 = false;
float filter_cutoff = 100.0f; // Hz
float filter_res = 0.0f;
float detune_hz = 1.0f;    // most Hz chaos can add to a pitch
float motion_hz = 1000.0f; // most Hz chaos can add to the cutoff
float chaos_mult = 0.0f;
float wobble = 1.0f;
float noise_level = 0.1f;
float volume = 0.8f;
// Voices: how many oscillators play, taken in this order so the chord
// keeps its shape as it thins: the root first, then the fifth, the sub,
// and last the two upper notes. (In most chords oscillator 0 is the
// root, 1 the fifth or third, 3 the sub.) Silent ones aren't computed
// at all, so fewer voices also saves CPU: about 4% each, measured.
const int kVoiceOrder[NUM_OSC] = {0, 1, 3, 2, 4};
int voices = NUM_OSC;
// Fewer oscillators sum to a quieter signal, so turn them up to keep
// the level roughly the same. sqrt(5 / voices): in between the 5/n a
// perfectly lined-up sum would need and the 1 that assumes it adds up
// randomly. Measured on the board: 5, 3 and 1 voices all peak at
// about 0.65-0.85.
float voiceGain = 1.0f;
float echoSamples = 22050.0f; // the Echo Time setting, in samples
float echoFeedback = 0.4f, echoMix = 0.0f;
// Spread: each oscillator's place, -1 (left) to +1 (right), at full
// Spread. Root and sub stay in the middle so the bass stays centred;
// the others pair off. Change these to rearrange the field.
const float kPan[NUM_OSC] = {0.5f, -1.0f, 1.0f, 0.0f, -0.5f};
float panL[NUM_OSC] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
float panR[NUM_OSC] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

// Ratios for each fixed chord; the Colour chord is built in applyMenu().
const float kChordRatios[kColourChord][NUM_OSC] = {
    {1.0f, 1.5f, 2.0f, 0.5f, 3.0f},         // Fifths (the original)
    {1.0f, 2.0f, 4.0f, 0.5f, 0.25f},        // Octaves
    {1.0f, 4.0f / 3, 2.0f, 0.5f, 8.0f / 3}, // Fourths
    {1.0f, 1.25f, 1.5f, 0.5f, 2.0f},        // Major
    {1.0f, 1.2f, 1.5f, 0.5f, 2.0f},         // Minor
    {0.5f, 1.0f, 1.5f, 2.0f, 2.5f},         // Harmonics
};

// ── Control-rate state (audio interrupt only) ────────────────────────
int controlCounter = 0;
float echoTime = 22050.0f; // where the echo reads now, gliding to echoSamples
float echoTone = 0.0f;     // the feedback low-pass's state

// ── Screensaver scope ────────────────────────────────────────────────
// The audio interrupt fills scope[] with one screen's width of output;
// loop() draws it. scopeState hands the buffer back and forth, so the
// two never touch it at the same time:
//   SCOPE_IDLE   nobody wants a trace (the menu is showing)
//   SCOPE_ARMED  loop() wants one: the interrupt waits for the left
//                channel to cross zero going up, so each trace starts
//                at the same point in the wave and the picture holds
//                still instead of jittering sideways
//   SCOPE_FILL   the interrupt is writing scope[]
//   SCOPE_READY  full: loop() draws it, then arms again
// Cost: a compare per sample, plus a store every kScopeEvery samples.
enum ScopeState { SCOPE_IDLE, SCOPE_ARMED, SCOPE_FILL, SCOPE_READY };
const int kScopeWidth = 128;
// One point every 4 samples: 128 points span 11.6 ms, about two cycles
// of D3 (147 Hz).
const int kScopeEvery = 4;
volatile int scopeState = SCOPE_IDLE;
int8_t scope[kScopeWidth]; // -31..31: rows above and below the middle
int scopeIndex = 0, scopeSkip = 0, scopeWait = 0;
float scopePrev = 0.0f;
// Each oscillator's level into each channel: wobble times pan.
float ampL[NUM_OSC], ampR[NUM_OSC];
float noise_amount = 0.0f;

/**
 * Read every menu value into the patch settings. Called from loop()
 * after any change; uses library maths (powf, midiToFreq), so control
 * rate only.
 */
void applyMenu() {
  // Root note, then Fine: semitonesToRatio(cents / 100) is the
  // frequency ratio for that fraction of a semitone.
  base_freq = midiToFreq((uint8_t)items[ROOT].value) *
              semitonesToRatio(items[FINE].value / 100.0f);

  // 1. Chord ratios: a fixed row, or built from the Colour.
  int chord = items[CHORD].value;
  if (chord == kColourChord) {
    // With the root at 1 Hz, getNote() returns the ratio itself:
    // note 1 is the lower variable note, note 4 the upper one.
    colour.setColour(COLOUR_BY_BRIGHTNESS[items[COLOUR].value]);
    colour.setRoot(1.0f);
    float r[NUM_OSC] = {1.0f, 1.5f, 2.0f * colour.getNote(0, 1), 0.5f,
                        2.0f * colour.getNote(0, 4)};
    for (int i = 0; i < NUM_OSC; i++)
      ratios[i] = r[i];
  } else {
    for (int i = 0; i < NUM_OSC; i++)
      ratios[i] = kChordRatios[chord][i];
  }

  // 2. Oscillators and filter. Cutoff is exponential, 30 Hz times
  //    2^(value / 32): 30 Hz to about 7.6 kHz, eight octaves.
  for (int i = 0; i < NUM_OSC; i++)
    oscillators[i].setShape(kShapes[items[WAVE].value]);
  useMs20 = items[FILTER].value;
  filter_cutoff = 30.0f * powf(2.0f, items[CUTOFF].value / 32.0f);
  filter_res = items[RESONANCE].value / 100.0f;

  // 3. Chaos and what it moves.
  detune_hz = items[DETUNE].value * 0.08f;
  motion_hz = items[MOTION].value * 20.0f;
  chaos_mult = items[CHAOS].value / 100.0f;
  // Rate 0.30 to 0.50: from about a loop a minute to about three a
  // second. Below 0.30 the attractor barely moves.
  chaos.setRate(0.30f + 0.20f * items[CHAOS_RATE].value / 100.0f);
  wobble = items[WOBBLE].value / 100.0f;
  noise_level = items[NOISE].value * 0.01f;

  // 4. Spread: equal-power panning. With p from -1 to +1, the gains
  //    sqrt(1 - p) and sqrt(1 + p) keep left^2 + right^2 the same
  //    wherever the oscillator sits, so it doesn't get louder or
  //    quieter as it moves. In the middle both are 1, so Spread 0 is
  //    exactly the old mono sound.
  for (int i = 0; i < NUM_OSC; i++) {
    float p = kPan[i] * items[SPREAD].value / 100.0f;
    panL[i] = sqrtf(1.0f - p);
    panR[i] = sqrtf(1.0f + p);
  }
  volume = items[VOLUME].value / 100.0f;
  voices = items[VOICES].value;
  voiceGain = sqrtf((float)NUM_OSC / voices);
  echoSamples = items[ECHO_TIME].value * (SAMPLE_RATE / 1000.0f);
  echoFeedback = items[FEEDBACK].value / 100.0f;
  echoMix = items[ECHO_MIX].value / 100.0f;

  // 5. Reverb: decay 0.5 (small room) to 0.98 (near endless).
  spring.setTime(0.5f + 0.48f * items[SPACE].value / 100.0f);
  spring.setLp(0.3f + 0.7f * items[TONE].value / 100.0f);
  spring.setAmount(items[REVERB].value / 100.0f);
}

/**
 * Fill one block of audio. Runs in interrupt context: no Serial, no
 * analogRead(), no allocation.
 *
 * @param buffer  Interleaved stereo int16 samples [L, R, L, R, ...]
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    // 1. Control update, every kControlEvery samples: step the chaos
    //    and set pitches, levels and the cutoff from it.
    if (controlCounter == 0) {
      controlCounter = kControlEvery;
      chaos.process();
      float cx = chaos.x() * chaos_mult;
      float cy = chaos.y() * chaos_mult;
      float cz = chaos.z() * chaos_mult;

      // Which axis moves which oscillator: pitch, then level.
      float pitchMod[NUM_OSC] = {cx, cy, cz, cx, cy};
      float levelMod[NUM_OSC] = {cx, cy, cz, cx, cz};
      for (int o = 0; o < NUM_OSC; o++) {
        oscillators[o].setFrequency(base_freq * ratios[o] +
                                    pitchMod[o] * detune_hz);
        float amp = 1.0f + levelMod[o] * wobble;
        ampL[o] = amp * panL[o];
        ampR[o] = amp * panR[o];
      }
      // Noise is always there at its level, and chaos swells it.
      // (It used to be noise_level * cz: silent with Chaos at 0,
      // and z spends most of its time near 0.)
      noise_amount = noise_level * (1.0f + cz * wobble);

      float hz = clampf(filter_cutoff + cy * motion_hz, 80.0f, kFilterMax);
      // Only the filter in use is updated; the other one sits
      // idle and picks up the settings when switched to.
      for (int c = 0; c < 2; c++) {
        if (useMs20) {
          ms20[c].setFrequency(hz);
          ms20[c].setResonance(filter_res);
        } else {
          moog[c].setFrequency(hz);
          moog[c].setResonance(filter_res);
        }
      }
    }
    controlCounter--;

    // 2. Oscillators and noise.
    float left = 0.0f, right = 0.0f;
    // Read voices once: loop() may change it mid-block.
    int count = voices;
    for (int v = 0; v < count; v++) {
      int o = kVoiceOrder[v];
      float s = oscillators[o].process();
      left += s * ampL[o];
      right += s * ampR[o];
    }
    left *= voiceGain;
    right *= voiceGain;
    float n = noise.process() * noise_amount; // centred
    left = (left + n) * 0.33f;
    right = (right + n) * 0.33f;

    // 3. Filter and its make-up gain (see kMoogGain).
    if (useMs20) {
      left = ms20[0].process(left) * kMs20Gain;
      right = ms20[1].process(right) * kMs20Gain;
    } else {
      left = moog[0].process(left) * kMoogGain;
      right = moog[1].process(right) * kMoogGain;
    }

    // 4. Echo, before the reverb so the repeats blur into it.
    //    a. The read point glides to a new Echo Time over about
    //       50 ms instead of jumping, which would click; the glide
    //       bends the repeats' pitch briefly, like a tape echo.
    echoTime += 0.0005f * (echoSamples - echoTime);
    float wet = echo.read(echoTime);
    //    b. A gentle low-pass in the feedback: each repeat comes back
    //       a little darker, so they fade into the drone.
    echoTone += 0.35f * (wet - echoTone);
    //    c. Mono in: the middle of the stereo signal, at half level
    //       because the delay's 16-bit storage clips at +-1 and the
    //       filter's output reaches about 1.1. Back out x 2.
    float mid = 0.5f * (left + right);
    echo.write(0.5f * mid + echoFeedback * echoTone);
    left += 2.0f * echoMix * wet;
    right += 2.0f * echoMix * wet;

    // 5. Reverb.
    spring.process(left, right);

    // 6. To 16 bits. Five oscillators with chaos on their levels
    //    can reach about +-3, and the MS-20's resonance more, so a
    //    soft clip rounds off the peaks instead of chopping them.
    //    Volume comes after the clip, so it only changes the level,
    //    never how hard the peaks are rounded off.
    float outL = softLimitClip(left), outR = softLimitClip(right);
    float gain = volume * 32000.0f;
    buffer[i] = (int16_t)(outL * gain);
    buffer[i + 1] = (int16_t)(outR * gain);

    // 7. Screensaver scope (see ScopeState). Taken before Volume,
    //    so the trace fills the screen however loud it's playing.
    if (scopeState == SCOPE_ARMED) {
      // Wait for an upward zero crossing, but not forever: after
      // a block's worth, take whatever comes (a drone with the
      // filter shut may never cross cleanly).
      if ((scopePrev < 0.0f && outL >= 0.0f) || ++scopeWait > 256) {
        scopeIndex = scopeSkip = scopeWait = 0;
        scopeState = SCOPE_FILL;
      }
      scopePrev = outL;
    }
    if (scopeState == SCOPE_FILL && scopeSkip-- == 0) {
      scopeSkip = kScopeEvery - 1;
      scope[scopeIndex++] = (int8_t)(outL * 31.0f);
      if (scopeIndex == kScopeWidth)
        scopeState = SCOPE_READY;
    }
  }
}

/** Draw one frame: a title bar with the root and chord, then the menu. */
/**
 * Draw the screensaver: the last captured trace, one line segment per
 * column, centred on row 32. Returns false while no new trace is ready.
 */
bool drawScope() {
  if (scopeState != SCOPE_READY)
    return false;
  oled.clear();
  for (int x = 1; x < kScopeWidth; x++)
    oled.line(x - 1, 32 - scope[x - 1], x, 32 - scope[x]);
  scopeState = SCOPE_ARMED; // ask for the next one
  return true;
}

void drawScreen() {
  oled.rect(0, 0, 128, 8, true);
  int x = oled.text(2, 0, "DRONE  ", false);
  x = oled.text(x, 0, noteNames[items[ROOT].value - kLowNote], false);
  x = oled.text(x, 0, " ", false);
  oled.text(x, 0, kChords[items[CHORD].value], false);
  oled.rect(0, 8, 128, 8, true, false);
  menu.draw(oled, 16, 6);
}

void setup() {
  Serial.begin(115200);

  // The Lorenz steps once per control update, so tell it that rate;
  // its Rate setting then means the same speed as at full rate.
  chaos.init(SAMPLE_RATE / kControlEvery);

  static const char *const kNames[12] = {"C",  "C#", "D",  "D#", "E",  "F",
                                         "F#", "G",  "G#", "A",  "A#", "B"};
  for (int n = 0; n < kNotes; n++) {
    int midi = kLowNote + n;
    // MIDI 24 is C1: octave = note / 12 - 1.
    snprintf(noteText[n], sizeof(noteText[n]), "%s%d", kNames[midi % 12],
             midi / 12 - 1);
    noteNames[n] = noteText[n];
  }
  applyMenu();

  encoder.begin();
  push.begin();
  haveScreen = oled.begin();

  AudioEngine::begin();
}

void loop() {
  // 1. Knobs. Only a real move gets through a knob's hysteresis, and
  //    then it sets its menu item, so the screen shows where it is.
  //    At power-up every knob counts as moved: the panel wins.
  bool moved = false;
  if (kKnobs) {
    for (Knob &k : knobs) {
      k.input.update();
      float v = k.input.read();
      if (v == k.last)
        continue;
      k.last = v;
      MenuItem &m = items[k.item];
      m.value = m.min + (int)(v * (m.max - m.min) + 0.5f);
      moved = true;
    }
    if (moved)
      applyMenu();
  }

  // 2. Menu: hand it the encoder and the switch. While the
  //    screensaver is up, the first turn or press only wakes the
  //    menu, so it can't change a setting you can't see.
  push.update();
  int turns = encoder.delta();
  bool pressed = push.triggered();
  static uint32_t lastActivity = 0;
  static bool sleeping = false;
  if (turns || pressed) {
    lastActivity = millis();
    if (sleeping) {
      sleeping = false;
      scopeState = SCOPE_IDLE;
      turns = 0;
      pressed = false;
    }
  }
  if (menu.update(turns, pressed) == Menu::CHANGED)
    applyMenu();
  if (!sleeping && haveScreen && millis() - lastActivity > kScreensaverMs) {
    sleeping = true;
    scopeState = SCOPE_ARMED;
  }

  // 3. CPU load over USB serial once a second (pio device monitor).
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.println("%");
  }

  // 4. Display: 20 frames a second, one page (8 rows) per loop() so
  //    the knobs are read in between.
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    if (!sleeping)
      drawScreen();
    else if (!drawScope())
      lastFrame -= 50; // no trace yet: try again next loop()
  }
  if (haveScreen)
    oled.display(1);
}
