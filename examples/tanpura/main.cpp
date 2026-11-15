/**
 * @file main.cpp
 * @brief A four-string tanpura: buzzing-bridge strings plucked in a 5- or
 *        6-beat cycle, with tuning, tempo, stereo spread and reverb.
 *
 * The tanpura is the drone of Indian classical music. Its four strings
 * are plucked one after another, over and over, and never damped, so
 * each is still ringing when it is plucked again. The usual tuning, low
 * to high in pitch order of plucking:
 *   pa  SA  SA  sa
 * SA is the root (the singer's tonic), sa the same note an octave down,
 * and pa the fifth below SA. Lower case means the lower octave. For
 * ragas without Pa the first string goes to ma, ni or dha instead.
 *
 * JAWARI, THE BUZZING BRIDGE: the strings rest on a wide, gently curved
 * bridge, and a cotton thread under each string sets where it touches.
 * Each swing towards the bridge slaps against it, which bends the wave
 * and throws energy up into the high harmonics: the tanpura's long,
 * shimmering buzz. TanpuraString models the bridge as a soft wall on
 * one side of the string: past it, the string gives way only partly.
 * Like a real bridge it can only take energy away, never add it, so the
 * string always dies away. (An earlier version bent the loop's delay
 * with each swing, as Elements' curved bridge does. With a tanpura's
 * long sustain that rang forever in simulation: bending the delay twice
 * per cycle pumps energy in, as a child pumps a swing.)
 *
 * WHY NOT StringModel: it has a curved bridge, but costs 427 cycles a
 * sample (20% of the CPU), so four strings plus a reverb won't fit.
 * TanpuraString keeps only what a tanpura needs: one delay line, the
 * bridge, a DC blocker and a one-pole loss filter.
 *
 * THE CYCLE: one string per beat, then a rest, the last string ringing
 * through it:
 *   5 beats   pa  SA  SA  sa  -
 *   6 beats   pa  -   SA  SA  sa  -
 * Tempo is beats a minute: at the default 48 a 5-beat cycle takes
 * about 6 seconds.
 * Edit kPatterns to change them.
 *
 * THE MENU (encoder; Menu in io/menu.h does the work):
 *   turn   move between items
 *   press  start editing the item (its value shows as <value>)
 *   turn   change the value; press again to finish
 *
 * Wiring:
 *   Audio    PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *            VIN -> 5V, GND -> GND (5 V, not 3V3)
 *   OLED     VCC -> 3V3, GND -> GND, SCL -> PB8, SDA -> PB9
 *   Encoder  A -> PB6, B -> PB7, C (middle pin) -> GND
 *   Push     one leg -> PB5, the other -> GND
 *   Knobs    8 pots, outer legs to 3V3 and GND, wipers to:
 *              PA1 Volume   PA2 Tempo    PA3 Jawari   PA4 Bright
 *              PA5 Sustain  PA6 Reverb   PA7 Spread   PB0 Space
 *            Set kKnobs = false without them. (PA0 is skipped: the
 *            board's KEY button is on it.)
 *
 * Cost: 67% of the F411's CPU, measured on the board (the reverb is
 * 27%; each string about 170 cycles a sample, 8%). 76 KB of RAM.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/midi_utils.h"
#include "buddhabox/engine/audio_engine.h"

#include "buddhabox/filter/dc_blocker.h"
#include "buddhabox/fx/delay.h"
#include "buddhabox/fx/reverb.h"
#include "buddhabox/physical/plucker.h"

#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/encoder_input.h"
#include "buddhabox/io/menu.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/io/switch_input.h"

using namespace bb;

/**
 * One tanpura string: a Karplus-Strong loop with a buzzing bridge.
 *
 * Each sample: read the wave one period back, let the bridge squash the
 * part that swings past it, take out DC, lose a little treble and
 * level, add the pluck, write it back.
 *
 * @tparam N  Delay length in samples: the lowest note is SAMPLE_RATE / N
 *            (2048: 21.5 Hz). Float storage, 4 bytes a sample.
 *
 * Example:
 *   TanpuraString<2048> s;
 *   s.set(138.6f, 0.6f, 0.7f, 8.0f); // C#3, some jawari, 8 s decay
 *   float out = s.process(pick.process());
 */
template <uint16_t N> class TanpuraString {
public:
  /**
   * Tune and voice the string. Uses powf: control rate only.
   * @param hz      Pitch in Hz
   * @param jawari  0..1: how hard the string hits the bridge (the buzz).
   *                More jawari also shortens the sustain a little.
   * @param bright  0..1: 0 = mellow, 1 = bright. The loop's treble
   *                loss is gentle at both ends, since the jawari's buzz
   *                lives in the treble and losing it shortens the note
   * @param rt60    Seconds for the note to fall by 60 dB without
   *                jawari. With it, the buzz carries energy into the
   *                treble, where the loop loses it faster: in
   *                simulation 60 s gives about 20 s at Jawari 60.
   */
  void set(float hz, float jawari, float bright, float rt60) {
    // The loss filter is a one-pole low-pass, y += a * (x - y). At low
    // frequencies it delays the wave by (1 - a) / a samples, which
    // would flatten the note, so the delay line is that much shorter.
    float a = 0.97f + 0.029f * bright;
    float delay = SAMPLE_RATE / hz - (1.0f - a) / a;
    // Loop gain: the wave passes hz times a second, and after rt60
    // seconds it must be down 60 dB (x 0.001): g^(hz * rt60) = 0.001.
    float g = powf(0.001f, 1.0f / (hz * rt60));
    // Each is one float, so the audio interrupt sees old or new values,
    // never half of one; a re-tune lands within a sample or two.
    a_ = a;
    delay_ = delay;
    gain_ = g;
    // How much of a swing past the bridge it absorbs, each pass. Squared
    // so the knob's first half gives the finer, gentler settings; at
    // full it absorbs 3%. (10% made the attack far louder than the
    // ring that followed, and halved the sustain.)
    give_ = 1.0f - 0.03f * jawari * jawari;
  }

  /**
   * Process one sample.
   * @param in  Excitation (the pluck), 0 when not plucking
   * @return    The string's output
   */
  float process(float in) {
    // 1. Read one period back. Hermite interpolation, not straight-line:
    //    a straight line between samples is itself a low-pass, which
    //    the loop would apply on every pass and dull the string.
    float s = line_.readHermite(delay_ - 1.0f); // (0) is the newest

    // 2. The bridge: a wall at the string's resting line, on one side.
    //    Anything that swings past it is squashed by give_. Squashing
    //    a wave's tops is what adds the harmonics.
    if (s > 0.0f)
      s *= give_;

    // 3. Squashing one side only pushes the wave off centre: take
    //    that DC out, or the string would drift away from the wall.
    s = dc_.process(s);

    // 4. Losses: treble (the low-pass), then level (the gain).
    lp_ += a_ * (s - lp_);
    s = lp_ * gain_ + in;
    line_.write(s);
    return s;
  }

private:
  DelayLine<N, float> line_;
  DcBlocker dc_;
  float delay_ = 300.0f, a_ = 0.95f, gain_ = 0.999f, give_ = 1.0f;
  float lp_ = 0.0f;
};

// ── Settings ─────────────────────────────────────────────────────────
const bool kKnobs = true;      // pots on PA1-PA7 and PB0 (see Wiring)
const float kPluckGain = 0.5f; // the pick's level into the string
// The four strings into the reverb. Set on the board with the default
// settings: peaks of 0.5-0.8 at full Volume (one second can fall in
// the rest, hence the spread).
const float kMixGain = 4.4f;

// ── The menu ─────────────────────────────────────────────────────────
enum Item {
  VOLUME,
  ROOT,
  FINE,
  TUNING,
  PATTERN,
  TEMPO,
  JAWARI,
  BRIGHT,
  SUSTAIN,
  SPREAD,
  SPACE,
  TONE,
  REVERB,
  kItems
};

// Tunings: the first string's note, as a ratio to SA; the others are
// always SA, SA and sa. Just intonation, as a tanpura is tuned by ear:
// pa = 3/2 an octave down, ma = 4/3, ni = 15/8, dha = 5/3.
const char *const kTunings[] = {"pa SA SA sa", "ma SA SA sa", "ni SA SA sa",
                                "dha SA SA sa"};
const float kFirstString[] = {3.0f / 4, 2.0f / 3, 15.0f / 16, 5.0f / 6};
const float kOtherStrings[3] = {1.0f, 1.0f, 0.5f}; // SA, SA, sa

// Patterns: which string each beat plucks; -1 is a rest.
const char *const kPatternNames[] = {"5 beat", "6 beat"};
const int kPatternLength[] = {5, 6};
const int8_t kPatterns[2][6] = {
    {0, 1, 2, 3, -1, -1}, // pa SA SA sa -
    {0, -1, 1, 2, 3, -1}, // pa - SA SA sa -
};

// Root: SA, C2 to C4 by name. Men's tanpuras sit around C3-D3, women's
// around G3-A#3.
const int kLowNote = NOTE_C1 + 12, kHighNote = NOTE_C1 + 36; // C2-C4
const int kNotes = kHighNote - kLowNote + 1;                 // 25
char noteText[kNotes][4];
const char *noteNames[kNotes];

MenuItem items[kItems] = {
    {"Volume", 80, 0, 100, 2},
    {"Root", NOTE_C1 + 26, kLowNote, kHighNote, 1, noteNames}, // D3
    // Fine tune in cents, to match another instrument. A = 432 Hz is
    // about -32.
    {"Fine", 0, -50, 50, 1, nullptr, " ct"},
    {"Tuning", 0, 0, 3, 1, kTunings},
    {"Pattern", 0, 0, 1, 1, kPatternNames},
    {"Tempo", 48, 20, 120, 1, nullptr, " bpm"},
    {"Jawari", 60, 0, 100, 2},
    {"Bright", 70, 0, 100, 2},
    // Seconds to fall 60 dB before the jawari's losses (see set()).
    {"Sustain", 60, 4, 120, 2, nullptr, " s"},
    {"Spread", 50, 0, 100, 2},
    {"Space", 70, 0, 100, 2}, // reverb decay
    {"Tone", 60, 0, 100, 2},  // reverb damping: 100 = brightest
    {"Reverb", 40, 0, 100, 2},
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
    {AnalogInput(PA1, 0.7f), VOLUME},  {AnalogInput(PA2, 0.7f), TEMPO},
    {AnalogInput(PA3, 0.7f), JAWARI},  {AnalogInput(PA4, 0.7f), BRIGHT},
    {AnalogInput(PA5, 0.7f), SUSTAIN}, {AnalogInput(PA6, 0.7f), REVERB},
    {AnalogInput(PA7, 0.7f), SPREAD},  {AnalogInput(PB0, 0.7f), SPACE},
};

// ── Modules ──────────────────────────────────────────────────────────
#define NUM_STRINGS (4)
// 2048 samples reach down to 21.5 Hz; the lowest string is sa at C2,
// 32.7 Hz. 8 KB each.
TanpuraString<2048> strings[NUM_STRINGS];
// One pick for all four strings: only one is plucked at a time, and a
// burst lasts a few milliseconds, far less than a beat.
Plucker pick;
Reverb<> reverb;

EncoderInput encoder;
SwitchInput push(PB5);
Oled oled;
bool haveScreen = false;

// ── Patch settings, from the menu ────────────────────────────────────
// loop() writes these and the audio interrupt reads them; each is one
// float or int, written in one instruction.
float freqs[NUM_STRINGS] = {110.1f, 146.8f, 146.8f, 73.4f}; // D3
float samplesPerBeat = SAMPLE_RATE;
int pattern = 0;
float volume = 0.8f;
// Spread: each string's place at full Spread, -1 (left) to +1 (right).
// pa hard left, sa hard right, the two SAs in the middle; at Spread 0
// all four are in the middle, so the output is mono.
const float kPan[NUM_STRINGS] = {-1.0f, 0.0f, 0.0f, 1.0f};
float panL[NUM_STRINGS] = {1.0f, 1.0f, 1.0f, 1.0f};
float panR[NUM_STRINGS] = {1.0f, 1.0f, 1.0f, 1.0f};

// ── Audio-interrupt state ────────────────────────────────────────────
float beatCounter = 0.0f;      // samples until the next beat
int beat = -1;                 // position in the pattern
int plucking = 0;              // the string the pick is feeding
volatile int lastPlucked = -1; // for the screen

/**
 * Read every menu value into the patch settings. Called from loop()
 * after any change; uses library maths (powf, sqrtf), so control rate
 * only.
 */
void applyMenu() {
  // 1. Pitches: Root plus Fine is SA; each string is a ratio of it.
  float sa = midiToFreq((uint8_t)items[ROOT].value) *
             semitonesToRatio(items[FINE].value / 100.0f);
  freqs[0] = sa * kFirstString[items[TUNING].value];
  for (int i = 1; i < NUM_STRINGS; i++)
    freqs[i] = sa * kOtherStrings[i - 1];

  // 2. Voice every string the same.
  float jawari = items[JAWARI].value / 100.0f;
  float bright = items[BRIGHT].value / 100.0f;
  float rt60 = (float)items[SUSTAIN].value;
  for (int i = 0; i < NUM_STRINGS; i++)
    strings[i].set(freqs[i], jawari, bright, rt60);

  // 3. Rhythm.
  pattern = items[PATTERN].value;
  samplesPerBeat = 60.0f * SAMPLE_RATE / items[TEMPO].value;

  // 4. Spread: equal-power panning (see examples/drone). Spread 0 puts
  //    every string in the middle at full level.
  for (int i = 0; i < NUM_STRINGS; i++) {
    float p = kPan[i] * items[SPREAD].value / 100.0f;
    panL[i] = sqrtf(1.0f - p);
    panR[i] = sqrtf(1.0f + p);
  }

  // 5. Reverb: decay 0.5 (small room) to 0.98 (near endless).
  reverb.setTime(0.5f + 0.48f * items[SPACE].value / 100.0f);
  reverb.setLp(0.3f + 0.7f * items[TONE].value / 100.0f);
  reverb.setAmount(items[REVERB].value / 100.0f);
  volume = items[VOLUME].value / 100.0f;
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
    // 1. The beat clock. On each beat, step the pattern and pluck its
    //    string, if any: tune the pick to it so the burst's length and
    //    comb suit that string, then send the pick to it.
    beatCounter -= 1.0f;
    if (beatCounter <= 0.0f) {
      beatCounter += samplesPerBeat;
      int p = pattern;
      beat = (beat + 1) % kPatternLength[p];
      int s = kPatterns[p][beat];
      if (s >= 0) {
        plucking = s;
        lastPlucked = s;
        pick.setFrequency(freqs[s]);
        pick.trigger();
      }
    }

    // 2. The strings. The pick's burst (and its short tail) goes to
    //    the string it last plucked; the others ring on by themselves.
    float excite = pick.process() * kPluckGain;
    float left = 0.0f, right = 0.0f;
    for (int s = 0; s < NUM_STRINGS; s++) {
      float out = strings[s].process(s == plucking ? excite : 0.0f);
      left += out * panL[s];
      right += out * panR[s];
    }
    left *= kMixGain;
    right *= kMixGain;

    // 3. Reverb.
    reverb.process(left, right);

    // 4. To 16 bits: a soft clip rounds off any peaks, then Volume, so
    //    Volume only changes the level.
    float gain = volume * 32000.0f;
    buffer[i] = (int16_t)(softLimitClip(left) * gain);
    buffer[i + 1] = (int16_t)(softLimitClip(right) * gain);
  }
}

/**
 * Draw one frame: a title bar with the root and the strings, the one
 * just plucked shown inverted, then the menu.
 */
void drawScreen() {
  oled.rect(0, 0, 128, 8, true);
  int x = oled.text(2, 0, "TANPURA ", false);
  oled.text(x, 0, noteNames[items[ROOT].value - kLowNote], false);
  // The four strings as four boxes along the second row: outlined, or
  // filled for the one just plucked.
  int playing = lastPlucked;
  oled.rect(0, 8, 128, 8, true, false);
  for (int s = 0; s < NUM_STRINGS; s++)
    oled.rect(s * 32 + 2, 9, 28, 6, s == playing);
  menu.draw(oled, 16, 6, 50);
}

void setup() {
  Serial.begin(115200);

  static const char *const kNames[12] = {"C",  "C#", "D",  "D#", "E",  "F",
                                         "F#", "G",  "G#", "A",  "A#", "B"};
  for (int n = 0; n < kNotes; n++) {
    int midi = kLowNote + n;
    // MIDI 24 is C1: octave = note / 12 - 1.
    snprintf(noteText[n], sizeof(noteText[n]), "%s%d", kNames[midi % 12],
             midi / 12 - 1);
    noteNames[n] = noteText[n];
  }
  // A soft pluck: the pad of a finger, not a plectrum. (3 kHz made the
  // attack stand out too far above the ring.)
  pick.setCutoff(1500.0f);
  pick.setPosition(0.2f);
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
  if (kKnobs) {
    bool moved = false;
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

  // 2. Menu: hand it the encoder and the switch.
  push.update();
  if (menu.update(encoder.delta(), push.triggered()) == Menu::CHANGED)
    applyMenu();

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
    drawScreen();
  }
  if (haveScreen)
    oled.display(1);
}
