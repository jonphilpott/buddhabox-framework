/**
 * @file main.cpp
 * @brief A tanpura resynthesised from a recording: each string replays
 *        its measured harmonics, frame by frame, from pluck to pluck.
 *
 * examples/tanpura models the strings' physics. This one doesn't model
 * anything: tools/analyse_tanpura.cpp measured a real tanpura, from a
 * vintage tape of raga Malkauns, with FFTs, and spectra.h holds the
 * result. For each of the three kinds of string (low sa, the two SAs and
 * the first string) it has 16 frames of 32 harmonic levels, spread over
 * the time from one pluck to the next.
 *
 * FROM SPECTRA TO WAVETABLES: the tool also turns each frame's 32
 * harmonic levels into one cycle of the waveform they make (an inverse
 * Fourier transform, done once on the computer), so spectra.h holds 48
 * single-cycle waveforms. Playing 32 harmonics live with
 * HarmonicOscillator measured about 500 cycles a sample per string on
 * the board, too much for four strings and a reverb; reading two
 * waveforms and blending them is a few dozen.
 *
 * HOW IT PLAYS: a pluck starts a string at frame 0; every 32 samples it
 * moves along the frames, blending between neighbours, so the frames
 * last exactly one cycle of the pattern whatever the tempo. On the tape
 * a cycle took about 2.1 seconds. Slower tempos stretch the same
 * evolution out.
 *
 * WHAT THE RECORDING SHOWED:
 * - The fundamentals hardly change from pluck to pluck: a tanpura rings
 *   until it is plucked again.
 * - The pluck refreshes the upper harmonics. On the low sa the jawari
 *   then blooms: the share of its power above the 8th harmonic climbs
 *   from a third to nearly three-quarters over 0.4 s, and fades to under
 *   a tenth by the next pluck. The SAs bloom less, about 0.65 s after
 *   their pluck.
 * - The recording's first string was ma (raga Malkauns has no Pa). Its
 *   spectrum is used for pa, ni and dha too.
 *
 * THE CYCLE, as in examples/tanpura:
 *   5 beats   pa  SA  SA  sa  -
 *   6 beats   pa  -   SA  SA  sa  -
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
 *              PA1 Volume   PA2 Tempo    PA3 Bright   PA4 Fine
 *              PA5 Reverb   PA6 Spread   PA7 Space    PB0 Tone
 *            Set kKnobs = false without them. (PA0 is skipped: the
 *            board's KEY button is on it.)
 *
 * Cost: 54% of the F411's CPU, measured on the board (the reverb is
 * about 27%). 42 KB of RAM; the waveforms take 49 KB of flash.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/midi_utils.h"
#include "buddhabox/engine/audio_engine.h"

#include "buddhabox/fx/reverb.h"

#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/encoder_input.h"
#include "buddhabox/io/menu.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/io/switch_input.h"

#include "spectra.h"

using namespace bb;

// ── Settings ─────────────────────────────────────────────────────────
const bool kKnobs = true; // pots on PA1-PA7 and PB0 (see Wiring)
// Samples between steps along the frames.
const int kControlEvery = 32;
// The four strings into the reverb. Set on the board with the default
// settings: peaks of about 0.6-0.75 at full Volume.
const float kMixGain = 0.62f;
// The two SA strings are never exactly in tune with each other; this
// many Hz apart they beat slowly. Identical, they'd be one string.
const float kSaBeatHz = 0.25f;

/**
 * One string, played back from its measured waveforms.
 *
 * A wavetable oscillator with two tables at once: frame f and frame
 * f + 1, blended by how far the string has moved between them.
 *
 * Example:
 *   SpectralString s;
 *   s.setFrequency(146.8f);
 *   s.pluck(kWaves[1]);                     // an SA string
 *   s.step(16.0f * 32 / cycleSamples);      // every 32 samples
 *   float out = s.process();
 */
class SpectralString {
public:
  /** @param hz  Pitch in Hz (one float write: safe from loop()) */
  void setFrequency(float hz) { increment_ = hz / SAMPLE_RATE; }

  /**
   * Start from the pluck frame.
   * @param frames  kWaves[string]: 16 frames of one cycle each
   */
  void pluck(const float (*frames)[kWaveSize + 1]) {
    frames_ = frames;
    pos_ = 0.0f;
    step(0.0f);
  }

  /**
   * Move along the frames. Control rate.
   * @param frames  How many frames to advance (16 per cycle)
   */
  void step(float frames) {
    // Hold the last frame if the next pluck is late (Tempo just slowed).
    const float last = kWaveFrames - 1;
    pos_ = minf(pos_ + frames, last);
    frame_ = (int)pos_;
    next_ = frame_ < last ? frame_ + 1 : frame_;
    blend_ = pos_ - frame_;
  }

  /** @return  The next sample (silent until the first pluck) */
  float process() {
    if (!frames_)
      return 0.0f;
    phase_ += increment_;
    if (phase_ >= 1.0f)
      phase_ -= 1.0f;
    // Position in the table, and the fraction between two samples.
    float x = phase_ * kWaveSize;
    int i = (int)x;
    float t = x - i;
    const float *a = frames_[frame_], *b = frames_[next_];
    float sa = a[i] + t * (a[i + 1] - a[i]); // sample 256 repeats 0
    float sb = b[i] + t * (b[i + 1] - b[i]);
    return sa + blend_ * (sb - sa);
  }

private:
  const float (*frames_)[kWaveSize + 1] = nullptr;
  float increment_ = 0.0f, phase_ = 0.0f, pos_ = 0.0f, blend_ = 0.0f;
  int frame_ = 0, next_ = 0;
};

// ── The menu ─────────────────────────────────────────────────────────
enum Item {
  VOLUME,
  ROOT,
  FINE,
  TUNING,
  PATTERN,
  TEMPO,
  BRIGHT,
  SPREAD,
  SPACE,
  TONE,
  REVERB,
  kItems
};

// Tunings: the first string's note, as a ratio to SA (see
// examples/tanpura); the others are always SA, SA and sa.
const char *const kTunings[] = {"pa SA SA sa", "ma SA SA sa", "ni SA SA sa",
                                "dha SA SA sa"};
const float kFirstString[] = {3.0f / 4, 2.0f / 3, 15.0f / 16, 5.0f / 6};

// Patterns: which string each beat plucks; -1 is a rest.
const char *const kPatternNames[] = {"5 beat", "6 beat"};
const int kPatternLength[] = {5, 6};
const int8_t kPatterns[2][6] = {
    {0, 1, 2, 3, -1, -1}, // pa SA SA sa -
    {0, -1, 1, 2, 3, -1}, // pa - SA SA sa -
};
// Which measured spectrum each string plays: first, SA, SA, sa.
const int kSpectrumOf[4] = {2, 1, 1, 0};

// Root: SA, C2 to C4 by name.
const int kLowNote = NOTE_C1 + 12, kHighNote = NOTE_C1 + 36; // C2-C4
const int kNotes = kHighNote - kLowNote + 1;                 // 25
char noteText[kNotes][4];
const char *noteNames[kNotes];

MenuItem items[kItems] = {
    {"Volume", 80, 0, 100, 2},
    {"Root", NOTE_C1 + 26, kLowNote, kHighNote, 1, noteNames}, // D3
    {"Fine", 0, -50, 50, 1, nullptr, " ct"},
    {"Tuning", 0, 0, 3, 1, kTunings},
    {"Pattern", 0, 0, 1, 1, kPatternNames},
    // The tape's tanpura ran at about 140 (a cycle every 2.1 s).
    {"Tempo", 48, 20, 160, 1, nullptr, " bpm"},
    {"Bright", 50, 0, 100, 2}, // 50 = as recorded (see applyMenu())
    {"Spread", 50, 0, 100, 2},
    {"Space", 70, 0, 100, 2}, // reverb decay
    {"Tone", 60, 0, 100, 2},  // reverb damping: 100 = brightest
    {"Reverb", 40, 0, 100, 2},
};
Menu menu(items, kItems);

// ── The knobs ────────────────────────────────────────────────────────
struct Knob {
  AnalogInput input;
  int item;           // which menu item it sets
  float last = -1.0f; // its last reading; -1 = never read
};

Knob knobs[] = {
    {AnalogInput(PA1, 0.7f), VOLUME}, {AnalogInput(PA2, 0.7f), TEMPO},
    {AnalogInput(PA3, 0.7f), BRIGHT}, {AnalogInput(PA4, 0.7f), FINE},
    {AnalogInput(PA5, 0.7f), REVERB}, {AnalogInput(PA6, 0.7f), SPREAD},
    {AnalogInput(PA7, 0.7f), SPACE},  {AnalogInput(PB0, 0.7f), TONE},
};

// ── Modules ──────────────────────────────────────────────────────────
#define NUM_STRINGS (4)
SpectralString strings[NUM_STRINGS];
Reverb<> reverb;

EncoderInput encoder;
SwitchInput push(PB5);
Oled oled;
bool haveScreen = false;

// ── Patch settings, from the menu ────────────────────────────────────
float samplesPerBeat = SAMPLE_RATE;
int pattern = 0;
float volume = 0.8f;
// Bright: a tilt filter on the mix. A one-pole low-pass splits each
// channel at about 1.5 kHz; the output is the lows plus the highs
// times highGain (0 = dark, 1 = as recorded, 2 = about +6 dB above).
float highGain = 1.0f;
const float kSplit = 0.19f; // one-pole coefficient: about 1.5 kHz
float lowL = 0.0f, lowR = 0.0f;
// Spread: pa hard left, sa hard right, the SAs in the middle.
const float kPan[NUM_STRINGS] = {-1.0f, 0.0f, 0.0f, 1.0f};
float panL[NUM_STRINGS] = {1.0f, 1.0f, 1.0f, 1.0f};
float panR[NUM_STRINGS] = {1.0f, 1.0f, 1.0f, 1.0f};

// ── Audio-interrupt state ────────────────────────────────────────────
float beatCounter = 0.0f; // samples until the next beat
int beat = -1;            // position in the pattern
int controlCounter = 0;
volatile int lastPlucked = -1; // for the screen

/**
 * Read every menu value into the patch settings. Called from loop()
 * after any change; uses library maths (powf, sqrtf), so control rate
 * only.
 */
void applyMenu() {
  // 1. Pitches.
  float sa = midiToFreq((uint8_t)items[ROOT].value) *
             semitonesToRatio(items[FINE].value / 100.0f);
  float hz[NUM_STRINGS] = {sa * kFirstString[items[TUNING].value], sa,
                           sa + kSaBeatHz, sa * 0.5f};
  for (int i = 0; i < NUM_STRINGS; i++)
    strings[i].setFrequency(hz[i]);

  // 2. Bright: 0 to 100 sets highGain from 0 to 2.
  highGain = items[BRIGHT].value / 50.0f;

  // 3. Rhythm, spread and reverb, as in examples/tanpura.
  pattern = items[PATTERN].value;
  samplesPerBeat = 60.0f * SAMPLE_RATE / items[TEMPO].value;
  for (int i = 0; i < NUM_STRINGS; i++) {
    float p = kPan[i] * items[SPREAD].value / 100.0f;
    panL[i] = sqrtf(1.0f - p);
    panR[i] = sqrtf(1.0f + p);
  }
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
    // 1. The beat clock: on each beat, pluck that beat's string.
    beatCounter -= 1.0f;
    if (beatCounter <= 0.0f) {
      beatCounter += samplesPerBeat;
      int p = pattern;
      beat = (beat + 1) % kPatternLength[p];
      int s = kPatterns[p][beat];
      if (s >= 0) {
        strings[s].pluck(kWaves[kSpectrumOf[s]]);
        lastPlucked = s;
      }
    }

    // 2. Every kControlEvery samples, move each string along its
    //    frames: 16 frames per cycle of the pattern.
    if (controlCounter == 0) {
      controlCounter = kControlEvery;
      float cycle = samplesPerBeat * kPatternLength[pattern];
      float frames = (float)kWaveFrames * kControlEvery / cycle;
      for (int s = 0; s < NUM_STRINGS; s++)
        strings[s].step(frames);
    }
    controlCounter--;

    // 3. The strings, panned.
    float left = 0.0f, right = 0.0f;
    for (int s = 0; s < NUM_STRINGS; s++) {
      float out = strings[s].process();
      left += out * panL[s];
      right += out * panR[s];
    }
    left *= kMixGain;
    right *= kMixGain;
    // Bright: lows plus highGain times the highs.
    lowL += kSplit * (left - lowL);
    lowR += kSplit * (right - lowR);
    left = lowL + highGain * (left - lowL);
    right = lowR + highGain * (right - lowR);

    // 4. Reverb, then a soft clip and Volume.
    reverb.process(left, right);
    float gain = volume * 32000.0f;
    buffer[i] = (int16_t)(softLimitClip(left) * gain);
    buffer[i + 1] = (int16_t)(softLimitClip(right) * gain);
  }
}

/** Title bar, the four strings (the one just plucked filled), menu. */
void drawScreen() {
  oled.rect(0, 0, 128, 8, true);
  int x = oled.text(2, 0, "TANPURA FFT ", false);
  oled.text(x, 0, noteNames[items[ROOT].value - kLowNote], false);
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
  // 1. Knobs: a real move sets its menu item (see examples/tanpura).
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

  // 2. Menu.
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

  // 4. Display: 20 frames a second, one page per loop().
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    drawScreen();
  }
  if (haveScreen)
    oled.display(1);
}
