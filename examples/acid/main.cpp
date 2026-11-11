/**
 * @file main.cpp
 * @brief A 303-style acid bass line: a 16-step sequence with accents and
 *        slides through a resonant MS-20 or ladder filter, edited from a
 *        menu on a 128x64 OLED with a rotary encoder.
 *
 * THE VOICE (the TB-303's signal path):
 *
 *   saw/square ─► low-pass ─► VCA ─► overdrive ─► delay ─► spring ─► out
 *                  ▲    ▲              ▲
 *        cutoff ───┘    │              └── gate: on for half a step,
 *        + env mod × filter envelope       held through a slide
 *        + accent
 *
 * - The filter is an MS-20 (Ms20Filter) by default: its resonance
 *   keeps the bass. The Filter item switches to the Moog ladder
 *   (MoogFilter), whose bass thins as the resonance goes up.
 * - The filter envelope jumps up on every note and decays: that sweep
 *   is the "squelch". Env Mod sets how far it opens the filter; Decay
 *   how fast it closes.
 * - An accented step is louder and opens the filter further.
 * - A slide holds the gate into the next step and glides the pitch
 *   there, without re-striking the filter envelope: the 303's legato
 *   "wow".
 *
 * THE EFFECTS: a tempo-synced echo (3/16, the dotted eighth, by
 * default: each repeat lands off the beat), its repeats darkened by a
 * low-pass in the feedback path like a tape or analog delay; then the
 * spring reverb, which also spreads the mono voice into stereo.
 *
 * THE DRUMS: a kick on every beat and a snare on 2 and 4, kept dry (no
 * echo or spring) so they stay tight. The Drums item sets their level;
 * at 0 they're skipped altogether.
 *
 * WHY THE KICK AND SNARE RUN AT HALF RATE: in this patch they measured
 * 16% (kick) and 14% (snare) of the CPU at the full rate. A kick has
 * almost nothing above 5 kHz and a snare little above 10, so both are
 * computed every other sample (22 kHz) and the gaps filled by
 * straight-line interpolation, which roughly halves their cost.
 *
 * WHY NO HI-HAT: a HiHat (drums/hihat.h) took another 11%, which put
 * the patch at 85%. It ran, but loop() got so little time that the
 * encoder felt sluggish. A hat can't be halved like the others: its
 * sizzle is all high frequencies.
 *
 * THE SEQUENCE: notes come from a Colour, a five-note just scale with
 * 16 flavours from dark to bright (mod/colour.h). Randomize rolls a new
 * 16-step melody over two octaves, plus random accents, slides, rests
 * and octave jumps. Changing Colour keeps the melody and re-tunes it.
 *
 * THE MENU (encoder; Menu in io/menu.h does the work):
 *   turn          move between items
 *   press         start editing the item (its value shows as <value>)
 *   turn          change the value; press again to finish
 *   press on Randomize   roll a new sequence
 *
 * Wiring:
 *   Audio    PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN (UDA1334A)
 *   OLED     VCC -> 3V3, GND -> GND, SCL -> PB8, SDA -> PB9
 *   Encoder  A -> PB6, B -> PB7, C (middle pin) -> GND
 *   Push     one leg -> PB5, the other -> GND
 *   Knob     (optional) pot: outer legs to 3V3 and GND, wiper to PA1.
 *            Sets the cutoff; set kCutoffKnob = false without one, or
 *            the unconnected pin's noise will move the filter.
 *
 * Cost: 72% of the F411's CPU, measured on the board: 56% without the
 * drums (the spring alone is about 21%). 89 KB of RAM, 64 KB of it the
 * echo's buffer.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/random.h"
#include "buddhabox/drums/analog_kick.h"
#include "buddhabox/drums/synth_snare.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/filter/moog_filter.h"
#include "buddhabox/filter/ms20_filter.h"
#include "buddhabox/fx/delay.h"
#include "buddhabox/fx/overdrive.h"
#include "buddhabox/fx/spring_reverb.h"
#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/encoder_input.h"
#include "buddhabox/io/menu.h"
#include "buddhabox/io/oled.h"
#include "buddhabox/io/switch_input.h"
#include "buddhabox/mod/clock.h"
#include "buddhabox/mod/colour.h"
#include "buddhabox/mod/envelope.h"
#include "buddhabox/osc/basic.h"

using namespace bb;

// ── Settings ─────────────────────────────────────────────────────────
const float kRoot = 55.0f;         // A1: the bottom note of the sequence
const int kSteps = 16;             // one bar of sixteenth notes
const float kGlideSeconds = 0.06f; // a 303 slide takes about 60 ms
const bool kCutoffKnob = true;     // a pot on PA1 sets the cutoff
const int kFilterEvery = 8;        // samples between cutoff updates (see below)

// ── The menu ─────────────────────────────────────────────────────────
// Menu (io/menu.h) does the scrolling, editing and drawing; this file
// only lists the items. The voice reads their values in applyMenu().
enum Item {
  TEMPO,
  CUTOFF,
  RESONANCE,
  ENV_MOD,
  DECAY,
  ACCENT,
  DRIVE,
  WAVE,
  FILTER,
  COLOUR,
  DELAY_TIME,
  DELAY_MIX,
  FEEDBACK,
  REVERB,
  DRUMS,
  RANDOMIZE, // an action, not a value
  kItems
};

const char *const kWaves[] = {"Saw", "Square"};
const char *const kFilters[] = {"Ladder", "MS-20"};

MenuItem items[kItems] = {
    {"Tempo", 125, 60, 200, 1},
    {"Cutoff", 30, 0, 100, 2},
    {"Resonance", 75, 0, 100, 2},
    {"Env Mod", 60, 0, 100, 2},
    {"Decay", 40, 0, 100, 2},
    {"Accent", 60, 0, 100, 2},
    {"Drive", 30, 0, 100, 2},
    {"Wave", 0, 0, 1, 1, kWaves},
    {"Filter", 1, 0, 1, 1, kFilters},
    {"Colour", 6, 0, 15, 1},
    {"Delay Time", 3, 1, 6, 1, nullptr,
     "/16"}, // in sixteenths: 3 = dotted eighth
    {"Delay Mix", 30, 0, 100, 2},
    {"Feedback", 45, 0, 90, 2},
    {"Reverb", 25, 0, 100, 2},
    {"Drums", 70, 0, 100, 2},
    {"Randomize"}, // step 0: an action
};
Menu menu(items, kItems);

// ── Modules ──────────────────────────────────────────────────────────
BasicOscillator osc;
MoogFilter ladder;
Ms20Filter ms20;
bool useMs20 = true; // the Filter item
Envelope ampEnv, filterEnv;
Overdrive drive;
// 32768 samples (0.74 s) of 16-bit storage: 64 KB, half the F411's RAM.
// ponytail: echoes longer than 0.74 s are cut short (6/16 below 122
// BPM, 3/16 below 61); a float or longer buffer won't fit.
DelayLine<32768> echo;
SpringReverb<> spring;
AnalogKick kick;  // 808-style
SynthSnare snare; // 909-style: cheaper than AnalogSnare, and snappier
float drumLevel = 0.0f;
// Half-rate drums: the last two values computed, and which sample of
// each pair this is.
float slowPrevious = 0.0f, slowNow = 0.0f;
bool oddSample = false;
TempoClock tempo(125.0f, 4); // 4 ticks a beat: sixteenth notes
Colour colour;
LFSR rng(303);

EncoderInput encoder;
AnalogInput cutoffKnob(PA1, 0.3f); // quick: it's for wiggling
SwitchInput push(PB5);
Oled oled;
bool haveScreen = false;

// ── The sequence ─────────────────────────────────────────────────────
// Written by randomize() in loop() and read by the audio interrupt.
// Each entry is a single byte or bool, which the processor writes in
// one go, so the interrupt never sees half a value. At worst one step
// plays half old, half new, the moment you press Randomize.
uint8_t notes[kSteps]; // scale index: 0-4 first octave, 5-9 next...
bool accents[kSteps], slides[kSteps], rests[kSteps];

// ── Voice settings, from the menu ────────────────────────────────────
// Floats are written in one instruction too, so the same goes for
// these: loop() writes them, the audio interrupt reads them.
float cutoffSemis = 0.0f; // the knob's base cutoff, semitones over 40 Hz
float envModSemis = 0.0f; // how far the envelope opens it
float accentAmount = 0.0f;
float resonance = 0.0f;

// ── Playback state (audio interrupt only, except `step` for display)
volatile int step = kSteps - 1; // so the first tick plays step 0
uint32_t gateSamples = 0;       // samples left until the gate closes
bool accentNow = false;         // is the sounding note accented?
float pitchHz = kRoot;          // where the oscillator is now...
float targetHz = kRoot;         // ...and where it's heading
float glideCoefficient = 0.0f;
float echoSamples = 0.0f; // the delay time, from Tempo and Delay Time
float echoMix = 0.0f, echoFeedback = 0.0f;
float echoTone = 0.0f; // the feedback low-pass's state
int filterCounter = 0;

/**
 * Roll a new sequence: a Colour melody over two octaves, then accents,
 * slides, rests and octave jumps scattered over it.
 */
void randomize() {
  colour.newMelody(&rng, 9); // indices 0..9: two octaves of the scale
  for (int i = 0; i < kSteps; i++) {
    uint8_t n = colour.getMelody(i);
    // An octave jump now and then: the 303's yelping leaps.
    if (n < 5 && rng.nextFloat() < 0.2f)
      n += 5;
    notes[i] = n;
    accents[i] = rng.nextFloat() < 0.25f;
    slides[i] = rng.nextFloat() < 0.2f;
    // Step 0 always plays, so the bar has an anchor.
    rests[i] = i > 0 && rng.nextFloat() < 0.15f;
  }
}

/**
 * Turn the menu's numbers into the voice's settings. Called from loop()
 * after any change; uses library maths, so control rate only.
 */
void applyMenu() {
  tempo.setBPM((float)items[TEMPO].value);
  // Cutoff: 40 Hz to about 2.6 kHz over the knob (six octaves), before
  // the envelope adds its sweep on top.
  cutoffSemis = 0.72f * items[CUTOFF].value;
  // Env Mod: up to four more octaves at the envelope's peak.
  envModSemis = 0.48f * items[ENV_MOD].value;
  // Resonance stops short of 1.0, where both filters ring on their own.
  resonance = 0.0096f * items[RESONANCE].value;
  ladder.setResonance(resonance);
  ms20.setResonance(resonance);
  useMs20 = items[FILTER].value;
  // Decay: 0.1 to 2 seconds, squared so the short, snappy settings get
  // more of the range.
  float d = items[DECAY].value / 100.0f;
  filterEnv.setRelease(0.1f + 1.9f * d * d);
  accentAmount = items[ACCENT].value / 100.0f;
  drive.setDrive(items[DRIVE].value / 100.0f);
  osc.setShape(items[WAVE].value ? BasicOscillator::SQUARE
                                 : BasicOscillator::SAW);
  colour.setColour(COLOUR_BY_BRIGHTNESS[items[COLOUR].value]);
  // Echo: a number of sixteenth notes at the current tempo.
  float sixteenth = SAMPLE_RATE * 60.0f / (items[TEMPO].value * 4.0f);
  echoSamples = minf(sixteenth * items[DELAY_TIME].value, 32766.0f);
  echoMix = items[DELAY_MIX].value / 100.0f;
  echoFeedback = items[FEEDBACK].value / 100.0f;
  spring.setAmount(items[REVERB].value / 100.0f);
  drumLevel = items[DRUMS].value / 100.0f;
}

/**
 * Trigger this step's drums (audio interrupt). Steps are sixteenths,
 * so 0, 4, 8 and 12 are the beats.
 */
void playDrums(int s) {
  if (s % 4 == 0)
    kick.trigger();
  if (s == 4 || s == 12)
    snare.trigger();
}

/** Start the next step of the sequence (audio interrupt). */
void nextStep() {
  int s = (step + 1) % kSteps;
  step = s;
  int previous = (s + kSteps - 1) % kSteps;
  playDrums(s);
  // A slide on the previous step means its gate is still open: glide
  // into this note instead of striking it.
  bool sliding = slides[previous] && !rests[previous];

  // The gate stays open half a step, or the whole step if this one
  // slides into the next.
  float samplesPerStep = SAMPLE_RATE * 60.0f / (tempo.getBPM() * 4.0f);
  if (rests[s]) {
    ampEnv.gate(false);
    gateSamples = 0;
    return;
  }
  // A sliding gate runs a little past the next tick, so it can't close
  // a sample before the next step holds it open.
  gateSamples = slides[s] ? (uint32_t)samplesPerStep + 64
                          : (uint32_t)(samplesPerStep * 0.5f);
  targetHz = colour.getNote(0, notes[s]);

  if (!sliding) {
    // A fresh note: jump to the pitch, strike both envelopes.
    pitchHz = targetHz;
    accentNow = accents[s];
    ampEnv.gate(true);
    filterEnv.trigger();
  }
}

/** Fill one block of audio (interrupt context: no I2C, no Serial). */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    // 1. Sequencer: a tick starts a step; the gate closes on time.
    if (tempo.process())
      nextStep();
    if (gateSamples > 0 && --gateSamples == 0)
      ampEnv.gate(false);

    // 2. Pitch, gliding towards the target (a jump when not sliding,
    //    because nextStep() already set pitchHz = targetHz).
    pitchHz += glideCoefficient * (targetHz - pitchHz);
    osc.setFrequency(pitchHz);

    // 3. Envelopes.
    float amp = ampEnv.process();
    float fenv = filterEnv.process();
    float accent = accentNow ? accentAmount : 0.0f;

    // 4. Cutoff. setFrequency() uses expf() or tanf(), too slow for
    //    every sample, so it runs every kFilterEvery samples: 5.5 kHz,
    //    still far quicker than the fastest sweep. The cutoff adds up
    //    in semitones (base + envelope + accent), which matches how
    //    the ear hears a sweep.
    if (++filterCounter >= kFilterEvery) {
      filterCounter = 0;
      float semis = cutoffSemis + (envModSemis + 24.0f * accent) * fenv;
      float hz = 40.0f * semitonesToRatio(semis);
      if (useMs20)
        ms20.setFrequency(hz);
      else
        ladder.setFrequency(hz);
    }

    // 5. Oscillator, filter, VCA (an accent adds up to 80% level),
    //    then overdrive for grit.
    float x = osc.process();
    x = useMs20 ? ms20.process(x) : ladder.process(x);
    x *= amp * (0.6f + 0.8f * accent);
    x = drive.process(x);

    // 6. Echo. The repeats go back in through a low-pass, so each one
    //    is darker than the last. The voice is kept a little under full
    //    scale so the 16-bit buffer's clamp at ±1 is rarely reached.
    float wet = echo.read(echoSamples);
    echoTone += 0.35f * (wet - echoTone);
    echo.write(0.7f * x + echoFeedback * echoTone);
    x += echoMix * wet;

    // 7. Spring reverb: mono in, stereo out.
    float left = x, right = x;
    spring.process(left, right);

    // 8. Drums, added dry after the effects. Skipped at level 0.
    if (drumLevel > 0.0f) {
      // Kick and snare on even samples only. Output, per pair: the
      // midpoint of the last two values, then the newest one: a
      // straight line through them, a quarter of a millisecond late.
      if (!oddSample) {
        slowPrevious = slowNow;
        slowNow = kick.process() * 0.8f + snare.process() * 0.4f;
      }
      float slow = oddSample ? slowNow : 0.5f * (slowPrevious + slowNow);
      oddSample = !oddSample;
      left += drumLevel * slow;
      right += drumLevel * slow;
    }

    // 9. Out, through a soft limiter: bass and drums together can
    //    pass full scale, and this rounds the peaks off instead of
    //    clipping them flat.
    buffer[i] = (int16_t)(softLimitClip(left) * 30000.0f);
    buffer[i + 1] = (int16_t)(softLimitClip(right) * 30000.0f);
  }
}

/**
 * Draw one frame. The yellow top band (rows 0-15 on two-colour panels)
 * holds the title and the sequence; the blue rest holds the menu.
 */
void drawScreen() {
  // 1. Title bar: name and tempo, dark on lit.
  oled.rect(0, 0, 128, 8, true);
  int x = oled.text(2, 0, "ACID  ", false);
  x = oled.number(x, 0, (float)items[TEMPO].value, 0, false);
  oled.text(x, 0, " bpm ", false);

  // 2. The sequence: a filled box per accented note, an outline per
  //    plain one, a dot per rest, and a line under the playing step.
  oled.rect(0, 8, 128, 8, true, false);
  int playing = step;
  for (int i = 0; i < kSteps; i++) {
    int cx = i * 8;
    if (rests[i])
      oled.rect(cx + 3, 11, 2, 2, true);
    else
      oled.rect(cx + 1, 8, 6, 6, accents[i]);
    if (slides[i] && !rests[i])
      oled.rect(cx + 7, 11, 1, 1, true); // a tail: slides on
    if (i == playing)
      oled.rect(cx + 1, 15, 6, 1, true);
  }

  // 3. The menu: six rows under the yellow band.
  menu.draw(oled, 16, 6);
}

void setup() {
  Serial.begin(115200);

  // Voice: a short click-free attack on both envelopes; the amp
  // envelope closes quickly when the gate drops.
  ampEnv.setAttack(0.003f);
  ampEnv.setRelease(0.02f);
  filterEnv.setAttack(0.002f);
  // One-pole glide: the coefficient that covers most of the way to the
  // target in kGlideSeconds (library maths: once, here).
  kick.init(SAMPLE_RATE / 2); // half rate: see the file header
  snare.init(SAMPLE_RATE / 2);
  kick.setFrequency(48.0f);
  kick.setDecay(0.55f);
  snare.setFrequency(190.0f);
  snare.setSnappy(0.65f);
  spring.setTime(0.6f); // a medium splash, not a wash
  spring.setTone(0.5f);
  glideCoefficient = 1.0f - expf(-3.0f / (kGlideSeconds * SAMPLE_RATE));
  colour.setRoot(kRoot);
  applyMenu();
  randomize();

  encoder.begin();
  push.begin();
  haveScreen = oled.begin();

  AudioEngine::begin();
}

void loop() {
  // 1. Cutoff knob. Only a real move gets through its hysteresis, and
  //    then it takes over from the menu: the Cutoff item is set to
  //    match, so the screen shows where the knob is. The cutoff itself
  //    comes straight from the knob's reading, finer than the menu's
  //    whole-number steps, so a slow turn sweeps smoothly.
  if (kCutoffKnob) {
    static float lastKnob = -1.0f;
    cutoffKnob.update();
    float k = cutoffKnob.read();
    if (k != lastKnob) {
      lastKnob = k;
      items[CUTOFF].value = (int)(k * 100.0f + 0.5f);
      applyMenu();
      cutoffSemis = 72.0f * k;
    }
  }

  // 2. Menu: hand it the encoder and the switch; it says what changed.
  push.update();
  Menu::Event event = menu.update(encoder.delta(), push.triggered());
  if (event == Menu::CHANGED)
    applyMenu();
  else if (event == Menu::ACTION && menu.selected() == RANDOMIZE)
    randomize();

  // 3. CPU load over USB serial once a second (pio device monitor).
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.println("%");
  }

  // 4. Display at 20 frames a second.
  static uint32_t lastFrame = 0;
  if (haveScreen && millis() - lastFrame >= 50) {
    lastFrame = millis();
    drawScreen();
    oled.display();
  }
}
