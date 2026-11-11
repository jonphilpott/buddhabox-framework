/**
 * @file main.cpp
 * @brief An aeolian harp in D: wind-blown strings singing their harmonics.
 *
 * An aeolian harp is a box of strings, all tuned to the same note, left in
 * the wind. Nobody plucks it. Air flowing past a string sheds little
 * whirlpools (vortices) from alternate sides, so the string is pushed back
 * and forth at the "shedding frequency", which rises with wind speed and
 * falls with string thickness. When that frequency lands near one of the
 * string's harmonics (2, 3, 4 ... times the fundamental), that harmonic
 * locks on and sings. So the harp rarely sounds its fundamental: it plays
 * the harmonic series of D, climbing as gusts build and falling as they
 * die, with a breathy, whistling tone.
 *
 * HOW THIS PATCH MODELS IT:
 *
 *   SmoothRandom x2 ─► wind speed ─► which harmonics sing (control rate)
 *                                          │ gains
 *   WhiteNoise ─► 16 band-passes at D2 x 1..16 ─► pan ─┐
 *   PinkNoise ─► band-pass (air rush) ─────────────────┼─► Reverb ─► Limiter
 *
 * 1. Wind: two SmoothRandoms, a slow drift (a new target every 20 s) and
 *    gusts (one every 2.5 s or so), set the wind speed, 0 to 1.
 * 2. Strings: one band-pass filter per harmonic of D2 (73.4 Hz), up to
 *    the 16th (1175 Hz, D6). Fed white noise, a very narrow band-pass rings
 *    like a string being blown: a pure tone whose level wavers. Every
 *    filter is 0.25 Hz wide, so each tone also swells slowly (about 1.3 s),
 *    as a real string takes time to lock on.
 * 3. Lock-in: two strings of different thickness each have a "singing
 *    point" on the harmonic series that slides up with the wind: the thick
 *    string from harmonic 2 to 8, the thin one from 5 to 14. Harmonics near
 *    a singing point get louder, a few at a time. Done once per block.
 * 4. Air: pink noise through a soft band-pass, rising with the wind.
 * 5. Space: a long reverb, then a limiter so gusts can't clip.
 *
 * Cost: about 50% of the F411's CPU (16 filters are ~400 cycles, the
 * reverb ~510). Generative: no controls, just wire up the DAC.
 *
 * Wiring: PB12 -> LRCLK/WSEL, PB13 -> BCLK, PB15 -> DIN on the UDA1334A.
 */

#include <Arduino.h>

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/reverb.h"
#include "buddhabox/noise/noise.h"
#include "buddhabox/noise/smooth_random.h"

using namespace bb;

// ── Settings ─────────────────────────────────────────────────────────

constexpr float kRoot = 73.416f; // D2: every string is tuned to this
constexpr int kHarmonics = 16;   // up to D6

// Width of each string's band-pass, in Hz. The same for every harmonic,
// which means a higher Q for higher ones (Q = frequency / width). Why not
// one Q for all: a band-pass's skirts leak white noise from far away, and
// summed over 16 filters with the large gain below, that leak is a hiss
// that sounds like wind. At Q 200 the top harmonics are 6 Hz wide and the
// hiss sits only 26 dB under the tones; at a constant 0.25 Hz it is 40 dB
// under. Narrower is purer but swells more slowly (about 1 / (pi * width)
// seconds) and gains little: 0.15 Hz measured only 2 dB better.
constexpr float kBandwidth = 0.25f;

// Each string's singing point slides along the harmonic series between
// these two harmonic numbers as the wind goes from still to full.
struct StringRange {
  float low, high;
};
constexpr StringRange kStrings[] = {{2.0f, 8.0f}, {5.0f, 14.0f}};

// How many harmonics either side of a singing point join in.
constexpr float kLockWidth = 1.3f;

// Output level of the harmonics. Large, because a band-pass this narrow
// passes only a sliver of the noise (about 0.002 RMS). Set from a
// two-minute host simulation: loud gusts peak near 0.55 at the output,
// lulls sit about 30 dB lower.
constexpr float kHarmonicGain = 200.0f;

// ── Modules ──────────────────────────────────────────────────────────

WhiteNoise wind(12345);
PinkNoise air(678);
Svf strings[kHarmonics];
Svf airFilter;
SmoothRandom drift, gusts;
Reverb<> reverb; // 32 KB
Limiter limiter;

// Per-harmonic state, updated once per block.
float gain[kHarmonics];     // current level
float gainStep[kHarmonics]; // added each sample to glide to the next level
float panLeft[kHarmonics], panRight[kHarmonics];
float airLevel = 0.0f, airStep = 0.0f;

/**
 * A smooth bump: 1 at d = 0, falling to 0 at d = ±1, and 0 beyond.
 * (1 - d²)² has no corners, so harmonics fade in and out smoothly as a
 * singing point slides past them. Cheaper than a Gaussian (no expf).
 */
inline float bump(float d) {
  float x = 1.0f - d * d;
  return x > 0.0f ? x * x : 0.0f;
}

/**
 * Control rate, once per block: read the wind and work out how loud each
 * harmonic should be by the end of the block. The audio loop then glides
 * to those levels a little each sample, so there is no stepping.
 *
 * @param frames  Samples in the block, to spread each change across.
 */
void updateWind(int frames) {
  // 1. Wind speed, 0..1: a slow drift plus quicker gusts on top.
  float windSpeed = clampf(
      0.5f + 0.4f * drift.process() + 0.25f * gusts.process(), 0.0f, 1.0f);
  // Louder with more wind; squared so light air barely whispers.
  float loudness = windSpeed * windSpeed;

  // 2. Each harmonic's level: the sum of its closeness to each string's
  //    singing point.
  for (int n = 0; n < kHarmonics; n++) {
    float harmonic = (float)(n + 1);
    float lock = 0.0f;
    for (const StringRange &s : kStrings) {
      float point = s.low + windSpeed * (s.high - s.low);
      lock += bump((harmonic - point) / kLockWidth);
    }
    // Every filter has the same width, so each passes the same noise
    // power: one gain suits all the harmonics.
    float target = kHarmonicGain * lock * loudness;
    gainStep[n] = (target - gain[n]) / frames;
  }

  // 3. The rush of air, quieter than the strings.
  airStep = (0.005f * loudness - airLevel) / frames;
}

void audioCallback(int16_t *buffer, uint16_t length) {
  int frames = length / 2;
  updateWind(frames);

  for (uint16_t i = 0; i < length; i += 2) {
    // All strings hear the same wind noise; each filter keeps only its
    // own harmonic.
    float excite = wind.process();
    float left = 0.0f, right = 0.0f;
    for (int n = 0; n < kHarmonics; n++) {
      gain[n] += gainStep[n];
      // bp * r(): the normalised band-pass, unity gain at its peak.
      float s = strings[n].process(excite).bp * strings[n].r() * gain[n];
      left += s * panLeft[n];
      right += s * panRight[n];
    }

    airLevel += airStep;
    float rush = airFilter.process(air.process()).bp * airLevel;
    left += rush;
    right += rush;

    reverb.process(left, right);
    limiter.process(left, right);
    buffer[i] = (int16_t)(clampf(left, -1.0f, 1.0f) * 32000.0f);
    buffer[i + 1] = (int16_t)(clampf(right, -1.0f, 1.0f) * 32000.0f);
  }
}

void setup() {
  for (int n = 0; n < kHarmonics; n++) {
    float harmonic = (float)(n + 1);
    // Real strings are slightly stiff, which stretches their upper
    // harmonics a little sharp. A touch of that (factor B) keeps the
    // chord from sounding like a perfect organ.
    constexpr float B = 0.00015f;
    float hz = kRoot * harmonic * sqrtf(1.0f + B * harmonic * harmonic);
    strings[n].setFrequency(hz, hz / kBandwidth);

    // Spread harmonics across the stereo field: odd ones lean left, even
    // ones right, and the lowest few stay near the centre.
    float spread = minf(0.1f * harmonic, 0.6f);
    float pan = 0.5f + ((n & 1) ? spread : -spread) * 0.5f;
    panLeft[n] = 1.0f - pan;
    panRight[n] = pan;
    gain[n] = gainStep[n] = 0.0f;
  }
  airFilter.setFrequency(500.0f, 0.7f);

  // The wind is read once per block, not once per sample, so tell the
  // SmoothRandoms they run at the block rate (44100 / 256 = 172 Hz);
  // otherwise their frequencies would come out 256 times too slow.
  const float blockRate = (float)SAMPLE_RATE / AUDIO_BLOCK_SIZE;
  drift.init(blockRate);
  gusts.init(blockRate);
  drift.seed(1); // different seeds, or the two would move in step
  gusts.seed(2);
  drift.setFrequency(0.05f); // a new wind level every 20 seconds
  gusts.setFrequency(0.4f);  // a gust every 2.5 seconds or so

  reverb.setTime(0.92f);
  reverb.setAmount(0.4f);
  reverb.setLp(0.5f);

  Serial.begin(115200); // USB serial, for the CPU meter below
  AudioEngine::begin();
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU: ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.println("%");
  }
}
