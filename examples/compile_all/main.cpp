/**
 * @file main.cpp
 * @brief Build check: instantiates and runs every module once.
 *
 * Not a musical patch. Its job is to make the compiler and linker see
 * every class, so `pio run -e compile_all` proves the whole framework
 * builds for the Cortex-M4, and PlatformIO's RAM/flash report shows the
 * cost of everything together. Effects and physical models use small
 * template sizes so the lot fits in 64 KB of RAM.
 *
 * Each output is summed into a `volatile` variable: `volatile` tells the
 * compiler the value is observed from outside, so it cannot optimise the
 * DSP away as unused.
 */

#include <Arduino.h>

#include "buddhabox.h"

using namespace bb;

volatile float sink = 0.0f;

// ── Original BuddhaBox modules ──────────────────────────────────────────
Oscillator osc;
LFO lfo;
WhiteNoise white;
PinkNoise pink;
LFSR lfsr;
SVFilter svFilter;
MoogFilter moog;
AllpassFilter<512> allpass;
Envelope adsr;
SlewLimiter slewLimiter;
SampleAndHold sah;
TempoClock tempo;
LowPassGate lowPassGate;
DelayLine<1024> delayLine;
WaveFolder waveFolder;
KarplusStrong<512> karplus;
AnalogInput pot(PA0);
SwitchInput button(PB3);
UserLED led;

// ── Oscillators ─────────────────────────────────────────────────────────
SineOscillator sine;
BasicOscillator basic;
VariableShapeOscillator variableShape;
FormantOscillator formant;
VosimOscillator vosim;
GrainletOscillator grainlet;
ZOscillator zOsc;
HarmonicOscillator harmonic;
StringSynthOscillator stringSynth;

// ── Noise ───────────────────────────────────────────────────────────────
Dust dust;
Particle particle;
ClockedNoise clockedNoise;
SmoothRandom smoothRandom;

// ── Filters ─────────────────────────────────────────────────────────────
Svf svf;
OnePole onePoleFilter;
DcBlocker dcBlocker;
VowelFilter vowel;
Hilbert hilbert;

// ── Modulation ──────────────────────────────────────────────────────────
Lorenz lorenz;
BouncingBall ball;
RandomSequence randomSequence;
Quantizer quantizer;
HysteresisQuantizer hysteresis;
Colour colour;
ClockToRamp clockToRamp;
SegmentEnvelope segmentEnvelope;
Lag lag;
Slope slopeGen;

// ── Dynamics ────────────────────────────────────────────────────────────
Vactrol vactrol;
VactrolLpg vactrolLpg;
Follower follower;
Compressor compressor;

// ── Effects ─────────────────────────────────────────────────────────────
Reverb<4096> reverb;
Chorus<2048> chorus;
Ensemble<4096> ensemble;
Diffuser<2048> diffuser;
PitchShifter<2048> pitchShifter;
Decimator decimator;
Limiter limiter;
Overdrive overdrive;
TableWavefolder tableWavefolder;

// ── Physical models ─────────────────────────────────────────────────────
ModalResonator<64> modal;
StringModel<512> stringModel;
Plucker plucker;
Tube<512> tube;

// ── Drums ───────────────────────────────────────────────────────────────
AnalogKick analogKick;
AnalogSnare analogSnare;
HiHat hiHat;
SynthKick synthKick;
SynthSnare synthSnare;
FmDrum fmDrum;

// ── Spectral ────────────────────────────────────────────────────────────
FmOperators fm;

/**
 * Run every module for one sample and sum the results.
 * @param buffer  Interleaved stereo int16 samples
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    float in = white.process() * 0.1f;
    float acc = 0.0f;

    acc += osc.process() + lfo.process() + pink.process() + lfsr.nextFloat();
    svFilter.process(in);
    acc += svFilter.lowpass() + moog.process(in) + allpass.process(in);
    acc += adsr.process() + slewLimiter.process(in) + sah.process(in, true);
    acc += tempo.process() ? 1.0f : 0.0f;
    acc += lowPassGate.process(in) + waveFolder.process(in);
    delayLine.write(in);
    acc += delayLine.read(100.5f) + karplus.process();

    acc += sine.process() + basic.process() + variableShape.process();
    acc += formant.process() + vosim.process() + grainlet.process();
    acc += zOsc.process() + harmonic.process() + stringSynth.process();

    acc += dust.process() + particle.process() + clockedNoise.process();
    acc += smoothRandom.process();

    acc += svf.process(in).bp + onePoleFilter.process(in).lp;
    acc += dcBlocker.process(in) + vowel.process(in) + hilbert.process(in);

    acc += lorenz.process() + ball.process(false) + randomSequence.next();
    acc += quantizer.process(in, 0.5f) + hysteresis.process(in);
    acc += colour.getNote(0, colour.getMelody(i)) * 0.001f;
    acc += clockToRamp.process(false) + segmentEnvelope.process(false);
    acc += lag.process(in, 0.5f, 0.5f) + slopeGen.process(false);

    acc += vactrol.process(in) + vactrolLpg.process(in);
    acc += follower.process(in) + compressor.process(in);

    float l = in, r = in;
    reverb.process(l, r);
    chorus.process(l, r);
    ensemble.process(l, r);
    diffuser.process(l, r);
    acc += l + r + pitchShifter.process(in) + decimator.process(in);
    acc += limiter.process(in) + overdrive.process(in);
    acc += tableWavefolder.process(in);

    acc += modal.process(in, 0.5f) + stringModel.process(in);
    acc += plucker.process() + tube.process(in, 0.5f);

    acc += analogKick.process() + analogSnare.process() + hiHat.process();
    acc += synthKick.process() + synthSnare.process() + fmDrum.process();

    acc += fm.process() + Xmod::morph(0.5f, in, acc, 0.5f);

    sink = acc;
    buffer[i] = buffer[i + 1] = (int16_t)(clampf(acc, -1.0f, 1.0f) * 32000.0f);
  }
}

void setup() {
  modal.setBowedModes(true);
  colour.setColour(COLOUR_BY_BRIGHTNESS[8]);
  colour.newMelody(&lfsr, 7);
  AudioEngine::begin();
}

void loop() {
  pot.update();
  button.update();
  led.update();
}
