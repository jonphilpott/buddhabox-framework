/**
 * @file main.cpp
 * @brief Profiles every audio module and prints the results as CSV, for
 *        tools (such as a patch editor) to read.
 *
 * WHY THIS EXISTS: the `profile` example times the modules in
 * singing_bowls and predicts that patch's load. This one times every
 * audio module in the framework, so a tool can add up the cost of any
 * patch before it is flashed. Like `profile`, it never starts the audio
 * engine.
 *
 * HOW IT MEASURES (three refinements over `profile`):
 * 1. Interrupts are off while timing, so a USB interrupt landing in the
 *    loop can't inflate a figure.
 * 2. Each module runs kTrials times and the fastest run is kept. Slower
 *    runs were disturbed by something else, so the minimum is truest.
 * 3. The cost of the empty loop (counter, input read, sum) is measured
 *    first and subtracted, leaving only the module's own cost.
 *
 * Every module gets a varied input (white noise) and is kept sounding:
 * drums and plucks are re-struck and envelopes re-gated every 2048
 * samples. A silent voice often takes a cheaper path, and a patch must
 * be budgeted for its busiest moment.
 *
 * OUTPUT, one line per module, repeated every 10 seconds:
 *   module,header,cycles_per_sample,ram_bytes,pct_of_budget
 * `header` is the path under src/buddhabox, so the rows can be matched
 * to the classes tools/gen_docs.py reads. `ram_bytes` is sizeof() at the
 * template size shown in `module`. Template sizes are small so that
 * everything fits in 64 KB at once; buffer size changes RAM, not cycles.
 *
 * CAVEAT: a tight loop stays in the flash accelerator's cache, so these
 * figures are slightly optimistic for a full patch. Budget about 80% of
 * the 1,905 cycles (about 1,500).
 *
 * Usage: pio run -e profile_all -t upload, then pio device monitor.
 */

#include <Arduino.h>

#include "buddhabox.h"

using namespace bb;

// Samples per trial, and trials per module (the fastest is kept).
const int kSamples = 4096;
const int kTrials = 5;

// White noise in [-1, 1]. 256 entries so the index wraps with a cheap
// mask (n & 255) instead of a division.
float noise[256];

// Results go here so the compiler can't remove the work as unused:
// `volatile` means "something outside the program reads this".
volatile float sink;

// Fastest total from the last TIME(), and the empty loop's cost.
uint32_t best = 0;
uint32_t overhead = 0;

/** Cycles available per sample: CPU clock / sample rate. */
float budget() { return (float)SystemCoreClock / SAMPLE_RATE; }

/**
 * Time some code run kSamples times, kTrials times over, and leave the
 * fastest total in `best`.
 *
 * A macro rather than a function so the code is pasted inline and timed
 * exactly as a patch would run it. Inside the code, `in` is the input
 * sample, `n` the sample index and `acc` the running sum to add results
 * to. `...` and __VA_ARGS__ let the code contain commas.
 *
 * WHY THE LAMBDA: each use becomes its own small function instead of
 * more code in loop(). GCC stops inlining into a function once it grows
 * past a size limit, and with ~80 rows pasted in, loop() passed it: big
 * modules like Reverb were called rather than inlined, and timed up to
 * 2.5x slower than in a real audioCallback(). `noinline` stops GCC from
 * folding the lambda back into loop(), which would undo the fix.
 *
 * DWT->CYCCNT is the Cortex-M4's cycle counter. The subtraction is
 * unsigned, so it is correct even if the counter wraps mid-run.
 */
#define TIME(...)                                                              \
  []() __attribute__((noinline)) {                                             \
    best = UINT32_MAX;                                                         \
    for (int t = 0; t < kTrials; ++t) {                                        \
      float acc = 0.0f;                                                        \
      __disable_irq();                                                         \
      uint32_t start = DWT->CYCCNT;                                            \
      for (int n = 0; n < kSamples; ++n) {                                     \
        float in = noise[n & 255];                                             \
        (void)in; /* not every module takes an input */                        \
        __VA_ARGS__;                                                           \
      }                                                                        \
      uint32_t cycles = DWT->CYCCNT - start;                                   \
      __enable_irq();                                                          \
      sink = acc;                                                              \
      if (cycles < best)                                                       \
        best = cycles;                                                         \
    }                                                                          \
  }()

/**
 * Time some code and print a CSV row.
 * @param name    Module label, with template size if any
 * @param header  Path under src/buddhabox
 * @param bytes   RAM used: sizeof() the object, 0 for free functions
 */
#define PROFILE(name, header, bytes, ...)                                      \
  do {                                                                         \
    TIME(__VA_ARGS__);                                                         \
    report(name, header, bytes);                                               \
  } while (0)

/**
 * Print one CSV row from `best`, less the loop overhead.
 * @param name    Module label
 * @param header  Path under src/buddhabox
 * @param bytes   RAM used by the object
 */
void report(const char *name, const char *header, size_t bytes) {
  uint32_t net = best > overhead ? best - overhead : 0;
  float perSample = (float)net / kSamples;
  Serial.print(name);
  Serial.print(',');
  Serial.print(header);
  Serial.print(',');
  Serial.print(perSample, 1);
  Serial.print(',');
  Serial.print(bytes);
  Serial.print(',');
  Serial.println(100.0f * perSample / budget(), 2);
}

/**
 * Time a setter and print a CSV row.
 *
 * A setter only writes member variables, and nothing reads them inside
 * the timing loop. The compiler notices that only the last of the 4096
 * writes matters and deletes the rest, so a cheap setter would time as
 * free. The empty `asm volatile` with a "memory" clobber is a compiler
 * barrier: it emits no instructions, but tells the compiler that memory
 * may be read here, so every write must really happen.
 *
 * @param name    Setter label
 * @param header  Path under src/buddhabox
 */
#define SETTER(name, header, ...)                                              \
  PROFILE(name, header, 0, {                                                   \
    __VA_ARGS__;                                                               \
    asm volatile("" ::: "memory");                                             \
  })

// True for the first half of every 2048 samples: a gate held for 23 ms.
#define GATE ((n & 2047) < 1024)
// True once every 2048 samples: a trigger.
#define STRIKE ((n & 2047) == 0)

// ── Oscillators ─────────────────────────────────────────────────────────
Oscillator osc;
LFO lfo;
SineOscillator sine, quad;
BasicOscillator basic;
VariableShapeOscillator variableShape;
FormantOscillator formant;
VosimOscillator vosim;
GrainletOscillator grainlet;
ZOscillator zOsc;
HarmonicOscillator harmonic;
StringSynthOscillator stringSynth;

// ── Noise ───────────────────────────────────────────────────────────────
WhiteNoise white;
PinkNoise pink;
LFSR lfsr;
Dust dust;
Particle particle;
ClockedNoise clockedNoise;
SmoothRandom smoothRandom;

// ── Filters ─────────────────────────────────────────────────────────────
SVFilter svFilter;
Svf svf;
MoogFilter moog;
OnePole onePoleFilter;
DcBlocker dcBlocker;
VowelFilter vowel;
Hilbert hilbert;
AllpassFilter<512> allpass;

// ── Modulation ──────────────────────────────────────────────────────────
Envelope adsr;
SegmentEnvelope segmentEnvelope;
Slope slopeGen;
SlewLimiter slewLimiter;
Lag lag;
SampleAndHold sah;
TempoClock tempo;
ClockToRamp clockToRamp;
Lorenz lorenz;

// ── Utilities ───────────────────────────────────────────────────────────
Range rangeLin, rangeExp;
Mixer mixer;
Multiply multiply;
Param param;
BouncingBall ball;
RandomSequence randomSequence;
Quantizer quantizer;
HysteresisQuantizer hysteresis;

// ── Dynamics ────────────────────────────────────────────────────────────
LowPassGate lowPassGate;
Vactrol vactrol;
VactrolLpg vactrolLpg;
Follower follower;
Compressor compressor;

// ── Effects ─────────────────────────────────────────────────────────────
DelayLine<1024> delayLine;
Reverb<4096> reverb;
Chorus<2048> chorus;
Ensemble<4096> ensemble;
Diffuser<2048> diffuser;
PitchShifter<2048> pitchShifter;
Decimator decimator;
Limiter limiter;
// A second limiter for the quiet row: the first one's peak follower
// releases slowly, so after the noise row it would still be limiting.
Limiter quietLimiter;
Overdrive overdrive;
WaveFolder waveFolder;
TableWavefolder tableWavefolder;

// ── Physical models ─────────────────────────────────────────────────────
KarplusStrong<512> karplus;
ModalResonator<> modal12, modal24;
ModalResonator<64> bowed;
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

void setup() {
  // Enable the cycle counter (AudioEngine::begin() normally does this).
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  WhiteNoise seed(12345);
  for (int i = 0; i < 256; ++i)
    noise[i] = seed.process();

  // Fast attack, slow release: the gated envelope is never idle.
  adsr.setAttack(0.01f);
  adsr.setRelease(2.0f);
  modal12.setModes(12);
  modal24.setModes(24);
  bowed.setModes(12);
  bowed.setBowedModes(true);
  rangeExp.setInput(-1.0f, 1.0f); // the test signal is noise, -1 to 1
  rangeExp.setOutput(80.0f, 8000.0f);
  rangeExp.setExponential(true);
  fm.setFrequency(220.0f);
  // The same reverb settings as `profile`, so the two can be compared.
  reverb.setTime(0.9f);
  reverb.setAmount(0.45f);
  reverb.setLp(0.6f);

  Serial.begin(115200);
}

void loop() {
  // 1. The empty loop: everything TIME() does except a module.
  overhead = 0;
  TIME(acc += in);
  overhead = best;

  Serial.print("\n# CPU ");
  Serial.print(SystemCoreClock / 1000000);
  Serial.print(" MHz, budget ");
  Serial.print(budget(), 0);
  Serial.print(" cycles/sample, loop overhead ");
  Serial.print((float)overhead / kSamples, 1);
  Serial.println(" cycles/sample (subtracted)");
  Serial.println("module,header,cycles_per_sample,ram_bytes,pct_of_budget");

  // 2. Oscillators.
  PROFILE("Oscillator", "osc/oscillator.h", sizeof(osc), acc += osc.process());
  PROFILE("LFO", "osc/lfo.h", sizeof(lfo), acc += lfo.process());
  PROFILE("SineOscillator", "osc/sine.h", sizeof(sine), acc += sine.process());
  PROFILE("SineOscillator (sine and cosine)", "osc/sine.h", sizeof(quad), {
    float s, c;
    quad.next(0.3f / SAMPLE_RATE, 1.0f, s, c);
    acc += s + c;
  });
  PROFILE("BasicOscillator", "osc/basic.h", sizeof(basic),
          acc += basic.process());
  PROFILE("VariableShapeOscillator", "osc/variable_shape.h",
          sizeof(variableShape), acc += variableShape.process());
  PROFILE("FormantOscillator", "osc/formant.h", sizeof(formant),
          acc += formant.process());
  PROFILE("VosimOscillator", "osc/vosim.h", sizeof(vosim),
          acc += vosim.process());
  PROFILE("GrainletOscillator", "osc/grainlet.h", sizeof(grainlet),
          acc += grainlet.process());
  PROFILE("ZOscillator", "osc/z_osc.h", sizeof(zOsc), acc += zOsc.process());
  PROFILE("HarmonicOscillator", "osc/harmonic.h", sizeof(harmonic),
          acc += harmonic.process());
  PROFILE("StringSynthOscillator", "osc/string_synth.h", sizeof(stringSynth),
          acc += stringSynth.process());

  // 3. Noise and random sources.
  PROFILE("WhiteNoise", "noise/noise.h", sizeof(white), acc += white.process());
  PROFILE("PinkNoise", "noise/noise.h", sizeof(pink), acc += pink.process());
  PROFILE("LFSR", "core/random.h", sizeof(lfsr), acc += lfsr.nextFloat());
  PROFILE("Dust", "noise/dust.h", sizeof(dust), acc += dust.process());
  PROFILE("Particle", "noise/particle.h", sizeof(particle),
          acc += particle.process());
  PROFILE("ClockedNoise", "noise/clocked_noise.h", sizeof(clockedNoise),
          acc += clockedNoise.process());
  PROFILE("SmoothRandom", "noise/smooth_random.h", sizeof(smoothRandom),
          acc += smoothRandom.process());

  // 4. Filters.
  PROFILE("SVFilter", "filter/sv_filter.h", sizeof(svFilter), {
    svFilter.process(in);
    acc += svFilter.lowpass();
  });
  PROFILE("Svf", "filter/svf.h", sizeof(svf), acc += svf.process(in).lp);
  PROFILE("MoogFilter", "filter/moog_filter.h", sizeof(moog),
          acc += moog.process(in));
  PROFILE("OnePole", "filter/one_pole.h", sizeof(onePoleFilter),
          acc += onePoleFilter.process(in).lp);
  PROFILE("DcBlocker", "filter/dc_blocker.h", sizeof(dcBlocker),
          acc += dcBlocker.process(in));
  PROFILE("VowelFilter", "filter/vowel.h", sizeof(vowel),
          acc += vowel.process(in));
  PROFILE("Hilbert", "filter/hilbert.h", sizeof(hilbert),
          acc += hilbert.process(in));
  PROFILE("AllpassFilter<512>", "filter/allpass.h", sizeof(allpass),
          acc += allpass.process(in));

  // 5. Modulation.
  PROFILE("Envelope", "mod/envelope.h", sizeof(adsr), {
    adsr.gate(GATE);
    acc += adsr.process();
  });
  PROFILE("SegmentEnvelope", "mod/segment_envelope.h", sizeof(segmentEnvelope),
          acc += segmentEnvelope.process(GATE));
  PROFILE("Slope", "mod/slope.h", sizeof(slopeGen),
          acc += slopeGen.process(GATE));
  PROFILE("SlewLimiter", "mod/slew.h", sizeof(slewLimiter),
          acc += slewLimiter.process(in));
  PROFILE("Lag", "mod/lag.h", sizeof(lag),
          acc += lag.process(in, 0.5f, (n & 255) / 256.0f));
  PROFILE("SampleAndHold", "mod/sample_and_hold.h", sizeof(sah),
          acc += sah.process(in, (n & 63) == 0));
  PROFILE("TempoClock", "mod/clock.h", sizeof(tempo),
          acc += tempo.process() ? 1.0f : 0.0f);
  PROFILE("ClockToRamp", "mod/clock_to_ramp.h", sizeof(clockToRamp),
          acc += clockToRamp.process((n & 511) == 0));
  PROFILE("Lorenz", "mod/lorenz.h", sizeof(lorenz), acc += lorenz.process());
  PROFILE("BouncingBall", "mod/bouncing_ball.h", sizeof(ball),
          acc += ball.process(STRIKE));
  PROFILE("RandomSequence", "mod/random_sequence.h", sizeof(randomSequence),
          acc += randomSequence.next());
  PROFILE("Quantizer", "mod/quantizer.h", sizeof(quantizer),
          acc += quantizer.process(in, 0.5f));
  PROFILE("HysteresisQuantizer", "mod/hysteresis_quantizer.h",
          sizeof(hysteresis), acc += hysteresis.process(in));

  // 5b. Utilities. The exponential Range is the dearer curve, so it is
  // the one gen_modules.py keeps (it takes each class's worst row).
  PROFILE("Range (linear)", "util/range.h", sizeof(rangeLin),
          acc += rangeLin.process(in));
  PROFILE("Range (exponential)", "util/range.h", sizeof(rangeExp),
          acc += rangeExp.process(in));
  PROFILE("Mixer", "util/mixer.h", sizeof(mixer),
          acc += mixer.process(in, 0.5f, 0.25f, 0.125f));
  PROFILE("Multiply", "util/multiply.h", sizeof(multiply),
          acc += multiply.process(in, 0.5f));
  PROFILE("Param", "util/param.h", sizeof(param), acc += param.process());

  // 6. Dynamics.
  PROFILE("LowPassGate", "dynamics/low_pass_gate.h", sizeof(lowPassGate),
          acc += lowPassGate.process(in));
  PROFILE("Vactrol", "dynamics/vactrol.h", sizeof(vactrol),
          acc += vactrol.process(in));
  PROFILE("VactrolLpg", "dynamics/vactrol_lpg.h", sizeof(vactrolLpg), {
    if (STRIKE)
      vactrolLpg.trigger();
    acc += vactrolLpg.process(in);
  });
  PROFILE("Follower", "dynamics/follower.h", sizeof(follower),
          acc += follower.process(in));
  PROFILE("Compressor", "dynamics/compressor.h", sizeof(compressor),
          acc += compressor.process(in));

  // 7. Effects. The stereo ones process a pair in place.
  PROFILE("DelayLine<1024> (write and read)", "fx/delay.h", sizeof(delayLine), {
    delayLine.write(in);
    acc += delayLine.read(100.5f);
  });
  PROFILE("Reverb<4096> (stereo)", "fx/reverb.h", sizeof(reverb), {
    float l = in, r = -in;
    reverb.process(l, r);
    acc += l + r;
  });
  PROFILE("Chorus<2048> (stereo)", "fx/chorus.h", sizeof(chorus), {
    float l = in, r = -in;
    chorus.process(l, r);
    acc += l + r;
  });
  PROFILE("Ensemble<4096> (stereo)", "fx/ensemble.h", sizeof(ensemble), {
    float l = in, r = -in;
    ensemble.process(l, r);
    acc += l + r;
  });
  PROFILE("Diffuser<2048> (stereo)", "fx/diffuser.h", sizeof(diffuser), {
    float l = in, r = -in;
    diffuser.process(l, r);
    acc += l + r;
  });
  PROFILE("PitchShifter<2048>", "fx/pitch_shifter.h", sizeof(pitchShifter),
          acc += pitchShifter.process(in));
  PROFILE("Decimator", "fx/decimator.h", sizeof(decimator),
          acc += decimator.process(in));
  PROFILE("Limiter (stereo)", "fx/limiter.h", sizeof(limiter), {
    float l = in, r = -in;
    limiter.process(l, r);
    acc += l + r;
  });
  // Below the threshold the limiter skips its division and does less
  // work: its cost when idle, against the busy row above.
  PROFILE("Limiter (stereo idle)", "fx/limiter.h", sizeof(quietLimiter), {
    float l = 0.1f, r = 0.1f;
    quietLimiter.process(l, r);
    acc += l + r;
  });
  PROFILE("Overdrive", "fx/overdrive.h", sizeof(overdrive),
          acc += overdrive.process(in));
  PROFILE("WaveFolder", "fx/wave_folder.h", sizeof(waveFolder),
          acc += waveFolder.process(in));
  PROFILE("TableWavefolder", "fx/table_wavefolder.h", sizeof(tableWavefolder),
          acc += tableWavefolder.process(in));

  // 8. Physical models, re-struck so they keep ringing.
  PROFILE("KarplusStrong<512>", "physical/karplus.h", sizeof(karplus), {
    if (STRIKE)
      karplus.pluck(110.0f);
    acc += karplus.process();
  });
  PROFILE("ModalResonator (12 modes)", "physical/modal_resonator.h",
          sizeof(modal12), acc += modal12.process(in * 0.1f));
  PROFILE("ModalResonator (24 modes)", "physical/modal_resonator.h",
          sizeof(modal24), acc += modal24.process(in * 0.1f));
  PROFILE("ModalResonator<64> (12 modes bowed)", "physical/modal_resonator.h",
          sizeof(bowed), acc += bowed.process(in * 0.1f, 0.5f));
  PROFILE("StringModel<512>", "physical/string_model.h", sizeof(stringModel),
          acc += stringModel.process(in * 0.1f));
  PROFILE("Plucker", "physical/plucker.h", sizeof(plucker), {
    if (STRIKE)
      plucker.trigger();
    acc += plucker.process();
  });
  PROFILE("Tube<512>", "physical/tube.h", sizeof(tube),
          acc += tube.process(in * 0.5f, 0.8f));

  // 9. Drums, struck every 2048 samples (about 21 hits a second).
  PROFILE("AnalogKick", "drums/analog_kick.h", sizeof(analogKick), {
    if (STRIKE)
      analogKick.trigger();
    acc += analogKick.process();
  });
  PROFILE("AnalogSnare", "drums/analog_snare.h", sizeof(analogSnare), {
    if (STRIKE)
      analogSnare.trigger();
    acc += analogSnare.process();
  });
  PROFILE("HiHat", "drums/hihat.h", sizeof(hiHat), {
    if (STRIKE)
      hiHat.trigger();
    acc += hiHat.process();
  });
  PROFILE("SynthKick", "drums/synth_kick.h", sizeof(synthKick), {
    if (STRIKE)
      synthKick.trigger();
    acc += synthKick.process();
  });
  PROFILE("SynthSnare", "drums/synth_snare.h", sizeof(synthSnare), {
    if (STRIKE)
      synthSnare.trigger();
    acc += synthSnare.process();
  });
  PROFILE("FmDrum", "drums/fm_drum.h", sizeof(fmDrum), {
    if (STRIKE)
      fmDrum.trigger();
    acc += fmDrum.process();
  });

  // 10. Spectral.
  PROFILE("FmOperators", "spectral/fm_operators.h", sizeof(fm),
          acc += fm.process());
  PROFILE("Xmod::morph", "spectral/xmod.h", 0,
          acc += Xmod::morph(0.5f, in, noise[(n + 7) & 255], 0.5f));

  // 11. Setters. Modulating a parameter calls its setter, so a patch
  //     that does it every sample pays this on top of process().
  SETTER("Oscillator.setFrequency", "osc/oscillator.h",
         osc.setFrequency(220.0f + in * 10.0f));
  SETTER("SVFilter.setFrequency", "filter/sv_filter.h",
         svFilter.setFrequency(1000.0f + in * 500.0f));
  SETTER("Svf.setFrequency (exact tan)", "filter/svf.h",
         svf.setFrequency(1000.0f + in * 500.0f, 0.707f));
  SETTER("Svf.setCoefficients (fast tan)", "filter/svf.h",
         svf.setCoefficients((1000.0f + in * 500.0f) / SAMPLE_RATE, 0.707f,
                             TanApprox::Fast));
  SETTER("MoogFilter.setFrequency", "filter/moog_filter.h",
         moog.setFrequency(1000.0f + in * 500.0f));
  SETTER("OnePole.setFrequency", "filter/one_pole.h",
         onePoleFilter.setFrequency(1000.0f + in * 500.0f));
  SETTER("HarmonicOscillator.setFrequency", "osc/harmonic.h",
         harmonic.setFrequency(220.0f + in * 10.0f));
  SETTER("StringSynthOscillator.setFrequency", "osc/string_synth.h",
         stringSynth.setFrequency(220.0f + in * 10.0f));
  SETTER("Tube.setFrequency", "physical/tube.h",
         tube.setFrequency(220.0f + in * 10.0f));
  SETTER("FmOperators.setFrequency", "spectral/fm_operators.h",
         fm.setFrequency(220.0f + in * 10.0f));

  delay(10000);
}
