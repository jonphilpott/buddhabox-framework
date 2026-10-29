/**
 * @file compare_partials.cpp
 * @brief Host-side check that each port matches its Partials original.
 *
 * Runs the Partials class (namespace pt) and the BuddhaBox port
 * (namespace bb) side by side with the same settings and input, and
 * prints the largest difference between their outputs. Tables and maths
 * are shared, so most ports match to ~1e-6; ports that swapped exp() or
 * pow() for fastExp2() or a table differ slightly more (~1e-4).
 *
 * Build and run (from the repository root; needs ~/git/partials):
 *   clang++ -std=c++14 -O2 -I src -I ../partials/include \
 *       tests/compare_partials.cpp -o /tmp/compare && /tmp/compare
 */

#include <cmath>
#include <cstdio>

#include "pt/partials.h"

#include "buddhabox/drums/analog_kick.h"
#include "buddhabox/drums/analog_snare.h"
#include "buddhabox/drums/fm_drum.h"
#include "buddhabox/drums/hihat.h"
#include "buddhabox/drums/synth_kick.h"
#include "buddhabox/drums/synth_snare.h"
#include "buddhabox/dynamics/compressor.h"
#include "buddhabox/dynamics/follower.h"
#include "buddhabox/dynamics/vactrol.h"
#include "buddhabox/dynamics/vactrol_lpg.h"
#include "buddhabox/filter/dc_blocker.h"
#include "buddhabox/filter/hilbert.h"
#include "buddhabox/filter/one_pole.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/filter/vowel.h"
#include "buddhabox/fx/chorus.h"
#include "buddhabox/fx/decimator.h"
#include "buddhabox/fx/diffuser.h"
#include "buddhabox/fx/ensemble.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/overdrive.h"
#include "buddhabox/fx/pitch_shifter.h"
#include "buddhabox/fx/reverb.h"
#include "buddhabox/fx/table_wavefolder.h"
#include "buddhabox/mod/bouncing_ball.h"
#include "buddhabox/mod/clock_to_ramp.h"
#include "buddhabox/mod/lag.h"
#include "buddhabox/mod/lorenz.h"
#include "buddhabox/mod/quantizer.h"
#include "buddhabox/mod/random_sequence.h"
#include "buddhabox/mod/segment_envelope.h"
#include "buddhabox/mod/slope.h"
#include "buddhabox/noise/clocked_noise.h"
#include "buddhabox/noise/dust.h"
#include "buddhabox/noise/particle.h"
#include "buddhabox/noise/smooth_random.h"
#include "buddhabox/osc/basic.h"
#include "buddhabox/osc/formant.h"
#include "buddhabox/osc/grainlet.h"
#include "buddhabox/osc/harmonic.h"
#include "buddhabox/osc/string_synth.h"
#include "buddhabox/osc/variable_shape.h"
#include "buddhabox/osc/vosim.h"
#include "buddhabox/osc/z_osc.h"
#include "buddhabox/physical/modal_resonator.h"
#include "buddhabox/physical/plucker.h"
#include "buddhabox/physical/string_model.h"
#include "buddhabox/physical/tube.h"
#include "buddhabox/spectral/fm_operators.h"
#include "buddhabox/spectral/xmod.h"

static const float kRate = 44100.0f;
static const int kSamples = 44100 * 2;
static int failures = 0;

/**
 * Deterministic test input: a slow saw plus noise bursts. The noise comes
 * from a hash of n (not a running generator), so calling it twice for
 * the same sample gives the same value.
 */
static float input(int n) {
  uint32_t h = static_cast<uint32_t>(n) * 2654435761u;
  h ^= h >> 15;
  h *= 2246822519u;
  h ^= h >> 13;
  float noise = static_cast<float>(h) / 4294967296.0f * 2.0f - 1.0f;
  float saw = (n % 200) / 100.0f - 1.0f;
  return 0.5f * saw + ((n / 4000) % 2 ? 0.3f * noise : 0.0f);
}

/** Compare two output streams; report max error relative to peak. */
template <typename F> static void check(const char *name, float tol, F f) {
  double maxDiff = 0, peak = 0;
  for (int n = 0; n < kSamples; ++n) {
    float a, b;
    f(n, a, b);
    maxDiff = fmax(maxDiff, fabs(a - b));
    peak = fmax(peak, fabs(a));
  }
  double rel = peak > 0 ? maxDiff / peak : maxDiff;
  bool ok = rel <= tol;
  if (!ok)
    ++failures;
  printf("%-22s peak %8.4f  max diff %.2e  (rel %.2e) %s\n", name, peak,
         maxDiff, rel, ok ? "ok" : "MISMATCH");
}

int main() {
  // ── Oscillators ──
  {
    pt::BasicOscillator p;
    bb::BasicOscillator b;
    p.init(kRate);
    for (int s = 0; s < 8; ++s) {
      p.setShape((pt::BasicOscillator::Shape)s);
      b.setShape((bb::BasicOscillator::Shape)s);
      p.setFrequency(220.0f);
      b.setFrequency(220.0f);
      p.setPulseWidth(0.3f);
      b.setPulseWidth(0.3f);
      char name[32];
      snprintf(name, 32, "BasicOscillator/%d", s);
      check(name, 1e-5f, [&](int, float &x, float &y) {
        x = p.process();
        y = b.process();
      });
    }
  }
  {
    pt::VariableShapeOscillator p;
    bb::VariableShapeOscillator b;
    p.init(kRate);
    p.setFrequency(330);
    b.setFrequency(330);
    p.setSyncFrequency(110);
    b.setSyncFrequency(110);
    p.setShape(0.3f);
    b.setShape(0.3f);
    check("VariableShape", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::FormantOscillator p;
    bb::FormantOscillator b;
    p.init(kRate);
    p.setCarrierFrequency(110);
    b.setCarrierFrequency(110);
    p.setFormantFrequency(900);
    b.setFormantFrequency(900);
    check("Formant", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::VosimOscillator p;
    bb::VosimOscillator b;
    p.init(kRate);
    p.setCarrierFrequency(98);
    b.setCarrierFrequency(98);
    p.setFormantFrequencies(700, 1200);
    b.setFormantFrequencies(700, 1200);
    p.setShape(0.3f);
    b.setShape(0.3f);
    check("Vosim", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::GrainletOscillator p;
    bb::GrainletOscillator b;
    p.init(kRate);
    p.setCarrierFrequency(130);
    b.setCarrierFrequency(130);
    p.setFormantFrequency(1500);
    b.setFormantFrequency(1500);
    p.setShape(0.5f);
    b.setShape(0.5f);
    p.setBleed(0.3f);
    b.setBleed(0.3f);
    check("Grainlet", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::ZOscillator p;
    bb::ZOscillator b;
    p.init(kRate);
    p.setCarrierFrequency(55);
    b.setCarrierFrequency(55);
    p.setFormantFrequency(600);
    b.setFormantFrequency(600);
    p.setMode(0.5f);
    b.setMode(0.5f);
    p.setShape(0.3f);
    b.setShape(0.3f);
    check("ZOscillator", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::HarmonicOscillator p;
    bb::HarmonicOscillator b;
    p.init(kRate);
    float a[8] = {1, 0.5f, 0.3f, 0.2f, 0.1f, 0.1f, 0.05f, 0.05f};
    p.setAmplitudes(a, 8);
    b.setAmplitudes(a, 8);
    p.setFrequency(110);
    b.setFrequency(110);
    check("Harmonic", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::StringSynthOscillator p;
    bb::StringSynthOscillator b;
    p.init(kRate);
    p.setFrequency(65.4f);
    b.setFrequency(65.4f);
    p.setRegistration(0.3f);
    b.setRegistration(0.3f);
    check("StringSynth", 1e-5f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  // ── Noise ──
  {
    pt::Dust p;
    bb::Dust b;
    p.init(kRate);
    p.setDensity(50);
    b.setDensity(50);
    check("Dust", 1e-6f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::Particle p;
    bb::Particle b;
    p.init(kRate);
    p.setDensity(20);
    b.setDensity(20);
    p.setFrequency(880);
    b.setFrequency(880);
    p.setQ(100);
    b.setQ(100);
    check("Particle", 1e-4f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::ClockedNoise p;
    bb::ClockedNoise b;
    p.init(kRate);
    p.setFrequency(500);
    b.setFrequency(500);
    check("ClockedNoise", 1e-6f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::SmoothRandom p;
    bb::SmoothRandom b;
    p.init(kRate);
    p.setFrequency(5);
    b.setFrequency(5);
    check("SmoothRandom", 1e-6f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  // ── Filters ──
  {
    pt::Svf p;
    bb::Svf b;
    p.init(kRate);
    p.setFrequency(1200, 3);
    b.setFrequency(1200, 3);
    check("Svf", 1e-5f, [&](int n, float &x, float &y) {
      x = p.process(input(n)).bp;
      y = b.process(input(n)).bp;
    });
  }
  {
    pt::OnePole p;
    bb::OnePole b;
    p.init(kRate);
    p.setFrequency(500);
    b.setFrequency(500);
    check("OnePole", 1e-5f, [&](int n, float &x, float &y) {
      x = p.process(input(n)).lp;
      y = b.process(input(n)).lp;
    });
  }
  {
    pt::DcBlocker p;
    bb::DcBlocker b;
    p.init(kRate);
    check("DcBlocker", 1e-5f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::VowelFilter p;
    bb::VowelFilter b;
    p.init(kRate);
    p.setVowel(0.4f);
    b.setVowel(0.4f);
    p.setVoice(0.3f);
    b.setVoice(0.3f);
    check("VowelFilter", 1e-4f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Hilbert p;
    bb::Hilbert b;
    p.init(kRate);
    check("Hilbert", 1e-5f, [&](int n, float &x, float &y) {
      p.process(input(n));
      b.process(input(n));
      x = p.q();
      y = b.q();
    });
  }
  // ── Dynamics ──
  {
    pt::Vactrol p;
    bb::Vactrol b;
    p.init(kRate);
    check("Vactrol", 1e-4f, [&](int n, float &x, float &y) {
      float c = (n / 8000) % 2 ? 1.0f : 0.0f;
      x = p.process(c);
      y = b.process(c);
    });
    p.setPlucked(true);
    b.setPlucked(true);
    check("Vactrol/plucked", 1e-4f, [&](int n, float &x, float &y) {
      float c = (n / 8000) % 2 ? 1.0f : 0.0f;
      x = p.process(c);
      y = b.process(c);
    });
  }
  {
    pt::LowPassGate p;
    bb::VactrolLpg b;
    p.init(kRate);
    p.setDecay(0.6f);
    b.setDecay(0.6f);
    p.setColour(0.7f);
    b.setColour(0.7f);
    check("VactrolLpg", 1e-4f, [&](int n, float &x, float &y) {
      if (n % 10000 == 0) {
        p.trigger();
        b.trigger();
      }
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Follower p;
    bb::Follower b;
    p.init(kRate);
    check("Follower", 1e-5f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Compressor p;
    bb::Compressor b;
    p.init(kRate);
    p.setThreshold(-20);
    b.setThreshold(-20);
    check("Compressor", 1e-3f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  // ── Modulation ──
  {
    pt::Lorenz p;
    bb::Lorenz b;
    p.init(kRate);
    p.setRate(0.6f);
    b.setRate(0.6f);
    check("Lorenz", 1e-3f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::BouncingBall p;
    bb::BouncingBall b;
    p.init(kRate);
    p.setGravity(0.4f);
    b.setGravity(0.4f);
    p.setBounce(0.8f);
    b.setBounce(0.8f);
    check("BouncingBall", 1e-3f, [&](int n, float &x, float &y) {
      bool g = n % 20000 < 10;
      x = p.process(g);
      y = b.process(g);
    });
  }
  {
    pt::RandomSequence p;
    bb::RandomSequence b;
    p.init();
    p.setDejaVu(0.3f);
    b.setDejaVu(0.3f);
    check("RandomSequence", 1e-6f, [&](int, float &x, float &y) {
      x = p.next();
      y = b.next();
    });
  }
  {
    pt::Quantizer p;
    bb::Quantizer b;
    p.init(pt::Quantizer::preset(pt::Quantizer::PENTATONIC));
    b.init(bb::Quantizer::preset(bb::Quantizer::PENTATONIC));
    check("Quantizer", 1e-6f, [&](int n, float &x, float &y) {
      float v = input(n) * 2.0f;
      x = p.process(v, 0.6f);
      y = b.process(v, 0.6f);
    });
  }
  {
    pt::ClockToRamp p;
    bb::ClockToRamp b;
    p.init(kRate);
    p.setRatio(3, 2);
    b.setRatio(3, 2);
    check("ClockToRamp", 1e-4f, [&](int n, float &x, float &y) {
      bool c = (n % 11025) < 2000;
      x = p.process(c);
      y = b.process(c);
    });
  }
  {
    pt::Envelope p;
    bb::SegmentEnvelope b;
    p.init(kRate);
    p.setAdsr(0.01f, 0.2f, 0.5f, 0.4f);
    b.setAdsr(0.01f, 0.2f, 0.5f, 0.4f);
    check("SegmentEnvelope", 1e-3f, [&](int n, float &x, float &y) {
      bool g = (n % 30000) < 15000;
      x = p.process(g);
      y = b.process(g);
    });
  }
  {
    pt::Lag p;
    bb::Lag b;
    p.init();
    check("Lag", 1e-4f, [&](int n, float &x, float &y) {
      float phase = (n % 4410) / 4410.0f;
      float v = (n / 4410) % 3 * 0.3f;
      if (n % 4410 == 0) {
        p.resetRamp();
        b.resetRamp();
      }
      x = p.process(v, 0.8f, phase);
      y = b.process(v, 0.8f, phase);
    });
  }
  {
    pt::Slope p;
    bb::Slope b;
    p.init(kRate);
    p.setFrequency(3);
    b.setFrequency(3);
    p.setSlope(0.7f);
    b.setSlope(0.7f);
    p.setShape(0.3f);
    b.setShape(0.3f);
    p.setSmoothness(0.7f);
    b.setSmoothness(0.7f);
    check("Slope/loop", 1e-4f, [&](int, float &x, float &y) {
      x = p.process(false);
      y = b.process(false);
    });
    p.setMode(pt::Slope::AD);
    b.setMode(bb::Slope::AD);
    p.setSmoothness(0.3f);
    b.setSmoothness(0.3f);
    check("Slope/AD", 1e-4f, [&](int n, float &x, float &y) {
      bool g = n % 20000 < 100;
      x = p.process(g);
      y = b.process(g);
    });
  }
  // ── Effects (int16 storage: ~1e-4 of quantisation noise expected) ──
  {
    pt::Reverb p;
    bb::Reverb<32768> b;
    p.init(kRate, pt::Reverb::CLOUDS);
    check("Reverb<32768>", 5e-3f, [&](int n, float &x, float &y) {
      float l = input(n), r = l, l2 = l, r2 = l;
      p.process(l, r);
      b.process(l2, r2);
      x = r;
      y = r2;
    });
  }
  {
    pt::Chorus p;
    bb::Chorus<> b;
    p.init(kRate);
    check("Chorus", 5e-3f, [&](int n, float &x, float &y) {
      float l = input(n), r = l, l2 = l, r2 = l;
      p.process(l, r);
      b.process(l2, r2);
      x = l;
      y = l2;
    });
  }
  {
    pt::Ensemble p;
    bb::Ensemble<> b;
    p.init(kRate);
    check("Ensemble", 1e-3f, [&](int n, float &x, float &y) {
      float l = input(n), r = l, l2 = l, r2 = l;
      p.process(l, r);
      b.process(l2, r2);
      x = r;
      y = r2;
    });
  }
  {
    pt::Diffuser p;
    bb::Diffuser<> b;
    p.init(kRate);
    check("Diffuser", 5e-3f, [&](int n, float &x, float &y) {
      float l = input(n), r = l, l2 = l, r2 = l;
      p.process(l, r);
      b.process(l2, r2);
      x = l;
      y = l2;
    });
  }
  {
    pt::PitchShifter p;
    bb::PitchShifter<> b;
    p.init(kRate);
    p.setRatio(1.5f);
    b.setRatio(1.5f);
    p.setSize(0.5f);
    b.setSize(0.5f);
    check("PitchShifter", 1e-3f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Decimator p;
    bb::Decimator b;
    p.init(kRate);
    p.setRate(5000);
    b.setRate(5000);
    check("Decimator", 1e-6f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Limiter p;
    bb::Limiter b;
    p.init(kRate);
    p.setPreGain(3);
    b.setPreGain(3);
    check("Limiter", 1e-6f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Overdrive p;
    bb::Overdrive b;
    p.init(kRate);
    p.setDrive(0.7f);
    b.setDrive(0.7f);
    check("Overdrive", 1e-6f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
  }
  {
    pt::Wavefolder p;
    bb::TableWavefolder b;
    p.init();
    p.setAmount(0.7f);
    b.setAmount(0.7f);
    check("TableWavefolder", 1e-5f, [&](int n, float &x, float &y) {
      x = p.process(input(n));
      y = b.process(input(n));
    });
    check("TableWavefolder/an", 1e-5f, [&](int n, float &x, float &y) {
      p.process(input(n));
      b.process(input(n));
      x = p.analog();
      y = b.analog();
    });
  }
  // ── Physical models ──
  {
    pt::ModalResonator p;
    bb::ModalResonator<> b;
    p.init(kRate);
    p.setModes(24);
    b.setModes(24);
    p.setFrequency(220);
    b.setFrequency(220);
    p.setStructure(0.15f);
    b.setStructure(0.15f);
    p.setDamping(0.8f);
    b.setDamping(0.8f);
    check("ModalResonator", 1e-4f, [&](int n, float &x, float &y) {
      float e = n % 22050 < 20 ? 0.8f : 0.0f;
      x = p.process(e);
      y = b.process(e);
    });
  }
  {
    pt::ModalResonator p;
    bb::ModalResonator<1536> b;
    p.init(kRate);
    p.setModes(16);
    b.setModes(16);
    p.setBowedModes(true);
    b.setBowedModes(true);
    p.setFrequency(220);
    b.setFrequency(220);
    check("ModalResonator/bow", 1e-3f, [&](int, float &x, float &y) {
      x = p.process(0.0f, 0.6f);
      y = b.process(0.0f, 0.6f);
    });
  }
  {
    pt::Plucker p;
    bb::Plucker b;
    p.init(kRate);
    p.setFrequency(196);
    b.setFrequency(196);
    p.setCutoff(4000);
    b.setCutoff(4000);
    check("Plucker", 1e-5f, [&](int n, float &x, float &y) {
      if (n % 11025 == 0) {
        p.trigger();
        b.trigger();
      }
      x = p.process();
      y = b.process();
    });
  }
  {
    pt::String p;
    bb::StringModel<> b;
    p.init(kRate);
    p.setFrequency(220);
    b.setFrequency(220);
    p.setDispersion(0.7f);
    b.setDispersion(0.7f);
    p.setDamping(0.8f);
    b.setDamping(0.8f);
    check("StringModel/stiff", 1e-3f, [&](int n, float &x, float &y) {
      float e = n % 22050 < 30 ? input(n + 4000) : 0.0f;
      x = p.process(e);
      y = b.process(e);
    });
    p.init(kRate);
    b.init();
    p.setFrequency(110);
    b.setFrequency(110);
    p.setDispersion(0.1f);
    b.setDispersion(0.1f);
    check("StringModel/bridge", 1e-3f, [&](int n, float &x, float &y) {
      float e = n % 22050 < 30 ? input(n + 4000) : 0.0f;
      x = p.process(e);
      y = b.process(e);
    });
  }
  {
    pt::Tube p;
    bb::Tube<2048> b;
    p.init(kRate);
    p.setFrequency(146.8f);
    b.setFrequency(146.8f);
    check("Tube", 1e-5f, [&](int n, float &x, float &y) {
      x = p.process(input(n) * 0.3f, 0.8f);
      y = b.process(input(n) * 0.3f, 0.8f);
    });
  }
  // ── Drums ──
#define DRUM(P, B, NAME, TOL)                                                  \
  {                                                                            \
    P p;                                                                       \
    B b;                                                                       \
    p.init(kRate);                                                             \
    check(NAME, TOL, [&](int n, float &x, float &y) {                          \
      if (n % 22050 == 0) {                                                    \
        p.trigger();                                                           \
        b.trigger();                                                           \
      }                                                                        \
      x = p.process();                                                         \
      y = b.process();                                                         \
    });                                                                        \
  }
  DRUM(pt::AnalogKick, bb::AnalogKick, "AnalogKick", 1e-4f)
  DRUM(pt::AnalogSnare, bb::AnalogSnare, "AnalogSnare", 1e-4f)
  DRUM(pt::HiHat, bb::HiHat, "HiHat", 1e-4f)
  DRUM(pt::SynthKick, bb::SynthKick, "SynthKick", 1e-4f)
  DRUM(pt::SynthSnare, bb::SynthSnare, "SynthSnare", 1e-4f) {
    pt::FmDrum p;
    bb::FmDrum b;
    p.init(kRate);
    p.setFrequency(60);
    b.setFrequency(60);
    p.setFmAmount(0.4f);
    b.setFmAmount(0.4f);
    p.setNoise(0.5f);
    b.setNoise(0.5f);
    check("FmDrum", 2e-3f, [&](int n, float &x, float &y) {
      if (n % 22050 == 0) {
        p.trigger();
        b.trigger();
      }
      x = p.process();
      y = b.process();
    });
  }
  // ── Spectral ──
  {
    pt::FmOperators p;
    bb::FmOperators b;
    p.init(kRate);
    p.setAlgorithm(5);
    b.setAlgorithm(5);
    p.setFrequency(220);
    b.setFrequency(220);
    for (int op = 1; op <= 6; ++op) {
      p.setRatio(op, op * 1.5f);
      b.setRatio(op, op * 1.5f);
      p.setLevel(op, 0.4f);
      b.setLevel(op, 0.4f);
    }
    p.setFeedback(0.5f);
    b.setFeedback(0.5f);
    check("FmOperators", 1e-4f, [&](int, float &x, float &y) {
      x = p.process();
      y = b.process();
    });
  }
  for (int a = 0; a <= 6; ++a) {
    char name[32];
    snprintf(name, 32, "Xmod/%d", a);
    check(name, 1e-4f, [&](int n, float &x, float &y) {
      float m = input(n), c = input(n + 77);
      x = pt::Xmod::process((pt::Xmod::Algorithm)a, m, c, 0.4f);
      y = bb::Xmod::process((bb::Xmod::Algorithm)a, m, c, 0.4f);
    });
  }
  printf("%s\n", failures ? "SOME PORTS DIFFER" : "all ports match");
  return failures ? 1 : 0;
}
