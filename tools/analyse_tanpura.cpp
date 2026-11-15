/**
 * @file analyse_tanpura.cpp
 * @brief Measures a recorded tanpura's harmonics, string by string, over
 *        one pluck cycle, and writes them as a table for
 *        examples/tanpura_spectral.
 *
 * Host tool, plain C++ (no libraries). Usage:
 *
 *   ffmpeg -t 20 -i recording.flac -ac 1 -ar 44100 -f f32le tanpura.f32
 *   clang++ -std=c++14 -O2 tools/analyse_tanpura.cpp -o /tmp/analyse
 *   /tmp/analyse tanpura.f32 4 14.5 161.69 \
 *       > examples/tanpura_spectral/spectra.h
 *
 * Arguments: the raw mono 32-bit float file, the start and end of a
 * stretch where only the tanpura plays (seconds), and the low sa's pitch
 * in Hz (find it as the strongest low peak of a long FFT). The first
 * string is assumed tuned to ma (4/3 of sa), as for raga Malkauns.
 *
 * WHAT IT DOES:
 * 1. A short-time FFT (4096 points, a frame every 10 ms) of the stretch.
 * 2. Finds each cycle's start from the SA strings: SA's 7th harmonic
 *    jumps by 15-20 dB at their pluck. Peaks at least 1.7 s apart.
 * 3. Folds: every frame's place in its own cycle (0..1, start to next
 *    start) puts it in one of 16 bins, so tape wobble, which stretches
 *    and shrinks cycles, doesn't smear the average.
 * 4. Separates the strings. With sa at 1, SA is at 2 and ma at 4/3:
 *    - sa's odd harmonics belong to sa alone;
 *    - its even harmonics are sa plus SA: sa's share is guessed from its
 *      odd neighbours, and the rest is SA's (two strings, so halved);
 *    - ma's harmonics are its own except every third, which lands on
 *      one of sa's (3 x 4/3 = 4): those are guessed from neighbours.
 * 5. Removes the tape's noise floor, measured between harmonics.
 * 6. Finds each string's pluck as the bin where its harmonics rise most,
 *    and rotates its 16 frames to start there.
 * 7. Turns each frame's 32 harmonic levels into one cycle of waveform
 *    (256 samples), an inverse Fourier transform done as a sum of sines.
 * 8. Prints a C++ header: 3 strings x 16 frames x 257 samples, scaled
 *    so the loudest sample of any string is 1.
 */

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

typedef std::complex<double> cd;

const int N = 4096;    // FFT length: 93 ms, 10.8 Hz between bins
const int HOP = 441;   // 10 ms
const int B = 16;      // frames per cycle in the output
const int H = 32;      // harmonics per string in the output
const int KSA = 2 * H; // sa harmonics measured (SA's 32 are sa's evens)
const int W = 256;     // samples in one cycle of the output waveforms
const double FS = 44100.0;

/** In-place radix-2 FFT. */
void fft(std::vector<cd> &a) {
  int n = a.size();
  for (int i = 1, j = 0; i < n; i++) {
    int b = n >> 1;
    for (; j & b; b >>= 1)
      j ^= b;
    j ^= b;
    if (i < j)
      std::swap(a[i], a[j]);
  }
  for (int len = 2; len <= n; len <<= 1) {
    cd w(cos(-2 * M_PI / len), sin(-2 * M_PI / len));
    for (int i = 0; i < n; i += len) {
      cd wn(1);
      for (int j = 0; j < len / 2; j++) {
        cd u = a[i + j], v = a[i + j + len / 2] * wn;
        a[i + j] = u + v;
        a[i + j + len / 2] = u - v;
        wn *= w;
      }
    }
  }
}

/** Power spectrum of N samples from start, Hann window. */
std::vector<double> power(const std::vector<float> &x, long start) {
  std::vector<cd> a(N);
  for (int i = 0; i < N; i++)
    a[i] = x[start + i] * (0.5 - 0.5 * cos(2 * M_PI * i / N));
  fft(a);
  std::vector<double> p(N / 2);
  for (int i = 0; i < N / 2; i++)
    p[i] = std::norm(a[i]) / ((double)N * N);
  return p;
}

/** Strongest bin within one bin of f: a harmonic's power. */
double at(const std::vector<double> &p, double f) {
  int i = (int)(f * N / FS + 0.5);
  if (i < 1 || i >= N / 2 - 1)
    return 0;
  return std::max(p[i - 1], std::max(p[i], p[i + 1]));
}

double db(double p) { return 10 * log10(p + 1e-20); }

int main(int argc, char **argv) {
  if (argc < 5) {
    fprintf(stderr, "usage: %s file.f32 start end sa_hz\n", argv[0]);
    return 1;
  }
  FILE *f = fopen(argv[1], "rb");
  if (!f) {
    fprintf(stderr, "can't open %s\n", argv[1]);
    return 1;
  }
  fseek(f, 0, SEEK_END);
  long n = ftell(f) / 4;
  fseek(f, 0, SEEK_SET);
  std::vector<float> x(n);
  if (fread(x.data(), 4, n, f) != (size_t)n)
    return 1;
  fclose(f);
  double t0 = atof(argv[2]), t1 = atof(argv[3]), sa = atof(argv[4]);
  double ma = sa * 4 / 3;

  // 1. Short-time FFT: per frame, every harmonic's power and the noise
  //    floor (between sa's harmonics, 1-4 kHz).
  std::vector<double> times;
  std::vector<std::vector<double>> pSa, pMa;
  std::vector<double> floorP;
  for (long s = (long)(t0 * FS); s + N < (long)(t1 * FS) && s + N < n;
       s += HOP) {
    std::vector<double> p = power(x, s);
    std::vector<double> a(KSA + 1), m(H + 1);
    for (int k = 1; k <= KSA; k++)
      a[k] = at(p, sa * k);
    for (int k = 1; k <= H; k++)
      m[k] = at(p, ma * k);
    // Between harmonics of sa there may be ma's: use the quietest gaps.
    std::vector<double> gaps;
    for (int k = 6; k < 25; k++)
      gaps.push_back(at(p, sa * (k + 0.5)));
    std::sort(gaps.begin(), gaps.end());
    floorP.push_back(gaps[gaps.size() / 4]);
    pSa.push_back(a);
    pMa.push_back(m);
    times.push_back((s + N / 2) / FS);
  }

  // 2. Cycle starts: peaks of SA's 7th harmonic (sa x 14), 1.7 s apart.
  std::vector<double> starts;
  int w = (int)(0.85 * FS / HOP);
  for (int i = 0; i < (int)times.size(); i++) {
    bool peak = true;
    for (int k = std::max(0, i - w); k < std::min((int)times.size(), i + w);
         k++)
      if (pSa[k][14] > pSa[i][14])
        peak = false;
    if (peak && i > 0 && i + 1 < (int)times.size())
      starts.push_back(times[i]);
  }
  fprintf(stderr, "cycle starts:");
  for (double s : starts)
    fprintf(stderr, " %.2f", s);
  fprintf(stderr, "\n");
  if (starts.size() < 3) {
    fprintf(stderr, "need at least two whole cycles\n");
    return 1;
  }

  // 3. Fold into B bins per cycle.
  std::vector<std::vector<double>> fSa(B, std::vector<double>(KSA + 1)),
      fMa(B, std::vector<double>(H + 1));
  std::vector<double> fFloor(B);
  std::vector<int> count(B);
  for (size_t i = 0; i < times.size(); i++) {
    for (size_t c = 0; c + 1 < starts.size(); c++) {
      if (times[i] < starts[c] || times[i] >= starts[c + 1])
        continue;
      double phase = (times[i] - starts[c]) / (starts[c + 1] - starts[c]);
      int b = std::min(B - 1, (int)(phase * B));
      for (int k = 1; k <= KSA; k++)
        fSa[b][k] += pSa[i][k];
      for (int k = 1; k <= H; k++)
        fMa[b][k] += pMa[i][k];
      fFloor[b] += floorP[i];
      count[b]++;
    }
  }

  // 4 and 5. Separate the strings and take off the floor. amp[string]
  //    [bin][harmonic], harmonic 0-based; strings: 0 sa, 1 SA, 2 ma.
  double amp[3][B][H] = {};
  for (int b = 0; b < B; b++) {
    double fl = fFloor[b] / count[b];
    auto clean = [&](double p) { return std::max(0.0, p / count[b] - fl); };
    // sa: odd harmonics as measured; even ones from the neighbours'
    // geometric mean (the average in dB), but never above the measured
    // total.
    std::vector<double> s(KSA + 2);
    for (int k = 1; k <= KSA; k += 2)
      s[k] = clean(fSa[b][k]);
    for (int k = 2; k <= KSA; k += 2) {
      double lo = s[k - 1], hi = k + 1 <= KSA ? s[k + 1] : s[k - 1];
      s[k] = std::min(sqrt(lo * hi), clean(fSa[b][k]));
    }
    for (int k = 1; k <= H; k++)
      amp[0][b][k - 1] = sqrt(s[k]);
    // SA: what's left of sa's even harmonics, shared by two strings.
    for (int j = 1; j <= H; j++)
      amp[1][b][j - 1] =
          sqrt(std::max(0.0, clean(fSa[b][2 * j]) - s[2 * j]) / 2);
    // ma: every third harmonic sits on one of sa's; guess it from its
    // neighbours instead.
    std::vector<double> m(H + 2);
    for (int k = 1; k <= H; k++)
      m[k] = clean(fMa[b][k]);
    for (int k = 3; k <= H; k += 3) {
      double hi = k + 1 <= H ? m[k + 1] : m[k - 1];
      m[k] = sqrt(m[k - 1] * hi);
    }
    for (int k = 1; k <= H; k++)
      amp[2][b][k - 1] = sqrt(m[k]);
  }

  // 6. Each string's pluck: the bin where the power of its upper
  //    harmonics (5th and up) rises most from the bin before. A pluck
  //    barely changes the fundamental, which rings on from the last one,
  //    but it refreshes the treble. Then rotate so frame 0 is the pluck.
  const char *names[3] = {"sa", "SA", "ma"};
  int pluck[3];
  for (int s = 0; s < 3; s++) {
    double best = -1e9;
    pluck[s] = 0;
    for (int b = 0; b < B; b++) {
      double now = 0, before = 0;
      for (int k = 4; k < H; k++) {
        now += amp[s][b][k] * amp[s][b][k];
        before += amp[s][(b + B - 1) % B][k] * amp[s][(b + B - 1) % B][k];
      }
      double rise = db(now) - db(before);
      if (rise > best) {
        best = rise;
        pluck[s] = b;
      }
    }
    fprintf(stderr, "%s plucked in bin %d of %d (rise %.1f dB)\n", names[s],
            pluck[s], B, best);
  }

  // 7. Turn every frame into one cycle of its waveform: the inverse
  //    Fourier transform, done the slow, plain way (a sum of sines),
  //    since it runs once. Each harmonic gets a fixed random starting
  //    phase, the same in every frame of a string so neighbouring
  //    frames blend smoothly; random phases keep the peaks lower than
  //    all-in-step sines would.
  static double wave[3][B][W + 1];
  double peak = 0;
  for (int s = 0; s < 3; s++) {
    double phase[H];
    uint32_t seed = 12345 + 777 * s;
    for (int k = 0; k < H; k++) {
      seed = seed * 1664525u + 1013904223u; // a simple LCG
      phase[k] = 2 * M_PI * (seed >> 8) / 16777216.0;
    }
    for (int f = 0; f < B; f++) {
      int b = (pluck[s] + f) % B;
      for (int i = 0; i < W; i++) {
        double v = 0;
        for (int k = 0; k < H; k++)
          v += amp[s][b][k] * sin(2 * M_PI * (k + 1) * i / W + phase[k]);
        wave[s][f][i] = v;
        peak = std::max(peak, fabs(v));
      }
      wave[s][f][W] = wave[s][f][0]; // guard point for interpolation
    }
  }

  // 8. Print, scaled so the loudest sample of any string is 1 (which
  //    keeps the strings' levels relative to each other).
  printf("/**\n * @file spectra.h\n * @brief Generated by "
         "tools/analyse_tanpura.cpp: never edit by hand.\n *\n");
  printf(" * A recorded tanpura, %.1f-%.1f s, sa = %.2f Hz, %d cycles of "
         "%.2f s\n * on average. kWaves[string][frame][sample]: one cycle "
         "of each\n * string's waveform (%d harmonics), at %d moments from "
         "its pluck\n * (frame 0) to just before the next (frame %d). "
         "Strings: 0 sa,\n * 1 SA (each of the two), 2 the first string "
         "(measured as ma).\n * Sample %d repeats sample 0, for "
         "interpolation.\n */\n\n#pragma once\n\n",
         t0, t1, sa, (int)starts.size() - 1,
         (starts.back() - starts.front()) / (starts.size() - 1), H, B, B - 1,
         W);
  printf("const int kWaveFrames = %d;\nconst int kWaveSize = %d;\n\n", B, W);
  printf("const float kWaves[3][%d][%d] = {\n", B, W + 1);
  for (int s = 0; s < 3; s++) {
    printf("    // %s\n    {\n", names[s]);
    for (int f = 0; f < B; f++) {
      printf("        {");
      for (int i = 0; i <= W; i++)
        printf("%s%.4ff%s", i % 8 ? "" : "\n         ", wave[s][f][i] / peak,
               i < W ? ", " : "");
      printf("},\n");
    }
    printf("    },\n");
  }
  printf("};\n");

  // Diagnostics: each string's level and brightness over its cycle.
  for (int s = 0; s < 3; s++) {
    fprintf(stderr, "%s frames: level dB / share above harmonic 8\n  ",
            names[s]);
    for (int f = 0; f < B; f++) {
      int b = (pluck[s] + f) % B;
      double all = 0, hi = 0;
      for (int k = 0; k < H; k++) {
        double p = amp[s][b][k] * amp[s][b][k];
        all += p;
        if (k >= 8)
          hi += p;
      }
      fprintf(stderr, "%.0f/%.2f ", db(all), hi / (all + 1e-30));
    }
    fprintf(stderr, "\n");
  }
  return 0;
}
