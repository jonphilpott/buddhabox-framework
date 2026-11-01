/**
 * @file hilbert.h
 * @brief Hilbert transformer: two outputs 90 degrees apart (Warps).
 *
 * Splits a signal into two copies 90 degrees apart at every frequency (an
 * "analytic signal": in-phase I and quadrature Q). It is the key part of
 * a frequency shifter (Bode / single-sideband modulation), which moves
 * every partial up or down by the same number of Hz — lovely, eerie
 * detuning on drones. It also gives a smooth amplitude envelope,
 * sqrt(I^2 + Q^2).
 *
 * HOW IT WORKS: two chains of first-order all-pass filters. All-passes
 * change only phase, not level. The chains are designed so their phase
 * responses stay 90 degrees apart (within about 1 degree) from 10 Hz to
 * 20 kHz. The design is an elliptic half-band filter decomposed into
 * all-passes (as in Warps' resource script), computed for the actual
 * sample rate when the object is built.
 *
 * Both outputs are delayed, phase-shifted versions of the input, so use
 * I and Q together, not I as a stand-in for the input.
 *
 * COST: 17 first-order all-passes, 235 cycles per sample (12% of the
 * Black Pill), measured with the profile example.
 *
 * Derived from Warps, Copyright 2014 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * 90-degree phase splitter.
 *
 * Example — a frequency shifter (moves every partial up by 3 Hz):
 *   Hilbert hilbert;
 *   SineOscillator shift;
 *   float s, c;
 *
 *   hilbert.process(in);
 *   shift.next(3.0f / SAMPLE_RATE, 1.0f, s, c);
 *   float up = hilbert.i() * c - hilbert.q() * s;   // shifted up
 *   float down = hilbert.i() * c + hilbert.q() * s; // shifted down
 */
class Hilbert {
public:
  /** All-pass sections across both chains. */
  static const int kNumFilters = 17;

  Hilbert() { init(); }

  /**
   * Design the filters for this rate and clear state. Uses double
   * precision, about a millisecond on the Black Pill: call from setup.
   * @param sampleRate  Sample rate in Hz (default SAMPLE_RATE)
   */
  void init(float sampleRate = SAMPLE_RATE) {
    designPoles(sampleRate, coefficient_);
    for (int i = 0; i < kNumFilters; ++i)
      x_[i] = y_[i] = 0.0f;
    i_ = q_ = 0.0f;
  }

  /**
   * Process one sample.
   * @param in  Input sample
   * @return    The in-phase output I; q() then holds Q, 90 degrees behind
   */
  float process(float in) {
    // Filters alternate between the two chains: even ones build I, odd
    // ones Q. The first of each chain takes the input.
    //
    // SPEED: the two chains are independent, so each pass runs one
    // section of each. The I and Q values stay in registers, and while
    // one chain waits for its multiply-add (3 cycles on the M4) the CPU
    // works on the other. Picking the chain with an index (out[k & 1])
    // forced both through memory and took twice as long.
    float i = in, q = in;
    for (int k = 0; k < kNumFilters - 1; k += 2) {
      i = section(k, i);
      q = section(k + 1, q);
    }
    i = section(kNumFilters - 1, i); // 17 sections: I has the extra one
    i_ = i;
    q_ = q;
    return i_;
  }

  /** @return  The in-phase output of the last process() call */
  float i() const { return i_; }

  /** @return  The quadrature output (90 degrees behind i()) */
  float q() const { return q_; }

  /**
   * Compute the all-pass coefficients (Warps' design, for any rate).
   * @param sampleRate    Sample rate in Hz
   * @param coefficients  Receives kNumFilters values, sorted ascending
   */
  static void designPoles(float sampleRate, float *coefficients) {
    const double pi = 3.14159265358979323846;
    // 1. Elliptic low-pass prototype poles: half-band (bandwidth 0.495),
    // order 17 (C. Britton Rorabaugh, "Digital Filter Designer's
    // Handbook", p. 94).
    const double cutoff = 0.495;
    const int order = 17;
    double wp = cutoff * pi, ws = pi - wp;
    double k = tan(0.5 * wp) / tan(0.5 * ws);
    double r = pow(1.0 - k * k, 0.25);
    double u = 0.5 * (1.0 - r) / (1.0 + r);
    double q = u + 2.0 * pow(u, 5) + 15.0 * pow(u, 9) + 150.0 * pow(u, 13);
    double lpPoles[(order - 1) / 2 + 1];
    int numLp = 0;
    for (int i = 0; i < (order - 1) / 2; ++i) {
      double w = (i + 1) * pi / order;
      double num = 0.0, den = 0.0;
      for (int m = 0; m < 7; ++m)
        num +=
            ((m & 1) ? -1.0 : 1.0) * pow(q, m * (m + 1)) * sin((2 * m + 1) * w);
      for (int m = 1; m < 7; ++m)
        den += ((m & 1) ? -1.0 : 1.0) * pow(q, m * m) * cos(2 * m * w);
      double l = 2.0 * pow(q, 0.25) * num / (1.0 + 2.0 * den);
      double b = sqrt((1.0 - k * l * l) * (1.0 - l * l / k));
      double c = 2.0 * b / (1.0 + l * l);
      lpPoles[numLp++] = (2.0 - c) / (2.0 + c);
    }
    lpPoles[numLp++] = 0.0;

    // 2. Warp them into all-pass poles covering 10 Hz..20 kHz (capped
    // below Nyquist at low sample rates). Each nonzero pole gives two.
    double high = 0.45 * sampleRate < 20000.0 ? 0.45 * sampleRate : 20000.0;
    double beta =
        sqrt(tan(pi * 10.0 / sampleRate) * tan(pi * high / sampleRate));
    double bw = (beta - 1.0) / (beta + 1.0);
    double ap[kNumFilters];
    int numAp = 0;
    for (int i = 0; i < numLp; ++i) {
      double pole = lpPoles[i];
      double p = sqrt(pole);
      if (pole != 0.0) {
        ap[numAp++] = (p + bw) / (p * bw + 1.0);
        ap[numAp++] = (-p + bw) / (-p * bw + 1.0);
      } else {
        ap[numAp++] = bw;
      }
    }

    // 3. Sort ascending, as Warps stores them. An insertion sort: for 17
    // values it is short and needs no library code.
    for (int i = 1; i < numAp; ++i) {
      double v = ap[i];
      int j = i - 1;
      while (j >= 0 && ap[j] > v) {
        ap[j + 1] = ap[j];
        --j;
      }
      ap[j + 1] = v;
    }
    for (int i = 0; i < kNumFilters; ++i)
      coefficients[i] = static_cast<float>(ap[i]);
  }

private:
  /**
   * One first-order all-pass section: y = c * (x - y[n-1]) + x[n-1].
   * @param k  Section index
   * @param x  Input
   * @return   Output (the input of the next section in its chain)
   */
  float section(int k, float x) {
    float y = coefficient_[k] * (x - y_[k]) + x_[k];
    x_[k] = x;
    y_[k] = y;
    return y;
  }

  float coefficient_[kNumFilters] = {};
  float x_[kNumFilters] = {}, y_[kNumFilters] = {};
  float i_ = 0.0f, q_ = 0.0f;
};

} // namespace bb
