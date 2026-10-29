/**
 * @file table_wavefolder.h
 * @brief Plaits' table wavefolder: sine fold and analog-circuit fold.
 *
 * West Coast-style waveshaping that folds a signal back on itself as it
 * is driven harder, adding bright, shifting harmonics. Plaits' waveshaping
 * engine has two folding curves, both computed by this class:
 * - process(): a smooth, sine-based fold. Bright and bell-like at high
 *   amounts.
 * - analog(): a model of an analog folder circuit (five op-amp stages and
 *   a diode clipper, like the Buchla 259's timbre circuit). Rougher and
 *   more vocal.
 *
 * HOW THIS DIFFERS FROM WaveFolder (wave_folder.h): WaveFolder reflects
 * the signal at a threshold (sharp corners, simple maths). This one reads
 * smooth transfer curves from flash tables, with Hermite interpolation.
 *
 * HOW IT WORKS: the input, scaled by the fold amount, indexes a transfer
 * curve. Near the centre the curve is almost a straight line. Further out
 * it bends back and forth, so the input's peaks are "folded" into extra
 * wiggles, which are extra harmonics.
 *
 * ALIASING: folding creates harmonics far above the input's own. Feed it
 * smooth, band-limited waveforms (sine, triangle, filtered saw), and keep
 * the amount modest at high pitches.
 *
 * Derived from Plaits, Copyright 2016 Emilie Gillet. MIT licence.
 */

#pragma once

#include "buddhabox/core/dsp_common.h"
#include "buddhabox/fx/lut_wavefolder.h"

namespace bb {

/**
 * Table-based wavefolder with two curves.
 *
 * Example — a West Coast "bongo" voice:
 *   SineOscillator sine;
 *   TableWavefolder folder;
 *   folder.setAmount(0.6f);
 *
 *   float bell = folder.process(sine.process() * env);
 *   float growl = folder.analog(); // same sample, analog curve
 */
class TableWavefolder {
public:
  /** Steps in each transfer-curve table. */
  static const int kTableSize = 512;

  /** @param x  Fold amount: 0.0 = nearly clean, 1.0 = many folds */
  void setAmount(float x) { gain_ = 0.03f + 0.46f * clampf(x, 0.0f, 1.0f); }

  /**
   * Fold one sample.
   * @param in  Input, about ±1
   * @return    The smooth (sine) fold; analog() then holds the
   *            analog-circuit fold of the same sample
   */
  float process(float in) {
    // Map the input to a position on the curve: 0.5 is the centre. The
    // tables start one entry early (a Hermite guard point), hence + 1.
    float index = clampf(in * gain_ + 0.5f, 0.0f, 1.0f);
    analog_ = -interpolateHermite(lutFoldAnalog() + 1, index, kTableSize);
    return interpolateHermite(lutFoldSine() + 1, index, kTableSize);
  }

  /** @return  The analog-circuit fold from the last process() call */
  float analog() const { return analog_; }

private:
  float gain_ = 0.25f;
  float analog_ = 0.0f;
};

} // namespace bb
