/**
 * @file wave_folder.h
 * @brief Single-stage wave folder for west-coast/Buchla-style synthesis.
 *
 * Wave folding is a form of waveshaping that reflects the signal back on
 * itself when it crosses a threshold — like folding a piece of paper.
 * Where soft clipping rounds off peaks smoothly, a wave folder bounces
 * the signal back in the opposite direction, adding harmonics that are
 * rich and metallic rather than warm and even.
 *
 * HOW THE ALGORITHM WORKS:
 * ────────────────────────
 * For a given threshold T:
 *
 *   - If input >  T: output = 2T - input  (reflect back downward)
 *   - If input < -T: output = -2T - input (reflect back upward)
 *   - Otherwise:     output = input       (pass through unchanged)
 *
 * This is a single folding stage, equivalent to one op-amp fold circuit
 * in the original Buchla hardware. Stack multiple WaveFolder instances in
 * series to add more folds — each stage multiplies harmonic complexity.
 *
 * DRIVING THE FOLDER:
 * ───────────────────
 * A signal at full amplitude (-1.0 to +1.0) with a high threshold will
 * barely fold. To get harmonic richness you need the signal to *exceed*
 * the threshold. There are two ways:
 *
 *   1. Reduce the threshold (e.g., setThreshold(0.3f)) so a normal-level
 *      signal clips and folds.
 *   2. Pre-amplify the signal before passing it in:
 *        folder.process(osc.process() * 3.0f)
 *
 * The user is responsible for managing signal levels — if a very hot
 * signal only folds once per stage, chain more stages.
 *
 * WEST-COAST CONTEXT:
 * ───────────────────
 * Wave folding is a defining technique of "west-coast synthesis" (Buchla),
 * as opposed to the "east-coast" (Moog) approach of subtractive filtering.
 * Rather than removing harmonics from a complex source, you ADD harmonics
 * to a simple source by pushing it through non-linearities like folders.
 * A sine wave into a wave folder produces increasingly complex waveforms
 * as the drive increases — going from pure sine, through harmonically rich
 * "folded" shapes, all the way to metallic, inharmonic timbres.
 *
 * Example usage — basic folding:
 *   WaveFolder folder;
 *   folder.setThreshold(0.7f);
 *
 *   // In audio callback:
 *   float folded = folder.process(osc.process() * 2.0f);
 *
 * Example usage — two stages for more complexity:
 *   WaveFolder fold1, fold2;
 *   fold1.setThreshold(0.8f);
 *   fold2.setThreshold(0.6f);
 *
 *   // In audio callback:
 *   float sig = osc.process() * 3.0f;
 *   sig = fold2.process(fold1.process(sig));
 *
 * Example usage — LFO-modulated threshold (animated timbre):
 *   WaveFolder folder;
 *   LFO lfo;
 *   lfo.setFrequency(0.2f);
 *
 *   // In audio callback:
 *   float thresh = mapf(lfo.process(), -1.0f, 1.0f, 0.2f, 0.9f);
 *   folder.setThreshold(thresh);
 *   float out = folder.process(osc.process() * 2.5f);
 */

#pragma once

#include "buddhabox/core/dsp_common.h"

namespace bb {

/**
 * Single-stage wave folder for west-coast/Buchla-style synthesis.
 *
 * See the file header above for how it works and usage examples.
 */
class WaveFolder {
public:
  /**
   * Construct a WaveFolder with a default threshold of 0.8.
   *
   * A threshold of 0.8 means signals between -0.8 and +0.8 pass unchanged.
   * Values outside that range are reflected back.
   */
  WaveFolder() = default;

  /**
   * Set the folding threshold.
   *
   * This is the level at which the signal is reflected. Lower thresholds
   * fold more aggressively, adding more harmonics. Combine with pre-gain
   * on the input for rich, metallic timbres.
   *
   * @param t  Threshold level, 0.001 to 1.0.
   *           0.8 = gentle fold (signal must exceed ±0.8 to fold)
   *           0.5 = moderate fold
   *           0.2 = aggressive fold (folds even moderate-level signals)
   *
   * Example:
   *   folder.setThreshold(0.5f);  // Fold at ±0.5
   */
  void setThreshold(float t) { threshold_ = clampf(t, 0.001f, 1.0f); }

  /**
   * Process one audio sample through the wave folder.
   *
   * Signal values within ±threshold pass through unchanged.
   * Values outside that range are reflected back — each reflection
   * inverts and offsets the excess, creating the characteristic
   * fold shape.
   *
   * @param input  Audio sample to fold. Any range is safe; a signal
   *               that never exceeds ±threshold will pass unchanged.
   * @return       Folded sample. May be outside the input range
   *               momentarily at fold boundaries but will not grow
   *               unboundedly for bounded inputs.
   *
   * Example:
   *   float out = folder.process(osc.process() * 2.0f);
   */
  float process(float input) {
    // Upper fold: if the signal has exceeded +threshold, reflect it
    // back downward. The formula 2*T - x mirrors the value around T:
    //   input = T + excess → output = T - excess
    if (input > threshold_)
      return 2.0f * threshold_ - input;

    // Lower fold: same logic for the negative side.
    if (input < -threshold_)
      return -2.0f * threshold_ - input;

    // Signal is within range — no folding needed, pass through.
    return input;
  }

  /** @return  The current threshold value. */
  float getThreshold() const { return threshold_; }

private:
  // Threshold at which folding occurs. Signals beyond ±threshold_ are
  // reflected back. Default 0.8 means a full-scale sine (-1 to +1) is
  // barely clipped — drive the input harder for interesting folds.
  float threshold_ = 0.8f;
};

} // namespace bb
