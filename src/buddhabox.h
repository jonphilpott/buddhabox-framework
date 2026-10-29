/**
 * @file buddhabox.h
 * @brief Includes every BuddhaBox module at once.
 *
 * Handy for sketches and experiments. Larger projects can include only the
 * headers they use (e.g. "buddhabox/osc/oscillator.h"), which compiles a
 * little faster. Either way, nothing is linked unless you use it: the
 * modules are header-only and unused tables are dropped by the linker.
 */

#pragma once

// ── core ──
#include "buddhabox/core/dsp_common.h"
#include "buddhabox/core/lut_sine.h"
#include "buddhabox/core/luts.h"
#include "buddhabox/core/midi_utils.h"
#include "buddhabox/core/random.h"

// ── engine ──
#include "buddhabox/engine/audio_engine.h"

// ── io ──
#include "buddhabox/io/analog_input.h"
#include "buddhabox/io/led.h"
#include "buddhabox/io/matrix_keypad.h"
#include "buddhabox/io/mpr121_input.h"
#include "buddhabox/io/mux_input.h"
#include "buddhabox/io/switch_input.h"

// ── osc ──
#include "buddhabox/osc/basic.h"
#include "buddhabox/osc/formant.h"
#include "buddhabox/osc/grainlet.h"
#include "buddhabox/osc/harmonic.h"
#include "buddhabox/osc/lfo.h"
#include "buddhabox/osc/oscillator.h"
#include "buddhabox/osc/polyblep.h"
#include "buddhabox/osc/sine.h"
#include "buddhabox/osc/string_synth.h"
#include "buddhabox/osc/variable_shape.h"
#include "buddhabox/osc/vosim.h"
#include "buddhabox/osc/z_osc.h"

// ── noise ──
#include "buddhabox/noise/clocked_noise.h"
#include "buddhabox/noise/dust.h"
#include "buddhabox/noise/noise.h"
#include "buddhabox/noise/particle.h"
#include "buddhabox/noise/smooth_random.h"

// ── filter ──
#include "buddhabox/filter/allpass.h"
#include "buddhabox/filter/dc_blocker.h"
#include "buddhabox/filter/hilbert.h"
#include "buddhabox/filter/moog_filter.h"
#include "buddhabox/filter/one_pole.h"
#include "buddhabox/filter/sv_filter.h"
#include "buddhabox/filter/svf.h"
#include "buddhabox/filter/vowel.h"

// ── mod ──
#include "buddhabox/mod/bouncing_ball.h"
#include "buddhabox/mod/clock.h"
#include "buddhabox/mod/clock_to_ramp.h"
#include "buddhabox/mod/colour.h"
#include "buddhabox/mod/envelope.h"
#include "buddhabox/mod/hysteresis_quantizer.h"
#include "buddhabox/mod/lag.h"
#include "buddhabox/mod/lorenz.h"
#include "buddhabox/mod/lut_slope.h"
#include "buddhabox/mod/quantizer.h"
#include "buddhabox/mod/random_sequence.h"
#include "buddhabox/mod/sample_and_hold.h"
#include "buddhabox/mod/segment_envelope.h"
#include "buddhabox/mod/slew.h"
#include "buddhabox/mod/slope.h"

// ── dynamics ──
#include "buddhabox/dynamics/compressor.h"
#include "buddhabox/dynamics/follower.h"
#include "buddhabox/dynamics/low_pass_gate.h"
#include "buddhabox/dynamics/vactrol.h"
#include "buddhabox/dynamics/vactrol_lpg.h"

// ── fx ──
#include "buddhabox/fx/chorus.h"
#include "buddhabox/fx/decimator.h"
#include "buddhabox/fx/delay.h"
#include "buddhabox/fx/diffuser.h"
#include "buddhabox/fx/ensemble.h"
#include "buddhabox/fx/fx_engine.h"
#include "buddhabox/fx/limiter.h"
#include "buddhabox/fx/lut_wavefolder.h"
#include "buddhabox/fx/overdrive.h"
#include "buddhabox/fx/pitch_shifter.h"
#include "buddhabox/fx/reverb.h"
#include "buddhabox/fx/table_wavefolder.h"
#include "buddhabox/fx/wave_folder.h"

// ── physical ──
#include "buddhabox/physical/karplus.h"
#include "buddhabox/physical/modal_resonator.h"
#include "buddhabox/physical/plucker.h"
#include "buddhabox/physical/string_model.h"
#include "buddhabox/physical/tube.h"

// ── drums ──
#include "buddhabox/drums/analog_kick.h"
#include "buddhabox/drums/analog_snare.h"
#include "buddhabox/drums/fm_drum.h"
#include "buddhabox/drums/hihat.h"
#include "buddhabox/drums/synth_kick.h"
#include "buddhabox/drums/synth_snare.h"

// ── spectral ──
#include "buddhabox/spectral/fm_operators.h"
#include "buddhabox/spectral/xmod.h"
