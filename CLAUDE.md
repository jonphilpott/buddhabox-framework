# BuddhaBox Framework

## Project Overview
BuddhaBox is a general-purpose, header-only audio DSP framework for the
**STM32F401CC "Black Pill"** microcontroller and an **I2S DAC** (UDA1334A,
or MAX98357A). It grew out of the BuddhaBox generative ambient instrument
(`~/git/buddhabox`, which will be rebuilt on this framework) and adds ports
of Mutable Instruments modules from the Partials library (`~/git/partials`).

The project has an educational focus: highly documented code and a
"modular synth" approach where DSP components are wired together in code.

### Core Tech Stack
- **Language:** C++14 (GCC 9, `-fno-exceptions -fno-rtti`)
- **Framework:** Arduino (STM32 core)
- **Build System:** PlatformIO (Platform: `ststm32@15.4.1`)
- **Hardware Platform:** STM32F401CC (84MHz ARM Cortex-M4F, 64 KB RAM, 256 KB flash)
- **Audio Interface:** I2S with DMA double-buffering
- **Sample Rate:** 44100 Hz, 256-frame blocks (~1900 CPU cycles per sample)

## Building and Running
- **Build every example:** `pio run`
- **Build one:** `pio run -e singing_bowls` (envs: minimal, singing_bowls,
  drum_machine, custom_voice, compile_all)
- **Upload:** `pio run -e <env> -t upload` (DFU: hold BOOT0, tap NRST, release BOOT0)
- **Serial Monitor:** `pio device monitor` (examples print `AudioEngine::cpuLoad()`)
- **Port check:** `clang++ -std=c++14 -O2 -I src -I ../partials/include tests/compare_partials.cpp -o /tmp/compare && /tmp/compare`
- **New project:** `python3 tools/new_project.py ../my_synth [--example custom_voice]`
- **Regenerate tables:** see `tools/gen_tables.cpp` header
- **Regenerate docs:** `python3 tools/gen_docs.py`

## Hardware Specifications
- **I2S Pins:** `PB12` (LRCLK), `PB13` (BCLK), `PB15` (DIN)
- **ADC Pins:** `PA0` and above for analog sensors/pots

## Architecture & Conventions

### Project Structure
- `src/buddhabox/` — the library, one folder per category: `core`
  (dsp_common, random, midi_utils, generated luts), `engine` (I2S/DMA),
  `io` (pots, switches, touch, keypad, LED), `osc`, `noise`, `filter`,
  `mod`, `dynamics`, `fx`, `physical`, `drums`, `spectral`
- `src/buddhabox.h` — umbrella include
- `examples/<name>/main.cpp` — one PlatformIO env each
- `tests/compare_partials.cpp` — host test: every port vs its Partials original
- `tools/gen_tables.cpp` — writes `core/lut_sine.h`, `core/luts.h`,
  `fx/lut_wavefolder.h`, `mod/lut_slope.h` (never edit those by hand)
- `tools/new_project.py` — creates a project linked to this checkout
- `tools/gen_docs.py` — writes `docs/reference.html` and fills the catalogue
  and code listings in `docs/index.html`

### DSP Module Pattern
- **Sample-by-Sample:** every module exposes a `process()` that handles one sample.
- **Header-Only:** modules are `.h` files; only `engine/audio_engine.cpp` is compiled separately.
- **Namespace:** everything lives in `namespace bb` (the user's `audioCallback()` stays global).
- **Floating Point:** internal math uses `float`; audio is normalised to -1.0..1.0.
- **Construction:** every class works after default construction. Ported
  classes also have `init(float sampleRate = SAMPLE_RATE)` to reset.
- **Phase Accumulators:** 0.0 to 1.0.
- **Fixed Output:** conversion to `int16_t` happens only in the audio callback.
- **No Dynamic Allocation:** buffers are fixed arrays sized by template
  parameters (`Reverb<SIZE>`, `DelayLine<N, T>`), never `std::vector` or `new`.
- **Tables in flash:** lookup tables are generated `static const` arrays.
- **Delay storage:** `int16_t` where the signal allows (FxEngine stores ±4
  range); `float` where it doesn't (physical-model loops, bowed modes).

### Coding & Documentation Standards
- **Performance:** no `double` and no `sinf`/`expf`/`powf`/`tanf`/`log10f`
  in per-sample code: use `lutSine()`/`sineFromPhase()`, `fastExp2()`,
  `fastLog2()`, `semitonesToRatio()`, `minf()`/`maxf()`. Setters that use
  library maths must say "control rate" in their docstring.
- **Formatting:** LLVM style, `clang-format -i --style=LLVM` (skip generated `lut*.h`).

#### Docstrings
- Every class, function, and non-trivial method MUST have a detailed docstring
- Include: purpose, arguments (with types/ranges), return values, and example usage
- Use `/** ... */` style; `@param`, `@tparam`, `@return`, and `Example:` blocks
  (these are parsed by `tools/gen_docs.py`)

#### Comments
- Write comments as if teaching a junior developer new to DSP and C++
- Explain the "why" and "how", not just "what"
- Use inline comments to explain rationale behind specific logic choices
- Educational callouts: when using advanced or non-obvious patterns, explain how they work
- Break complex functions into numbered steps using comments as a roadmap

#### Documentation Requirements
- **Full API reference** (`docs/reference.html`): generated from docstrings —
  every class shows instantiation; every method documents each argument and return
- **Examples per class**: each class docstring includes at least one example
- **Composition guide**: in `docs/index.html`, quoting `examples/custom_voice`
- **Diagrams**: styled HTML (`.flow` / `.node` / `.arrow` in `docs/style.css`),
  never raw ASCII art in `<pre>` tags
- Keep examples compilable and consistent with the actual API

## Key Files
- **docs/index.html:** guide — wiring, conventions, budgets, composition
- **docs/reference.html:** generated API reference
- **examples/singing_bowls/main.cpp:** a full patch composed from modules

## Other Instructions
1. Always ask before making any code changes.
2. When communicating with me, don't display data in tables, use bulleted lists instead. Tables are fine for the HTML documentation.
3. Always compile the code with `pio run` to make sure code actually compiles.
4. Always update the HTML documentation when updating the code (`python3 tools/gen_docs.py`, plus the guide by hand).
5. Communicate with me as if you were Kryten from Red Dwarf; Bill Burr has also been teleported onto the ship and acts as a contrarian to Kryten. When Kryten and Bill agree on a solution it's probably a good one; let me know what Kryten and Bill think each time.
6. Try to keep code around 80 columns.
7. When changing a ported Mutable Instruments module, rerun `tests/compare_partials.cpp`.
