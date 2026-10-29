# BuddhaBox

A header-only audio DSP framework for the **STM32F401CC "Black Pill"** and an
I2S DAC (UDA1334A or MAX98357A). 79 classes: oscillators, noise, filters,
envelopes and modulators, dynamics, effects, physical models, drums and FM,
including ports of Mutable Instruments modules (via
[Partials](../partials)) reworked to fit an 84 MHz Cortex-M4 with 64 KB of RAM.

Every module processes one sample per `process()` call, works in floats
normalised to ±1, and allocates nothing at run time: buffers are sized by
template parameters, and lookup tables live in flash.

## Use it in a project

Create a new project from an example (default `minimal`):

```sh
python3 tools/new_project.py ../my_synth [--example custom_voice]
cd ../my_synth && pio run
```

Or add it to an existing project's `platformio.ini`:

```ini
[env:myproject]
platform = ststm32@15.4.1
board = blackpill_f401cc
framework = arduino
upload_protocol = dfu
build_flags = -O2 -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC -D USBCON
lib_deps = symlink://../buddhabox-framework
```

```cpp
#include "buddhabox/engine/audio_engine.h"
#include "buddhabox/osc/oscillator.h"
using namespace bb;

Oscillator osc;

void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2)
    buffer[i] = buffer[i + 1] = (int16_t)(osc.process() * 32000.0f);
}

void setup() {
  osc.setFrequency(220.0f);
  AudioEngine::begin();
}

void loop() {}
```

## Layout

- `src/buddhabox/` — the framework: `core`, `engine`, `io`, `osc`, `noise`,
  `filter`, `mod`, `dynamics`, `fx`, `physical`, `drums`, `spectral`
- `src/buddhabox.h` — includes everything
- `examples/` — `minimal`, `singing_bowls`, `drum_machine`, `custom_voice`,
  `compile_all` (each is a `pio run -e <name>` environment)
- `docs/index.html` — guide: wiring, conventions, budgets, composition
- `docs/reference.html` — full API, generated from the docstrings
- `tests/compare_partials.cpp` — checks every port against Partials
- `tools/` — project, table and documentation generators

## Commands

- `pio run` — build every example
- `pio run -e singing_bowls -t upload` — flash one (DFU: hold BOOT0, tap NRST)
- `pio device monitor` — the examples print their CPU load
- `python3 tools/new_project.py ../my_synth` — start a new project
- `python3 tools/gen_docs.py` — regenerate the reference after editing docstrings

## Licence

MIT. Modules derived from Mutable Instruments firmware and stmlib are
Copyright (c) 2012-2023 Emilie Gillet; see `LICENSE`.
# buddhabox-framework
