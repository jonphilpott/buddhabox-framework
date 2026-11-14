# BuddhaBox

A header-only audio DSP framework for the **STM32F411CE "Black Pill"** (the
F401CC board works too) and an
I2S DAC (UDA1334A or MAX98357A). 79 classes: oscillators, noise, filters,
envelopes and modulators, dynamics, effects, physical models, drums and FM,
including ports of Mutable Instruments modules (via
[Partials](../partials)) reworked to fit a 96 MHz Cortex-M4 with 128 KB of RAM.

Every module processes one sample per `process()` call, works in floats
normalised to ±1, and allocates nothing at run time: buffers are sized by
template parameters, and lookup tables live in flash.

It also runs on the RP2350 (Raspberry Pi Pico 2 and other boards), with
the audio on the chip's second core: `pio run -e minimal_rp2350 -t upload`
(hold BOOTSEL while plugging in). I2S on GP10 (BCLK), GP11 (LRCLK) and
GP12 (DIN); see the guide.

Power the DAC board from 5 V (the 5V or VBUS pin), not 3.3 V.

## Set up the build environment

You need git, Python 3 and [PlatformIO Core](https://platformio.org) (the
`pio` command). The first build downloads everything else into
`~/.platformio`: the ARM compiler, the STM32 Arduino core and the ST-Link
upload tool, a few hundred MB in all.

### macOS

```sh
xcode-select --install     # git and clang++ (for the host tests)
brew install platformio    # from https://brew.sh
```

On macOS 26, the first time you plug in an ST-Link or a USB hub the system
asks whether to allow it. Until you do, uploads fail.

### Linux

Install git, Python (with `venv`) and clang, then PlatformIO with its
official installer:

```sh
sudo apt install git python3 python3-venv clang curl   # Debian, Ubuntu
# sudo dnf install git python3 clang curl              # Fedora

curl -fsSLO https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
echo 'export PATH="$PATH:$HOME/.platformio/penv/bin"' >> ~/.bashrc
```

(`pipx install platformio` also works.) Linux won't let an ordinary user
open the ST-Link or the board's USB serial port until you add
PlatformIO's udev rules and join the serial group:

```sh
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules \
  | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -a -G dialout,plugdev $USER   # Arch: uucp instead of dialout
```

Log out and back in for the groups to take effect.

### Check it works

```sh
git clone https://github.com/jonphilpott/buddhabox-framework.git
git clone https://github.com/jonphilpott/partials.git   # optional: port tests
cd buddhabox-framework
pio run -e minimal             # the first build downloads the toolchain
pio run -e minimal -t upload   # with an ST-Link wired up (see the guide)
```

`partials` must sit next to `buddhabox-framework`, because the port test
looks for it at `../partials`.

## Use it in a project

Create a new project from an example (default `minimal`):

```sh
python3 tools/new_project.py ../my_synth [--example custom_voice]
cd ../my_synth && pio run
```

Or add it to an existing project's `platformio.ini`:

```ini
[env:myproject]
platform = ststm32@20.0.0
board = blackpill_f411ce   ; or blackpill_f401cc
framework = arduino
upload_protocol = stlink   ; an ST-Link on the SW header; see the guide
build_unflags = -Os   ; the core adds it after -O2, and it wins
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
  `filter`, `mod`, `util`, `dynamics`, `fx`, `physical`, `drums`, `spectral`
- `src/buddhabox.h` — includes everything
- `examples/` — `minimal`, `singing_bowls`, `drum_machine`, `custom_voice`,
  `compile_all`, `profile`, `profile_all`, `patch_minimal`, `patch_demo`, `patch_hardware`, `patch_param` (each is a `pio run -e <name>` environment)
- `docs/index.html` — guide: wiring, conventions, budgets, composition
- `docs/reference.html` — full API, generated from the docstrings
- `docs/editor.html` — patch editor: wire modules, knobs, switches and LEDs,
  save `patch.json` (open it from disk; no server needed)
- `tests/compare_partials.cpp` — checks every port against Partials
- `tools/` — project, table, documentation and patch generators

## Commands

- `pio run` — build every example
- `pio run -e singing_bowls -t upload` — flash one with an ST-Link (DIO to
  SWDIO, SCK to SWCLK, GND; no buttons to press)
- `pio device monitor` — the examples print their CPU load; the blue LED
  lights if the audio ever overruns (`-D BB_OVERRUN_LED=0` turns that off)
- `pio run -e profile -t upload` — measure each module's cycles per sample
- `pio run -e profile_all -t upload` — the same for every audio module, as CSV
- `python3 tools/new_project.py ../my_synth` — start a new project
- `python3 tools/gen_docs.py` — regenerate the reference after editing docstrings
- `python3 tools/gen_patch.py path/patch.json` — generate a patch's C++ from JSON
- `python3 tools/gen_modules.py` — refresh the module list the editor and
  generator use

## Credits and licence

Many of the modules are ports of code from
[Mutable Instruments](https://mutable-instruments.net)' Eurorack
firmware and its stmlib library, by Emilie Gillet: oscillators,
resonators, reverbs, filters, envelopes and drums from Plaits, Rings,
Elements, Clouds, Peaks, Streams, Tides and Warps, among others. They
came through the [Partials](https://github.com/jonphilpott/partials)
library, and a ported file's header comment names the module it came
from. Any mistakes made in porting are ours, not Mutable Instruments'.

That code is Copyright (c) 2012-2023 Emilie Gillet and released under
the MIT licence. The full notice is in `LICENSE.mutable`, and it must be
kept with any copy of this code or substantial part of it.

The rest of the framework is Copyright (c) 2026 Jon Philpott, also under
the MIT licence: see `LICENSE`.

The original firmware is at
[github.com/pichenettes/eurorack](https://github.com/pichenettes/eurorack).
# buddhabox-framework
