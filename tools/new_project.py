#!/usr/bin/env python3
"""Create a new PlatformIO project that builds on the BuddhaBox framework.

Usage:
    python3 tools/new_project.py ../my_synth
    python3 tools/new_project.py ../bells --example custom_voice

What it writes into the new folder:
    platformio.ini   Black Pill settings plus a lib_deps link back to
                     this checkout
    src/main.cpp     a copy of one of the examples (default: minimal) to
                     start from
    .gitignore       ignores PlatformIO's build folder

HOW THE LINK WORKS: lib_deps = symlink://<path> tells PlatformIO to use
this folder as a library in place, rather than copying it. Edits to the
framework show up in every project on the next build. The path is
written relative to the new project when the two share a parent folder
(for example, both under ~/git), so they can move together without
breaking it; otherwise it is absolute.

Uses only the Python standard library.
"""

import argparse
import os
import shutil
import sys

# The framework's root: this script lives in <root>/tools/.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLES = os.path.join(ROOT, "examples")

PLATFORMIO_INI = """\
; {name} — built on the BuddhaBox framework.
;
;   pio run              build
;   pio run -t upload    flash (DFU: hold BOOT0, tap NRST, release BOOT0)
;   pio device monitor   serial output

[env:{name}]
platform = ststm32@15.4.1
board = blackpill_f401cc
framework = arduino
upload_protocol = dfu
; The STM32 core adds -Os after build_flags and GCC obeys the last -O,
; so remove it or -O2 is silently ignored.
build_unflags = -Os
build_flags =
    -O2
    -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
    -D USBCON
;   -D BB_OVERRUN_LED=0   ; frees the blue LED (PC13) from the CPU overrun
;                         ; indicator, e.g. to use UserLED
lib_deps =
    symlink://{framework}
"""

GITIGNORE = ".pio\n.vscode\n.DS_Store\n"


def main():
    examples = sorted(os.listdir(EXAMPLES))
    parser = argparse.ArgumentParser(
        description="Create a new project on the BuddhaBox framework.")
    parser.add_argument("path", help="folder to create, e.g. ../my_synth")
    parser.add_argument("--example", default="minimal", choices=examples,
                        help="example to copy as src/main.cpp")
    args = parser.parse_args()

    # 1. Refuse to write over anything that is already there.
    target = os.path.abspath(args.path)
    if os.path.exists(target) and os.listdir(target):
        sys.exit("error: %s exists and is not empty" % target)

    # 2. The environment name must be a valid PlatformIO section name,
    #    so keep letters, digits and underscores from the folder name.
    name = "".join(c if c.isalnum() else "_"
                   for c in os.path.basename(target)) or "project"

    # 3. Write the three files.
    os.makedirs(os.path.join(target, "src"), exist_ok=True)
    # Relative when the two folders share a parent below /, otherwise a
    # run of ../../.. up to the root would be fragile: use absolute.
    if os.path.commonpath([ROOT, target]) == os.sep:
        framework = ROOT
    else:
        framework = os.path.relpath(ROOT, target)
    with open(os.path.join(target, "platformio.ini"), "w") as f:
        f.write(PLATFORMIO_INI.format(name=name, framework=framework))
    with open(os.path.join(target, ".gitignore"), "w") as f:
        f.write(GITIGNORE)
    shutil.copy(os.path.join(EXAMPLES, args.example, "main.cpp"),
                os.path.join(target, "src", "main.cpp"))

    print("created %s (from examples/%s)" % (target, args.example))
    print("next: cd %s && pio run" % args.path)


if __name__ == "__main__":
    main()
