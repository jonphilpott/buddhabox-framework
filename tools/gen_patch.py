#!/usr/bin/env python3
"""
Generate patch_graph.h from a patch.json: the C++ that creates a patch's
modules, sets their starting parameters and wires them together.

WHY: a patch's signal flow is plumbing: declare objects, call setters,
pass each output to the next input in the right order. A tool (and later
a graphical editor) can write that; a person writes the musical logic.
So the generated header is never edited by hand, and main.cpp, which
calls it, belongs to you and is only written if it doesn't exist yet.

USAGE (from the repository root):
    python3 tools/gen_patch.py examples/patch_minimal/patch.json

PATCH FORMAT:
    {
      "modules": {
        "osc": {"type": "Oscillator",
                "set": {"setFrequency": 220, "setWaveform": "Waveform::SINE"}},
        "verb": {"type": "Reverb", "template": "8192"}
      },
      "wires": [["osc.out", "verb.left"], ["verb.left", "out.left"]]
    }
- `type` is a class from tools/modules.json; `template` overrides its
  template arguments.
- `set` calls each setter once in setup(); a list passes several
  arguments (`"setFrequency": [800, 0.7]` on an Svf).
- A wire joins `module.output` to one of three things on a module:
  - an input (`filt.in`): fed every sample. `out.left` and `out.right`
    are the speakers. Several wires into one input are summed, each
    scaled by its `amount` if given (a mix level); an unwired input gets
    its default (or 0). A float into a bool input counts as
    true above 0.5.
  - a setter (`filt.setFrequency`): modulation. The setter's first
    argument becomes its `set` value plus `amount` times the signal, e.g.
    ["lfo.out", "filt.setFrequency", {"amount": 600}] sweeps 800 Hz by
    +-600 Hz. Other arguments keep their `set` values. Setters are slow
    (filter cutoffs use tanf), so these run at control rate: once every
    `control_every` samples (top-level key, default 32).
  - an event: `trigger` fires when the signal rises past 0.5, `gate`
    follows it (true above 0.5). Checked every sample, because a clock
    pulse lasts only one sample.
- Module ids may not end in `_`: generated names do (`osc_v_`), so the
  two can never collide.
- Hardware: {"type": "Knob", "pin": "PA1"} gives 0..1, a "Switch" gives
  1.0 while closed, an "LED" lights while its `in` is above 0.5. They
  are read and driven in patch::control(), which main.cpp calls from
  loop(), never in the audio interrupt (analogRead() is far too slow
  there). Each value crosses between the two through a volatile float.
  tools/modules.json lists the pins each may use.

HOW IT WORKS:
1. Check every module type, setter and wire against modules.json.
2. Sort the modules so each runs after the modules feeding it (a
   topological sort). A loop of wires has no such order and is refused.
3. Write the header: includes, objects, setup(), process().
4. Add up the measured CPU and RAM costs and print them. Modulated
   setters add their cost divided by `control_every`.
"""

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULES = os.path.join(ROOT, 'tools', 'modules.json')

BUDGET = round(84e6 / 44100)  # CPU cycles per sample: 1905
SAFE = 0.8             # warn above this share of the estimate
# ponytail: assumed cost of a setter profile_all hasn't measured. The
# measured ones run 20-190 cycles; measure the setter to replace this.
UNMEASURED_SETTER = 200
# RAM used by the framework itself (engine, DMA buffers, USB): the
# `minimal` example's 6124 bytes less its 20-byte oscillator.
BASE_RAM = 6104
RAM_SIZE = 65536

MAIN_CPP = '''/**
 * @file main.cpp
 * @brief Runs the patch in patch_graph.h. This file is yours: edit it
 *        freely. gen_patch.py wrote it once and will not overwrite it.
 *
 * patch_graph.h is generated from patch.json: change the patch there and
 * rerun tools/gen_patch.py. Reach any module from here as patch::<id>,
 * e.g. patch::osc.setFrequency(440.0f), to add your own logic.
 */

#include <Arduino.h>

#include "buddhabox/engine/audio_engine.h"
#include "patch_graph.h"

using namespace bb;

/**
 * Fill one block of audio. Runs in interrupt context: no Serial, no
 * analogRead(), no allocation.
 *
 * @param buffer  Interleaved stereo int16 samples [L, R, L, R, ...]
 * @param length  Number of int16 values (frames * 2)
 */
void audioCallback(int16_t *buffer, uint16_t length) {
  for (uint16_t i = 0; i < length; i += 2) {
    float left, right;
    patch::process(left, right);
    buffer[i] = (int16_t)(clampf(left, -1.0f, 1.0f) * 32000.0f);
    buffer[i + 1] = (int16_t)(clampf(right, -1.0f, 1.0f) * 32000.0f);
  }
}

void setup() {
  patch::setup();
  Serial.begin(115200); // USB serial, for the CPU meter below
  AudioEngine::begin();
}

/**
 * Print the CPU load once a second. `peak` is the worst block since the
 * last print, so it catches spikes the once-a-second load would miss.
 * Above 100% a block ran late (the blue LED lights). If nothing prints
 * at all, the patch is over budget: the audio interrupt never leaves
 * time for loop() to run.
 */
void loop() {
  // Knobs, switches and LEDs. Call often: a knob is only as smooth as
  // this loop is fast, so never delay() here.
  patch::control();

  // millis() rather than delay(1000), so control() keeps running.
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.print("CPU: ");
    Serial.print(AudioEngine::cpuLoad() * 100.0f, 1);
    Serial.print("%  peak: ");
    Serial.print(AudioEngine::cpuPeak() * 100.0f, 1);
    Serial.println("%");
  }
}
'''


def fail(msg):
    sys.exit('gen_patch: ' + msg)


def literal(value, ctype):
    """A JSON value as C++ source for an argument of type `ctype`."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)) and ctype == 'float':
        return repr(float(value)) + 'f'
    return str(value)  # ints, and enum names such as Waveform::SINE


def port(ref):
    """'osc.out' -> ('osc', 'out')."""
    if not re.match(r'^\w+\.\w+$', ref):
        fail('bad port "%s": expected module.port' % ref)
    return tuple(ref.split('.'))


def main():
    if len(sys.argv) != 2:
        fail('usage: gen_patch.py path/to/patch.json')
    path = sys.argv[1]
    out_dir = os.path.dirname(os.path.abspath(path))
    patch = json.load(open(path))
    lib = json.load(open(MODULES))
    mods = patch.get('modules', {})
    every = int(patch.get('control_every', 32))

    # 1. Check modules, setters and wires.
    for mid, m in mods.items():
        if not re.match(r'^[A-Za-z_]\w*$', mid) or mid == 'out' or \
                mid.endswith('_'):
            fail('module id "%s" must be a C++ name, not "out", and not '
                 'end in "_"' % mid)
        if m['type'] not in lib:
            fail('%s: unknown type %s' % (mid, m['type']))
        for setter in m.get('set', {}):
            if setter not in lib[m['type']]['params']:
                fail('%s: %s has no %s()' % (mid, m['type'], setter))
        if lib[m['type']].get('hardware'):
            if m.get('pin') not in lib[m['type']]['pins']:
                fail('%s: a %s needs "pin", one of %s' % (
                    mid, m['type'], ', '.join(lib[m['type']]['pins'])))
    pins = [m['pin'] for m in mods.values() if 'pin' in m]
    for pin in sorted({p for p in pins if pins.count(p) > 1}):
        fail('pin %s is used twice' % pin)
    # Sort each wire by what it ends on: an input (`wires`), a setter
    # (`mods_in`, with its amount) or an event (`events_in`).
    wires, mods_in, events_in = [], [], []
    gain = {}  # (src, sport, dst, dport) -> amount, for input wires
    for w in patch.get('wires', []):
        if len(w) not in (2, 3):
            fail('a wire is ["from", "to"] or ["from", "to", {options}]')
        (src, sport), (dst, dport) = port(w[0]), port(w[1])
        gain[src, sport, dst, dport] = (w[2] if len(w) == 3 else {}).get(
            'amount', 1.0)
        if src not in mods:
            fail('wire from unknown module "%s"' % src)
        if sport not in [o['name'] for o in lib[mods[src]['type']]['outputs']]:
            fail('%s has no output "%s"' % (src, sport))
        if dst == 'out':
            if dport not in ('left', 'right'):
                fail('out has inputs left and right, not "%s"' % dport)
            wires.append(((src, sport), (dst, dport)))
            continue
        if dst not in mods:
            fail('wire to unknown module "%s"' % dst)
        spec = lib[mods[dst]['type']]
        if dport in [i['name'] for i in spec['inputs']]:
            wires.append(((src, sport), (dst, dport)))
        elif dport in spec['params']:
            if spec['params'][dport]['args'][0]['type'] != 'float':
                fail('%s.%s: only a setter whose first argument is a float '
                     'can be modulated' % (dst, dport))
            mods_in.append(((src, sport), (dst, dport),
                            gain[src, sport, dst, dport]))
        elif dport in spec['events']:
            events_in.append(((src, sport), (dst, dport)))
        else:
            fail('%s has no input, setter or event "%s"' % (dst, dport))
    links = wires + [w[:2] for w in mods_in] + events_in

    # 2. Topological sort (Kahn's algorithm): repeatedly take a module
    #    whose feeders have all been placed. If none is left to take but
    #    modules remain, the wires form a loop.
    feeders = {mid: {s for (s, _), (d, _) in links if d == mid}
               for mid in mods}
    order = []
    while len(order) < len(mods):
        ready = [mid for mid in mods if mid not in order and
                 feeders[mid] <= set(order)]
        if not ready:
            loop = [mid for mid in mods if mid not in order]
            fail('wires form a loop through: ' + ', '.join(loop))
        order.append(ready[0])

    def raw_expr(mid, name):
        """An output's expression with {id} still in it."""
        outs = lib[mods[mid]['type']]['outputs']
        return next(o['expr'] for o in outs if o['name'] == name)

    def output_expr(mid, name):
        return raw_expr(mid, name).replace('{id}', mid)

    def input_expr(mid, name, ctype='float', default=None):
        """The C++ feeding one input: its wires summed, or a default."""
        srcs = []
        for (s, sp), (d, dp) in wires:
            if d == mid and dp == name:
                g = gain[s, sp, d, dp]
                srcs.append(output_expr(s, sp) if g == 1 else
                            '%s * %s' % (literal(g, 'float'), output_expr(s, sp)))
        if not srcs:
            return default or ('false' if ctype == 'bool' else '0.0f')
        expr = ' + '.join(srcs)
        return '(%s) > 0.5f' % expr if ctype == 'bool' else expr

    # 3. Write the header.
    includes = sorted({lib[m['type']]['header'] for m in mods.values()} -
                      {None})
    hw = [mid for mid in order if lib[mods[mid]['type']].get('hardware')]
    if hw:
        includes.insert(0, 'Arduino.h')
    objects, setters, body, control = [], [], [], []
    if mods_in:
        objects.append('int ctl_count_ = 0; // samples since last control '
                       'update')
        body.append('  // Control rate: update modulated setters once every '
                    '%d samples.' % every)
        body.append('  const bool ctl_ = ++ctl_count_ >= %d;' % every)
        body.append('  if (ctl_)')
        body.append('    ctl_count_ = 0;')
    for mid in order:
        m, spec = mods[mid], lib[mods[mid]['type']]

        # Hardware: read or driven in control(), shared with process()
        # through a volatile float. `volatile` tells the compiler another
        # context (loop() or the interrupt) changes it, so it must really
        # read memory each time instead of reusing an old value.
        if spec.get('hardware'):
            pin, kind = m['pin'], m['type']
            objects.append('volatile float %s_v_ = 0.0f; // shared with '
                           'the audio interrupt' % mid)
            if kind == 'Knob':
                objects.append('AnalogInput %s(%s);' % (mid, pin))
                control.append('  %s.update();' % mid)
                control.append('  %s_v_ = %s.read();' % (mid, mid))
            elif kind == 'Switch':
                objects.append('SwitchInput %s(%s);' % (mid, pin))
                setters.append('  %s.begin();' % mid)
                control.append('  %s.update();' % mid)
                control.append('  %s_v_ = %s.isHeld() ? 1.0f : 0.0f;' %
                               (mid, mid))
            else:  # LED. PC13, the on-board LED, lights when driven LOW.
                on, off = ('LOW', 'HIGH') if pin == 'PC13' else ('HIGH', 'LOW')
                setters.append('  pinMode(%s, OUTPUT);' % pin)
                control.append('  digitalWrite(%s, %s_v_ > 0.5f ? %s : %s);'
                               % (pin, mid, on, off))
                body.append('  %s_v_ = %s;' % (mid, input_expr(mid, 'in')))
            continue

        tmpl = '<%s>' % m['template'] if 'template' in m else (
            '<>' if spec['template'] else '')
        objects.append('%s%s %s;' % (m['type'], tmpl, mid))
        for setter, value in m.get('set', {}).items():
            args = spec['params'][setter]['args']
            values = value if isinstance(value, list) else [value]
            if len(values) != len(args):
                fail('%s.%s takes %d arguments' % (mid, setter, len(args)))
            setters.append('  %s.%s(%s);' % (mid, setter, ', '.join(
                literal(v, a['type']) for v, a in zip(values, args))))

        # Events: `trigger` fires on a rising edge, so it remembers whether
        # its input was high last sample; `gate` simply follows the level.
        for event in sorted({dp for _, (d, dp) in events_in if d == mid}):
            level = '(%s) > 0.5f' % ' + '.join(
                output_expr(s, sp) for (s, sp), (d, dp) in events_in
                if d == mid and dp == event)
            if event == 'gate':
                body.append('  %s.gate(%s);' % (mid, level))
                continue
            was = '%s_%s_was_' % (mid, event)
            objects.append('bool %s = false; // input high last sample?'
                           % was)
            body.append('  {')
            body.append('    const bool high = %s;' % level)
            body.append('    if (high && !%s)' % was)
            body.append('      %s.%s();' % (mid, event))
            body.append('    %s = high;' % was)
            body.append('  }')

        # Modulated setters: the `set` value plus each wire's amount times
        # its signal, applied at control rate.
        for setter in sorted({dp for _, (d, dp), _ in mods_in if d == mid}):
            args = spec['params'][setter]['args']
            base = m.get('set', {}).get(setter, 0.0)
            base = base if isinstance(base, list) else [base]
            if len(base) != len(args):
                fail('%s.%s takes %d arguments: give them all in "set" so '
                     'the unmodulated ones have values' %
                     (mid, setter, len(args)))
            terms = [literal(base[0], 'float')] + [
                '%s * %s' % (literal(a, 'float'), output_expr(s, sp))
                for (s, sp), (d, dp), a in mods_in
                if d == mid and dp == setter]
            values = [' + '.join(terms)] + [
                literal(v, a['type']) for v, a in zip(base[1:], args[1:])]
            body.append('  if (ctl_)')
            body.append('    %s.%s(%s);' % (mid, setter, ', '.join(values)))

        # Only name the result if a wire reads it, to avoid unused-variable
        # warnings.
        used = any(s == mid and '{id}_v_' in raw_expr(mid, sp)
                   for (s, sp), _ in links)
        if spec['returns'] == 'stereo':
            body.append('  float %s_left_ = %s;' % (mid, input_expr(mid, 'left')))
            body.append('  float %s_right_ = %s;' % (
                mid, input_expr(mid, 'right')))
            body.append('  %s.process(%s_left_, %s_right_);' % (mid, mid, mid))
        else:
            call = '%s.%s(%s)' % (mid, spec['call'], ', '.join(
                input_expr(mid, i['name'], i['type'], i['default'])
                for i in spec['inputs']))
            body.append('  auto %s_v_ = %s;' % (mid, call) if used
                        else '  %s;' % call)
    body.append('  left = %s;' % input_expr('out', 'left'))
    body.append('  right = %s;' % input_expr('out', 'right'))

    # 4. Costs. A templated module's RAM is mostly its buffer, so scale
    #    the measured RAM by (size used / size measured).
    def template_size(tmpl):
        """'uint32_t SIZE = 16384' or '8192' -> 16384 or 8192."""
        m = re.search(r'(\d+)\s*$', tmpl or '')
        return int(m.group(1)) if m else None

    def module_ram(m):
        spec = lib[m['type']]
        ram = spec['ram'] or 0
        used = template_size(m.get('template') or spec['template'])
        measured = spec['ram_size'] or template_size(spec['template'])
        return ram * used // measured if used and measured else ram

    def module_cycles(m):
        spec = lib[m['type']]
        sc = spec.get('scaling')
        if not sc:
            return spec['cycles'] or 0
        sets = m.get('set', {})
        per = sc['per'] + (sc['extra_per'] if sets.get(sc['extra_if']) else 0)
        return sc['base'] + per * sets.get(sc['setter'], sc['default'])

    cycles = sum(module_cycles(m) for m in mods.values())
    for k, m in mods.items():
        if lib[m['type']]['cycles'] is None:
            print('note: %s (%s) is unmeasured; counted as 0 cycles until '
                  'profile_all is rerun' % (k, m['type']))
    for d, setter in {(d, dp) for _, (d, dp), _ in mods_in}:
        c = lib[mods[d]['type']]['params'][setter]['cycles']
        if c is None:
            print('note: %s.%s is unmeasured; assuming %d cycles' %
                  (d, setter, UNMEASURED_SETTER))
            c = UNMEASURED_SETTER
        cycles += c / every
    cycles = round(cycles)
    ram = BASE_RAM + sum(module_ram(m) for m in mods.values())
    share = cycles / BUDGET

    src = os.path.relpath(path, ROOT)
    header = '\n'.join([
        '// GENERATED by tools/gen_patch.py from %s.' % os.path.basename(path),
        '// Do not edit: change the patch and rerun',
        '//   python3 tools/gen_patch.py %s' % src,
        '//',
        '// Rough estimate: CPU %d of %d cycles per sample (%d%%), RAM %d'
        % (cycles, BUDGET, share * 100, ram),
        '// of %d bytes. Real CPU can be well above this (patch_demo: 75%%'
        % RAM_SIZE,
        '// estimated, 98% measured); check cpuLoad() on the board.',
        '//',
        '// Include this file from one .cpp only: it defines the module',
        '// objects.',
        '#pragma once',
        '',
        *[('#include <%s>' if h == 'Arduino.h' else '#include "%s"') % h
          for h in includes],
        *([
            '',
            '// The engine lights PC13 when audio runs late. An LED patched',
            '// to PC13 needs that turned off: add -D BB_OVERRUN_LED=0 to',
            '// build_flags.',
            '#include "buddhabox/engine/audio_engine.h"',
            '#if BB_OVERRUN_LED',
            '#warning "LED on PC13 clashes with the overrun LED: add '
            '-D BB_OVERRUN_LED=0"',
            '#endif',
        ] if any(mods[h].get('pin') == 'PC13' for h in hw) else []),
        '',
        '// A namespace keeps module names such as `slope` from clashing',
        '// with framework functions of the same name.',
        'namespace patch {',
        'using namespace bb;',
        '',
        *objects,
        '',
        '/** Apply the starting parameters. Call once, before',
        ' *  AudioEngine::begin(). */',
        'inline void setup() {',
        *setters,
        '}',
        '',
        '/**',
        ' * Read knobs and switches, drive LEDs. Call from loop(), often;',
        ' * never from the audio interrupt (analogRead() is too slow).',
        ' */',
        'inline void control() {',
        *control,
        '}',
        '',
        '/**',
        ' * Run every module for one sample, in signal order.',
        ' * @param left   Left output',
        ' * @param right  Right output',
        ' */',
        'inline void process(float &left, float &right) {',
        *body,
        '}',
        '',
        '} // namespace patch',
        '',
    ])
    open(os.path.join(out_dir, 'patch_graph.h'), 'w').write(header)
    print('wrote %s' % os.path.relpath(os.path.join(out_dir, 'patch_graph.h'),
                                       ROOT))
    main_cpp = os.path.join(out_dir, 'main.cpp')
    if not os.path.exists(main_cpp):
        open(main_cpp, 'w').write(MAIN_CPP)
        print('wrote %s (yours from now on)' % os.path.relpath(main_cpp, ROOT))
    print('estimate: CPU %d of %d cycles (%d%%), RAM %d of %d bytes (%d%%)'
          % (cycles, BUDGET, share * 100, ram, RAM_SIZE,
             100 * ram / RAM_SIZE))
    print('a rough guide: real CPU can run well above it; the starter '
          'main.cpp prints the measured load')
    if share > SAFE:
        print('warning: estimate over %d%% of the CPU; audio may glitch' %
              (SAFE * 100))
    if ram > RAM_SIZE:
        print('warning: over %d bytes of RAM; it will not link' % RAM_SIZE)


if __name__ == '__main__':
    sys.exit(main())
