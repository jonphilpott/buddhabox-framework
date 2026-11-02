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
- A wire joins `module.output` to `module.input`. `out.left` and
  `out.right` are the speakers. Several wires into one input are summed.
  An unwired input gets its default (or 0). A float wired into a bool
  input counts as true above 0.5.

HOW IT WORKS:
1. Check every module type, setter and wire against modules.json.
2. Sort the modules so each runs after the modules feeding it (a
   topological sort). A loop of wires has no such order and is refused.
3. Write the header: includes, objects, setup(), process().
4. Add up the measured CPU and RAM costs and print them.
"""

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULES = os.path.join(ROOT, 'tools', 'modules.json')

BUDGET = round(84e6 / 44100)  # CPU cycles per sample: 1905
SAFE = 0.8             # leave 20% for USB, loop() and estimate error

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
  AudioEngine::begin();
}

void loop() {
  // Read pots or buttons here and pass the values to patch::<id>.
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
    wires = [(port(a), port(b)) for a, b in patch.get('wires', [])]

    # 1. Check modules, setters and wires.
    for mid, m in mods.items():
        if not re.match(r'^[A-Za-z_]\w*$', mid) or mid == 'out':
            fail('module id "%s" must be a C++ name and not "out"' % mid)
        if m['type'] not in lib:
            fail('%s: unknown type %s' % (mid, m['type']))
        for setter in m.get('set', {}):
            if setter not in lib[m['type']]['params']:
                fail('%s: %s has no %s()' % (mid, m['type'], setter))
    for (src, sport), (dst, dport) in wires:
        if src not in mods:
            fail('wire from unknown module "%s"' % src)
        if sport not in [o['name'] for o in lib[mods[src]['type']]['outputs']]:
            fail('%s has no output "%s"' % (src, sport))
        if dst == 'out':
            if dport not in ('left', 'right'):
                fail('out has inputs left and right, not "%s"' % dport)
        elif dst not in mods:
            fail('wire to unknown module "%s"' % dst)
        elif dport not in [i['name'] for i in lib[mods[dst]['type']]['inputs']]:
            fail('%s has no input "%s"' % (dst, dport))

    # 2. Topological sort (Kahn's algorithm): repeatedly take a module
    #    whose feeders have all been placed. If none is left to take but
    #    modules remain, the wires form a loop.
    feeders = {mid: {s for (s, _), (d, _) in wires if d == mid}
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
        srcs = [output_expr(s, sp) for (s, sp), (d, dp) in wires
                if d == mid and dp == name]
        if not srcs:
            return default or ('false' if ctype == 'bool' else '0.0f')
        expr = ' + '.join(srcs)
        return '(%s) > 0.5f' % expr if ctype == 'bool' else expr

    # 3. Write the header.
    includes = sorted({lib[m['type']]['header'] for m in mods.values()})
    objects, setters, body = [], [], []
    for mid in order:
        m, spec = mods[mid], lib[mods[mid]['type']]
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

        # Only name the result if a wire reads it, to avoid unused-variable
        # warnings.
        used = any(s == mid and '{id}_v' in raw_expr(mid, sp)
                   for (s, sp), _ in wires)
        if spec['returns'] == 'stereo':
            body.append('  float %s_left = %s, %s_right = %s;' % (
                mid, input_expr(mid, 'left'), mid, input_expr(mid, 'right')))
            body.append('  %s.process(%s_left, %s_right);' % (mid, mid, mid))
        else:
            call = '%s.%s(%s)' % (mid, spec['call'], ', '.join(
                input_expr(mid, i['name'], i['type'], i['default'])
                for i in spec['inputs']))
            body.append('  auto %s_v = %s;' % (mid, call) if used
                        else '  %s;' % call)
    body.append('  left = %s;' % input_expr('out', 'left'))
    body.append('  right = %s;' % input_expr('out', 'right'))

    # 4. Costs. RAM is measured at the profiler's template sizes, so it
    #    is only a guide when a patch changes a template size.
    cycles = round(sum(lib[m['type']]['cycles'] or 0 for m in mods.values()))
    ram = sum(lib[m['type']]['ram'] or 0 for m in mods.values())
    share = cycles / BUDGET

    src = os.path.relpath(path, ROOT)
    header = '\n'.join([
        '// GENERATED by tools/gen_patch.py from %s.' % os.path.basename(path),
        '// Do not edit: change the patch and rerun',
        '//   python3 tools/gen_patch.py %s' % src,
        '//',
        '// Estimated CPU: %d of %d cycles per sample (%d%%). Estimated RAM:'
        % (cycles, BUDGET, share * 100),
        '// %d bytes. Include this file from one .cpp only: it defines the'
        % ram,
        '// module objects.',
        '#pragma once',
        '',
        *['#include "%s"' % h for h in includes],
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
    print('CPU %d of %d cycles (%d%%), RAM %d bytes' % (
        cycles, BUDGET, share * 100, ram))
    if share > SAFE:
        print('warning: over %d%% of the CPU; expect audio glitches' %
              (SAFE * 100))


if __name__ == '__main__':
    sys.exit(main())
