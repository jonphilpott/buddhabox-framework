#!/usr/bin/env python3
"""
Generate tools/modules.json: every audio module's inputs, outputs,
parameters and measured cost, for the patch generator (gen_patch.py) and
a future patch editor.

WHY: a patch tool must know how to wire each class: what process()
takes, what it returns, which setters exist and what it all costs.
Writing that by hand for ~70 classes would drift from the code, so this
script reads it from the headers (reusing gen_docs.py's parser) and the
costs from tools/costs.csv (the output of `pio run -e profile_all`).

USAGE (from the repository root):
    python3 tools/gen_modules.py

HOW EACH CLASS IS READ:
1. The first public `process(...)` (or `next(...)` if there is none) is
   the per-sample call. Its arguments become inputs.
2. Outputs come from its return value:
   - float, bool or int: one output, `out`
   - a struct such as SvfOut: one output per field (`lp`, `bp`, `hp`)
   - `float &left, float &right`: stereo in place, outputs `left`/`right`
   Every public `float name() const` getter adds one more output
   (ModalResonator's `side`, SVFilter's `lowpass`).
3. Every public `setX(...)` is a parameter; `trigger()` and `gate(bool)`
   are events.
4. Costs: the worst measured row for the class (so ModalResonator gets
   its 24-mode figure), plus each measured setter.

Each output carries a C++ expression with `{id}` standing for the
object's name, so gen_patch.py never needs to know the rules above.
Generated local names end in `_` (`osc_v_`); module ids may not, so the
two can never collide.
"""

import csv
import json
import os
import re
import sys

# Don't leave a __pycache__ folder in tools/ when importing gen_docs.
sys.dont_write_bytecode = True
import gen_docs  # noqa: E402

ROOT = gen_docs.ROOT
OUT = os.path.join(ROOT, 'tools', 'modules.json')
COSTS = os.path.join(ROOT, 'tools', 'costs.csv')

# Folders holding audio modules. core, engine and io are not patchable.
FOLDERS = ['osc', 'noise', 'filter', 'mod', 'dynamics', 'fx', 'physical',
           'drums', 'spectral']

# Building blocks rather than patchable modules.
SKIP = {'FxEngine', 'DelayLine', 'Colour', 'Xmod'}

# Costs that depend on a setter's value. gen_patch.py computes
#   base + (per + extra_per if `extra_if` is set true) * setter value
# ModalResonator measured 704 cycles at 12 modes and 1294 at 24: about
# 115 + 49 per mode. Bowed, it measured 1294 at 12 modes: about 49 more
# per mode. The default is 24 modes.
SCALING = {
    'ModalResonator': {'setter': 'setModes', 'default': 24, 'base': 115,
                       'per': 49, 'extra_if': 'setBowedModes',
                       'extra_per': 49},
}


def parse_args(text):
    """'float in, bool gate = false' -> [{name, type, default}, ...]."""
    args = []
    for part in filter(None, (p.strip() for p in text.split(','))):
        default = None
        if '=' in part:
            part, default = (s.strip() for s in part.split('=', 1))
        # type, optional & or *, name, optional array size: `float r[7]`
        m = re.match(r'(.*?)([&*]?)\s*(\w+)(\[\d*\])?$', part)
        args.append({'name': m.group(3),
                     'type': (m.group(1) + m.group(2)).strip() +
                     (m.group(4) or ''),
                     'default': default})
    return args


def struct_fields(header, name):
    """Field names of `struct name { float a; float b; };` in a header."""
    m = re.search(r'struct\s+%s\s*\{(.*?)\};' % name, open(header).read(),
                  re.S)
    return re.findall(r'float\s+(\w+)\s*;', m.group(1)) if m else []


def load_costs():
    """{class: worst cycles, 'Class.setX': cycles} and
    {class: (bytes, template size profiled or None)}. RAM comes from the
    class's first row: buffer sizes vary between rows, so a maximum would
    mix sizes."""
    cycles, ram = {}, {}
    for line in open(COSTS).read().splitlines()[1:]:
        if not line or line.startswith('#'):
            continue
        # Split from the right: only the module name could hold a comma.
        module, _, c, b, _ = line.rsplit(',', 4)
        key = re.match(r'[\w:]+(\.\w+)?', module).group(0)
        cycles[key] = max(cycles.get(key, 0.0), float(c))
        size = re.search(r'<(\d+)>', module)
        ram.setdefault(key, (int(b), int(size.group(1)) if size else None))
    return cycles, ram


def read_class(cls, header, include, folder, cycles, ram):
    """Build one module entry, or None if the class has no per-sample
    call."""
    members = [m['decl'] for m in cls['members']]

    # 1. The per-sample call: the first process(), else the first next().
    call = None
    for verb in ('process', 'next'):
        for d in members:
            m = re.match(r'(static\s+)?(\w+)\s+%s\((.*)\)' % verb, d)
            if m and not m.group(1):
                call = (verb, m.group(2), parse_args(m.group(3)))
                break
        if call:
            break
    if not call:
        return None
    verb, ret, args = call

    # 2. Inputs and outputs.
    stereo = [a['name'] for a in args] == ['left', 'right'] and \
        all(a['type'] == 'float &' or a['type'] == 'float&' for a in args)
    if stereo:
        inputs = [{'name': 'left', 'type': 'float', 'default': '0.0f'},
                  {'name': 'right', 'type': 'float', 'default': '0.0f'}]
        outputs = [{'name': 'left', 'expr': '{id}_left_'},
                   {'name': 'right', 'expr': '{id}_right_'}]
    else:
        inputs = args
        if ret == 'float':
            outputs = [{'name': 'out', 'expr': '{id}_v_'}]
        elif ret == 'bool':
            outputs = [{'name': 'out', 'expr': '({id}_v_ ? 1.0f : 0.0f)'}]
        elif ret == 'int':
            outputs = [{'name': 'out', 'expr': '(float){id}_v_'}]
        elif ret == 'void':
            outputs = []
        else:  # a struct of floats
            outputs = [{'name': f, 'expr': '{id}_v_.%s' % f}
                       for f in struct_fields(header, ret)]
    for d in members:
        m = re.match(r'float\s+(\w+)\(\)\s*const$', d)
        if m and not m.group(1).startswith('get'):
            outputs.append({'name': m.group(1),
                            'expr': '{id}.%s()' % m.group(1)})

    # 3. Parameters and events.
    params, events = {}, []
    for d in members:
        m = re.match(r'void\s+(set\w+)\((.*)\)$', d)
        if m:
            params[m.group(1)] = {
                'args': parse_args(m.group(2)),
                'cycles': cycles.get('%s.%s' % (cls['name'], m.group(1)))}
        elif re.match(r'void\s+(trigger\(\)|gate\(bool \w+\))$', d):
            events.append(re.match(r'void\s+(\w+)', d).group(1))

    template = None
    if cls['decl'].startswith('template'):
        template = re.match(r'template\s*<(.*)>', cls['decl']).group(1)

    return {
        'header': include,
        'category': folder,
        'brief': gen_docs.brief(cls['doc']),
        'template': template,
        'call': verb,
        'returns': 'stereo' if stereo else ret,
        'inputs': inputs,
        'outputs': outputs,
        'params': params,
        'events': events,
        'cycles': cycles.get(cls['name']),
        'scaling': SCALING.get(cls['name']),
        'ram': ram.get(cls['name'], (None, None))[0],
        # The template size `ram` was measured at (None: the default).
        'ram_size': ram.get(cls['name'], (None, None))[1],
    }


def main():
    cycles, ram = load_costs()
    modules = {}
    for folder in FOLDERS:
        d = os.path.join(gen_docs.SRC, folder)
        for f in sorted(os.listdir(d)):
            if not f.endswith('.h') or f.startswith('lut'):
                continue
            header = os.path.join(d, f)
            _, classes, _ = gen_docs.parse_header(header)
            for cls in classes:
                if cls['kind'] != 'class' or cls['name'] in SKIP:
                    continue
                entry = read_class(cls, header,
                                   'buddhabox/%s/%s' % (folder, f), folder,
                                   cycles, ram)
                if entry:
                    modules[cls['name']] = entry
    unmeasured = sorted(n for n, m in modules.items() if m['cycles'] is None)
    with open(OUT, 'w') as fh:
        json.dump(modules, fh, indent=1)
        fh.write('\n')
    print('wrote %s: %d modules' % (os.path.relpath(OUT, ROOT), len(modules)))
    if unmeasured:
        print('not in costs.csv:', ', '.join(unmeasured))


if __name__ == '__main__':
    sys.exit(main())
