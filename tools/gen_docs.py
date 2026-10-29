#!/usr/bin/env python3
"""
Generate docs/reference.html from the docstrings in src/buddhabox.

WHY: the API reference must match the code. Writing it by hand for ~75
classes would drift the first time a parameter changes, so this script
reads the /** ... */ docstrings straight from the headers instead.

USAGE (from the repository root):
    python3 tools/gen_docs.py

HOW IT WORKS:
1. Each header is scanned line by line, counting braces to know when we
   are inside a class body, and tracking public:/private: labels.
2. A docstring followed by `class X` documents the class; one followed by
   a public member documents that member (its signature is everything up
   to the opening brace or semicolon); one at namespace level followed by
   `inline`, `constexpr`, `enum` or `struct` documents a free item.
3. Docstring text becomes HTML: @param/@tparam/@return become labelled
   lines, "Example...:" blocks become code, "- " lines become lists.
"""

import html
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, 'src', 'buddhabox')
OUT = os.path.join(ROOT, 'docs', 'reference.html')

CATEGORIES = [
    ('core', 'Core', 'Shared maths, tables, random numbers and pitch.'),
    ('engine', 'Audio engine', 'I2S + DMA output to the DAC.'),
    ('osc', 'Oscillators', 'Pitched sound sources.'),
    ('noise', 'Noise', 'Random and sparse sources.'),
    ('filter', 'Filters', 'Tone shaping.'),
    ('mod', 'Modulation', 'Envelopes, LFOs, clocks, sequencers.'),
    ('dynamics', 'Dynamics', 'Gates, followers, compressors.'),
    ('fx', 'Effects', 'Delays, reverbs, choruses, distortion.'),
    ('physical', 'Physical models', 'Strings, resonators, tubes.'),
    ('drums', 'Drums', 'Analog and digital drum voices.'),
    ('spectral', 'Spectral', 'FM and cross-modulation.'),
    ('io', 'Inputs and outputs', 'Pots, switches, touch, keypads, LED.'),
]


# ── 1. Parsing ──────────────────────────────────────────────────────────

def code_part(line):
    """The line without // comments or string/char literals."""
    line = re.sub(r'"(\\.|[^"\\])*"', '""', line)
    line = re.sub(r"'(\\.|[^'\\])*'", "''", line)
    return line.split('//')[0]


def clean_doc(lines):
    """Strip the /** */ framing and the leading ' * ' from each line,
    keeping any further indentation (code examples rely on it)."""
    out = []
    for raw in lines:
        s = raw.strip()
        if s.startswith('/**'):
            s = s[3:]
        if s.endswith('*/'):
            s = s[:-2]
        if s.startswith('*'):
            s = s[1:]
        if s.startswith(' '):
            s = s[1:]
        out.append(s.rstrip())
    while out and not out[0].strip():
        out.pop(0)
    while out and not out[-1].strip():
        out.pop()
    return out


def parse_header(path):
    """Return (file_doc, classes, free_items) for one header."""
    lines = open(path).read().split('\n')
    file_doc = None
    classes = []        # dicts: name, decl, doc, members
    free = []           # dicts: decl, doc
    depth = 0
    stack = []          # open classes: [class_dict, body_depth, access]
    pending = None      # docstring waiting for its declaration
    i = 0
    while i < len(lines):
        line = lines[i]
        s = line.strip()
        # Collect a docstring.
        if s.startswith('/**'):
            block = [line]
            while '*/' not in lines[i]:
                i += 1
                block.append(lines[i])
            doc = clean_doc(block)
            if file_doc is None and any('@file' in l for l in doc):
                file_doc = doc
            else:
                pending = doc
            i += 1
            continue
        if not s or s.startswith('//') or s.startswith('#'):
            i += 1
            continue
        # Access labels.
        if stack and re.match(r'^(public|private|protected):', s):
            stack[-1][2] = s.split(':')[0]
            i += 1
            continue
        # Class declarations (possibly after a template line).
        decl = s
        if s.startswith('template') and not re.search(r'\bclass\b|\bstruct\b',
                                                       s.split('>')[-1]):
            decl = s + ' ' + lines[i + 1].strip()
        m = re.match(r'^(template\s*<.*>\s*)?(class|struct)\s+(\w+)'
                     r'(?:\s*:[^{]*)?\s*\{', decl)
        in_public_class = stack and stack[-1][2] == 'public'
        if m and (not stack or in_public_class):
            cls = {'name': m.group(3),
                   'decl': re.sub(r'\s*\{.*$', '', decl).strip(),
                   'doc': pending, 'members': [], 'kind': m.group(2)}
            if not stack:
                classes.append(cls)
            elif pending:
                stack[-1][0]['members'].append(
                    {'decl': cls['decl'], 'doc': pending})
            # Count braces on the declaration lines, then open the class.
            span = 2 if decl != s else 1
            for k in range(span):
                c = code_part(lines[i + k])
                depth += c.count('{') - c.count('}')
            stack.append([cls, depth,
                          'public' if m.group(2) == 'struct' else 'private'])
            pending = None
            i += span
            continue
        # Members and free items: gather the declaration up to { or ;.
        if pending is not None:
            sig_lines = []
            j = i
            while j < len(lines):
                c = code_part(lines[j]).strip()
                sig_lines.append(c)
                if '{' in c or ';' in c:
                    break
                j += 1
            sig = ' '.join(sig_lines)
            sig = re.split(r'\{', sig)[0].strip()
            sig = re.sub(r'\s*;\s*$', '', sig)
            sig = re.sub(r'\s+', ' ', sig)
            sig = re.sub(r'\s*:\s*\w+\(.*$', '', sig)  # constructor inits
            if stack and stack[-1][2] == 'public' and depth == stack[-1][1]:
                stack[-1][0]['members'].append({'decl': sig, 'doc': pending})
            elif not stack:
                free.append({'decl': sig, 'doc': pending})
            pending = None
        # Track braces and close classes.
        c = code_part(line)
        depth += c.count('{') - c.count('}')
        while stack and depth < stack[-1][1]:
            stack.pop()
        i += 1
    return file_doc, classes, free


# ── 2. Docstring text to HTML ───────────────────────────────────────────

def inline(text):
    """Escape, then mark up `code` spans."""
    text = html.escape(text)
    return re.sub(r'`([^`]+)`', r'<code>\1</code>', text)


def doc_to_html(doc, skip_file=True):
    """Convert cleaned docstring lines to HTML (prose, lists, code, tags)."""
    if not doc:
        return ''
    out, para, items, code = [], [], [], []
    params = []
    mode = 'prose'

    def flush_para():
        if para:
            out.append('<p>%s</p>' % inline(' '.join(para)))
            para.clear()

    def flush_items():
        if items:
            out.append('<ul>%s</ul>' %
                       ''.join('<li>%s</li>' % inline(x) for x in items))
            items.clear()

    def flush_code():
        if code:
            while code and not code[-1].strip():
                code.pop()
            # Remove the common indentation.
            ind = min(len(l) - len(l.lstrip()) for l in code if l.strip())
            body = '\n'.join(l[ind:] for l in code)
            out.append('<pre><code>%s</code></pre>' % html.escape(body))
            code.clear()

    for line in doc:
        s = line.strip()
        if skip_file and (s.startswith('@file') or s.startswith('@brief')):
            continue
        tag = re.match(r'^@(param|tparam|return|returns)\s*(.*)$', s)
        if tag:
            flush_para(); flush_items(); flush_code(); mode = 'tag'
            kind, rest = tag.group(1), tag.group(2)
            if kind in ('param', 'tparam'):
                name, _, desc = rest.partition(' ')
                params.append([kind, name, desc.strip()])
            else:
                params.append(['return', '', rest])
            continue
        if re.match(r'^Example.*:$', s):
            flush_para(); flush_items(); flush_code()
            out.append('<p class="example-label">%s</p>' % inline(s))
            mode = 'code'
            continue
        if mode == 'tag':
            if s and params:
                params[-1][2] += ' ' + s
                continue
            if not s:
                continue
        if mode == 'code':
            if not s or line.startswith('  '):
                code.append(line)
                continue
            flush_code(); mode = 'prose'
        if not s:
            flush_para(); flush_items()
            continue
        if re.match(r'^[A-Z][A-Z0-9 ()/,\'-]+:', s):  # "HOW IT WORKS:" etc.
            flush_para(); flush_items()
            head, _, rest = s.partition(':')
            out.append('<h5>%s</h5>' % inline(head.title()))
            if rest.strip():
                para.append(rest.strip())
            continue
        m = re.match(r'^(-|\d+\.)\s+(.*)$', s)
        if m:
            flush_para()
            items.append(m.group(2))
            continue
        if items and line.startswith('  '):
            items[-1] += ' ' + s
            continue
        flush_items()
        para.append(s)
    flush_para(); flush_items(); flush_code()
    for kind, name, desc in params:
        if kind == 'return':
            out.append('<div class="method-returns">Returns: %s</div>' %
                       inline(desc))
        else:
            label = 'Template' if kind == 'tparam' else 'Param'
            out.append('<div class="method-param">%s <code>%s</code> — %s'
                       '</div>' % (label, html.escape(name), inline(desc)))
    return '\n'.join(out)


def brief(doc):
    for l in doc or []:
        if l.startswith('@brief'):
            return l[len('@brief'):].strip()
    return ''


# ── 3. Page assembly ────────────────────────────────────────────────────

def anchor(name):
    return re.sub(r'[^a-z0-9]+', '-', name.lower()).strip('-')


def render_class(cls, file_doc):
    parts = ['<div class="module-card" id="%s">' % anchor(cls['name'])]
    parts.append('<h4>%s</h4>' % html.escape(cls['name']))
    parts.append('<div class="method-sig">%s</div>' %
                 html.escape(cls['decl']))
    doc = cls['doc']
    if not doc or len(doc) < 2:
        doc = (doc or []) + [''] + (file_doc or [])
    parts.append(doc_to_html(doc))
    # Instantiation: constructors (members named like the class).
    ctors = [m for m in cls['members']
             if re.match(r'^(explicit\s+)?%s\s*\(' % cls['name'], m['decl'])]
    if '<' in cls['decl']:
        targs = re.search(r'<(.*)>', cls['decl']).group(1)
        inst = '%s<> name;   // template defaults: %s' % (cls['name'], targs)
    elif ctors and '()' not in ctors[0]['decl']:
        inst = '%s name%s;' % (cls['name'],
                               ctors[0]['decl'][len(cls['name']):])
    else:
        inst = '%s name;' % cls['name']
    parts.append('<p class="example-label">Instantiation</p>'
                 '<pre><code>%s</code></pre>' % html.escape(inst))
    for m in cls['members']:
        parts.append('<div class="method-block"><div class="method-sig">%s'
                     '</div><div class="method-desc">%s</div></div>' %
                     (html.escape(m['decl']), doc_to_html(m['doc'])))
    parts.append('</div>')
    return '\n'.join(parts)


def main():
    nav, body = [], []
    for folder, title, blurb in CATEGORIES:
        d = os.path.join(SRC, folder)
        if not os.path.isdir(d):
            continue
        files = sorted(f for f in os.listdir(d)
                       if f.endswith('.h') and not f.startswith('lut'))
        nav.append('<a href="#%s">%s</a>' % (folder, title))
        body.append('<section id="%s"><h2>%s</h2><p>%s</p>' %
                    (folder, title, blurb))
        for f in files:
            include = 'buddhabox/%s/%s' % (folder, f)
            file_doc, classes, free = parse_header(os.path.join(d, f))
            body.append('<h3 id="%s">%s</h3>' % (anchor(f), html.escape(f)))
            body.append('<p class="include"><code>#include "%s"</code> — %s'
                        '</p>' % (include, inline(brief(file_doc))))
            body.append(doc_to_html(file_doc))
            for item in free:
                body.append('<div class="method-block"><div class="method-sig">'
                            '%s</div><div class="method-desc">%s</div></div>' %
                            (html.escape(item['decl']),
                             doc_to_html(item['doc'])))
            for cls in classes:
                body.append(render_class(cls, file_doc))
        body.append('</section>')
    page = TEMPLATE.replace('{{NAV}}', '\n'.join(nav))
    page = page.replace('{{BODY}}', '\n'.join(body))
    open(OUT, 'w').write(page)
    print('wrote', os.path.relpath(OUT, ROOT))
    update_guide()


def replace_between(text, start, end, content):
    """Replace whatever sits between two marker comments."""
    a = text.index(start) + len(start)
    b = text.index(end, a)
    return text[:a] + '\n' + content + '\n' + text[b:]


def update_guide():
    """Fill the catalogue and the code listings in docs/index.html, so the
    guide always quotes the real, compiled example files."""
    path = os.path.join(ROOT, 'docs', 'index.html')
    page = open(path).read()
    # 1. Catalogue: one line per header, grouped by category.
    cat = []
    for folder, title, blurb in CATEGORIES:
        d = os.path.join(SRC, folder)
        if not os.path.isdir(d):
            continue
        cat.append('<h3>%s</h3><div class="catalogue">' % title)
        for f in sorted(os.listdir(d)):
            if not f.endswith('.h') or f.startswith('lut'):
                continue
            file_doc, classes, _ = parse_header(os.path.join(d, f))
            names = ', '.join('<a href="reference.html#%s">%s</a>' %
                              (anchor(c['name']), html.escape(c['name']))
                              for c in classes) or (
                '<a href="reference.html#%s">%s</a>' % (anchor(f), f))
            cat.append('<div>%s — %s</div>' % (names, inline(brief(file_doc))))
        cat.append('</div>')
    page = replace_between(page, '<!-- CATALOGUE -->', '<!-- /CATALOGUE -->',
                           '\n'.join(cat))
    # 2. Code listings: <!-- INCLUDE path --> ... <!-- /INCLUDE -->.
    pos = 0
    while True:
        m = re.compile(r'<!-- INCLUDE (\S+) -->').search(page, pos)
        if not m:
            break
        code = open(os.path.join(ROOT, m.group(1))).read().rstrip()
        listing = '<pre><code>%s</code></pre>' % html.escape(code)
        end = page.index('<!-- /INCLUDE -->', m.end())
        page = page[:m.end()] + '\n' + listing + '\n' + page[end:]
        pos = m.end() + len(listing)
    open(path, 'w').write(page)
    print('updated docs/index.html')


TEMPLATE = '''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BuddhaBox API Reference</title>
<link rel="stylesheet" href="style.css">
</head>
<body>
<header>
  <h1>Buddha<span>Box</span> API Reference</h1>
  <p>Every module, generated from the source docstrings.
     <a href="index.html">Back to the guide</a>.</p>
</header>
<nav>
{{NAV}}
</nav>
<div class="container">
<div class="tip"><strong>Generated file.</strong> Edit the docstrings in
<code>src/buddhabox/</code> and run <code>python3 tools/gen_docs.py</code>
instead of editing this page.</div>
{{BODY}}
</div>
<footer>BuddhaBox framework. Modules derived from Mutable Instruments
firmware are Copyright Emilie Gillet, MIT licence.</footer>
</body>
</html>
'''

if __name__ == '__main__':
    sys.exit(main())
