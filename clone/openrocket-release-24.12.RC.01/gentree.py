#!/usr/bin/env python3
"""
Generate a Markdown package/class tree for packages under info.openrocket.core.

Usage:
  python3 generate_openrocket_tree.py /path/to/openrocket/core/src/main/java > tree.md
"""
import os
import re
import sys
from collections import defaultdict

if len(sys.argv) < 2:
    print("Usage: python3 generate_openrocket_tree.py /path/to/core/src/main/java")
    sys.exit(1)

src_root = sys.argv[1]
target_prefix = "info.openrocket.core"

java_files = []
for root, dirs, files in os.walk(src_root):
    for f in files:
        if f.endswith(".java"):
            path = os.path.join(root, f)
            # only include files under the target package path
            rel = os.path.relpath(path, src_root).replace(os.sep, ".")
            if rel.startswith(target_prefix):
                java_files.append(path)

pkg_map = defaultdict(list)

decl_re = re.compile(
    r'(?P<preamble>[\w\s]*)\b(?P<kind>class|interface|enum)\s+'
    r'(?P<name>[A-Za-z0-9_]+)'
    r'(?:\s+extends\s+(?P<extends>[^\\{implements]+?))?'
    r'(?:\s+implements\s+(?P<implements>[^\\{]+?))?'
    , re.M
)

pkg_re = re.compile(r'^\s*package\s+([\w\.]+)\s*;', re.M)

for f in java_files:
    try:
        text = open(f, encoding='utf-8').read()
    except Exception:
        continue
    pkg_match = pkg_re.search(text)
    if not pkg_match:
        continue
    pkg = pkg_match.group(1)
    for m in decl_re.finditer(text):
        kind = m.group('kind')
        name = m.group('name')
        extends = m.group('extends') or ''
        implements = m.group('implements') or ''
        # clean up extends/implements lists
        extends = ",".join(p.strip() for p in re.split(r'[,<>\s]+', extends) if p.strip())
        implements = ",".join(p.strip() for p in re.split(r'[,<>\s]+', implements) if p.strip())
        pkg_map[pkg].append({
            'name': name,
            'kind': kind,
            'extends': extends,
            'implements': implements,
            'file': os.path.relpath(f, src_root)
        })

# Print Markdown tree
print(f"- {target_prefix}")
for pkg in sorted(pkg_map.keys()):
    if not pkg.startswith(target_prefix):
        continue
    rel_pkg = pkg[len(target_prefix):].lstrip('.')
    indent = "  "
    if rel_pkg:
        # show subpackages as nested bullet
        parts = rel_pkg.split('.')
        # print subpackage path
        subs = []
        for i in range(len(parts)):
            subs.append(parts[i])
        print(f"{indent}- {'.'.join([target_prefix]+parts)}")
        pkg_indent = indent + "  "
    else:
        pkg_indent = indent + "  "
        print(f"{pkg_indent}- {target_prefix} (root package)")
    entries = pkg_map[pkg]
    for e in sorted(entries, key=lambda x: x['name']):
        line = f"{pkg_indent}- {e['name']} ({e['kind']})"
        extras = []
        if e['extends']:
            extras.append(f"extends {e['extends']}")
        if e['implements']:
            extras.append(f"implements {e['implements']}")
        if extras:
            line += " — " + ", ".join(extras)
        line += f"  <!-- {e['file']} -->"
        print(line)