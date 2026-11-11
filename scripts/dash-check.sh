#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
export LC_ALL=C
paths=(
  README.md NOTICE PLAN.md TASKS.md REFERENCES.md
  src tests scripts .github web
  pyproject.toml requirements.in
)
python3 -c "
import pathlib
import sys

chars = ('\u2014', '\u2013', '\u2212')
paths = sys.argv[1:]
bad = []
for p in paths:
    root = pathlib.Path(p)
    if not root.exists():
        continue
    files = [root] if root.is_file() else [f for f in root.rglob('*') if f.is_file()]
    for f in files:
        if 'node_modules' in f.parts or f.name.endswith('.lock'):
            continue
        if f.suffix in ('.png', '.json') and 'package' in f.name:
            continue
        try:
            text = f.read_text(encoding='utf-8', errors='replace')
        except OSError:
            continue
        for ch in chars:
            if ch in text:
                bad.append((str(f), ch))
if bad:
    for b in bad[:20]:
        print('typo dash', b)
    sys.exit(1)
" "${paths[@]}"
