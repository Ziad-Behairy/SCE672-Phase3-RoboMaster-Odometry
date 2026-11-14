#!/usr/bin/env python3
"""Print where every [EQ ...] formula tag sits in the code (file:line), for theory/FORMULA_MAP.md
and for pointing at the screen during the presentation.   python3 scripts/formula_lines.py"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TAG = re.compile(r'\[EQ ([^\]]+)\]')

for path in sorted((ROOT / 'src').rglob('*.py')):
    if 'test' in path.parts:
        continue
    for n, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        for m in TAG.finditer(line):
            if m.group(1) != '...':
                print(f'{path.relative_to(ROOT).as_posix()}:{n:<4d} [EQ {m.group(1)}]')
