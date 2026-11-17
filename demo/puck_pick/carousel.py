#!/usr/bin/env python3
"""Carousel memory: what sits in each clock slot (carousel.yaml, written by execute.py when a puck is released).

  carousel.py            show the 12 slots
  carousel.py next N     print the next N free slots (load order 12, 11, ... 1), comma separated; exit 1 if fewer
  carousel.py set SLOT CLASS | clear SLOT | reset     fix it by hand (reset: empty carousel, a new run)
"""
import sys
from pathlib import Path

import yaml

FILE = Path(__file__).resolve().parent / 'carousel.yaml'
ORDER = list(range(12, 0, -1))
KO = {'red': '빨강', 'green': '초록', 'cylinder': '원기둥', 'prism': '사각'}


def load():
    return (yaml.safe_load(open(FILE)) if FILE.exists() else None) or {}


def main():
    a, d = sys.argv[1:], load()
    if a[:1] == ['next']:
        free = [s for s in ORDER if s not in d][:int(a[1])]
        print(','.join(map(str, free)))
        return 0 if len(free) == int(a[1]) else 1
    if a[:1] == ['reset']:
        d = {}
    elif a[:1] == ['clear']:
        d.pop(int(a[1]), None)
    elif a[:1] == ['set']:
        d[int(a[1])] = {'class': a[2], 'time': 'by hand', 'grasped': True}
    if a:
        yaml.safe_dump(d, open(FILE, 'w'), sort_keys=True)
    for s in ORDER:
        e = d.get(s)
        if not e:
            print(f'  slot {s:2d}: -'); continue
        c = e['class']
        ko = ' '.join(KO.get(w, w) for w in c.split('_'))
        print(f'  slot {s:2d}: {c:15s} {ko:10s} {e.get("time", "")}' + ('' if e.get('grasped', True) else '  !! no grasp reported'))
    print(f'  {len(d)}/12 loaded')
    return 0


if __name__ == '__main__':
    sys.exit(main())
