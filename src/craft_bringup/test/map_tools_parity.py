#!/usr/bin/env python3
"""map_tools.py vs the C++ port (src/map_tools.hpp) on a real map and a synthetic scan: same pose, same scores.
  python3 test/map_tools_parity.py <build>/map_tools_cli <map.yaml>
"""
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'demo' / 'corner'))
from map_tools import MapGrid, ScanMatcher  # noqa: E402

cli, map_yaml = sys.argv[1], sys.argv[2]
grid = MapGrid(map_yaml)
true = (0.648, 1.0, math.pi)
# synthetic scan: ray-cast the map from the true pose (every 1.5 deg, 7.5 m), as base_link points
pts = []
for a in np.radians(np.arange(0, 360, 1.5)):
    for r in np.arange(0.06, 7.5, grid.res / 2):
        x, y = true[0] + r * math.cos(true[2] + a), true[1] + r * math.sin(true[2] + a)
        c, w = grid.to_cell(x, y)
        c, w = int(math.floor(c)), int(math.floor(w))
        if not (0 <= c < grid.w and 0 <= w < grid.h):
            break
        if grid.occupied[w, c]:
            pts.append((r * math.cos(a), r * math.sin(a)))
            break
pts = np.array(pts)
for near in [None, (0.648, 1.0, 180.0)]:
    py = ScanMatcher(grid).localize(pts, near=near[:2] if near else None, yaw=math.radians(near[2]) if near else None)
    with tempfile.NamedTemporaryFile('w', suffix='.txt') as f:
        np.savetxt(f, pts)
        f.flush()
        out = subprocess.run([cli, map_yaml, f.name] + ([str(v) for v in near] if near else []),
                             capture_output=True, text=True, check=True).stdout.split()
    cx, cy, cyaw, cs, c2 = map(float, out[:5])
    px, py_, pyaw = py['pose']
    print(f'near={near}: python ({px:.3f}, {py_:.3f}, {math.degrees(pyaw):.1f}) {py["score"]:.4f} / {py["second"]:.4f}'
          f'   c++ ({cx:.3f}, {cy:.3f}, {math.degrees(cyaw):.1f}) {cs:.4f} / {c2:.4f}')
    assert math.hypot(cx - px, cy - py_) < 1e-6 and abs(math.remainder(cyaw - pyaw, 2 * math.pi)) < 1e-6, 'pose differs'
    assert abs(cs - py['score']) < 1e-4 and abs(c2 - py['second']) < 1e-4, 'score differs'
    assert bool(int(out[5])) == py['ambiguous']
print('map_tools parity OK')
