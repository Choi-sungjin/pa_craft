#!/usr/bin/python3
"""Camera pitch / roll / height from a view of the floor (depth only).

  /usr/bin/python3 floor_pose.py out/<prefix> [--height 0.915]    # rs_snap capture of (mostly) floor
RANSAC plane (8 mm) over all valid points, SVD refit on the inliers, normal n pointing up (toward the camera).
Optical frame: x right, y down, z forward.
  pitch (optical axis below horizontal) = asin(-n_z)
  roll  (image tilt, + = right side down) = atan2(n_x, -n_y)
  height = distance from the camera to the plane = -n.c
"""
import sys
import numpy as np

P = sys.argv[1]
ref = float(sys.argv[sys.argv.index('--height') + 1]) if '--height' in sys.argv else None
w, h, fx, fy, cx, cy, sc, ser = open(P + '_meta.txt').read().split()
w, h = int(w), int(h); fx, fy, cx, cy = map(float, (fx, fy, cx, cy))
d = np.fromfile(P + '_depth.u16', dtype='<u2').reshape(h, w) * float(sc)
vv, uu = np.mgrid[0:h, 0:w]
X = np.dstack([(uu - cx) * d / fx, (vv - cy) * d / fy, d])[d > 0]
rng = np.random.default_rng(0)
S = X[rng.choice(len(X), min(20000, len(X)), replace=False)]
best = (0, None)
for _ in range(500):
    a, b, c = S[rng.choice(len(S), 3, replace=False)]
    n = np.cross(b - a, c - a); nn = np.linalg.norm(n)
    if nn < 1e-9: continue
    n /= nn; k = (np.abs((S - a) @ n) < 0.008).sum()
    if k > best[0]: best = (k, (a, n))
a, n = best[1]
inl = X[np.abs((X - a) @ n) < 0.008]
c = inl.mean(0); n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
for _ in range(2):
    r = (inl - c) @ n; inl = inl[np.abs(r) < 3 * r.std()]; c = inl.mean(0); n = np.linalg.svd(inl - c, full_matrices=False)[2][2]
if n @ (-c) < 0: n = -n
pitch = np.degrees(np.arcsin(-n[2])); roll = np.degrees(np.arctan2(n[0], -n[1])); height = -(n @ c)
frac = len(inl) / len(X); rms = np.abs((inl - c) @ n).std()
print(f'D435 {ser}: floor plane from {len(inl)} of {len(X)} valid pixels ({100 * frac:.0f}%), plane rms {rms * 1000:.1f} mm, '
      f'depth range {np.percentile(inl[:, 2], 5):.2f}-{np.percentile(inl[:, 2], 95):.2f} m')
print(f'  pitch (optical axis below horizontal) {pitch:.2f} deg')
print(f'  roll  (image tilt, + = right side down) {roll:.2f} deg')
print(f'  height above the floor {height * 1000:.1f} mm' + (f'  (tape {ref * 1000:.0f} mm: {1000 * (height - ref):+.1f} mm)' if ref else ''))
