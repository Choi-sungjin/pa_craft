#!/usr/bin/python3
"""Box top from depth only (no colour): table plane by RANSAC, then the flat blob ~box_h above it.

  /usr/bin/python3 depthtop.py out/<prefix>      # reads <prefix>_{depth.u16,meta.txt,color.ppm}; writes <prefix>_depthtop.png
1. every valid depth pixel -> 3D point (X=(u-cx)Z/fx, Y=(v-cy)Z/fy, Z)
2. table plane: RANSAC over points nearer than 1.0 m (the floor and the far room are not the table)
3. height of every point above that plane; box top = height 70 mm +-15 mm, the largest connected blob
4. box top plane refit on the blob's inner pixels, corners cast onto it (as boxsize.py) -> side lengths
"""
import sys
import numpy as np
from PIL import Image
from scipy import ndimage
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
import boxsize

BOX_H, BAND = 0.070, 0.015
for f in fm.findSystemFonts():
    if 'NotoSansCJK' in f:
        fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break

P = sys.argv[1]
w, h, fx, fy, cx, cy, sc, ser = open(P + '_meta.txt').read().split()
w, h = int(w), int(h); fx, fy, cx, cy = map(float, (fx, fy, cx, cy))
d = np.fromfile(P + '_depth.u16', dtype='<u2').reshape(h, w) * float(sc)
vv, uu = np.mgrid[0:h, 0:w]
XYZ = np.dstack([(uu - cx) * d / fx, (vv - cy) * d / fy, d])
valid = d > 0
# 2-3. planes one after the other (RANSAC, 6 mm), each split into connected blobs; the box top is the blob whose real
# area is nearest 205 x 205 mm (the white table returns little depth, so it need not be the biggest plane). A plane
# parallel to it and below is the table: its offset is the box height.
near = valid & (d < 1.0)
pix_area = np.where(valid, (d / fx) * (d / fy), 0.0)            # m^2 a pixel covers face-on
rng = np.random.default_rng(0)
left = near.copy(); planes = []
for _ in range(4):
    cand = np.argwhere(left)
    if len(cand) < 500: break
    sub = XYZ[cand[:, 0], cand[:, 1]][rng.choice(len(cand), min(20000, len(cand)), replace=False)]
    best = (0, None)
    for _ in range(300):
        a_, b_, c_ = sub[rng.choice(len(sub), 3, replace=False)]
        nv = np.cross(b_ - a_, c_ - a_); nn = np.linalg.norm(nv)
        if nn < 1e-9: continue
        nv /= nn; cnt = (np.abs((sub - a_) @ nv) < 0.006).sum()
        if cnt > best[0]: best = (cnt, (a_, nv))
    a_, nv = best[1]
    on = left & (np.abs((XYZ - a_) @ nv) < 0.006)
    c0 = XYZ[on].mean(0); nv = np.linalg.svd(XYZ[on] - c0, full_matrices=False)[2][2]
    if nv @ (-c0) < 0: nv = -nv
    on = left & (np.abs((XYZ - c0) @ nv) < 0.008)
    lab, k = ndimage.label(ndimage.binary_opening(on, iterations=2))
    for i in range(1, k + 1):
        blob = lab == i
        cos = abs(nv @ (XYZ[blob].mean(0) / np.linalg.norm(XYZ[blob].mean(0))))
        area = pix_area[blob].sum() / max(cos, 0.2)
        if blob.sum() > 300: planes.append(dict(c=c0, n=nv, blob=blob, area=area, px=int(blob.sum())))
    left &= ~on
for q in planes:
    print(f'  plane blob {q["px"]:6d} px, area {q["area"] * 1e4:6.0f} cm^2, centre {np.linalg.norm(q["blob"].sum() and XYZ[q["blob"]].mean(0)):.3f} m')
box_q = min(planes, key=lambda q: abs(q['area'] - 0.205 * 0.205))
top = ndimage.binary_fill_holes(ndimage.binary_closing(box_q['blob'], iterations=3))
c0, n = box_q['c'], box_q['n']
under = [q for q in planes if q is not box_q and abs(q['n'] @ n) > 0.98 and (q['c'] - c0) @ n < -0.03]
if under:
    tq = max(under, key=lambda q: q['px']); table = tq['blob']; hbox_tab = -((tq['c'] - c0) @ n) * 1000
else:
    table = np.zeros_like(top); hbox_tab = float('nan')
height = np.where(valid, (XYZ - (c0 - n * (hbox_tab / 1000 if under else BOX_H))) @ n, np.nan)
# 4. top plane + corner rays (boxsize method)
inner = ndimage.binary_erosion(top, iterations=8) & valid
X = XYZ[inner]; cen = X.mean(0); nrm = np.linalg.svd(X - cen, full_matrices=False)[2][2]
for _ in range(3):
    keep = np.abs((X - cen) @ nrm) < 3 * np.abs((X - cen) @ nrm).std() + 1e-4
    X = X[keep]; cen = X.mean(0); nrm = np.linalg.svd(X - cen, full_matrices=False)[2][2]
# Outline pixels cast onto the top plane (their depth is not used: edges fly), then the minimum-area rectangle there
# (rotating calipers over the hull): robust to a box turned on the table.
from scipy.spatial import ConvexHull
edge = top & ~ndimage.binary_erosion(top)
er, ec = np.nonzero(edge)
ray = np.column_stack([(ec - cx) / fx, (er - cy) / fy, np.ones(len(er))])
E3 = ray * ((cen @ nrm) / (ray @ nrm))[:, None]
e1 = np.cross(nrm, [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(nrm, e1)
E2 = np.column_stack([(E3 - cen) @ e1, (E3 - cen) @ e2])
hull = E2[ConvexHull(E2).vertices]
best_r = None
for i in range(len(hull)):
    edir = hull[(i + 1) % len(hull)] - hull[i]; edir /= np.linalg.norm(edir); perp = np.array([-edir[1], edir[0]])
    a_, b_ = hull @ edir, hull @ perp
    area = (a_.max() - a_.min()) * (b_.max() - b_.min())
    if best_r is None or area < best_r[0]:
        best_r = (area, edir, perp, a_.min(), a_.max(), b_.min(), b_.max())
_, edir, perp, a0, a1, b0, b1 = best_r
# The hull picks the orientation; the sides come from the 1-99 % spread of all outline points (flying pixels ignored).
pa, pb = E2 @ edir, E2 @ perp
a0, a1, b0, b1 = np.percentile(pa, 1), np.percentile(pa, 99), np.percentile(pb, 1), np.percentile(pb, 99)
rect2 = np.array([edir * a0 + perp * b0, edir * a1 + perp * b0, edir * a1 + perp * b1, edir * a0 + perp * b1])
Pk = cen + rect2[:, :1] * e1 + rect2[:, 1:] * e2
K = np.column_stack([fx * Pk[:, 0] / Pk[:, 2] + cx, fy * Pk[:, 1] / Pk[:, 2] + cy])
side = [np.linalg.norm(Pk[(i + 1) % 4] - Pk[i]) * 1000 for i in range(4)]
turn = np.degrees(np.arctan2(edir[1], edir[0])) % 90
hbox = hbox_tab
print(f'box top: blob {box_q["px"]} px, real area {box_q["area"] * 1e4:.0f} cm^2 (205 x 205 mm = 420); '
      f'table found: {bool(under)}, box top {hbox:.1f} mm above it (spec 70); top plane rms {np.abs((X - cen) @ nrm).std() * 1000:.1f} mm')
print(f'box top from depth only: {top.sum()} px; minimum-area rectangle on the top plane {side[0]:.0f} x {side[1]:.0f} mm '
      f'(spec 205 x 205), rectangle turned {min(turn, 90 - turn):.0f} deg from the plane axes')
rgb = np.asarray(Image.open(P + '_color.ppm').convert('RGB')) / 255.0
fig, ax = plt.subplots(1, 3, figsize=(20, 4.9), dpi=105)
dd = np.where(valid, d, np.nan)
im = ax[0].imshow(dd, cmap='turbo', vmin=0, vmax=1.5); ax[0].set_title('① 깊이 (m)'); plt.colorbar(im, ax=ax[0], fraction=.03)
hm = np.where(valid & (d < 1.0), height * 1000, np.nan)
im = ax[1].imshow(hm, cmap='viridis', vmin=-20, vmax=150); ax[1].set_title('② 책상 기준 높이 (mm): 평면을 차례로 찾고\n넓이가 205x205 mm에 가까운 평면 = 박스 윗면')
plt.colorbar(im, ax=ax[1], fraction=.03)
ov = rgb * 0.45; ov[table] = ov[table] * .5 + np.array([.2, .4, 1]) * .5; ov[top] = np.array([1, .55, 0]) * .8 + rgb[top] * .2
ax[2].imshow(ov); quad = np.vstack([K, K[:1]]); ax[2].plot(quad[:, 0], quad[:, 1], 'r-', lw=2)
ax[2].set_title(f'③ 깊이만으로 뽑은 윗면 (주황), 책상 (파랑)\n평면 위 최소 사각형 {side[0]:.0f} x {side[1]:.0f} mm (실제 205 x 205), '
                f'책상에서 높이 {hbox:.0f} mm (실제 70)')
for A in ax: A.axis('off')
fig.tight_layout(); fig.savefig(P + '_depthtop.png'); print(P + '_depthtop.png')
