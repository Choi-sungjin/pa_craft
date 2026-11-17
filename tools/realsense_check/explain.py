#!/usr/bin/python3
"""Figure that explains the measurement on one capture: depth per pixel -> 3D, pixel-count occlusion, plane + corner rays."""
import sys
import numpy as np
from PIL import Image
from scipy import ndimage
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Polygon
import boxsize

for f in fm.findSystemFonts():
    if 'NotoSansCJK' in f:
        fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break
P = sys.argv[1] if len(sys.argv) > 1 else 'out/series_050mm'
w, h, fx, fy, cx, cy, sc, ser = open(P + '_meta.txt').read().split()
w, h = int(w), int(h); fx, fy, cx, cy = map(float, (fx, fy, cx, cy))
d = np.fromfile(P + '_depth.u16', dtype='<u2').reshape(h, w) * float(sc)
img = Image.open(P + '_color.ppm').convert('RGB'); rgb = np.asarray(img)
hsv = np.asarray(img.convert('HSV')).astype(float); H, S, V = hsv[..., 0] * 360 / 255, hsv[..., 1] / 255, hsv[..., 2] / 255
card = (H > 30) & (H < 62) & (S > 0.2) & (S < 0.65) & (V > 0.3) & (V < 0.8) & ~((d > 0) & (d < 0.55))
card = ndimage.binary_opening(card, iterations=1); lab, n = ndimage.label(card)
box = lab == (np.argmax(ndimage.sum(card, lab, range(1, n + 1))) + 1)
box = ndimage.binary_fill_holes(ndimage.binary_closing(box, iterations=3))
deproj = lambda r, c, z: np.column_stack([(c - cx) * z / fx, (r - cy) * z / fy, z])
inner = ndimage.binary_erosion(box, iterations=12) & (d > 0)
r, c = np.nonzero(inner); X = deproj(r, c, d[inner])
cen = X.mean(0); nrm = np.linalg.svd(X - cen, full_matrices=False)[2][2]
for _ in range(3):  # same outlier trimming as boxsize.py
    keep = np.abs((X - cen) @ nrm) < 3 * np.abs((X - cen) @ nrm).std() + 1e-4
    X = X[keep]; cen = X.mean(0); nrm = np.linalg.svd(X - cen, full_matrices=False)[2][2]
K = boxsize.corners_from_mask(box)
rays = np.column_stack([(K[:, 0] - cx) / fx, (K[:, 1] - cy) / fy, np.ones(4)])
Pk = rays * ((cen @ nrm) / (rays @ nrm))[:, None]
side = [np.linalg.norm(Pk[(i + 1) % 4] - Pk[i]) * 1000 for i in range(4)]

fig = plt.figure(figsize=(20, 11), dpi=100)
# 1. colour + depth, one pixel followed through the formula
ax = fig.add_subplot(2, 2, 1)
u0, v0 = int((K[0, 0] + K[2, 0]) / 2), int((K[0, 1] + K[2, 1]) / 2); z0 = d[v0, u0]
dd = np.where(d > 0, d, np.nan)
ax.imshow(np.hstack([rgb / 255.0, plt.cm.turbo(np.clip(np.nan_to_num(dd, nan=0) / 1.5, 0, 1))[..., :3] * np.where(np.isnan(dd), 1, 1)[..., None]]))
for off in (0, w):
    ax.plot(u0 + off, v0, 'w+', ms=18, mew=3); ax.plot(u0 + off, v0, 'k+', ms=14, mew=1.5)
ax.text(w * 0.02, 22, '컬러', color='w', fontsize=13, weight='bold'); ax.text(w * 1.02, 22, '깊이 (같은 픽셀 = 같은 위치)', color='w', fontsize=13, weight='bold')
x0, y0 = (u0 - cx) * z0 / fx, (v0 - cy) * z0 / fy
ax.set_title(f'① 픽셀마다 거리(깊이)가 있다 → 3D 점으로 바꾼다\n'
             f'픽셀 (u={u0}, v={v0}), 깊이 Z={z0 * 1000:.0f} mm  →  X=(u−cx)·Z/fx={x0 * 1000:.0f} mm,  '
             f'Y=(v−cy)·Z/fy={y0 * 1000:.0f} mm,  Z={z0 * 1000:.0f} mm', fontsize=12)
ax.axis('off')
# 2. occlusion = pixel counting inside the outline
ax = fig.add_subplot(2, 2, 2)
near = (d > 0) & (d < 0.55); grip = near | ((d == 0) & (V < 0.3))
from scipy.spatial import ConvexHull
from matplotlib.path import Path
pts = np.column_stack(np.nonzero(box)[::-1]); poly = pts[ConvexHull(pts).vertices]
yy, xx = np.mgrid[0:h, 0:w]; inside = Path(poly).contains_points(np.column_stack([xx.ravel(), yy.ravel()])).reshape(h, w)
ov = rgb / 255.0; ov = ov.copy()
ov[inside & box] = ov[inside & box] * .35 + np.array([1, .6, 0]) * .65
ov[inside & grip & ~box] = ov[inside & grip & ~box] * .3 + np.array([1, 0, 0]) * .7
ov[grip & ~inside] = ov[grip & ~inside] * .5 + np.array([0, .4, 1]) * .5
ax.imshow(ov); ax.add_patch(Polygon(poly, fill=False, ec='yellow', lw=2))
A = inside.sum(); vis = (inside & box).sum(); occ = (inside & grip & ~box).sum()
ax.set_xlim(K[:, 0].min() - 90, K[:, 0].max() + 90); ax.set_ylim(min(h, K[:, 1].max() + 25), K[:, 1].min() - 40)
ax.set_title('② 가림 비율 = 화면 픽셀 개수 세기\n'
             f'노란 선 안 전체 {A} px 중  주황(박스 색 + 박스 깊이 ±0.1 m) {vis} px = {100 * vis / A:.0f}%,  '
             f'빨강(0.55 m보다 가까움 / 어둡고 깊이 없음) {occ} px = {100 * occ / A:.0f}%', fontsize=12)
ax.axis('off')
# 3. 3D: plane from the inner points, corner rays hit the plane
ax = fig.add_subplot(2, 2, 3, projection='3d')
sel = np.random.default_rng(0).choice(len(X), min(1500, len(X)), replace=False)
ax.scatter(X[sel, 0], X[sel, 2], -X[sel, 1], s=2, c='tab:orange', alpha=.4, label='윗면 안쪽 점 (12 px 깎음) → 평면 맞춤')
ax.scatter([0], [0], [0], s=80, c='k', marker='^', label='카메라')
for i in range(4):
    ax.plot([0, Pk[i, 0]], [0, Pk[i, 2]], [0, -Pk[i, 1]], 'b-', lw=.8, alpha=.6)
quad = np.vstack([Pk, Pk[:1]])
ax.plot(quad[:, 0], quad[:, 2], -quad[:, 1], 'r-', lw=2.5, label='꼭짓점 광선 ∩ 평면 = 3D 꼭짓점')
for i, name in enumerate(['먼 변', '오른 변', '가까운 변', '왼 변']):
    m = (Pk[i] + Pk[(i + 1) % 4]) / 2
    ax.text(m[0], m[2], -m[1] + 0.01, f'{name}\n{side[i]:.0f} mm', fontsize=10, color='darkred')
ax.set_xlabel('X 좌우 (m)'); ax.set_ylabel('Z 앞 (m)'); ax.set_zlabel('−Y 위 (m)'); ax.view_init(elev=25, azim=-60)
ax.set_title('③ 박스 크기 = 3D에서 잰다 (깊이를 쓰는 곳)', fontsize=12); ax.legend(loc='upper left', fontsize=9)
# 4. the four steps in words with the numbers
ax = fig.add_subplot(2, 2, 4); ax.axis('off')
txt = (f'박스 크기 재는 순서 ({P.split("/")[-1]})\n\n'
       f'1. 박스 안쪽 픽셀 {len(X)}개를 깊이로 3D 점으로 바꾼다 (①의 식)\n'
       f'    · 가장자리 12 px는 버린다: 모서리 깊이가 튀고, 옆면이 섞이기 때문\n\n'
       f'2. 그 점들에 평면을 맞춘다 (SVD, 튀는 점 3번 제거) → 평면에서 벗어난 정도 {np.abs((X - cen) @ nrm).std() * 1000:.1f} mm\n\n'
       f'3. 화면에서 박스 모서리 4개를 고른다 (u+v 최소 = 왼위, u−v 최대 = 오른위 …)\n'
       f'    · 모서리 픽셀의 깊이는 쓰지 않는다 (튐)\n'
       f'    · 카메라 → 그 픽셀 방향 광선을 2의 평면과 만나게 해서 3D 꼭짓점을 얻는다\n'
       f'      광선 r = ((u−cx)/fx, (v−cy)/fy, 1),  꼭짓점 = r × (c·n)/(r·n)\n\n'
       f'4. 이웃 꼭짓점 사이 거리 = 변 길이\n'
       f'    먼 {side[0]:.0f} / 오른 {side[1]:.0f} / 가까운 {side[2]:.0f} / 왼 {side[3]:.0f} mm  (실제 205)\n\n'
       f'※ ②의 % 는 화면 픽셀 개수라서, 가까운 쪽이 1 mm²당 픽셀이 더 많다 (먼 쪽보다 ~14%).\n'
       f'   ③의 mm 는 3D 평면에서 잰 실제 길이다.')
ax.text(0.0, 1.0, txt, va='top', fontsize=13, family=plt.rcParams['font.family'])
fig.tight_layout(); fig.savefig(P + '_explain.png'); print(P + '_explain.png')
