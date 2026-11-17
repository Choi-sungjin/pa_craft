#!/usr/bin/python3
"""Side view and top view of the geometry box_range.py uses, drawn from one capture.

  /usr/bin/python3 geometry_views.py [out/series_010mm]   -> out/geometry_views.png
Side view (ahead x, height z): camera at cam_height, optical axis pitch, vertical FOV edges, floor, box (70 mm),
LiDAR, the robot part that hides the box's near edge and the sight line over it. Top view (orthographic onto the
floor): the camera's floor footprint (HFOV), LiDAR centre, the 205 mm square, centre-to-centre XY distance.
"""
import sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Rectangle, Circle, Polygon
import box_range as br

for f in fm.findSystemFonts():
    if 'NotoSansCJK' in f:
        fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break
P = sys.argv[1] if len(sys.argv) > 1 else 'out/series_010mm'
CAM_H, PITCH, LID_AHEAD = 0.627, 43.6, 0.2525
d, K, ser = br.load(P)
t = np.radians(PITCH); up = np.array([0.0, -np.cos(t), -np.sin(t)])
fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd); right = -np.cross(up, fwd)
top, cen, n, area, rms = br.box_top(d, K, up, CAM_H, np.random.default_rng(0), 0.5, LID_AHEAD + br.MAX_GAP + br.SIDE)
C3, px = br.rectangle(top, K, cen, n)
_, cut, width, depth_seen, g, S = br.geometry(C3, px, K, up, CAM_H)
xy = br.xy_from_lidar(S, up, LID_AHEAD)
h, w = d.shape; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
# seen near boundary and square edges along "ahead"
a_seen = np.sort(C3 @ fwd)[:2].mean(); a_near = np.sort(S @ fwd)[:2].mean(); a_far = np.sort(S @ fwd)[2:].mean()
# occluder: pixels just under the box outline, higher than the box top, nearest to the camera along ahead
vv, uu = np.mgrid[0:h, 0:w]; Pc = np.dstack([(uu - cx) * d / fx, (vv - cy) * d / fy, d])
low = int(np.nonzero(top.any(1))[0].max())
cols = np.nonzero(top.any(0))[0]
band = np.zeros_like(top); band[low - 4:, cols.min() + 10:cols.max() - 10] = True
pts = Pc[band & (d > 0)]; za = CAM_H + pts @ up; aa = pts @ fwd
occ = (za > br.BOX_H + 0.01) & (za < 0.30) & (aa > 0.05)
# the part that makes the seen boundary: its sight line meets the box top where the box stops being seen
hits = aa * (CAM_H - br.BOX_H) / (CAM_H - za)
if occ.sum() > 20:
    k = np.argmin(np.where(occ, np.abs(hits - a_seen), np.inf)); occ_a, occ_z = aa[k], za[k]
else:
    occ_a = occ_z = np.nan
# FOV edges in the vertical plane (rows 0 and h-1 at the centre column)
ang = lambda v: np.degrees(np.arctan((v - cy) / fy))
top_deg, bot_deg = PITCH + ang(0), PITCH + ang(h - 1)                 # below horizontal
hfov = 2 * np.degrees(np.arctan(w / 2 / fx))

fig, ax = plt.subplots(1, 2, figsize=(19, 8.2), dpi=105, gridspec_kw=dict(width_ratios=[1.35, 1]))
# ---- side view
A = ax[0]
A.axhline(0, color='k', lw=1.5); A.fill_between([-0.1, 1.3], -0.03, 0, color='0.85')
A.plot(0, CAM_H, 'ks', ms=10); A.text(0.01, CAM_H + 0.02, f'어퍼캠 (높이 {CAM_H * 1000:.0f} mm)', fontsize=11)
L = 1.3
for deg, lab, st in ((PITCH, f'광축 {PITCH:.1f}°', '-'), (top_deg, f'화면 위 끝 {top_deg:.1f}°', '--'), (bot_deg, f'화면 아래 끝 {bot_deg:.1f}°', '--')):
    x_end = min(L, CAM_H / np.tan(np.radians(deg)))
    A.plot([0, x_end], [CAM_H, CAM_H - x_end * np.tan(np.radians(deg))], st, color='tab:blue', lw=1.4)
    A.text(x_end * 0.55, CAM_H - x_end * 0.55 * np.tan(np.radians(deg)) + 0.012, lab, color='tab:blue', fontsize=10)
A.add_patch(Rectangle((a_near, 0), a_far - a_near, br.BOX_H, fc='#d9a35f', ec='k', label='박스 205 x 70 mm'))
A.add_patch(Rectangle((LID_AHEAD - 0.044, 0.019), 0.044, 0.021, fc='red', ec='k', label='LiDAR'))
A.axvline(LID_AHEAD, color='red', lw=1, ls=':'); A.text(LID_AHEAD - 0.06, -0.025, 'LiDAR 앞면', color='red', fontsize=10)
def near_details(X, labels):
    X.add_patch(Rectangle((a_near, 0), a_far - a_near, br.BOX_H, fc='#d9a35f', ec='k'))
    X.add_patch(Rectangle((LID_AHEAD - 0.044, 0.019), 0.044, 0.021, fc='red', ec='k'))
    X.axvline(LID_AHEAD, color='red', lw=1, ls=':')
    xs = np.array([0, 1.0])
    X.plot(xs, CAM_H - xs * np.tan(np.radians(bot_deg)), '--', color='tab:blue', lw=1.2)
    if not np.isnan(occ_a):
        X.add_patch(Rectangle((occ_a - 0.012, occ_z - 0.008), 0.012, 0.008, fc='0.25', ec='k'))
        hit = occ_a * (CAM_H - br.BOX_H) / (CAM_H - occ_z)
        X.plot([0, hit], [CAM_H, br.BOX_H], 'm-', lw=1.6)
        X.plot([a_near, hit], [br.BOX_H, br.BOX_H], color='m', lw=4, alpha=.6)
    if labels:
        X.annotate(f'LiDAR 앞면 ~ 박스: 간격 {(a_near - LID_AHEAD) * 1000:.0f} mm', (LID_AHEAD + 0.005, 0.03),
                   (0.20, 0.135), fontsize=10, arrowprops=dict(arrowstyle='->', color='k'))
        if not np.isnan(occ_a):
            X.annotate(f'숨는 띠 {(hit - a_near) * 1000:.0f} mm\n(보인 끝 {a_seen * 1000:.0f} mm)', ((a_near + hit) / 2, br.BOX_H),
                       (0.335, 0.03), fontsize=10, color='m', arrowprops=dict(arrowstyle='->', color='m'))
            X.annotate(f'가리는 부품 위끝\n앞 {occ_a * 1000:.0f} mm, 높이 {occ_z * 1000:.0f} mm', (occ_a - 0.006, occ_z),
                       (0.31, 0.125), fontsize=10, arrowprops=dict(arrowstyle='->', color='0.2'))
        X.annotate(f'화면 아래 끝 {bot_deg:.1f}°', (0.29, CAM_H - 0.29 * np.tan(np.radians(bot_deg))), (0.205, 0.075),
                   fontsize=9, color='tab:blue', arrowprops=dict(arrowstyle='->', color='tab:blue'))


if not np.isnan(occ_a):
    hit = occ_a * (CAM_H - br.BOX_H) / (CAM_H - occ_z)
    A.plot([0, hit], [CAM_H, br.BOX_H], 'm-', lw=1.6, label='가리는 부품 위를 스치는 시선')
    A.add_patch(Rectangle((occ_a - 0.012, occ_z - 0.008), 0.012, 0.008, fc='0.25', ec='k', label='가리는 로봇 부품'))
ins = A.inset_axes([0.47, 0.30, 0.52, 0.52])
near_details(ins, True)
ins.set_xlim(0.19, 0.40); ins.set_ylim(-0.005, 0.15); ins.set_aspect('equal'); ins.grid(alpha=.3)
ins.set_title('박스 앞 모서리 확대', fontsize=10); ins.tick_params(labelsize=8)
A.indicate_inset_zoom(ins, edgecolor='0.4')
A.set_xlim(-0.05, 0.9); A.set_ylim(-0.04, 0.70); A.set_aspect('equal'); A.grid(alpha=.3)
A.set_xlabel('카메라 바로 아래에서 앞쪽 (m)'); A.set_ylabel('바닥 위 높이 (m)')
A.set_title(f'측면도 (위에서 본 박스 중심 단면) — {P.split("/")[-1]}', fontsize=13); A.legend(loc='upper right', fontsize=9)
# ---- top view
B = ax[1]
corners = [(0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)]
def floor_hit(u, v, zplane=0.0):
    r = np.array([(u - cx) / fx, (v - cy) / fy, 1.0]); s = (zplane - CAM_H) / (r @ up)
    p = r * s; return p @ fwd, p @ right
for zp, colr, lab in ((0.0, 'tab:blue', f'바닥에 닿는 시야 (HFOV {hfov:.1f}°)'), (br.BOX_H, 'tab:cyan', '박스 윗면 높이(70 mm)의 시야')):
    fp = np.array([floor_hit(u, v, zp) for u, v in corners])
    fp[:, 0] = np.minimum(fp[:, 0], 2.0)
    B.add_patch(Polygon(np.column_stack([fp[:, 1], fp[:, 0]]), fill=False, ec=colr, lw=1.5, ls='--', label=lab))
B.plot(0, 0, 'ks', ms=9); B.text(0.01, -0.03, '어퍼캠 바로 아래', fontsize=10)
lc = (xy and (LID_AHEAD - br.LIDAR_FACE_TO_CENTRE, 0.0))
B.add_patch(Circle((lc[1], lc[0]), 0.022, fc='red', ec='k', label='LiDAR 중심 (앞면 19 mm 뒤)'))
B.plot([-0.15, 0.15], [LID_AHEAD, LID_AHEAD], 'r:', lw=1)
Sa, Sr = S @ fwd, S @ right
B.add_patch(Polygon(np.column_stack([Sr, Sa]), fc='#d9a35f', ec='k', alpha=.8, label='박스 205 mm 윗면 (가려진 앞쪽 복원)'))
bc = (Sr.mean(), Sa.mean())
B.plot([lc[1], bc[0]], [lc[0], bc[1]], 'k-', lw=2)
B.annotate(f'LiDAR 중심 → 박스 중심\nXY {xy["centre"] * 1000:.0f} mm', ((lc[1] + bc[0]) / 2, (lc[0] + bc[1]) / 2),
           (0.20, 0.62), fontsize=12, weight='bold', arrowprops=dict(arrowstyle='->'))
B.plot([lc[1], lc[1]], [lc[0], bc[1]], color='tab:orange', lw=1); B.plot([lc[1], bc[0]], [bc[1], bc[1]], color='tab:orange', lw=1)
B.text(lc[1] - 0.16, (lc[0] + bc[1]) / 2, f'x {xy["x"] * 1000:+.0f} mm', color='tab:orange', fontsize=11, weight='bold')
B.text((lc[1] + bc[0]) / 2 - 0.03, bc[1] + 0.22, f'y {xy["y"] * 1000:+.0f} mm', color='tab:orange', fontsize=11, weight='bold')
B.set_xlim(-0.55, 0.55); B.set_ylim(-0.08, 1.0); B.set_aspect('equal'); B.grid(alpha=.3)
B.set_xlabel('오른쪽 (m)'); B.set_ylabel('앞쪽 (m)')
B.set_title('탑뷰 (바닥 XY 평면 정사영)', fontsize=13); B.legend(loc='upper right', fontsize=9)
fig.tight_layout(); fig.savefig('out/geometry_views.png'); print('out/geometry_views.png')
print(f'FOV below horizontal: top {top_deg:.1f}, bottom {bot_deg:.1f} deg; box near {a_near*1000:.0f} far {a_far*1000:.0f} '
      f'seen near {a_seen*1000:.0f} mm; occluder at {occ_a*1000:.0f} mm, {occ_z*1000:.0f} mm high; XY {xy}')
