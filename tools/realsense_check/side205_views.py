#!/usr/bin/python3
"""How the 205 mm square is used (box_range.py), drawn from one capture with its real numbers.

  /usr/bin/python3 side205_views.py [out/series_010mm]   -> out/side205_views.png
A) similar triangles on the far edge: the two far-corner rays; the plane distance at which the chord between them
   is 205 mm (D_205 = D_depth x 205 / chord_depth) - the pinhole relation f / w_px = D / 205 mm.
B) top view: the seen (cut) rectangle, its complete far edge, the square laid 205 mm from the far edge toward the
   robot, seen centre vs completed centre, LiDAR centre and the gap.
C) side view of B: the hidden strip and the completed depth.
"""
import sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
from matplotlib.patches import Polygon, Rectangle, Circle
import box_range as br

for f in fm.findSystemFonts():
    if 'NotoSansCJK' in f:
        fm.fontManager.addfont(f); plt.rcParams['font.family'] = fm.FontProperties(fname=f).get_name(); break
P = sys.argv[1] if len(sys.argv) > 1 else 'out/series_010mm'
CAM_H, PITCH, LID = 0.627, 43.6, 0.2525
d, K, ser = br.load(P); fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
t = np.radians(PITCH); up = np.array([0.0, -np.cos(t), -np.sin(t)])
fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd); right = -np.cross(up, fwd)
top, cen, n, area, rms = br.box_top(d, K, up, CAM_H, np.random.default_rng(0), 0.5, LID + br.MAX_GAP + br.SIDE)
C3, px = br.rectangle(top, K, cen, n)
S, cut, chord, depth_seen = br.complete_square(C3, up)
far = np.argsort(C3 @ fwd)[-2:]; A3, B3 = C3[far[0]], C3[far[1]]
pa, pb = px[far[0]], px[far[1]]; w_px = np.linalg.norm(pa - pb)
D_depth = np.linalg.norm((A3 + B3) / 2); scale = br.SIDE / chord; D_205 = D_depth * scale
TA = lambda Q: np.column_stack([Q @ right, Q @ fwd])                 # top view (right, ahead)

fig, ax = plt.subplots(1, 3, figsize=(22, 7.6), dpi=100, gridspec_kw=dict(width_ratios=[0.9, 1, 1.1]))
# ---- A: similar triangles on the far edge (drawn in the plane of the two rays)
X = ax[0]
ua, ub = A3 / np.linalg.norm(A3), B3 / np.linalg.norm(B3)
e1 = (ua + ub) / np.linalg.norm(ua + ub); e2 = ub - (ub @ e1) * e1; e2 /= np.linalg.norm(e2)
to2 = lambda v: np.array([v @ e2, v @ e1])
a2, b2 = to2(A3), to2(B3)
X.plot(0, 0, 'ks', ms=10); X.text(0.02, -0.02, '카메라 중심', fontsize=10)
for p2 in (a2 * 1.25, b2 * 1.25):
    X.plot([0, p2[0]], [0, p2[1]], color='tab:blue', lw=1.2)
X.plot(*zip(a2, b2), 'o-', color='tab:orange', lw=3, label=f'깊이로 잰 먼 변: {chord * 1000:.0f} mm (거리 {D_depth * 1000:.0f} mm)')
a5, b5 = a2 * scale, b2 * scale
X.plot(*zip(a5, b5), 's-', color='tab:green', lw=3, label=f'205 mm가 되는 거리: {D_205 * 1000:.0f} mm  (x {scale:.3f})')
fpl = 0.12                                                            # image plane drawn at a symbolic depth
ia, ib = ua / (ua @ e1) * fpl, ub / (ub @ e1) * fpl
X.plot(*zip(to2(ia), to2(ib)), color='k', lw=2); X.text(to2(ib)[0] + 0.005, fpl - 0.005, f'영상면: 먼 변 {w_px:.0f} px', fontsize=10)
X.set_xlim(-0.35, 0.35); X.set_ylim(-0.05, 0.95); X.grid(alpha=.3); X.legend(loc='upper left', fontsize=10)
X.annotate('', (a5[0], a5[1] + 0.03), (b5[0], b5[1] + 0.03), arrowprops=dict(arrowstyle='<->', color='tab:green'))
X.text((a5[0] + b5[0]) / 2 - 0.04, (a5[1] + b5[1]) / 2 + 0.05, '205 mm', color='tab:green', fontsize=11, weight='bold')
X.set_title('A) 닮은 삼각형: 같은 두 광선 사이 폭 ∝ 거리\n'
            f'f / w_px = D / 205 mm  →  D = {D_depth * 1000:.0f} x 205 / {chord * 1000:.0f} = {D_205 * 1000:.0f} mm', fontsize=12)
X.set_xlabel('광선 평면 가로 (m)'); X.set_ylabel('광선 평면 깊이 (m)')
# ---- B: top view, square completion
Y = ax[1]
Y.add_patch(Polygon(TA(C3), fc='none', ec='tab:orange', lw=2.5, label=f'보인 사각형 {chord * 1000:.0f} x {depth_seen * 1000:.0f} mm (앞쪽 잘림)'))
Y.add_patch(Polygon(TA(S), fc='#d9a35f', ec='tab:green', lw=2, ls='--', alpha=.45, label='먼 변 기준 205 x 205 mm 복원'))
fa, fb = TA(S[:2]); Y.plot(*zip(fa, fb), color='tab:green', lw=4)
Y.text(*(fa + fb) / 2 + [0, 0.012], '먼 변 (항상 보임)', ha='center', fontsize=10, color='tab:green')
mid = (fa + fb) / 2; nearmid = TA(S[2:]).mean(0)
Y.annotate('', nearmid, mid, arrowprops=dict(arrowstyle='<->', color='k'))
Y.text(mid[0] + 0.01, (mid[1] + nearmid[1]) / 2, '205 mm\n로봇 쪽으로', fontsize=10)
cs, cq = TA(C3).mean(0), TA(S).mean(0)
Y.plot(*cs, 'o', color='tab:orange', ms=8, label=f'보인 사각형 중심 (앞 {cs[1] * 1000:.0f} mm)')
Y.plot(*cq, 'P', color='tab:green', ms=11, label=f'복원 정사각형 중심 (앞 {cq[1] * 1000:.0f} mm)')
lc = np.array([0.0, LID - br.LIDAR_FACE_TO_CENTRE])
Y.add_patch(Circle(lc, 0.022, fc='red', ec='k')); Y.plot([-0.15, 0.15], [LID, LID], 'r:', lw=1)
Y.text(0.03, LID - 0.03, 'LiDAR 중심 / 앞면', color='red', fontsize=10)
gap = TA(S[2:])[:, 1].mean() - LID
Y.annotate(f'간격 {gap * 1000:.0f} mm', (cq[0], LID), (cq[0] + 0.12, LID - 0.06), fontsize=10, arrowprops=dict(arrowstyle='->'))
Y.set_xlim(-0.25, 0.25); Y.set_ylim(0.18, 0.52); Y.set_aspect('equal'); Y.grid(alpha=.3); Y.legend(loc='upper left', fontsize=9)
Y.set_xlabel('오른쪽 (m)'); Y.set_ylabel('카메라 바로 아래에서 앞쪽 (m)')
Y.set_title(f'B) 탑뷰: 잘린 사각형 → 205 mm 정사각형 복원\n중심이 {(cs[1] - cq[1]) * 1000:.0f} mm 로봇 쪽으로 이동', fontsize=12)
# ---- C: side view of the same
Z = ax[2]
a_far, a_seen, a_near = TA(S[:2])[:, 1].mean(), np.sort(C3 @ fwd)[:2].mean(), TA(S[2:])[:, 1].mean()
Z.axhline(0, color='k'); Z.add_patch(Rectangle((a_near, 0), a_far - a_near, br.BOX_H, fc='#d9a35f', ec='k', alpha=.6))
Z.plot([a_seen, a_far], [br.BOX_H] * 2, color='tab:orange', lw=5, label=f'깊이로 보인 윗면 {(a_far - a_seen) * 1000:.0f} mm')
Z.plot([a_near, a_seen], [br.BOX_H] * 2, color='m', lw=5, label=f'숨은 띠 {(a_seen - a_near) * 1000:.0f} mm (205로 채움)')
Z.add_patch(Rectangle((LID - 0.044, 0.019), 0.044, 0.021, fc='red', ec='k')); Z.axvline(LID, color='red', ls=':', lw=1)
Z.annotate('', (a_near, 0.10), (a_far, 0.10), arrowprops=dict(arrowstyle='<->')); Z.text((a_near + a_far) / 2 - 0.02, 0.105, '205 mm', fontsize=11)
Z.annotate('', (LID, 0.035), (a_near, 0.035), arrowprops=dict(arrowstyle='<->', color='k'))
Z.text(LID - 0.005, 0.045, f'{gap * 1000:.0f} mm', fontsize=10)
Z.set_xlim(0.18, 0.52); Z.set_ylim(-0.01, 0.15); Z.set_aspect('equal'); Z.grid(alpha=.3); Z.legend(loc='upper right', fontsize=9)
Z.set_xlabel('카메라 바로 아래에서 앞쪽 (m)'); Z.set_ylabel('높이 (m)')
Z.set_title('C) 측면도: 먼 변은 보이고, 앞 띠는 205 mm로 채운다', fontsize=12)
fig.tight_layout(); fig.savefig('out/side205_views.png'); print('out/side205_views.png')
print(f'far edge {w_px:.0f} px, chord {chord*1000:.1f} mm at {D_depth*1000:.0f} mm -> 205 at {D_205*1000:.0f} mm; seen depth {depth_seen*1000:.0f}; '
      f'centre seen {cs[1]*1000:.0f} -> square {cq[1]*1000:.0f}; gap {gap*1000:.0f}')
