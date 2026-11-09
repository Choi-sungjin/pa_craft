import sys, math, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
MOUNT = {'/lidar/front/scan': (0.155, -1, 1, 0.012), '/lidar/rear/scan': (-0.1295, 1, -1, 0.011)}
acc = {k: [] for k in MOUNT}; n = 0
while r.has_next():
    topic, data, t = r.read_next()
    if topic in acc:
        n += 1
        if n % 3: continue
        m = deserialize_message(data, get_message(types[topic]))
        rg = np.array(m.ranges); ang = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
        acc[topic].append(np.c_[rg[ok] * np.cos(ang[ok]), rg[ok] * np.sin(ang[ok])])
fig, axes = plt.subplots(1, 2, figsize=(15, 7.5))
for ax, (topic, ch) in zip(axes, acc.items()):
    x0, sx, sy, h = MOUNT[topic]
    P = np.vstack(ch)
    B = np.c_[x0 + sx * P[:, 0], sy * P[:, 1]]                     # base_link
    body = np.hypot(B[:, 0] - 0.01275, B[:, 1]) < 0.30
    floor = ~body & (P[:, 0] > -0.10) & (np.hypot(P[:, 0], P[:, 1]) < 1.8)
    F = P[floor]
    mu = F.mean(0); _, sv, vt = np.linalg.svd(F - mu, full_matrices=False); nrm = vt[1]; d = nrm @ mu
    if d < 0: nrm, d = -nrm, -d
    res = F @ nrm - d
    alpha = math.degrees(math.asin(min(1, h / d)))
    # straightness per side: separate line fits
    print(f'{topic}: {len(F)} floor points; one line through both sides: d {d:.3f} m, rms {res.std()*100:.1f} cm, max {np.abs(res).max()*100:.1f} cm -> alpha {alpha:.2f} deg')
    for side, sel in (('laser +y', F[:, 1] > 0), ('laser -y', F[:, 1] < 0)):
        G = F[sel]; g = G.mean(0); _, _, v = np.linalg.svd(G - g, full_matrices=False); nn = v[1]; dd = nn @ g
        if dd < 0: nn, dd = -nn, -dd
        rr = G @ nn - dd
        # curvature check: quadratic fit along the line direction
        tang = np.array([-nn[1], nn[0]]); s = G @ tang; q = np.polyfit(s, rr, 2)
        print(f'    {side}: {len(G)} pts, own line d {dd:.3f} m (alpha {math.degrees(math.asin(min(1, h/dd))):.2f} deg), normal ({nn[0]:+.2f},{nn[1]:+.2f}), '
              f'rms {rr.std()*100:.1f} cm, bow (quadratic) {q[0]*np.ptp(s)**2/8*100:+.1f} cm over {np.ptp(s):.2f} m')
    ax.scatter(P[:, 0], P[:, 1], s=1, c='#c3c2b7', alpha=0.3, label='all (laser frame)')
    ax.scatter(P[body][:, 0], P[body][:, 1], s=2, c='#898781', label='robot body (removed)')
    ax.scatter(F[:, 0], F[:, 1], s=2, c='#eb6834', label='floor hits')
    t = np.linspace(-1.5, 1.5, 2); tang = np.array([-nrm[1], nrm[0]])
    ax.plot(nrm[0] * d + t * tang[0], nrm[1] * d + t * tang[1], color='#2a78d6', lw=1.2, label=f'best straight line d={d:.2f} m')
    ax.plot(0, 0, 'k+', ms=12); ax.set_aspect('equal'); ax.set_xlim(-1, 1.5); ax.set_ylim(-1.5, 1.5); ax.grid(alpha=.3)
    ax.set_title(f'{topic}  (+x = toward robot centre)'); ax.legend(fontsize=8, loc='lower left')
fig.tight_layout(); fig.savefig(sys.argv[2], dpi=80)
