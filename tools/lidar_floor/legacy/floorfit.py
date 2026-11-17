import sys, math, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, out = sys.argv[1], sys.argv[2]
H = {'/lidar/front/scan': 0.012, '/lidar/rear/scan': 0.011}
# laser frame -> base_link rotation (static TF): front = 180 deg about y (x->-x, y->y), rear = 180 deg about x (x->x, y->-y)
TO_BASE = {'/lidar/front/scan': np.array([[-1, 0], [0, 1]]), '/lidar/rear/scan': np.array([[1, 0], [0, -1]])}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
acc = {k: [] for k in H}
while r.has_next():
    topic, data, t = r.read_next()
    if topic in acc and len(acc[topic]) < 400 and (sum(map(len, acc.values())) % 3 == 0 or True):
        m = deserialize_message(data, get_message(types[topic]))
        rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
        acc[topic].append(np.c_[rg[ok] * np.cos(a[ok]), rg[ok] * np.sin(a[ok])])
rng = np.random.default_rng(0)
fig, axes = plt.subplots(1, 2, figsize=(15, 7.5))
for ax, (topic, ch) in zip(axes, acc.items()):
    P = np.vstack(ch[::3])
    # Candidates: not the outward environment (x > -0.25), beyond the body ring (r > 0.33 or x > 0.2), within 1.6 m.
    rr = np.hypot(P[:, 0], P[:, 1])
    cand = P[(P[:, 0] > -0.25) & (rr < 1.6) & (rr > 0.18)]
    best = None
    if len(cand) > 8000: cand = cand[rng.choice(len(cand), 8000, replace=False)]
    for _ in range(3000):
        i, j = rng.integers(len(cand), size=2); a, b = cand[i], cand[j]
        d = b - a; L = np.hypot(*d)
        if L < 0.3: continue
        n = np.array([d[1], -d[0]]) / L; c = n @ a
        if c < 0: n, c = -n, -c                      # normal points from the LiDAR to the line
        inl = np.abs(cand @ n - c) < 0.01
        span = np.ptp(cand[inl] @ np.array([-n[1], n[0]])) if inl.sum() > 1 else 0
        score = inl.sum() * min(span, 1.5)
        if best is None or score > best[0]: best = (score, n, c, inl)
    _, n, c, inl = best
    # refine by least squares on inliers (total least squares)
    Q = cand[inl]; mu = Q.mean(0); _, _, vt = np.linalg.svd(Q - mu); n = vt[1]; c = n @ mu
    if c < 0: n, c = -n, -c
    resid = Q @ n - c
    h = H[topic]; alpha = math.asin(h / c)
    nb = TO_BASE[topic] @ n                           # descent direction in base_link
    # base_link slope: z drops by sin(alpha) per metre along nb -> pitch (about y, + = nose down) / roll (about x, + = left up)
    slope_x, slope_y = -math.sin(alpha) * nb[0], -math.sin(alpha) * nb[1]   # dz/dx, dz/dy of the scan plane
    pitch = -math.degrees(math.atan(slope_x)); roll = math.degrees(math.atan(slope_y))
    print(f'{topic}: {inl.sum()} inliers, line distance d = {c:.3f} m (rms {resid.std()*1000:.1f} mm), '
          f'normal (laser) ({n[0]:+.2f},{n[1]:+.2f}) -> descent toward base ({nb[0]:+.2f},{nb[1]:+.2f})')
    print(f'    tilt alpha = asin(h/d) = asin({h}/{c:.3f}) = {math.degrees(alpha):.2f} deg  ->  plane slope dz/dx {slope_x*100:+.2f} %, dz/dy {slope_y*100:+.2f} %'
          f'  (pitch {pitch:+.2f} deg, roll {roll:+.2f} deg in base_link)')
    for h_box in (0.07,):
        rise = math.sin(alpha) * max(1e-9, -nb[0] if topic.endswith('front/scan') else nb[0])  # rise per metre straight outward
        out_dir = np.array([1, 0]) if topic.endswith('front/scan') else np.array([-1, 0])
        rise = -math.sin(alpha) * (nb @ out_dir)
        print(f'    straight outward the beam rises {rise*100:.1f} cm per metre: a {h_box*100:.0f} cm box top is under the beam beyond {(h_box - h) / rise:.2f} m')
    ax.scatter(P[:, 0], P[:, 1], s=1, c='#898781', alpha=0.15, label='all points')
    ax.scatter(Q[:, 0], Q[:, 1], s=3, c='#eb6834', label='floor line inliers')
    t = np.linspace(-1.6, 1.6, 2); base = n * c; tang = np.array([-n[1], n[0]])
    ax.plot(base[0] + t * tang[0], base[1] + t * tang[1], color='#2a78d6', lw=1.5, label=f'fit: d={c:.3f} m, alpha={math.degrees(alpha):.2f} deg')
    ax.plot(0, 0, 'k+', ms=12); ax.set_aspect('equal'); ax.set_xlim(-2, 2); ax.set_ylim(-2, 2); ax.grid(alpha=.3)
    ax.set_title(f'{topic} (laser frame: +x = toward the robot centre)'); ax.legend(loc='lower left', fontsize=9)
fig.tight_layout(); fig.savefig(out, dpi=80)
