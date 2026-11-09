import sys, math, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, out = sys.argv[1], sys.argv[2]
edges = [0, 15, 30, 1e9]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
pts = {(tp, k): [] for tp in ('/lidar/front/scan', '/lidar/rear/scan') for k in range(3)}; t0 = None; yaw = {k: [] for k in range(3)}
while r.has_next():
    topic, data, t = r.read_next(); t /= 1e9
    if t0 is None: t0 = t
    k = next(i for i in range(3) if edges[i] <= t - t0 < edges[i + 1])
    if topic == '/perception/imu/data':
        q = deserialize_message(data, get_message(types[topic])).orientation
        yaw[k].append(math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
    elif topic in ('/lidar/front/scan', '/lidar/rear/scan'):
        m = deserialize_message(data, get_message(types[topic]))
        rg = np.array(m.ranges); a = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
        pts[(topic, k)].append(np.c_[rg[ok] * np.cos(a[ok]), rg[ok] * np.sin(a[ok])])
fig, axes = plt.subplots(2, 3, figsize=(18, 12))
for j, tp in enumerate(('/lidar/front/scan', '/lidar/rear/scan')):
    for k in range(3):
        ax = axes[j, k]; ch = pts[(tp, k)]
        if not ch: ax.set_title(f'{tp} part {k}: no data'); continue
        P = np.vstack(ch)
        ax.scatter(P[:, 0], P[:, 1], s=1, c='#2a78d6', alpha=0.25); ax.plot(0, 0, 'k+', ms=12)
        ax.set_aspect('equal'); ax.set_xlim(-4, 4); ax.set_ylim(-4, 4); ax.grid(alpha=.3)
        y = yaw[k]; ys = f'IMU yaw {np.median(y):+.0f} deg' if y else ''
        ax.set_title(f'{tp} {edges[k]:.0f}-{min(edges[k+1], 43):.0f} s, {len(ch)} scans  {ys}', fontsize=10)
        print(tp, k, len(ch), 'scans', ys)
fig.tight_layout(); fig.savefig(out, dpi=75)
