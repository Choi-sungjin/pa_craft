import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
# laser -> base_link: front at x 0.155 (x->-x, y->y), rear at x -0.1295 (x->x, y->-y); body centre x 0.01275, radius 0.25
MOUNT = {'/lidar/front/scan': (0.155, -1, 1), '/lidar/rear/scan': (-0.1295, 1, -1)}
acc = {k: [] for k in MOUNT}; rp = []; n = 0
while r.has_next():
    topic, data, t = r.read_next()
    if topic == '/perception/imu/data':
        m = deserialize_message(data, get_message(types[topic])); q = m.orientation; a = m.linear_acceleration
        rp.append((math.degrees(math.atan2(a.y, a.z)), math.degrees(math.atan2(-a.x, math.hypot(a.y, a.z)))))
    elif topic in acc:
        n += 1
        if n % 3: continue
        m = deserialize_message(data, get_message(types[topic]))
        rg = np.array(m.ranges); ang = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
        x0, sx, sy = MOUNT[topic]
        bx, by = x0 + sx * rg[ok] * np.cos(ang[ok]), sy * rg[ok] * np.sin(ang[ok])
        acc[topic].append(np.c_[bx, by, rg[ok], np.degrees(ang[ok])])
rp = np.array(rp); print(f'IMU (gravity) roll {np.median(rp[:,0]):+.2f} deg, pitch {np.median(rp[:,1]):+.2f} deg  (board frame, sign as mounted)')
for topic, ch in acc.items():
    P = np.vstack(ch); x0 = MOUNT[topic][0]
    body = np.hypot(P[:, 0] - 0.01275, P[:, 1]) < 0.30
    inward = (P[:, 0] - x0) * np.sign(-x0) > -0.10   # roughly the inward / side half of this LiDAR
    near = np.hypot(P[:, 0] - x0, P[:, 1]) < 1.8
    F = P[~body & inward & near]
    print(f'{topic}: points outside the body, inward/side half, < 1.8 m: {len(F)}')
    if len(F):
        # bin by side and report where they are (base_link) and the beam angle they come from
        for side, sel in (('left (+y)', F[:, 1] > 0), ('right (-y)', F[:, 1] < 0)):
            G = F[sel]
            if len(G) < 20: print(f'    {side}: {len(G)} points'); continue
            print(f'    {side}: {len(G)} points, base x {G[:,0].min():+.2f}..{G[:,0].max():+.2f}, y {G[:,1].min():+.2f}..{G[:,1].max():+.2f}, '
                  f'range {G[:,2].min():.2f}..{G[:,2].max():.2f} m, beam {G[:,3].min():+.0f}..{G[:,3].max():+.0f} deg (laser)')
