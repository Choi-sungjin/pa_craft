import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
MOUNT = {'/lidar/front/scan': (0.155, -1, 1, +1), '/lidar/rear/scan': (-0.1295, 1, -1, -1)}   # x0, sx, sy, outward sign in base x
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
BC = 0.01275
for topic, ch in acc.items():
    x0, sx, sy, out = MOUNT[topic]
    P = np.vstack(ch); bx, by = x0 + sx * P[:, 0], sy * P[:, 1]
    body = np.hypot(bx - BC, by) < 0.30
    floor = ~body & (P[:, 0] > -0.10) & (np.hypot(P[:, 0], P[:, 1]) < 1.8)     # the floor hits found before
    ang_body = np.degrees(np.abs(np.arctan2(by, out * (bx - BC))))            # from the body centre, 0 = outward
    ang_lidar = np.degrees(np.abs(np.arctan2(by, out * (bx - x0))))           # from the LiDAR itself
    print(f'{topic}: {floor.sum()} floor hits')
    for fov in (180, 160, 140, 120, 90):
        kb = (ang_body <= fov / 2) & floor; kl = (ang_lidar <= fov / 2) & floor
        print(f'    FOV {fov:3d}: kept by the current body-centred cut {kb.sum():5d}  | by a LiDAR-centred cut {kl.sum():5d}')
    fb = ang_lidar[floor]
    print(f'    floor hits seen from the LiDAR: {fb.min():.0f}..{fb.max():.0f} deg from its outward axis (90 = level line)')
