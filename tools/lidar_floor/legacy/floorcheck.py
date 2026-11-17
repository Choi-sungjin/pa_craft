import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/home/craft/craft/demo/corner')
from map_tools import MapGrid
g = MapGrid('/home/craft/maps/arena3_clean.yaml')
def clr(x, y):
    c = int((x - g.ox) / g.res); r = int(g.h - (y - g.oy) / g.res)
    return g.dist[r, c] if 0 <= r < g.h and 0 <= c < g.w else 0
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
tf = {}; t0 = None; path = None; merged = None; lastT = -9
def T2(t): q = t.transform.rotation; return (t.transform.translation.x, t.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
def comp(a, b): return (a[0] + math.cos(a[2])*b[0] - math.sin(a[2])*b[1], a[1] + math.sin(a[2])*b[0] + math.cos(a[2])*b[1], a[2] + b[2])
print('time  robot            merged pts near the rest of the path (<24 cm):  beyond 3 m | on map wall | kept as obstacle | removed by filter')
while r.has_next():
    topic, data, t = r.read_next(); t /= 1e9
    if t0 is None: t0 = t
    T = t - t0
    if topic == '/tf':
        for tr in deserialize_message(data, get_message(types[topic])).transforms: tf[(tr.header.frame_id, tr.child_frame_id)] = T2(tr)
    elif topic == '/control/tracking_path':
        m = deserialize_message(data, get_message(types[topic]))
        if m.poses: path = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])
    elif topic == '/lidar/merged':
        merged = deserialize_message(data, get_message(types[topic]))
    elif topic == '/perception/lidar/obstacle' and merged is not None and path is not None and ('map','odom') in tf and T - lastT >= 1.0 and T > 16:
        lastT = T
        obs = deserialize_message(data, get_message(types[topic]))
        base = comp(tf[('map','odom')], tf[('odom','base_link')])
        k = int(np.argmin(np.hypot(path[:, 0] - base[0], path[:, 1] - base[1]))); rest = path[k:]
        rg = np.array(merged.ranges); a = merged.angle_min + np.arange(len(rg)) * merged.angle_increment
        ok = np.isfinite(rg) & (rg > merged.range_min) & (rg < merged.range_max)
        bx, by = rg[ok] * np.cos(a[ok]), rg[ok] * np.sin(a[ok])
        P = np.array([comp(base, (x, y, 0))[:2] for x, y in zip(bx, by)]) if ok.any() else np.zeros((0, 2))
        O = np.array([comp(base, (p.position.x, p.position.y, 0))[:2] for p in obs.poses]) if obs.poses else np.zeros((0, 2))
        if not len(P): continue
        d = np.hypot(P[:, None, 0] - rest[None, :, 0], P[:, None, 1] - rest[None, :, 1]).min(axis=1)
        near = d < 0.24
        far = rg[ok][near] > 3.0
        idx = np.where(near)[0]
        wall = np.array([clr(*P[i]) <= 0.06 for i in idx], bool)
        kept = np.array([len(O) > 0 and np.hypot(O[:, 0] - P[i, 0], O[:, 1] - P[i, 1]).min() < 0.005 for i in idx], bool)
        removed = ~far & ~wall & ~kept
        if near.sum():
            ex = P[idx[removed]][:2] if removed.any() else []
            print(f'{T:6.1f} ({base[0]:+.2f},{base[1]:+.2f})  {near.sum():3d}:  {far.sum():3d} | {(~far & wall).sum():3d} | {(kept).sum():3d} | {removed.sum():3d}  '
                  + ' '.join(f'({x:+.2f},{y:+.2f}) wall {clr(x, y)*100:.0f} cm, {math.hypot(x-base[0], y-base[1]):.2f} m away' for x, y in ex))
