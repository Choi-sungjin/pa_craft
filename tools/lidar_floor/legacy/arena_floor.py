import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/home/craft/craft/demo/corner')
from map_tools import MapGrid
g = MapGrid('/home/craft/maps/arena3_clean.yaml')
def clr(x, y):
    c = ((x - g.ox) / g.res).astype(int); r = (g.h - (y - g.oy) / g.res).astype(int)
    ok = (r >= 0) & (r < g.h) & (c >= 0) & (c < g.w); out = np.zeros(len(x)); out[ok] = g.dist[r[ok], c[ok]]; return out
MOUNT = {'/lidar/front/scan': (0.155, -1, 1), '/lidar/rear/scan': (-0.1295, 1, -1)}
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
tf = {}; t0 = None; rows = []; nscan = {k: 0 for k in MOUNT}
def T2(t): q = t.transform.rotation; return (t.transform.translation.x, t.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
while r.has_next():
    topic, data, t = r.read_next(); t /= 1e9
    if t0 is None: t0 = t
    if topic == '/tf':
        for tr in deserialize_message(data, get_message(types[topic])).transforms: tf[(tr.header.frame_id, tr.child_frame_id)] = T2(tr)
    elif topic in MOUNT and t - t0 > 16 and ('map', 'odom') in tf:   # after localization
        m = deserialize_message(data, get_message(types[topic])); nscan[topic] += 1
        a, b = tf[('map', 'odom')], tf[('odom', 'base_link')]
        base = (a[0] + math.cos(a[2])*b[0] - math.sin(a[2])*b[1], a[1] + math.sin(a[2])*b[0] + math.cos(a[2])*b[1], a[2] + b[2])
        rg = np.array(m.ranges); th = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > m.range_min) & (rg < 3.0)
        rg, th = rg[ok], th[ok]
        x0, sx, sy = MOUNT[topic]
        bx, by = x0 + sx * rg * np.cos(th), sy * rg * np.sin(th)
        off_out = np.degrees(np.abs(np.remainder(th - math.pi, 2 * math.pi) - math.pi * 0) ) # placeholder
        # angle from this LiDAR's outward axis (laser -x)
        from_out = np.degrees(np.abs(np.arctan2(np.sin(th - math.pi), np.cos(th - math.pi))))
        mx = base[0] + math.cos(base[2]) * bx - math.sin(base[2]) * by
        my = base[1] + math.sin(base[2]) * bx + math.cos(base[2]) * by
        body = np.hypot(bx - 0.01275, by) < 0.30
        w = clr(mx, my)
        keep = ~body & (from_out <= 90)
        rows.append(np.c_[from_out[keep], rg[keep], w[keep], np.full(keep.sum(), t - t0)])
R = np.vstack(rows); ang, rg, wall, tt = R.T
off = wall > 0.10        # more than 10 cm from any map wall: floor (or a person)
print(f'scans after localization: front {nscan["/lidar/front/scan"]}, rear {nscan["/lidar/rear/scan"]}; points in the outward half (< 3 m): {len(R)}, off-wall: {off.sum()}')
print('angle from outward | points | off-wall (floor?) | off-wall per scan | off-wall range median / 10th pct')
ns = sum(nscan.values())
for lo, hi in ((0, 45), (45, 60), (60, 70), (70, 75), (75, 80), (80, 85), (85, 90)):
    s = (ang >= lo) & (ang < hi); o = s & off
    rr = rg[o]
    print(f'   {lo:2d}-{hi:2d} deg       | {s.sum():6d} | {o.sum():6d} ({100*o.sum()/max(1,s.sum()):4.1f} %) | {o.sum()/ns:6.2f} | '
          + (f'{np.median(rr):.2f} / {np.percentile(rr, 10):.2f} m' if len(rr) else '-'))
