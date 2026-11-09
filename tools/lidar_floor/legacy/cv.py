import sys, os, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, '/home/craft/craft/demo/corner')
from map_tools import MapGrid
g = MapGrid('/home/craft/maps/arena3_clean.yaml')
def clr(x, y):
    c = ((x - g.ox) / g.res).astype(int); r = (g.h - (y - g.oy) / g.res).astype(int)
    ok = (r >= 0) & (r < g.h) & (c >= 0) & (c < g.w); out = np.zeros(len(x)); out[ok] = g.dist[r[ok], c[ok]]; return out
XF, XR, BC = 0.155, -0.1295, 0.01275
def T2(t): q = t.transform.rotation; return (t.transform.translation.x, t.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
def extract(bag, cache):
    if os.path.exists(cache): return np.load(cache)
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    tf = {}; localized = False; last = None; n = 0; rows = []
    while r.has_next():
        topic, data, t = r.read_next()
        if topic == '/tf':
            for tr in deserialize_message(data, get_message(types[topic])).transforms:
                tf[(tr.header.frame_id, tr.child_frame_id)] = T2(tr)
                if (tr.header.frame_id, tr.child_frame_id) == ('map', 'odom'):
                    cur = T2(tr)
                    if last and math.hypot(cur[0] - last[0], cur[1] - last[1]) > 0.3: localized = True
                    last = cur
        elif topic == '/lidar/merged' and localized and ('odom', 'base_link') in tf:
            n += 1
            if n % 2: continue
            m = deserialize_message(data, get_message(types[topic]))
            a, b = tf[('map', 'odom')], tf[('odom', 'base_link')]
            base = (a[0] + math.cos(a[2])*b[0] - math.sin(a[2])*b[1], a[1] + math.sin(a[2])*b[0] + math.cos(a[2])*b[1], a[2] + b[2])
            rg = np.array(m.ranges); th = m.angle_min + np.arange(len(rg)) * m.angle_increment
            ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
            bx, by = rg[ok] * np.cos(th[ok]), rg[ok] * np.sin(th[ok])
            front = bx >= XF - 0.005; rear = bx <= XR + 0.005; src = front | rear
            bx, by, front = bx[src], by[src], front[src]
            lx = np.where(front, bx - XF, XR - bx); ang = np.degrees(np.arctan2(np.abs(by), lx)); rng = np.hypot(lx, by)
            body = np.hypot(bx - BC, by) < 0.30
            mx = base[0] + math.cos(base[2]) * bx - math.sin(base[2]) * by; my = base[1] + math.sin(base[2]) * bx + math.cos(base[2]) * by
            keep = ~body & (rng < 3.0)
            rows.append(np.c_[(~front[keep]).astype(int), np.sign(by[keep]), ang[keep], rng[keep], clr(mx, my)[keep]])
    R = np.vstack(rows); np.save(cache, R); return R
def derive(R):
    lid, side, ang, rng, wall = R.T; floor = wall > 0.10; base = floor[ang < 45].mean(); table = {}
    for li in (0, 1):
        for sd in (1, -1):
            clear, ranges = 90.0, []
            for lo in range(45, 90, 5):
                s = (lid == li) & (side == sd) & (ang >= lo) & (ang < lo + 5); f = s & floor
                if f.sum() / max(1, s.sum()) >= 3 * base:
                    if clear == 90.0: clear = lo
                if clear != 90.0: ranges.append(max(0.0, np.percentile(rng[f], 1) - 0.10) if f.sum() >= 20 else 0.0)
            table[(li, sd)] = (clear, ranges)
    return table
def apply(R, table, step=5.0):
    lid, side, ang, rng, wall = R.T; keep = np.ones(len(R), bool)
    for (li, sd), (clear, ranges) in table.items():
        s = (lid == li) & (side == sd) & (ang > clear) & (ang <= 90)
        b = ((ang - clear) / step).astype(int)
        lim = np.array(ranges + [-1.0])[np.clip(b, 0, len(ranges))]
        keep &= ~(s & (rng > lim))
    return keep
def report(name, R, table):
    lid, side, ang, rng, wall = R.T; floor = wall > 0.10; walls = wall < 0.04; k = apply(R, table)
    edge = ang >= 55
    print(f'  {name}: off-wall (floor) points per 1000 points: before {1000*floor.mean():.1f}, after {1000*(floor&k).sum()/k.sum():.1f}'
          f'  | edge 55-90 deg: before {1000*floor[edge].mean():.1f}, after {1000*(floor&k&edge).sum()/max(1,(k&edge).sum()):.1f}'
          f'  | map-wall points kept: all {100*(walls&k).sum()/walls.sum():.1f} %, edge {100*(walls&k&edge).sum()/max(1,(walls&edge).sum()):.1f} %')
S = sys.argv[1]
A = np.vstack([extract(os.path.expanduser('~/bags/' + b), f'{S}/rows_{b}.npy') for b in ('pocket_2248', 'pocket_2309')])
B = extract(os.path.expanduser('~/bags/fov180_2359'), f'{S}/rows_fov180_2359.npy')
names = {(0, 1): 'front left', (0, -1): 'front right', (1, 1): 'rear left', (1, -1): 'rear right'}
for train, test, tn, sn in ((A, B, 'pocket bags', 'fov180_2359'), (B, A, 'fov180_2359', 'pocket bags')):
    tab = derive(train)
    print(f'table from {tn}: ' + ', '.join(f'{names[k]} clear {v[0]:.0f}' for k, v in tab.items()))
    report(f'applied to {sn} (unseen)', test, tab)
    report(f'applied to {tn} (seen)  ', train, tab)
tab = derive(np.vstack([A, B])); print('table from all three: ' + ', '.join(f'{names[k]} clear {v[0]:.0f} {np.round(v[1], 2).tolist()}' for k, v in tab.items()))
