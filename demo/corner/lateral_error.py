#!/usr/bin/env python3
"""Lateral (cross-track) error of a corner demo bag.

Robot pose = map->odom * odom->base_link from /tf; reference = /control/tracking_path (map), each path
valid until the next one. Nearest point is searched in a forward window so a U-shaped path never jumps
to the other branch. Sign: + = left of the path direction.

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/demo/corner/lateral_error.py ~/bags/<name> [--csv out.csv]
"""
import argparse
import bisect
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def read(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()
             if t.name in ('/tf', '/control/tracking_path', '/control/task_result')}
    m2o, o2b, paths, results = [], [], [], []
    while r.has_next():
        topic, data, stamp = r.read_next()
        if topic not in types:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == '/tf':
            for x in msg.transforms:
                s = x.header.stamp.sec + x.header.stamp.nanosec * 1e-9
                t = x.transform.translation
                rec = (s, t.x, t.y, yaw(x.transform.rotation))
                if x.header.frame_id == 'map' and x.child_frame_id == 'odom':
                    m2o.append(rec)
                elif x.header.frame_id == 'odom' and x.child_frame_id == 'base_link':
                    o2b.append(rec)
        elif topic == '/control/tracking_path':
            pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses]).reshape(-1, 2)
            paths.append((stamp * 1e-9, pts))
        else:
            results.append((stamp * 1e-9, msg.task_id, msg.status))
    return sorted(m2o), sorted(o2b), paths, results


def project(p, pts, lo, hi):
    best = (math.inf, lo, 0.0)
    for i in range(max(lo, 0), min(hi, len(pts) - 1)):
        a, ab = pts[i], pts[i + 1] - pts[i]
        l2 = ab @ ab
        u = 0.0 if l2 < 1e-12 else float(np.clip((p - a) @ ab / l2, 0.0, 1.0))
        d = p - (a + u * ab)
        dist = math.hypot(*d)
        if dist < best[0]:
            side = math.copysign(1.0, ab[0] * d[1] - ab[1] * d[0]) if l2 > 1e-12 else 1.0
            best = (dist, i, side * dist)
    return best


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--csv', help='write t, path_n, idx, e, x, y, yaw, v, w per sample')
    args = ap.parse_args()

    m2o, o2b, paths, results = read(args.bag)
    if not m2o or not o2b or not paths:
        raise SystemExit('need map->odom, odom->base_link and /control/tracking_path in the bag')
    mt = [a[0] for a in m2o]
    pose = []
    for s, x, y, th in o2b:
        i = bisect.bisect_right(mt, s) - 1
        if i < 0:
            continue
        _, mx, my, mth = m2o[i]
        pose.append((s, mx + math.cos(mth) * x - math.sin(mth) * y, my + math.sin(mth) * x + math.cos(mth) * y,
                     mth + th))
    pose = np.array(pose)

    rows, prev, idx = [], None, 0
    for k, (t0, pts) in enumerate(paths):
        t1 = paths[k + 1][0] if k + 1 < len(paths) else math.inf
        if len(pts) < 2:
            continue
        key = (len(pts), *np.round(pts[0], 3), *np.round(pts[-1], 3))
        if key != prev:  # a new path, not the same one republished
            idx, prev = 0, key
        for s, x, y, th in pose[(pose[:, 0] >= t0) & (pose[:, 0] < t1)]:
            _, i, e = project(np.array([x, y]), pts, idx - 5, idx + 40)
            idx = max(idx, i)
            rows.append((s, len(pts), i, e, x, y, th))
    if not rows:
        raise SystemExit('no pose samples while a path was active')
    R = np.array(rows)
    t = R[:, 0]
    dt = np.maximum(np.gradient(t), 1e-3)
    v = np.hypot(np.gradient(R[:, 4]), np.gradient(R[:, 5])) / dt
    w = np.abs(np.gradient(np.unwrap(R[:, 6]))) / dt

    def stat(name, mask):
        ix = np.flatnonzero(mask)
        if not len(ix):
            print(f'{name:22s} n=0')
            return
        e = R[ix, 3]
        k = ix[np.argmax(np.abs(e))]
        print(f'{name:22s} n={len(e):5d}  RMSE {1000 * np.sqrt(np.mean(e ** 2)):5.1f} mm  '
              f'mean {1000 * e.mean():+5.1f}  p95 {1000 * np.percentile(np.abs(e), 95):5.1f}  '
              f'max {1000 * abs(R[k, 3]):5.1f} mm ({1000 * R[k, 3]:+.1f}) at {t[k] - t[0]:5.1f} s '
              f'({R[k, 4]:.2f}, {R[k, 5]:.2f}) path{int(R[k, 1])} idx{int(R[k, 2])}')

    moving = v > 0.03
    print(f'bag {args.bag}: {t[-1] - t[0]:.1f} s with a path, {len(paths)} tracking_path msgs')
    stat('all', np.ones(len(R), bool))
    stat('moving (v>3cm/s)', moving)
    stat('turning in place', ~moving & (w > 0.1))
    for n in dict.fromkeys(R[:, 1].astype(int)):
        stat(f'path n={n}', R[:, 1] == n)

    final = {}
    for _, task, status in results:
        final[task] = status
    print('task results (1=SUCCEEDED):', list(final.values()))

    m = np.array(m2o)
    m = m[(m[:, 0] >= t[0]) & (m[:, 0] <= t[-1])]
    jump = np.hypot(np.diff(m[:, 1]), np.diff(m[:, 2]))
    if len(jump):
        print(f'map->odom while driving: max step {1000 * jump.max():.1f} mm, steps > 10 mm: {int(np.sum(jump > 0.01))}')

    print('per 2.5 s:')
    for a in np.arange(0.0, t[-1] - t[0] + 1e-9, 2.5):
        sel = (t - t[0] >= a) & (t - t[0] < a + 2.5)
        if sel.any():
            print(f'  {a:5.1f}s ({R[sel, 4].mean():5.2f},{R[sel, 5].mean():5.2f}) e mean {1000 * R[sel, 3].mean():+6.1f}'
                  f'  |e| max {1000 * np.abs(R[sel, 3]).max():5.1f}  v {v[sel].mean():.3f}  w {w[sel].mean():.2f}')

    if args.csv:
        np.savetxt(args.csv, np.column_stack([t - t[0], R[:, 1:], v, w]), delimiter=',', fmt='%.5f',
                   header='t,path_n,idx,e,x,y,yaw,v,w', comments='')


if __name__ == '__main__':
    main()
