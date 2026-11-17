#!/usr/bin/env python3
"""Tour bags side by side: completion, lateral error, heading offset (does the robot travel where the pose points),
pose jump on stopping, SLAM jumps. One line per bag in each table.

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/tools/tracking_rmse/tour_report.py BAG [BAG ...] [--plot OUT.png]

Heading offset = pose yaw - direction the position actually moves (over 0.4 s), on straight cruise (v > 0.12 m/s,
|yaw rate| < 0.15 rad/s): 0 when the robot travels where the pose says it points, - = the pose points right of the
travel. By travel direction in the map (right +x, up +y, left, down) and per 0.5 m cell for comparing bags.
Stop jump = how far the pose in use moved sideways within 2.5 s after the robot stopped (v from > 0.10 to < 0.02).
SLAM jump = one map->odom update moving the robot's map pose more than 6 cm or 2.5 deg (after the first 15 s).
Wall scatter = scatter of (wall direction in the scan + yaw) over the run, with the SLAM yaw and with the wheel + IMU
yaw: the walls do not move, so it is the error of that yaw (+ scan noise).
"""
import argparse
import collections
import math
import sys
import warnings
from pathlib import Path

import numpy as np

warnings.simplefilter('ignore')
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path.home() / 'craft/tools/tracking_rmse'))
import rosbag2_py  # noqa: E402
import tracking_rmse as T  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

yawq = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))  # noqa: E731
wrap = np.vectorize(lambda a: math.remainder(a, 2 * math.pi))
DIRS = ['→', '↑', '←', '↓']


def uri_of(bag):
    f = list(Path(bag).glob('*.mcap'))
    return (str(bag), '') if (Path(bag) / 'metadata.yaml').exists() else (str(f[0]), 'mcap')


def wall_dir(m):
    rg = np.array(m.ranges, float)
    a = m.angle_min + np.arange(len(rg)) * m.angle_increment
    ok = np.isfinite(rg) & (rg > 0.4) & (rg < 6)
    x, y = rg * np.cos(a), rg * np.sin(a)
    g = ok[:-4] & ok[4:]
    dx, dy = (x[4:] - x[:-4])[g], (y[4:] - y[:-4])[g]
    L = np.hypot(dx, dy)
    g2 = (L > 0.01) & (L < 0.12)
    if g2.sum() < 40:
        return None
    z = np.sum(np.exp(4j * np.arctan2(dy[g2], dx[g2])))
    return np.angle(z) / 4, abs(z) / g2.sum()


def load(bag):
    uri, sid = uri_of(bag)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id=sid), rosbag2_py.ConverterOptions('', ''))
    want = ('/tf', '/perception/state/odom', '/planning/state', '/lidar/merged')
    tt = {t.name: get_message(t.type) for t in r.get_all_topics_and_types() if t.name in want}
    r.set_filter(rosbag2_py.StorageFilter(topics=list(tt)))
    m2o, o2b, st, ps, sc = [], [], [], [], []
    while r.has_next():
        topic, d, t = r.read_next()
        t *= 1e-9
        m = deserialize_message(d, tt[topic])
        if topic == '/tf':
            for x in m.transforms:
                k = (x.header.frame_id, x.child_frame_id)
                tr = x.transform
                if k == ('map', 'odom'):
                    m2o.append((t, tr.translation.x, tr.translation.y, yawq(tr.rotation)))
                elif k == ('odom', 'base_link'):
                    o2b.append((t, tr.translation.x, tr.translation.y, yawq(tr.rotation)))
        elif topic == '/planning/state':
            ps.append((t, m.data))
        elif topic == '/lidar/merged':
            w = wall_dir(m)
            if w and w[1] > 0.5:
                sc.append((t, w[0]))
        else:
            p = m.pose.pose
            st.append((t, p.position.x, p.position.y, yawq(p.orientation), m.twist.twist.linear.x, m.twist.twist.angular.z))
    return [np.array(sorted(a)) for a in (m2o, o2b, st, sc)] + [sorted(ps)], uri


def report(bag):
    (m2o, o2b, st, sc, ps), uri = load(bag)
    out = {'name': Path(bag).name}
    t, v, w = st[:, 0], st[:, 4], st[:, 5]
    t0 = t[0]
    mov = st[np.abs(v) > 0.03]
    out['dur'] = t[-1] - t0
    out['dist'] = float(np.sum(np.hypot(np.diff(mov[:, 1]), np.diff(mov[:, 2]))))
    out['done'] = sum(1 for _, s in ps if s == 'Run.Complete')
    out['fault'] = sum(1 for _, s in ps if s == 'Fault')
    out['end'] = (st[-1, 1], st[-1, 2])
    # lateral error
    paths, odom = T.read_bag(Path(uri), '/perception/state/odom')
    rows, _ = T.analyze(paths, odom, 0.08, 0.5)
    r = np.array(rows)
    e, s = r[:, 3] * 100, r[:, 5]
    out['lat'] = (np.sqrt(np.mean(e ** 2)), e.mean(), e.std(), np.median(np.abs(e)), 100 * np.mean(np.abs(e) > 3))
    out['bys'] = [(e[m].mean(), e[m].std()) if m.sum() > 30 else (np.nan, np.nan) for m in (s < 0.5, (s >= 0.5) & (s < 1.5), s >= 1.5)]
    resid = []
    for k in np.unique(r[:, 4]):
        m = r[:, 4] == k
        if m.sum() >= 40:
            resid += list(e[m] - e[m].mean())
    out['wob'] = float(np.std(resid))
    # heading offset on straight cruise
    k = max(1, int(round(0.2 / np.median(np.diff(t)))))
    dx, dy = st[2 * k:, 1] - st[:-2 * k, 1], st[2 * k:, 2] - st[:-2 * k, 2]
    c = st[k:-k]
    course = np.arctan2(dy, dx)
    off = np.degrees(wrap(c[:, 3] - course))
    g = (c[:, 4] > 0.12) & (np.abs(c[:, 5]) < 0.15) & (np.hypot(dx, dy) > 0.04) & (np.abs(off) < 20)
    o, cr = off[g], np.degrees(course[g])
    q = ((cr + 45) % 360) // 90
    out['off'] = (np.median(o), np.percentile(o, 25), np.percentile(o, 75), 100 * np.mean(o < 0), len(o))
    out['offdir'] = [np.median(o[q == d]) if (q == d).sum() > 30 else np.nan for d in range(4)]
    cells = collections.defaultdict(list)
    for x, y, d, oo in zip(c[g, 1], c[g, 2], q, o):
        cells[(int(np.floor(x * 2)), int(np.floor(y * 2)), int(d))].append(oo)
    out['cells'] = {kk: float(np.median(vv)) for kk, vv in cells.items() if len(vv) >= 12}
    out['offpts'] = np.c_[c[g, 1], c[g, 2], cr, o]
    # pose in use against the raw SLAM pose, and the jump on stopping
    ox, oy = np.interp(t, o2b[:, 0], o2b[:, 1]), np.interp(t, o2b[:, 0], o2b[:, 2])
    oyaw = np.interp(t, o2b[:, 0], np.unwrap(o2b[:, 3]))
    i = np.clip(np.searchsorted(m2o[:, 0], t, side='right') - 1, 0, len(m2o) - 1)
    tx = m2o[i, 1] + np.cos(m2o[i, 3]) * ox - np.sin(m2o[i, 3]) * oy
    ty = m2o[i, 2] + np.sin(m2o[i, 3]) * ox + np.cos(m2o[i, 3]) * oy
    left = (-np.sin(st[:, 3]) * (tx - st[:, 1]) + np.cos(st[:, 3]) * (ty - st[:, 2])) * 100
    drive = v > 0.15
    out['gap'] = (np.median(left[drive]), np.percentile(left[drive], 5), np.percentile(left[drive], 95))
    ev, kk = [], 0
    fast = v > 0.10
    while kk < len(v) - 1:
        if fast[kk] and not fast[kk + 1]:
            j = kk + 1
            while j < len(v) and v[j] > 0.02 and t[j] - t[kk] < 3:
                j += 1
            if j < len(v) and v[j] <= 0.02:
                pre = (t > t[kk] - 1.0) & (t <= t[kk])
                post = (t >= t[j]) & (t < t[j] + 2.5)
                if pre.sum() > 5 and post.sum() > 5:
                    h = st[kk, 3]
                    side = (-math.sin(h) * (st[post, 1] - st[kk, 1]) + math.cos(h) * (st[post, 2] - st[kk, 2])) * 100
                    ev.append((left[pre].mean(), side[np.argmax(np.abs(side))], st[kk, 1], st[kk, 2]))
                kk = j
        kk += 1
    ev = np.array(ev) if ev else np.zeros((0, 4))
    out['stops'] = ev
    # SLAM jumps
    chg = np.where((np.abs(np.diff(m2o[:, 1])) + np.abs(np.diff(m2o[:, 2])) + np.abs(np.diff(m2o[:, 3]))) > 1e-9)[0] + 1
    jx, jy, jyaw = (np.interp(m2o[chg, 0], o2b[:, 0], o2b[:, col]) for col in (1, 2, 3))
    jumps = []
    for j, kq in enumerate(chg):
        if m2o[kq, 0] - t0 < 15:
            continue
        pose = lambda q_: (m2o[q_, 1] + math.cos(m2o[q_, 3]) * jx[j] - math.sin(m2o[q_, 3]) * jy[j],  # noqa: E731
                           m2o[q_, 2] + math.sin(m2o[q_, 3]) * jx[j] + math.cos(m2o[q_, 3]) * jy[j], m2o[q_, 3] + jyaw[j])
        x1, y1, a1 = pose(kq)
        x0, y0, a0 = pose(kq - 1)
        dth = math.degrees(math.remainder(a1 - a0, 2 * math.pi))
        dd = math.hypot(x1 - x0, y1 - y0) * 100
        if abs(dth) > 2.5 or dd > 6:
            jumps.append((m2o[kq, 0] - t0, x1, y1, dd, dth))
    out['jumps'] = np.array(jumps) if jumps else np.zeros((0, 5))
    out['updates'] = len(chg)
    # wall scatter after the first 15 s
    if len(sc) > 50:
        ts = sc[:, 0]
        keep = ts - t0 > 15
        oy_s = np.interp(ts, o2b[:, 0], np.unwrap(o2b[:, 3]))
        i2 = np.clip(np.searchsorted(m2o[:, 0], ts, side='right') - 1, 0, len(m2o) - 1)
        my = np.unwrap(m2o[:, 3])[i2]
        q4 = math.pi / 2
        w90 = lambda a: (a + q4 / 2) % q4 - q4 / 2  # noqa: E731
        res = []
        for yaw in (my + oy_s, oy_s):
            wm = w90(sc[keep, 1] + yaw[keep])
            cm = np.angle(np.mean(np.exp(4j * wm))) / 4
            dev = np.degrees(w90(wm - cm))
            res.append((np.std(dev), 100 * np.mean(np.abs(dev) > 5), dev))
        out['wall'] = res
        out['wall_t'] = ts[keep] - t0
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--plot')
    a = ap.parse_args()
    R = [report(b) for b in a.bags]
    n = max(len(x['name']) for x in R)
    print('== 완주')
    for x in R:
        print(f"  {x['name']:{n}s} {x['dur']:5.0f} s  {x['dist']:5.1f} m  목표점 {x['done']:2d}개 도착  고장 {x['fault']}  끝 위치 ({x['end'][0]:.2f}, {x['end'][1]:.2f})")
    print('== 횡오차 [cm] (속도 0.08 이상): RMSE  평균(왼쪽+)  표준편차  |D|중앙  >3cm | 구간 시작 후 0~0.5 m / 0.5~1.5 / 1.5 m~ 평균(표준편차) | 구간 안 흔들림')
    for x in R:
        b = '  '.join(f'{m:+5.2f}({s:.2f})' for m, s in x['bys'])
        print(f"  {x['name']:{n}s} {x['lat'][0]:5.2f}  {x['lat'][1]:+5.2f}  {x['lat'][2]:5.2f}  {x['lat'][3]:5.2f}  {x['lat'][4]:4.1f}% | {b} | {x['wob']:.2f}")
    print('== 방향 어긋남 [deg] (직선 순항, 방향값 - 실제 이동 방향; 0 = 비스듬히 안 감): 중앙  25%  75%  음수비율  n | 달린 방향별 중앙 ' + ' '.join(DIRS))
    for x in R:
        o = x['off']
        print(f"  {x['name']:{n}s} {o[0]:+5.2f}  {o[1]:+5.1f}  {o[2]:+5.1f}  {o[3]:3.0f}%  {o[4]:5d} | " + '  '.join(f'{d:+5.2f}' for d in x['offdir']))
    print('   같은 0.5 m 칸·같은 방향끼리, 첫 배그와의 차이:')
    for x in R[1:]:
        ks = sorted(set(x['cells']) & set(R[0]['cells']))
        if len(ks) < 8:
            print(f"  {x['name']:{n}s} 공통 칸 {len(ks)}개")
            continue
        u, v = np.array([R[0]['cells'][k] for k in ks]), np.array([x['cells'][k] for k in ks])
        print(f"  {x['name']:{n}s} 공통 {len(ks):2d}칸: {R[0]['name'][-9:]} {u.mean():+.2f} -> {v.mean():+.2f} (차이 평균 {np.mean(v - u):+.2f}, 표준편차 {np.std(v - u):.2f}, 상관 {np.corrcoef(u, v)[0, 1]:+.2f})")
    print('== 제어가 쓰는 위치와 SLAM 원래 위치 [cm]: 달리는 중 SLAM 쪽이 왼쪽으로 중앙 (5~95%) | 정지 때 옆으로 튄 크기: 중앙  3 cm 넘는 횟수 / 정지 횟수  최대')
    for x in R:
        ev = x['stops']
        s = np.abs(ev[:, 1]) if len(ev) else np.array([np.nan])
        print(f"  {x['name']:{n}s} {x['gap'][0]:+5.1f} ({x['gap'][1]:+5.1f} .. {x['gap'][2]:+5.1f}) | {np.median(s):4.1f}   {int((s > 3).sum()):2d} / {len(ev):2d}   {np.nanmax(s):4.1f}")
    print('== SLAM: 한 번에 위치 6 cm 또는 방향 2.5도 넘게 튄 횟수 / 갱신 수 (분당) | 벽 방향 흔들림 [deg]: SLAM 방향 기준 (5도 넘는 시간)  바퀴+IMU 방향 기준')
    for x in R:
        j = x['jumps']
        wl = x.get('wall')
        ws = f"{wl[0][0]:4.2f} ({wl[0][1]:4.1f}%)   {wl[1][0]:4.2f} ({wl[1][1]:4.1f}%)" if wl else '-'
        print(f"  {x['name']:{n}s} {len(j):3d} / {x['updates']:4d}  ({60 * len(j) / x['dur']:4.1f}/분) | {ws}")
        if len(j):
            cc = collections.Counter((int(np.floor(q[1] * 2)), int(np.floor(q[2] * 2))) for q in j)
            print(f"  {'':{n}s}   어디서: " + ', '.join(f'({k[0] / 2:.1f},{k[1] / 2:.1f}):{v}' for k, v in cc.most_common(6)))
    if a.plot:
        import logging
        import matplotlib
        matplotlib.use('Agg')
        import cv2
        import matplotlib.pyplot as plt
        logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
        plt.rcParams['font.family'] = ['Noto Sans CJK JP', 'DejaVu Sans']
        plt.rcParams['axes.unicode_minus'] = False
        OX, OY, RES = -0.79096, -0.246309, 0.01
        img = cv2.imread(str(Path.home() / 'maps/arena5_legend_opt_arena1cm.pgm'), cv2.IMREAD_UNCHANGED)
        ext = [OX, OX + img.shape[1] * RES, OY, OY + img.shape[0] * RES]
        fig, axs = plt.subplots(2, len(R), figsize=(5.2 * len(R), 12.5), squeeze=False)
        for k, x in enumerate(R):
            ax = axs[0, k]
            ax.imshow(img, cmap='gray', extent=ext, vmin=0, vmax=300)
            p = x['offpts']
            sc_ = ax.scatter(p[:, 0], p[:, 1], c=p[:, 3], s=6, cmap='RdBu', vmin=-8, vmax=8)
            j = x['jumps']
            if len(j):
                ax.plot(j[:, 1], j[:, 2], 'x', color='k', ms=7, mew=1.5)
            ax.set_xlim(0, 4.6)
            ax.set_ylim(0, 6.5)
            ax.set_title(f"{x['name']}\n방향 어긋남 중앙 {x['off'][0]:+.2f}°, × = SLAM이 튄 곳 {len(j)}번", fontsize=10)
            bx = axs[1, k]
            if 'wall' in x:
                bx.plot(x['wall_t'], x['wall'][0][2], '.', ms=2, color='#d62728', label='SLAM 방향 기준')
                bx.plot(x['wall_t'], x['wall'][1][2], '.', ms=2, color='#2a78d6', label='바퀴+IMU 방향 기준')
            bx.axhline(0, color='k', lw=.5)
            bx.set_ylim(-12, 12)
            bx.set_xlabel('시간 [s]')
            bx.grid(alpha=.3)
            bx.set_title('지도에서 본 벽 방향의 흔들림 [deg] (0에서 벗어나면 그 방향값이 틀린 것)', fontsize=9)
            if k == 0:
                bx.legend(markerscale=5, fontsize=8)
        fig.colorbar(sc_, ax=axs[0, :].tolist(), label='방향값 - 실제 이동 방향 [deg] (빨강 = 음수)', shrink=.6)
        fig.savefig(a.plot, dpi=90, bbox_inches='tight')
        print('그림:', a.plot)


if __name__ == '__main__':
    main()
