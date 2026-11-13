#!/usr/bin/python3
"""경로 추종 횡오차(cross-track error) RMSE — rosbag2 기반.

  source /opt/ros/jazzy/setup.bash
  python3 ~/craft/tools/tracking_rmse/tracking_rmse.py BAG [BAG ...] [--odom /perception/state/odom]
          [--min-speed 0.08] [--max-dist 0.5] [--curve 0.5] [--plot OUT_DIR] [--csv OUT.csv]

목표경로: /control/tracking_path (control_master 가 추종 중인 경로, map 프레임). 새 경로가 오면 그 시점부터
그 경로 기준으로 계산하고, 빈 경로가 오면 추종 종료로 본다.
실제경로: --odom (기본 /perception/state/odom, map 프레임). 각 샘플을 현재 목표경로의 가장 가까운 선분에 투영해
부호 있는 횡오차(진행방향 왼쪽 +)를 구한다.
제외: 속도 < --min-speed (정지·제자리 회전·목표점 도착 직후. 기본 0.08: 목표점에서 급정지하는 순간 SLAM 위치가
옆으로 9~11 cm 튀었다가 0.02 m/s 로 되돌아오는데, 그동안 속도가 0.01~0.05 로 찍혀 0.03 기준을 넘었다, tour_1111_1716
t=222 s. 출발과 도착의 저속 구간(v_min 0.03)도 같이 빠진다: 그 구간을 보려면 --min-speed 0.03), 최근접 거리 > --max-dist (경로 프레임 불일치·전환 직후 이상치),
경로 끝점 너머(투영이 끝 선분 밖) 샘플.

S-D: 각 샘플의 S = 그 목표경로 시작점부터 투영점까지의 경로 길이, D = 횡오차. --plot 그림의 세 번째 칸이 S-D
그래프(가로 S, 세로 D; 곡선 구간은 주황), --csv 에 s_m, 방향오차, 경로 곡률, 속도가 같이 나온다.
직선/곡선 분리: 경로 곡률 |k| >= --curve [1/m] 이면 곡선. 곡선의 "바깥쪽 평균"이 + 면 회전이 모자라 바깥으로
밀린 것(언더스티어), - 면 안쪽으로 파고든 것. 방향오차 = 로봇 진행 방향 - 경로 방향 (왼쪽 +).
"""
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit('ROS 2 환경이 필요합니다: source /opt/ros/jazzy/setup.bash')

PATH_TOPIC = '/control/tracking_path'
CURV_WINDOW = 0.06  # [m]


def read_bag(uri, odom_topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if PATH_TOPIC not in types or odom_topic not in types:
        raise RuntimeError(f'토픽 없음: {PATH_TOPIC if PATH_TOPIC not in types else odom_topic}')
    r.set_filter(rosbag2_py.StorageFilter(topics=[PATH_TOPIC, odom_topic]))
    ptype, otype = get_message(types[PATH_TOPIC]), get_message(types[odom_topic])
    paths, odom, frames = [], [], set()
    while r.has_next():
        topic, data, t = r.read_next()
        t *= 1e-9
        if topic == PATH_TOPIC:
            m = deserialize_message(data, ptype)
            xy = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses]).reshape(-1, 2)
            if len(xy):
                frames.add(m.header.frame_id)
            paths.append((t, xy))
        else:
            m = deserialize_message(data, otype)
            frames.add(m.header.frame_id)
            p, v, q = m.pose.pose.position, m.twist.twist.linear, m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            odom.append((t, p.x, p.y, math.hypot(v.x, v.y), yaw + (math.pi if v.x < 0 else 0.0)))  # 후진: 진행 방향
    if len(frames) > 1:
        print(f'  경고: 프레임이 섞여 있음 {frames}', file=sys.stderr)
    return paths, np.array(odom).reshape(-1, 5)


def wrap(a):
    return math.remainder(a, 2 * math.pi)


def cross_track(xy, path):
    """path 폴리라인에 대한 (부호 있는 횡오차 D, 경로 길이 S, 경로 방향, 경로 곡률). 끝점 너머로 투영되면 None."""
    if len(path) < 2:
        return None
    a, b = path[:-1], path[1:]
    d = b - a
    L2 = np.einsum('ij,ij->i', d, d)
    ok = L2 > 1e-12
    if not ok.any():
        return None
    a, d, L2 = a[ok], d[ok], L2[ok]
    s = np.einsum('ij,ij->i', xy - a, d) / L2
    sc = np.clip(s, 0.0, 1.0)
    diff = xy - (a + sc[:, None] * d)
    dist = np.hypot(diff[:, 0], diff[:, 1])
    i = int(np.argmin(dist))
    if (i == 0 and s[i] < 0) or (i == len(a) - 1 and s[i] > 1):
        return None
    cross = d[i, 0] * (xy[1] - a[i, 1]) - d[i, 1] * (xy[0] - a[i, 0])
    # ponytail: 경로 전체에서 가장 가까운 선분. 자기 자신과 겹치는 경로(U턴)면 S가 튈 수 있음 -> 그때는 이전 S 근처만 탐색.
    L = np.sqrt(L2)
    arc = np.concatenate(([0.0], np.cumsum(L)))
    s_here = arc[i] + sc[i] * L[i]
    head = np.arctan2(d[:, 1], d[:, 0])
    # 곡률: S 앞뒤 CURV_WINDOW 사이의 경로 방향 변화 / 그 길이 (2 cm 샘플 한 칸의 꺾임이 튀지 않게).
    s0, s1 = max(s_here - CURV_WINDOW, 0.0), min(s_here + CURV_WINDOW, arc[-1])
    j0, j1 = (min(int(np.searchsorted(arc, x, side='right')) - 1, len(L) - 1) for x in (s0, s1))
    kappa = wrap(head[j1] - head[j0]) / (s1 - s0) if s1 - s0 > 1e-6 else 0.0
    return math.copysign(dist[i], cross), s_here, head[i], kappa


def analyze(paths, odom, min_speed, max_dist):
    """반환: rows = [(t, x, y, e, seg_idx, s, heading_err, curvature, v)], segs = [(t0, path_xy)]"""
    segs, rows = [], []
    pi, cur = 0, None
    for t, x, y, v, yaw in odom:
        while pi < len(paths) and paths[pi][0] <= t:
            pt, pxy = paths[pi]
            if len(pxy) >= 2:
                if cur is None or not np.array_equal(segs[cur][1], pxy):
                    segs.append((pt, pxy)); cur = len(segs) - 1
            else:
                cur = None
            pi += 1
        if cur is None or v < min_speed:
            continue
        c = cross_track(np.array([x, y]), segs[cur][1])
        if c is None or abs(c[0]) > max_dist:
            continue
        e, s, head, kappa = c
        rows.append((t, x, y, e, cur, s, wrap(yaw - head), kappa, v))
    return rows, segs


def stats(e):
    e = np.asarray(e)
    return dict(n=len(e), rmse=float(np.sqrt(np.mean(e ** 2))), mean=float(e.mean()),
                max=float(np.abs(e).max()), p95=float(np.percentile(np.abs(e), 95)))


def plot(bag, rows, segs, odom, out_dir, curve):
    import logging
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)  # 없는 글꼴 경고 줄이기
    plt.rcParams['font.family'] = ['NanumGothic', 'Noto Sans CJK KR', 'NanumSquare', 'Noto Sans CJK JP', 'Noto Sans CJK TC',
                                   'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False
    out_dir.mkdir(parents=True, exist_ok=True)
    fig, (ax, bx, cx) = plt.subplots(1, 3, figsize=(21, 6))
    for k, (_, p) in enumerate(segs):
        ax.plot(p[:, 0], p[:, 1], '--', lw=1.2, label='목표경로' if k == 0 else None, color='tab:blue')
    ax.plot(odom[:, 1], odom[:, 2], lw=0.8, color='0.6', label='실제(전체)')
    r = np.array(rows)
    sc = ax.scatter(r[:, 1], r[:, 2], c=np.abs(r[:, 3]) * 100, s=4, cmap='viridis', label='실제(평가구간)')
    fig.colorbar(sc, ax=ax, label='|횡오차| [cm]')
    ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.legend(); ax.grid(alpha=.3)
    bx.plot(r[:, 0] - odom[0, 0], r[:, 3] * 100, '.', ms=2)
    bx.axhline(0, color='k', lw=.5); bx.set_xlabel('t [s]'); bx.set_ylabel('횡오차 [cm] (왼쪽 +)'); bx.grid(alpha=.3)
    # S-D: 경로마다 S가 0부터 다시 시작하므로, 앞 경로에서 간 거리만큼 밀어 한 줄로 잇는다 (세로 점선 = 새 경로).
    off = 0.0
    for k in np.unique(r[:, 4]):
        q = r[r[:, 4] == k]
        bend = np.abs(q[:, 7]) >= curve
        cx.plot(off + q[~bend, 5], q[~bend, 3] * 100, '.', ms=3, color='tab:blue', label='직선' if off == 0 else None)
        cx.plot(off + q[bend, 5], q[bend, 3] * 100, '.', ms=3, color='tab:orange', label='곡선' if off == 0 else None)
        cx.axvline(off, color='0.5', ls=':', lw=.8)
        cx.text(off, 1.0, f' #{int(k)}', transform=cx.get_xaxis_transform(), va='top', fontsize=8)
        off += q[:, 5].max() - min(q[:, 5].min(), 0.0)
    cx.axhline(0, color='k', lw=.5); cx.set_xlabel('S: 경로를 따라 간 거리 [m]'); cx.set_ylabel('D: 횡오차 [cm] (왼쪽 +)')
    cx.set_title('S-D 그래프'); cx.legend(); cx.grid(alpha=.3)
    s = stats(r[:, 3])
    fig.suptitle(f'{bag.name}  RMSE {s["rmse"]*100:.2f} cm  max {s["max"]*100:.2f} cm  (n={s["n"]})')
    fig.tight_layout()
    f = out_dir / f'{bag.name}_xte.png'
    fig.savefig(f, dpi=130); plt.close(fig)
    return f


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+', type=Path)
    ap.add_argument('--odom', default='/perception/state/odom')
    ap.add_argument('--min-speed', type=float, default=0.08, help='m/s, 이보다 느린 샘플 제외')
    ap.add_argument('--max-dist', type=float, default=0.5, help='m, 이보다 먼 샘플은 이상치로 제외')
    ap.add_argument('--curve', type=float, default=0.5, help='1/m, 경로 곡률이 이 이상이면 곡선 (0.5 = 반경 2 m)')
    ap.add_argument('--plot', type=Path, help='그림 저장 디렉터리')
    ap.add_argument('--csv', type=Path, help='샘플별 횡오차 CSV')
    a = ap.parse_args()

    allrows, writer = [], None
    if a.csv:
        fh = open(a.csv, 'w', newline=''); writer = csv.writer(fh)
        writer.writerow(['bag', 'segment', 't', 'x', 'y', 'xte_m', 's_m', 'heading_err_rad', 'curvature_1pm', 'v_mps'])
    for bag in a.bags:
        print(f'== {bag.name}')
        try:
            paths, odom = read_bag(bag, a.odom)
        except RuntimeError as ex:
            print(f'  건너뜀: {ex}'); continue
        rows, segs = analyze(paths, odom, a.min_speed, a.max_dist)
        if not rows:
            print('  평가할 샘플 없음 (추종 경로 없음 또는 정지 상태)'); continue
        e = np.array([r[3] for r in rows]); seg = np.array([r[4] for r in rows])
        for k in np.unique(seg):
            s = stats(e[seg == k])
            print(f'  경로#{k:<2d} ({len(segs[k][1]):3d}pt)  n={s["n"]:5d}  RMSE {s["rmse"]*100:6.2f} cm'
                  f'  평균 {s["mean"]*100:+6.2f}  p95 {s["p95"]*100:6.2f}  max {s["max"]*100:6.2f}')
        s = stats(e)
        print(f'  [전체] n={s["n"]}  RMSE {s["rmse"]*100:.2f} cm  평균 {s["mean"]*100:+.2f}  '
              f'p95 {s["p95"]*100:.2f}  max {s["max"]*100:.2f}')
        he, kap = np.array([r[6] for r in rows]), np.array([r[7] for r in rows])
        bend = np.abs(kap) >= a.curve
        if (~bend).any():
            s = stats(e[~bend])
            print(f'  [직선] n={s["n"]}  RMSE {s["rmse"]*100:.2f} cm  평균 {s["mean"]*100:+.2f} (왼쪽 +)  max {s["max"]*100:.2f}')
        if bend.any():
            s = stats(e[bend])
            print(f'  [곡선] n={s["n"]}  RMSE {s["rmse"]*100:.2f} cm  바깥쪽 평균 {-np.mean(e[bend] * np.sign(kap[bend]))*100:+.2f}'
                  f' (+ 바깥으로 밀림, - 안쪽으로 파고듦)  max {s["max"]*100:.2f}')
        # 중앙값: 경로 시작점의 제자리 회전이 --min-speed 를 넘겨 섞여 들어오면(수십 도) 평균은 그쪽으로 끌려간다.
        print(f'  [방향오차] 크기 중앙값 {math.degrees(np.median(np.abs(he))):.2f} deg  중앙값 {math.degrees(np.median(he)):+.2f} deg (왼쪽 +)')
        allrows.append(e)
        if writer:
            for t, x, y, ee, k, ss, hh, kk, vv in rows:
                writer.writerow([bag.name, k, f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{ee:.5f}', f'{ss:.4f}', f'{hh:.5f}',
                                 f'{kk:.4f}', f'{vv:.4f}'])
        if a.plot:
            print(f'  그림: {plot(bag, rows, segs, odom, a.plot, a.curve)}')
    if len(allrows) > 1:
        s = stats(np.concatenate(allrows))
        print(f'== 전체 배그 합산  n={s["n"]}  RMSE {s["rmse"]*100:.2f} cm  max {s["max"]*100:.2f} cm')


def demo():
    """자체 점검: python3 -c "import tracking_rmse as t; t.demo()" """
    line = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    e, s, head, kappa = cross_track(np.array([1.5, 0.1]), line)  # 직선 왼쪽 10 cm
    assert abs(e - 0.1) < 1e-9 and abs(s - 1.5) < 1e-9 and abs(head) < 1e-9 and abs(kappa) < 1e-9
    assert cross_track(np.array([2.5, 0.0]), line) is None  # 끝점 너머
    th = np.linspace(0, math.pi / 2, 80)  # 반경 1 m 좌회전: 곡률 +1, 바깥쪽(오른쪽) 5 cm = D -0.05
    arc = np.c_[np.sin(th), 1 - np.cos(th)]
    e, s, head, kappa = cross_track(np.array([1.05 * math.sin(0.8), 1 - 1.05 * math.cos(0.8)]), arc)
    assert abs(e + 0.05) < 1e-3 and abs(s - 0.8) < 1e-2 and abs(kappa - 1.0) < 0.05, (e, s, kappa)
    print('ok')


if __name__ == '__main__':
    main()
