#!/usr/bin/python3
"""Box-approach situations -> approach_sim (our planner + controllers) -> metrics, out/runs.json.

  /usr/bin/python3 run_cases.py [--goal-dist 0.55]

Per situation: the arena5_legend_arena map with that layout box erased (it is re-placed at the situation's yaw as
planner obstacle points along its outline), and one goal per box face: base_link goal_dist from the box centre
along the face normal (0.55 m as tools/path_compare, LiDAR face ~0.24 m from the box), heading toward the box. Each goal runs twice: goal_heading_required false
(planning.yaml today) and true (ALIGN, the planner rounds the heading to E/N/W/S). Metrics from the simulated
trajectory: arrival heading vs the face normal, LiDAR-face gap and lateral offset to the face centre, body
clearance to walls and to the box (V307 base_link outline x -0.141..0.2255, y +-0.1772).
"""
import argparse
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
MAP_YAML = Path.home() / 'maps' / 'arena5_legend_arena.yaml'
HALF, LF = 0.1025, 0.2097
BODY = (-0.141, 0.2255, 0.1772)
BOXES = {'S1': (1.463, 1.151), 'S2': (3.648, 3.421), 'S3': (3.083, 5.341), 'S4': (0.863, 5.381), 'S5': (2.263, 5.356),
         'S6': (3.613, 3.881), 'L1': (3.558, 1.071), 'L2': (2.348, 3.726), 'L3': (3.778, 5.321)}
# id, name, box, yaw [deg], robot start (x, y, yaw deg), one-line setting
SITS = [
    (1, '열린 공간', 'L2', 27, (2.25, 2.55, 90), '사방이 열린 가운데 박스, 로봇은 남쪽에서 북쪽을 보고 출발'),
    (2, '가까운 면이 벽 쪽', 'S4', 27, (0.65, 4.75, 90), '서쪽 벽과 세로벽 사이 박스, 로봇은 남서쪽에서 출발'),
    (3, '포켓 안 박스', 'S1', 27, (2.30, 0.60, 180), '벽 두 장에 둘러싸인 박스, 로봇은 남동쪽 통로에서 출발'),
    (4, '벽 너머 박스', 'S2', 27, (2.90, 2.75, 0), '세로벽 동쪽 통로 안 박스, 로봇은 벽 서쪽에서 출발'),
    (5, '좁은 슬롯', 'S6', 27, (2.95, 4.05, 0), '가로벽 두 장 사이 슬롯 안 박스, 로봇은 슬롯 입구 서쪽에서 출발'),
    (6, '좁은 슬롯, 박스 0°', 'S6', 0, (2.95, 4.05, 0), '5와 같은 자리, 박스를 벽과 나란히(0°)'),
    (7, '출발 패드에서 L2', 'L2', 27, (0.734, 0.950, 0), '미션 출발점(0.734, 0.950)에서 가운데 박스까지 먼 길'),
]


def erased_map(box_id, tag):
    y = yaml.safe_load(open(MAP_YAML))
    im = np.array(Image.open(MAP_YAML.parent / y['image']))
    res, (ox, oy) = y['resolution'], y['origin'][:2]
    H = im.shape[0]
    bx, by = BOXES[box_id]
    c0, c1 = int((bx - 0.125 - ox) / res), int((bx + 0.125 - ox) / res)
    r0, r1 = int((by - 0.125 - oy) / res), int((by + 0.125 - oy) / res)
    sub = im[H - 1 - r1:H - r0, c0:c1 + 1]
    sub[sub < 128] = 254                                   # hits of the mapped box -> free
    (OUT / 'maps').mkdir(parents=True, exist_ok=True)
    Image.fromarray(im).save(OUT / 'maps' / f'{tag}.pgm')
    y2 = dict(y); y2['image'] = f'{tag}.pgm'
    p = OUT / 'maps' / f'{tag}.yaml'; yaml.safe_dump(y2, open(p, 'w')); return p, im, y


def box_outline(bx, by, yaw, step=0.01):
    c, s = math.cos(yaw), math.sin(yaw); pts = []
    for t in np.arange(-HALF, HALF + 1e-9, step):
        for (a, b) in ((t, -HALF), (t, HALF), (-HALF, t), (HALF, t)):
            pts.append([round(bx + c * a - s * b, 4), round(by + s * a + c * b, 4)])
    return pts


def body_poly(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[x + c * a - s * b, y + s * a + c * b] for a, b in
                     ((BODY[0], -BODY[2]), (BODY[1], -BODY[2]), (BODY[1], BODY[2]), (BODY[0], BODY[2]))])


def body_points(x, y, th, step=0.02):
    c, s = math.cos(th), math.sin(th)
    a, b = np.meshgrid(np.arange(BODY[0], BODY[1] + 1e-9, step), np.arange(-BODY[2], BODY[2] + 1e-9, step))
    a, b = a.ravel(), b.ravel()
    return np.stack([x + c * a - s * b, y + s * a + c * b], 1)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--goal-dist', type=float, default=0.55); a = ap.parse_args()
    cases, meta = [], []
    walls = {}
    for sid, name, box, yaw_d, start, desc in SITS:
        mp, im, my = erased_map(box, f'sit{sid}')
        walls[sid] = (im, my)
        bx, by = BOXES[box]; yaw = math.radians(yaw_d)
        obs = box_outline(bx, by, yaw)
        for k in range(4):
            ang = yaw + k * math.pi / 2; n = (math.cos(ang), math.sin(ang))
            d = a.goal_dist
            goal = [bx + n[0] * d, by + n[1] * d, math.atan2(-n[1], -n[0])]
            for heading in (False, True):
                cases.append(dict(name=f's{sid}f{k + 1}{"h" if heading else "n"}', map=str(mp),
                                  start=[start[0], start[1], math.radians(start[2])], goal=goal, heading=heading, obstacles=obs))
                meta.append(dict(sit=sid, face=k + 1, heading=heading, normal=ang, goal=goal))
    json.dump({'cases': cases}, open(OUT / 'cases.yaml', 'w'), default=float)   # JSON is YAML for yaml-cpp
    subprocess.run([str(HERE / 'build' / 'approach_sim'), str(OUT / 'cases.yaml'), str(OUT / 'sim.json')], check=True)
    sims = json.load(open(OUT / 'sim.json'))

    runs = []
    for m, s in zip(meta, sims):
        sid = m['sit']; im, my = walls[sid]
        res, (ox, oy) = my['resolution'], my['origin'][:2]; H = im.shape[0]
        sit = next(t for t in SITS if t[0] == sid); bx, by = BOXES[sit[2]]; yaw = math.radians(sit[3])
        r = dict(m, result=s['result'], error=s['error'], t=s['t'], globals=s['globals'], locals=s['locals'],
                 events=s['events'], traj=(s['traj'][::2] + [s['traj'][-1]]) if s['traj'] else [])
        if s['traj']:
            fx, fy, fth = s['final']
            n = np.array([math.cos(m['normal']), math.sin(m['normal'])])
            r['head_err'] = math.degrees(math.remainder(fth - math.atan2(-n[1], -n[0]), 2 * math.pi))
            lidar = np.array([fx + LF * math.cos(fth), fy + LF * math.sin(fth)])
            rel = lidar - np.array([bx, by])
            r['gap'] = float(rel @ n - HALF)                                   # LiDAR-face centre to the face plane
            r['lateral'] = float(rel @ np.array([-n[1], n[0]]))
            r['goal_err'] = math.hypot(fx - m['goal'][0], fy - m['goal'][1])
            # body clearance: walls (map hits) and the box, over the whole trajectory
            wall_min, box_hit = 9.0, False
            cs, sn = math.cos(-yaw), math.sin(-yaw)
            for (_, x, y, th, *_r) in s['traj']:
                P = body_points(x, y, th)
                c = ((P[:, 0] - ox) / res).astype(int); rr = H - 1 - ((P[:, 1] - oy) / res).astype(int)
                ok = (c >= 0) & (rr >= 0) & (c < im.shape[1]) & (rr < H)
                if (~ok).any() or (im[rr[ok], c[ok]] < 128).any():
                    wall_min = 0.0
                d = P - np.array([bx, by]); lx = cs * d[:, 0] - sn * d[:, 1]; ly = sn * d[:, 0] + cs * d[:, 1]
                if ((np.abs(lx) <= HALF) & (np.abs(ly) <= HALF)).any():
                    box_hit = True
            r['wall_hit'] = wall_min == 0.0; r['box_hit'] = box_hit
        runs.append(r)
        print(f"s{sid} f{m['face']} {'ALIGN' if m['heading'] else 'free '} {s['result']:13s} t={s['t']:5.1f}"
              + (f" head {r['head_err']:+6.1f} gap {r['gap'] * 1000:+6.0f} lat {r['lateral'] * 1000:+5.0f} wall {r['wall_hit']} box {r['box_hit']}" if s['traj'] else '')
              + (f"  [{s['error']}]" if s['error'] else ''))
    json.dump(dict(sits=[dict(id=t[0], name=t[1], box=t[2], yaw=t[3], start=t[4], desc=t[5], bx=BOXES[t[2]][0], by=BOXES[t[2]][1]) for t in SITS],
                   goal_dist=a.goal_dist, runs=runs), open(OUT / 'runs.json', 'w'))


if __name__ == '__main__':
    main()
