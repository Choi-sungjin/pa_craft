#!/usr/bin/python3
"""Upper-cam bag -> video with the camera distance results written on each frame + a gap/latency timeline.

  source ~/craft/install/setup.bash
  /usr/bin/python3 ~/craft/tools/realsense_check/uppercam_bag_overlay.py ~/bags/L1cam_1109_2228 [--out DIR] [--fps 4]

Frames: /perception/uppercam/image_raw/compressed (the bridge preview, 2 Hz). On each: the box phase, robot speed
(/odometry/local), the camera result of that capture (/perception/uppercam/observation: gap, valid/reason,
uncertainty; /perception/uppercam/diagnostics: raw gap, processing, capture age) and the reference gap: the LiDAR gap
at the stage point (/perception/box/observation, last valid) minus the odom travel along the heading since then.
Timeline: camera gap at capture time and at arrival time, the reference, STOP commands, box phases.
"""
import argparse
import bisect
import json
import math
import os
import sys
from pathlib import Path

import cv2
import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lidar_floor'))
from floor_common import read_bag  # noqa: E402

STOP = 3  # craft_interfaces/TaskCommand.STOP (checked at run time below when importable)


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag'); ap.add_argument('--out'); ap.add_argument('--fps', type=float, default=4.0)
    a = ap.parse_args()
    name = os.path.basename(a.bag.rstrip('/'))
    out = Path(a.out or Path(__file__).resolve().parent / 'out' / name); out.mkdir(parents=True, exist_ok=True)
    try:
        from craft_interfaces.msg import TaskCommand
        stop_op = TaskCommand.STOP
    except Exception:
        stop_op = STOP
    imgs, cam, diag, lidar, odom, phase, stops = [], [], [], [], [], [], []
    topics = {'/perception/uppercam/image_raw/compressed', '/perception/uppercam/observation',
              '/perception/uppercam/diagnostics', '/perception/box/observation', '/odometry/local',
              '/planning/box_phase', '/planning/task_command'}
    for topic, m, t in read_bag(a.bag, topics):
        if topic == '/perception/uppercam/image_raw/compressed':
            imgs.append((stamp(m.header), t, bytes(m.data)))
        elif topic == '/perception/uppercam/observation':
            cam.append((stamp(m.header), t, m.valid, m.gap_m, m.gap_uncertainty_m, m.reason))
        elif topic == '/perception/uppercam/diagnostics':
            try:
                d = json.loads(m.data); diag.append((d.get('capture_stamp_s') or 0.0, t, d))
            except ValueError:
                pass
        elif topic == '/perception/box/observation':
            if m.valid and m.source == 'lidar':
                lidar.append((stamp(m.header), t, m.gap_m))
        elif topic == '/odometry/local':
            q = m.pose.pose.orientation
            odom.append((t, m.pose.pose.position.x, m.pose.pose.position.y,
                         math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), m.twist.twist.linear.x))
        elif topic == '/planning/box_phase':
            if not phase or phase[-1][1] != m.data:
                phase.append((t, m.data))
        elif topic == '/planning/task_command':
            if m.operation == stop_op:
                stops.append(t)
    if not imgs:
        raise SystemExit('no /perception/uppercam/image_raw/compressed in the bag')
    if not cam and diag:  # bag from a machine without craft_interfaces: the diagnostics JSON carries the result
        cam = [(d.get('capture_stamp_s') or ct, t, bool(d.get('valid')), (d.get('gap_mm') or float('nan')) / 1000,
                (d.get('gap_uncertainty_mm') or 0) / 1000, d.get('error') or d.get('state') or '') for ct, t, d in diag]
        print('no /perception/uppercam/observation: using /perception/uppercam/diagnostics')
    if not stops:  # no /planning/task_command: the box phase entering final_stop
        stops = [pt for pt, pn in phase if pn == 'final_stop']
    O = np.array(odom); ot = O[:, 0]
    cam_t = [c[0] for c in cam]; diag_t = [d[0] for d in diag]; ph_t = [p[0] for p in phase]

    def odom_at(t):
        i = min(max(bisect.bisect_left(ot, t), 0), len(ot) - 1); return O[i]

    # reference: last valid LiDAR gap (stage point) minus the travel along the heading since then
    ref0 = lidar[-1] if lidar else None

    def ref_gap(t):
        if not ref0 or t < ref0[1]:
            return None
        p0, p1 = odom_at(ref0[1]), odom_at(t)
        return ref0[2] - ((p1[1] - p0[1]) * math.cos(p0[3]) + (p1[2] - p0[2]) * math.sin(p0[3]))

    def nearest(ts, t, tol):
        i = bisect.bisect_left(ts, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(ts)), key=lambda j: abs(ts[j] - t), default=None)
        return best if best is not None and abs(ts[best] - t) <= tol else None

    t0 = imgs[0][1]
    # keep the camera window: from the uppercam request to a few seconds after the last STOP
    first = next((c[1] for c in cam), t0); last = (stops[-1] + 4) if stops else imgs[-1][1]
    frames = [im for im in imgs if first - 2 <= im[1] <= last]
    h, w = cv2.imdecode(np.frombuffer(frames[0][2], np.uint8), cv2.IMREAD_COLOR).shape[:2]
    vw = cv2.VideoWriter(str(out / f'{name}_overlay.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), a.fps, (w, h + 150))
    for k, (cs, rt, data) in enumerate(frames):
        img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        canvas = np.zeros((h + 150, w, 3), np.uint8); canvas[:h] = img
        ci = nearest(cam_t, cs, 0.02); di = nearest(diag_t, cs, 0.02)
        pi = bisect.bisect_right(ph_t, rt) - 1
        o = odom_at(rt); ref = ref_gap(rt)
        lines = [f't {rt - t0:6.1f} s  phase {phase[pi][1] if pi >= 0 else "-"}  v {o[4] * 1000:5.1f} mm/s',
                 f'ref (LiDAR stage gap - odom travel): {ref * 1000:6.1f} mm' if ref is not None else 'ref: -']
        if ci is not None:
            c = cam[ci]
            col = (80, 230, 80) if c[2] else (60, 60, 240)
            lines.append(f'camera: {"VALID" if c[2] else "invalid"} gap {c[3] * 1000:6.1f} mm +-{c[4] * 1000:.1f}  '
                         f'arrival {(c[1] - c[0]) * 1000:4.0f} ms after capture')
            if not c[2]:
                lines.append(f'  reason: {c[5][:70]}')
        else:
            col = (180, 180, 180); lines.append('camera: no result for this capture')
        if di is not None:
            d = diag[di][2]
            lines.append(f'diag: raw {d.get("raw_gap_mm")} mm  proc {d.get("processing_ms") or 0:.0f} ms  '
                         f'capture age {d.get("capture_age_ms") or 0:.0f} ms  {(d.get("error") or "")[:40]}')
        if any(abs(s - rt) < 0.5 for s in stops):
            cv2.putText(canvas, 'STOP', (w - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (8, h + 24 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        col if line.startswith('camera') else (230, 230, 230), 1, cv2.LINE_AA)
        if ci is not None and cam[ci][2]:
            cv2.putText(canvas, f'{cam[ci][3] * 1000:.1f} mm', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (80, 230, 80), 3)
        vw.write(canvas)
        if k % 10 == 0 or any(abs(s - rt) < 0.5 for s in stops):
            cv2.imwrite(str(out / f'frame_{rt - t0:06.1f}.jpg'), canvas)
    vw.release()

    # timeline
    fig, ax = plt.subplots(figsize=(13, 5.5))
    V = [(c[0] - t0, c[1] - t0, c[3] * 1000) for c in cam if c[2]]
    if V:
        V = np.array(V)
        ax.plot(V[:, 0], V[:, 2], '.', ms=3, color='#2ca02c', label='camera gap at capture time')
        ax.plot(V[:, 1], V[:, 2], '.', ms=3, color='#98df8a', label='camera gap at arrival time')
    tt = np.arange(first - 2, last, 0.05)
    R = [(x - t0, ref_gap(x) * 1000) for x in tt if ref_gap(x) is not None]
    if R:
        R = np.array(R); ax.plot(R[:, 0], R[:, 1], '-', color='#1f77b4', lw=1.2, label='ref: LiDAR stage gap - odom travel')
    for s in stops:
        ax.axvline(s - t0, color='r', lw=1, ls='--')
    for (pt, pn), nxt in zip(phase, phase[1:] + [(last, '')]):
        if pt <= last and nxt[0] >= first - 2:
            ax.text(pt - t0, 1.0, pn, rotation=90, fontsize=7, va='bottom', transform=ax.get_xaxis_transform())
    ax.axhline(10, color='k', lw=0.8, ls=':'); ax.axhline(0, color='k', lw=0.8)
    ax.set_xlim(first - 2 - t0, last - t0); ax.set_ylim(-20, 520); ax.grid(alpha=.3)
    ax.set_xlabel('bag time [s]'); ax.set_ylabel('gap [mm]'); ax.legend(fontsize=8, loc='upper right')
    ax.set_title(f'{name}: upper-cam gap vs reference (red dashed = STOP command)')
    fig.tight_layout(); fig.savefig(out / f'{name}_timeline.png', dpi=100)

    lat = [(c[1] - c[0]) * 1000 for c in cam if c[2]]
    print(f'frames in video {len(frames)}, camera results {len(cam)} (valid {len(V) if len(V) else 0}), STOP at '
          f'{[round(s - t0, 2) for s in stops]} s')
    if lat:
        print(f'valid camera result arrival after capture: median {np.median(lat):.0f} ms, p95 {np.percentile(lat, 95):.0f} ms')
    print('video   :', out / f'{name}_overlay.mp4')
    print('timeline:', out / f'{name}_timeline.png')
    print('frames  :', out)


if __name__ == '__main__':
    main()
