#!/usr/bin/env python3
"""Wheel odometry yaw vs IMU yaw from a teleop rosbag: does control_master's yaw rate map hold?

Record while driving with teleop_ros.py (control_master up, wheel_yaw_rate_coeffs in control.yaml):
  ros2 bag record -o <bag> --topics /control/wheel/odom /control/imu/wheel_clock
Start and end standing still for a few seconds. The reference is the raw wheel_clock gyro gz minus its
offset, the mean over samples taken while both wheels stand still (same reference as turn_map.py; the
perception imu_processor output is skipped because it learns its own bias/scale corrections).

Each wheel odometry sample is classed by its own twist: in-place (v reported 0, turning), arc (moving and
turning) or straight. Per class: wheel/IMU yaw rate ratio (least squares; 1.00 = the map is right) and the
yaw each source accumulated there. Plot: both yaws over time and their difference.

Gyro scale (IMU vs a floor reference, test/gyro_scale/README.md): turn in place N full turns from a tape
line back to it and measure the leftover angle; --turns N --residual-deg R gives the true turn
sign * (360 N + R), R > 0 past the line. Then scale = true / IMU and wheel odometry / true are printed and
appended to test/gyro_scale/results.csv; --summary prints the per-direction means.
--gyro-scale K multiplies the IMU rate (offset removed) by K, the measured scale, before comparing.

  odom_compare.py <bag dir> [--turning 0.05] [--gyro-scale K [--gyro-scale-cw K]]
  odom_compare.py <bag dir> --turns auto|5 --residual-deg 3.5 [--note text]
  odom_compare.py --summary
"""
import argparse
import math
import sys
import warnings
from pathlib import Path

import csv
from datetime import datetime

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ODOM, IMU = '/control/wheel/odom', '/control/imu/wheel_clock'
warnings.filterwarnings('ignore', message='Use read_next_ext', category=DeprecationWarning)  # stamps unused
STILL_V, STILL_W = 1e-4, 1e-4  # wheel odometry twist at wheel feedback ~0
SCALE_CSV = Path(__file__).resolve().parents[1] / 'gyro_scale' / 'results.csv'
SCALE_FIELDS = ['time', 'bag', 'board', 'dir', 'turns', 'residual_deg', 'true_deg', 'imu_deg', 'wheel_deg', 'gyro_scale',
                'wheel_over_true', 'gyro_offset', 'note']
SLIP_RATE, SLIP_S = 0.1, 0.3  # [rad/s], [s] wheel vs IMU yaw rate apart this much this long: slip event


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    missing = [t for t in (ODOM, IMU) if t not in types]
    if missing:
        sys.exit(f'{bag}: missing topics {missing}')
    odom, imu = [], []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in (ODOM, IMU):
            continue
        m = deserialize_message(data, types[topic])
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if topic == ODOM:
            q = m.pose.pose.orientation
            odom.append((t, m.twist.twist.linear.x, m.twist.twist.angular.z,
                         2 * math.atan2(q.z, q.w)))
        else:
            imu.append((t, m.angular_velocity.z))
    return np.array(sorted(odom)), np.array(sorted(imu))


def summary():
    if not SCALE_CSV.exists():
        sys.exit(f'no {SCALE_CSV}')
    rows = list(csv.DictReader(open(SCALE_CSV, newline='')))
    sd = (lambda a: a.std(ddof=1) if len(a) > 1 else float('nan'))
    print(f'{"board":>11} {"dir":>4} {"n":>2} {"gyro scale":>17}')
    for b in sorted({r['board'] for r in rows}):
        for d in ('ccw', 'cw', 'all'):
            g = [r for r in rows if r['board'] == b and d in ('all', r['dir'])]
            if g:
                k = np.array([float(r['gyro_scale']) for r in g])
                print(f'{b:>11} {d:>4} {len(g):2d} {k.mean():8.4f} +-{sd(k):.4f}')
    print(f'{"wheel odom":>11} {"dir":>4} {"n":>2} {"wheel/true":>17}')
    for d in ('ccw', 'cw', 'all'):  # one wheel value per run: wheel_clock rows (or the first board's)
        lead = 'wheel_clock' if any(r['board'] == 'wheel_clock' for r in rows) else rows[0]['board']
        q = np.array([float(r['wheel_over_true']) for r in rows if r['board'] == lead and d in ('all', r['dir'])])
        if len(q):
            print(f'{"":>11} {d:>4} {len(q):2d} {q.mean():8.4f} +-{sd(q):.4f}')
    print('[cmp] gyro scale -> --gyro-scale (CW apart with --gyro-scale-cw if the directions differ by more '
          'than their spread)')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', type=Path, nargs='?')
    ap.add_argument('--turning', type=float, default=0.05, help='[rad/s] wheel yaw rate counted as turning')
    ap.add_argument('--gyro-scale', type=float, default=1.0, help='IMU rate multiplier (CCW, or both)')
    ap.add_argument('--gyro-scale-cw', type=float, help='IMU rate multiplier for CW (default: --gyro-scale)')
    ap.add_argument('--turns', help='gyro scale run: full in-place turns from the tape line, or auto (nearest '
                    'whole turn to the IMU; safe while the IMU is within 180 deg)')
    ap.add_argument('--residual-deg', type=float, default=0.0, help='angle past (+) / short of (-) the line')
    ap.add_argument('--note', default='')
    ap.add_argument('--summary', action='store_true', help='per-direction means of test/gyro_scale/results.csv')
    args = ap.parse_args()
    if args.summary:
        return summary()
    if args.bag is None:
        ap.error('bag is required')
    if args.turns is not None and (args.gyro_scale != 1.0 or args.gyro_scale_cw is not None):
        ap.error('--turns measures the scale: leave --gyro-scale out')
    odom, imu = read(args.bag)
    if len(odom) < 10 or len(imu) < 10:
        sys.exit('too few samples')
    t0 = odom[0, 0]
    to, v, w, yaw_w = odom[:, 0] - t0, odom[:, 1], odom[:, 2], np.unwrap(odom[:, 3])
    ti, gz = imu[:, 0] - t0, imu[:, 1]

    # Gyro offset: IMU samples whose nearest odometry sample has both wheels still.
    still_o = (np.abs(v) < STILL_V) & (np.abs(w) < STILL_W)
    near = np.clip(np.searchsorted(to, ti), 0, len(to) - 1)
    still_i = still_o[near]
    if still_i.sum() < 50:
        sys.exit(f'only {still_i.sum()} still IMU samples: stand still a few seconds at the start and end')
    offset = gz[still_i].mean()
    rate_i = gz - offset
    k_cw = args.gyro_scale if args.gyro_scale_cw is None else args.gyro_scale_cw
    rate_i = rate_i * np.where(rate_i > 0, args.gyro_scale, k_cw)

    # IMU yaw at the odometry stamps, both starting at 0.
    yaw_i = np.concatenate([[0.0], np.cumsum(np.diff(ti) * (rate_i[1:] + rate_i[:-1]) / 2)])
    yaw_i_o = np.interp(to, ti, yaw_i) - np.interp(to[0], ti, yaw_i)
    yaw_w = yaw_w - yaw_w[0]
    rate_i_o = np.interp(to, ti, rate_i)
    err = np.degrees(yaw_w - yaw_i_o)

    turning = np.abs(w) > args.turning
    classes = {'in-place': turning & (v == 0.0), 'arc': turning & (v != 0.0),
               'straight': ~turning & (np.abs(v) > 1e-3)}
    dt = np.diff(to, append=to[-1])
    print(f'[cmp] {args.bag.name}: {to[-1]:.0f} s, odom {len(to)} / IMU {len(ti)} samples, '
          f'gyro offset {offset:+.5f} rad/s from {still_i.sum()} still samples'
          + (f', IMU x {args.gyro_scale:g} CCW / {k_cw:g} CW' if (args.gyro_scale, k_cw) != (1.0, 1.0) else ''))
    if args.turns is not None:
        imu_deg, wheel_deg = np.degrees(yaw_i_o[-1]), np.degrees(yaw_w[-1])
        if abs(imu_deg) < 180:
            sys.exit(f'IMU turned only {imu_deg:+.1f} deg: not a full-turn run')
        sign = 1.0 if imu_deg > 0 else -1.0
        if args.turns == 'auto':
            args.turns = round((abs(imu_deg) - args.residual_deg) / 360.0)
            print(f'[cmp] turns auto: {args.turns} (IMU {imu_deg:+.1f} deg)')
        elif not args.turns.isdigit():
            sys.exit('--turns must be a whole number or auto')
        args.turns = int(args.turns)
        true_deg = sign * (360.0 * args.turns + args.residual_deg)
        if abs(true_deg - imu_deg) > 90:
            sys.exit(f'true {true_deg:+.1f} vs IMU {imu_deg:+.1f} deg (wheel odometry {wheel_deg:+.1f}): wrong '
                     f'--turns or residual sign? IMU says {abs(imu_deg) / 360:.2f} turns (--turns auto)')
        scale, wq = true_deg / imu_deg, wheel_deg / true_deg
        d = 'ccw' if sign > 0 else 'cw'
        print(f'[cmp] gyro scale run {d}: true {true_deg:+.1f} deg, IMU {imu_deg:+.1f} deg, wheel odometry '
              f'{wheel_deg:+.1f} deg -> gyro scale {scale:.4f}, wheel/true {wq:.4f}')
        new = not SCALE_CSV.exists()
        with open(SCALE_CSV, 'a', newline='') as f:
            wr = csv.writer(f)
            if new:
                wr.writerow(SCALE_FIELDS)
            wr.writerow([datetime.now().isoformat(timespec='seconds'), args.bag.name, 'wheel_clock', d, args.turns,
                         args.residual_deg, round(true_deg, 2), round(imu_deg, 2), round(wheel_deg, 2),
                         round(scale, 5), round(wq, 5), round(offset, 6), args.note])
        print(f'[cmp] appended to {SCALE_CSV}')
    print(f'{"class":>13} {"time s":>7} {"wheel/IMU":>10} {"wheel deg":>10} {"IMU deg":>9} {"diff deg":>9}')
    rows = []
    for name, m in classes.items():
        rows.append((name, m))
        if name != 'straight':  # CW and CCW apart: an asymmetry cancels in the total
            rows += [(f'{name} CCW', m & (w > 0)), (f'{name} CW', m & (w < 0))]
    for name, m in rows:
        if m.sum() < 5:
            print(f'{name:>13} {dt[m].sum():7.1f}  (too few samples)')
            continue
        ratio = float(np.sum(w[m] * rate_i_o[m]) / np.sum(rate_i_o[m] ** 2)) if np.any(rate_i_o[m]) else math.nan
        yw, yi = np.degrees(np.sum(w[m] * dt[m])), np.degrees(np.sum(rate_i_o[m] * dt[m]))
        print(f'{name:>13} {dt[m].sum():7.1f} {ratio:10.3f} {yw:+10.1f} {yi:+9.1f} {yw - yi:+9.1f}')

    # Slip events: the wheel yaw rate leaves the IMU's by more than SLIP_RATE for at least SLIP_S.
    bad = np.abs(w - rate_i_o) > SLIP_RATE
    edges = np.flatnonzero(np.diff(np.concatenate([[0], bad.astype(int), [0]])))
    events = [(a, b) for a, b in zip(edges[::2], edges[1::2]) if to[b - 1] - to[a] >= SLIP_S]
    for a, b in events:
        print(f'[cmp] slip {to[a]:6.1f}-{to[b - 1]:6.1f} s: wheel {w[a:b].mean():+.3f} vs IMU '
              f'{rate_i_o[a:b].mean():+.3f} rad/s, yaw error {np.degrees(np.sum((w - rate_i_o)[a:b] * dt[a:b])):+.1f} deg')
    if not events:
        print(f'[cmp] no slip events (|wheel - IMU| > {SLIP_RATE} rad/s for >= {SLIP_S} s)')
    turned = np.degrees(np.sum(np.abs(rate_i_o) * dt))
    print(f'[cmp] total: IMU turned {turned:.0f} deg (absolute); final yaw wheel {np.degrees(yaw_w[-1]):+.1f} / '
          f'IMU {np.degrees(yaw_i_o[-1]):+.1f} deg, difference {err[-1]:+.1f} deg (max |{np.abs(err).max():.1f}|), '
          f'{abs(err[-1]) / max(turned, 1e-9) * 360:.1f} deg per 360 deg turned')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ink, muted, grid = '#0b0b0b', '#52514e', '#e4e3df'
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 6.5), dpi=130, sharex=True, height_ratios=[2, 1])
    fig.patch.set_facecolor('#fcfcfb')
    a1.plot(to, np.degrees(yaw_i_o), color=ink, lw=2, label='IMU (reference)')
    a1.plot(to, np.degrees(yaw_w), color='#2a78d6', lw=2, ls='--', label='wheel odometry')
    for m, c in ((classes['in-place'], '#eb6834'), (classes['arc'], '#1baf7a')):
        a1.fill_between(to, 0, 1, where=m, transform=a1.get_xaxis_transform(), color=c, alpha=0.12, lw=0)
    a1.plot([], [], color='#eb6834', alpha=0.3, lw=8, label='in-place turn')
    a1.plot([], [], color='#1baf7a', alpha=0.3, lw=8, label='arc')
    a1.set_ylabel('yaw [deg]', color=ink)
    a1.legend(frameon=False, fontsize=9, loc='best')
    a1.set_title(f'Wheel odometry vs IMU yaw ({args.bag.name})', color=ink, fontsize=11, loc='left')
    a2.plot(to, err, color='#2a78d6', lw=1.5)
    a2.axhline(0, color=muted, lw=0.8)
    a2.set_ylabel('wheel − IMU [deg]', color=ink)
    a2.set_xlabel('time [s]', color=ink)
    for a in (a1, a2):
        a.set_facecolor('#fcfcfb'); a.grid(color=grid, lw=0.8); a.set_axisbelow(True)
        a.tick_params(colors=muted)
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)
    out = args.bag.parent / f'{args.bag.name}_compare.png'
    fig.tight_layout(); fig.savefig(out)
    print(f'[cmp] plot {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
