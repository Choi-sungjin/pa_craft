#!/usr/bin/python3
"""Flat-floor bag -> LiDAR beam model: cone (all beams dip) + mount tilt, per LiDAR, with a figure.

  /usr/bin/python3 cone_fit.py ~/bags/floor_cal_0034 out/cone_fit.png [--h-front 0.012 --h-rear 0.011]

Record the robot standing on a flat floor with nothing within ~1.8 m on the outward sides; measure the beam height
above the floor (2026-11-01: front 1.2 cm, rear 1.1 cm). Model: h / r = c + A cos(th) + B sin(th) (small angles), c the
cone dip, hypot(A, B) the mount tilt. 2026-11-01 result: cone 0.6 deg, mounts 1.65 deg up outward, roll 0.1-0.2 deg,
rms 0.9-1.4 cm (a plain plane, pitch + roll, failed: rms 33-113 cm, see legacy/floorfit.py). Extrapolating to other
angles was inaccurate, so the ROI comes from arena bags (derive_roi.py), not from this model.
"""
import argparse
import math

import matplotlib
import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

from floor_common import BC, FRONT, LIDAR_H, REAR, read_bag, scan_xy, to_base  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag'); ap.add_argument('out')
    ap.add_argument('--h-front', type=float, default=LIDAR_H[FRONT]); ap.add_argument('--h-rear', type=float, default=LIDAR_H[REAR])
    a = ap.parse_args()
    H = {FRONT: a.h_front, REAR: a.h_rear}
    acc, n = {FRONT: [], REAR: []}, 0
    for topic, m, _ in read_bag(a.bag, set(acc)):
        n += 1
        if n % 3:
            continue
        rg, th = scan_xy(m)
        acc[topic].append(np.c_[rg, th])
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    for ax, (topic, ch) in zip(axes, acc.items()):
        h = H[topic]
        R = np.vstack(ch); rg, th = R[:, 0], R[:, 1]
        bx, by = to_base(topic, rg, th)
        floor = (np.hypot(bx - BC, by) >= 0.30) & (rg * np.cos(th) > -0.10) & (rg < 1.8)
        X = np.c_[np.ones(floor.sum()), np.cos(th[floor]), np.sin(th[floor])]
        (c, A, B), *_ = np.linalg.lstsq(X, h / rg[floor], rcond=None)
        err = rg[floor] - h / (X @ np.array([c, A, B]))
        tilt, phi = math.degrees(math.hypot(A, B)), math.degrees(math.atan2(B, A))
        out_rise = -(c - A)                                   # outward = laser th 180 deg
        print(f'{topic}: {floor.sum()} floor points  cone {math.degrees(c):+.2f} deg, mount tilt {tilt:.2f} deg toward '
              f'laser {phi:+.0f} deg; range rms {np.sqrt(np.mean(err ** 2)) * 100:.1f} cm')
        print(f'    straight outward: beam rises {out_rise * 100:.1f} cm/m' if out_rise > 0
              else f'    straight outward: beam falls {-out_rise * 100:.1f} cm/m -> floor at {h / -out_rise:.2f} m')
        for sd in (90, -90, 135, -135):
            e = c + A * math.cos(math.radians(sd)) + B * math.sin(math.radians(sd))
            print(f'    beam at laser {sd:+4d} deg: ' + (f'floor at {h / e:.2f} m' if e > 0 else f'rises {-e * 100:.1f} cm/m, no floor'))
        ax.scatter(np.degrees(th[floor]), rg[floor], s=2, c='#eb6834', label='floor hits')
        tt = np.radians(np.linspace(-180, 180, 721)); ee = c + A * np.cos(tt) + B * np.sin(tt)
        ax.plot(np.degrees(tt), np.where(ee > 0, h / np.maximum(ee, 1e-9), np.nan), color='#2a78d6', lw=1.5,
                label=f'model: cone {math.degrees(c):+.2f} deg + tilt {tilt:.2f} deg')
        ax.set_ylim(0, 2.5); ax.set_xlim(-180, 180); ax.grid(alpha=.3)
        ax.set_xlabel('beam angle in laser frame [deg] (0 = toward robot centre, +-180 = outward)')
        ax.set_ylabel('range to floor [m]'); ax.set_title(topic); ax.legend(fontsize=9)
    fig.tight_layout(); fig.savefig(a.out, dpi=80)
    print('figure:', a.out)


if __name__ == '__main__':
    main()
