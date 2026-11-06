#!/usr/bin/env python3
"""Both OpenCR IMUs side by side: tilt from the accelerometer, gyro, and the difference, until Ctrl+C.

Reads the normal CRAFT firmware feedback (wheel_clock and lm_arm, IMU tail imu_ok,gx,gy,gz,ax,ay,az) and
sends nothing. Each printed line is the mean of the lines received since the previous print. Roll/pitch
are in each board's own frame (both upright): roll = atan2(ay, az), pitch = atan2(-ax, sqrt(ay^2 + az^2)).
A board mounted lower at one end shows up as a pitch (or roll, depending on how it is turned) difference.
Stop control_master first; run under both board locks (_craft_locked all).

  imu_compare.py [--rate 2] [--seconds 0 (= until Ctrl+C)]
"""
import argparse
import math
import sys
import time

import serial

BOARDS = [('wheel', '/dev/opencr_wheel_clock', 12), ('arm', '/dev/opencr_arm_lm', 20)]  # CSV line length


def open_port(path):
    port = serial.Serial(path, 115200, timeout=0)
    try:
        port.dtr = True  # OpenCR USB CDC sends only with DTR set
    except OSError:
        pass
    port.reset_input_buffer()
    return port


def drain(port, buf, fields, out):
    """Append the IMU tail (imu_ok, gx, gy, gz, ax, ay, az) of every complete feedback line to out."""
    buf += port.read(port.in_waiting or 1)
    *lines, rest = buf.split(b'\n')
    for line in lines:
        try:
            v = [float(x) for x in line.decode(errors='replace').split(',')]
        except ValueError:
            continue
        if len(v) == fields and all(math.isfinite(x) for x in v):
            out.append(v[-7:])
    return rest


def summary(rows):
    n = len(rows)
    ok = min(int(r[0]) for r in rows)
    gx, gy, gz, ax, ay, az = (sum(r[i] for r in rows) / n for i in range(1, 7))
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
    return ok, roll, pitch, math.sqrt(ax * ax + ay * ay + az * az), (gx, gy, gz)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=2.0, help='printed lines per second')
    ap.add_argument('--seconds', type=float, default=0.0, help='stop after this long (0 = until Ctrl+C)')
    args = ap.parse_args()

    ports, bufs = [], []
    for name, path, _ in BOARDS:
        try:
            ports.append(open_port(path))
        except serial.SerialException as e:
            sys.exit(f'[imu] {name}: cannot open {path}: {e} (control_master running?)')
        bufs.append(b'')
    print('[imu] roll/pitch from the accelerometer [deg], |a| [g], gyro [rad/s]; diff = wheel - arm')
    t0 = last = time.monotonic()
    rows = [[] for _ in BOARDS]
    try:
        while not args.seconds or time.monotonic() - t0 < args.seconds:
            for k, (_, _, fields) in enumerate(BOARDS):
                bufs[k] = drain(ports[k], bufs[k], fields, rows[k])
            now = time.monotonic()
            if now - last < 1 / args.rate:
                time.sleep(0.005)
                continue
            last = now
            if not all(rows):
                missing = [BOARDS[k][0] for k in range(len(BOARDS)) if not rows[k]]
                print(f'[imu] no feedback from {", ".join(missing)} (firmware not running? reset / check)')
                continue
            s = [summary(r) for r in rows]
            parts = [f'{name} ok{ok} roll {roll:+6.2f} pitch {pitch:+6.2f} |a| {na:.3f} '
                     f'g {g[0]:+.3f},{g[1]:+.3f},{g[2]:+.3f}'
                     for (name, _, _), (ok, roll, pitch, na, g) in zip(BOARDS, s)]
            print(' | '.join(parts) + f' | diff roll {s[0][1] - s[1][1]:+6.2f} pitch {s[0][2] - s[1][2]:+6.2f}')
            rows = [[] for _ in BOARDS]
    except KeyboardInterrupt:
        pass
    finally:
        for p in ports:
            p.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
