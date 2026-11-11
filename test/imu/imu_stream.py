#!/usr/bin/env python3
"""Print the OpenCR on-board IMU continuously (firmware/imu_stream on the wheel board), until Ctrl+C.

Upload first: _craft_locked wheel _craft_fw imu_stream /dev/opencr_wheel_clock (robot still for 5 s).
Back to driving afterwards: resetwheel.

  imu_stream.py [--rate 10] [--log FILE.csv]
--rate only thins the terminal output; --log keeps every 100 Hz line as the board sent it.
"""
import argparse
import sys
import time

import serial

PORT = '/dev/opencr_wheel_clock'
FIELDS = ['t_ms', 'imu_ok', 'acc_raw_x', 'acc_raw_y', 'acc_raw_z', 'gyro_raw_x', 'gyro_raw_y', 'gyro_raw_z',
          'mag_raw_x', 'mag_raw_y', 'mag_raw_z', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps',
          'roll_deg', 'pitch_deg', 'yaw_deg']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rate', type=float, default=10.0, help='printed lines per second')
    ap.add_argument('--log', help='also write every line to this CSV')
    ap.add_argument('--port', default=PORT)
    args = ap.parse_args()

    port = serial.Serial(args.port, 115200, timeout=0.05)
    try:
        port.dtr = True  # OpenCR USB CDC sends only with DTR set
    except OSError:
        pass
    port.reset_input_buffer()
    log = open(args.log, 'w') if args.log else None
    if log:
        log.write(','.join(FIELDS) + '\n')

    buf, last_print, last_rx = b'', 0.0, time.monotonic()
    try:
        while True:
            buf += port.read(port.in_waiting or 1)
            *lines, buf = buf.split(b'\n')
            now = time.monotonic()
            for line in lines:
                v = line.decode(errors='replace').strip().split(',')
                if len(v) != len(FIELDS):
                    if len(v) == 7:
                        sys.exit('[imu] this is the wheel_clock firmware: upload imu_stream first')
                    continue
                last_rx = now
                if log:
                    log.write(','.join(v) + '\n')
                if now - last_print >= 1 / args.rate:
                    last_print = now
                    d = dict(zip(FIELDS, v))
                    print(f"{int(d['t_ms']) / 1000:8.2f} s ok {d['imu_ok']} | "
                          f"acc g {float(d['ax_g']):+.3f} {float(d['ay_g']):+.3f} {float(d['az_g']):+.3f} | "
                          f"gyro dps {float(d['gx_dps']):+8.3f} {float(d['gy_dps']):+8.3f} {float(d['gz_dps']):+8.3f} | "
                          f"rpy {float(d['roll_deg']):+7.2f} {float(d['pitch_deg']):+7.2f} {float(d['yaw_deg']):+7.2f} | "
                          f"raw acc {d['acc_raw_x']:>6} {d['acc_raw_y']:>6} {d['acc_raw_z']:>6} "
                          f"gyro {d['gyro_raw_x']:>6} {d['gyro_raw_y']:>6} {d['gyro_raw_z']:>6} "
                          f"mag {d['mag_raw_x']:>5} {d['mag_raw_y']:>5} {d['mag_raw_z']:>5}")
            if now - last_rx > 1.0:
                sys.exit(f'[imu] no IMU lines from {args.port} (imu_stream uploaded? control_master running?)')
    except KeyboardInterrupt:
        print()
    finally:
        port.close()
        if log:
            log.close()


if __name__ == '__main__':
    main()
