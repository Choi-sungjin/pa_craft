#!/usr/bin/python3
"""Arm moved by hand (relaxarm first, control_master NOT running): log the lm_arm board's motor angles to a bag.

Reads the board's 50 Hz CSV feedback straight off the serial port (read only, sends nothing) and writes
/hand/arm_motor (sensor_msgs/JointState, raw motor rad: LM shoulder elbow wrist_pitch tool_roll gripper,
velocity[0] = board arm_lm_state) to ~/bags/<name>_<MMDD_HHMM>, plus motor.csv (deg, flushed per line, so it
survives a kill that leaves the mcap unclosed). Ctrl+C or SIGTERM to stop.
Usage: source ~/craft/install/setup.bash && /usr/bin/python3 hand_bag.py [name]   (name default: hand)
"""
import math
import signal
import subprocess
import sys
import time
from pathlib import Path

import rosbag2_py
import serial
from rclpy.serialization import serialize_message
from sensor_msgs.msg import JointState

PORT = '/dev/opencr_arm_lm'
NAMES = ['LM', 'shoulder', 'elbow', 'wrist_pitch', 'tool_roll', 'gripper']
FIELDS = 20  # lm_arm CSV line length (6 positions, state, 6 currents, imu_ok, gx gy gz ax ay az)
TOPIC = '/hand/arm_motor'


def main():
    real = str(Path(PORT).resolve())
    if subprocess.run(['fuser', real], capture_output=True).returncode == 0:
        sys.exit(f'{PORT} is in use (control_master running?). Close it first.')
    out = Path.home() / 'bags' / f"{sys.argv[1] if len(sys.argv) > 1 else 'hand'}_{time.strftime('%m%d_%H%M')}"
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(out), storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    w.create_topic(rosbag2_py.TopicMetadata(id=0, name=TOPIC, type='sensor_msgs/msg/JointState',
                                            serialization_format='cdr'))
    csv = open(out / 'motor.csv', 'w', buffering=1)
    csv.write('t,' + ','.join(NAMES) + ',state\n')
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    s = serial.Serial(PORT, 115200, timeout=0.1); s.dtr = True
    time.sleep(0.2); s.reset_input_buffer()
    print(f'recording {TOPIC} -> {out}  (Ctrl+C to stop)')
    buf, n, last = b'', 0, 0.0
    try:
        while True:
            buf += s.read(4096)
            *lines, buf = buf.split(b'\n')
            for line in lines:
                try:
                    v = [float(x) for x in line.decode(errors='replace').split(',')]
                except ValueError:
                    continue
                if len(v) != FIELDS:
                    continue
                t = time.time_ns()
                m = JointState(name=NAMES, position=v[:6], velocity=[v[6]])
                m.header.stamp.sec, m.header.stamp.nanosec = divmod(t, 10**9)
                w.write(TOPIC, serialize_message(m), t)
                csv.write(f'{t / 1e9:.3f},' + ','.join(f'{math.degrees(x):.2f}' for x in v[:6]) + f',{int(v[6])}\n')
                n += 1
                if time.time() - last > 1.0:  # one live line per second
                    last = time.time()
                    print('  '.join(f'{k} {math.degrees(x):.1f}' for k, x in zip(NAMES, v[:6]))
                          + f'  state {int(v[6])}  ({n} samples)', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
        csv.close()
        del w
        print(f'\n{n} samples -> {out}')


if __name__ == '__main__':
    main()
