#!/usr/bin/python3
"""Throwaway: a sis bag's /control/arm_joints as show.py frames -> bag_replay.yaml (20 Hz, step name as label).
A hand_bag.py bag (/hand/arm_motor, raw motor rad) is mapped to URDF through config/control.yaml (offsets,
directions, LM offset and m/rad, gripper -> jaw as sis1_sim.py), label = raw motor degrees.

Usage: source ~/craft/install/setup.bash && /usr/bin/python3 bag_replay.py ~/bags/sis_MMDD_HHMM
then:  show.py bag_replay.yaml (show.sh's RViz). Visualization only.
"""
import sys
from pathlib import Path

import math

import numpy as np
import yaml

import plan

from bag_plot import read

HERE = Path(__file__).resolve().parent
STEPS = {1: 'arm', 2: 'gripper', 3: 'clock'}


def main():
    b = read(Path(sys.argv[1]).expanduser())
    if '/hand/arm_motor' in b:
        return hand(b)
    J = b['/control/arm_joints']
    t0 = J[0][0]
    ts = np.array([t - t0 for t, _ in J])
    names = list(J[0][1].name)
    pos = np.array([list(m.position) for _, m in J])
    cmds = [(t - t0, f'#{k + 1} {STEPS.get(c.kind, c.kind)} ' +
             ' '.join(f'{v:.2f}' for v in c.joints.position)) for k, (t, c) in enumerate(b['/planning/task_command'])]
    p = yaml.safe_load(open(HERE / 'plan.yaml'))
    puck = p['frames'][0]['puck']
    frames = []
    for t in np.arange(0, ts[-1], 0.05):
        i = min(np.searchsorted(ts, t), len(ts) - 1)
        lab = [c for tc, c in cmds if tc <= t]
        joints = {n: float(v) for n, v in zip(names, pos[i])}
        joints['jaw_upper_joint'] = float(p['assume']['jaw_grip'])
        frames.append({'joints': joints, 'puck': puck, 'label': f't {t:5.1f}  ' + (lab[-1] if lab else 'start')})
    out = HERE / 'bag_replay.yaml'
    yaml.safe_dump({'assume': p['assume'], 'frames': frames}, open(out, 'w'), default_flow_style=None)
    print(f'{len(frames)} frames ({ts[-1]:.0f} s) -> {out}')


def hand(b):
    c = yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']
    off, dirs = c['arm_joint_offsets'], c['arm_joint_directions']
    o, cl = c['gripper_open_angle'], c['gripper_close_angle']
    p = yaml.safe_load(open(HERE / 'plan.yaml'))
    a = p['assume']
    J = b['/hand/arm_motor']
    t0 = J[0][0]
    ts = np.array([t - t0 for t, _ in J])
    raw = np.array([list(m.position) for _, m in J])
    frames = []
    for t in np.arange(0, ts[-1], 0.05):
        r = raw[min(np.searchsorted(ts, t), len(ts) - 1)]
        joints = {'slide_joint': float((r[0] - c['lm_motor_offset']) * c['lm_meters_per_rad'])}
        for k, n in enumerate(['shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']):
            joints[n] = float((r[k + 1] - off[k]) / dirs[k])
        joints['jaw_upper_joint'] = float(a['jaw_open'] * (1 - (r[5] - o) / (cl - o)))
        lab = f't {t:5.1f}  raw ' + ' '.join(f'{math.degrees(x):.1f}' for x in r)
        frames.append({'joints': joints, 'puck': [0.0, 0.0, -1.0], 'label': lab})
    out = HERE / 'bag_replay.yaml'
    yaml.safe_dump({'assume': a, 'frames': frames}, open(out, 'w'), default_flow_style=None)
    print(f'{len(frames)} frames ({ts[-1]:.0f} s) -> {out}')


if __name__ == '__main__':
    main()
