#!/usr/bin/env python3
"""Throwaway demo: the whole sis1 step list (execute.steps_from_plan) as frames for show.py -> sis1_sim.yaml.

Each MANIPULATOR step moves every joint linearly from the previous target at the hardware profile speeds (arm
~41 deg/s, LM ~28 mm/s), so joints arrive together here (on the robot each arrives on its own profile). The puck
rides in the jaws from the close step until the clock release. Gripper motor angles map linearly onto the URDF
jaw (94 deg open, 229 deg closed). Visualization only: never talks to control or hardware.
Usage: source ~/craft/install/setup.bash && python3 sis1_sim.py [shoulder elbow wrist roll start, URDF rad]
"""
import math
import sys
from pathlib import Path

import numpy as np
import yaml

import execute as ex
import plan
from craft_interfaces.msg import TaskCommand

HERE = Path(__file__).resolve().parent
HZ = 20
ARM_SPEED = math.radians(41.2)  # [rad/s] profile velocity 30 x 0.229 rpm
LM_SPEED = 0.028  # [m/s] LM profile velocity 180
HOLD = 10  # frames between steps
# Start pose (URDF): the rest pose of the 2026-11-03 11:28 run; override with 4 numbers on the command line.
START = [0.055, -0.385, 3.106, 1.273, -0.003]


def jaw(motor_rad, a, ctrl):
    """Gripper motor angle -> URDF jaw_upper_joint (linear: open angle -> jaw_open, close angle -> 0)."""
    o, c = ctrl['gripper_open_angle'], ctrl['gripper_close_angle']
    return a['jaw_open'] * (1 - (motor_rad - o) / (c - o))


def quat(R):
    """Rotation matrix -> [x, y, z, w]."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [float(x), float(y), float(z), float(w)]


def main():
    start = list(START)
    if len(sys.argv) == 5:
        start[1:] = [float(v) for v in sys.argv[1:]]
    p = yaml.safe_load(open(HERE / 'plan.yaml'))
    a = p['assume']
    ctrl = yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']
    ch = plan.Chain(plan.URDF)
    steps = ex.steps_from_plan(start[0], start)

    q = np.array(start, dtype=float)
    j = a['jaw_grip']
    puck_world = np.array(p['frames'][0]['puck'], dtype=float)
    puck_rot = np.eye(3)
    held = None  # puck pose in the TCP frame (4x4) while gripped
    frames = []

    def tcp(qv):
        return ch.fk(dict(zip(ex.ARM, qv)))

    def emit(qv, jv, label, n=1):
        nonlocal puck_world, puck_rot
        for _ in range(n):
            if held is not None:
                T = tcp(qv) @ held
                puck_world, puck_rot = T[:3, 3], T[:3, :3]
            joints = {k: float(v) for k, v in zip(ex.ARM, qv)}
            joints['jaw_upper_joint'] = float(jv)
            frames.append({'joints': joints, 'puck': [float(v) for v in puck_world],
                           'puck_quat': quat(puck_rot), 'label': label})

    emit(q, j, 'start', HOLD)
    for name, kind, arg in steps:
        if kind == TaskCommand.MANIPULATOR:
            target = np.array(arg[-1], dtype=float)
            j1 = jaw(target[5], a, ctrl) if len(target) > 5 else j
            target = target[:5]
            if np.isnan(target[1]):
                target[1] = q[1]
            t = max(abs(target[0] - q[0]) / LM_SPEED, np.abs(target[1:] - q[1:]).max() / ARM_SPEED, 0.2)
            n = max(int(t * HZ), 2)
            for k in range(1, n + 1):
                f = k / n
                emit(q + (target - q) * f, j + (j1 - j) * f, name)
            q, j = target, j1
        elif kind == TaskCommand.GRIPPER:
            if arg is True:
                j1 = a['jaw_grip']
            elif arg is False:
                j1 = a['jaw_open']
            else:
                j1 = jaw(arg, a, ctrl)
            for k in range(1, HZ + 1):
                emit(q, j + (j1 - j) * k / HZ, name)
            j = j1
            if arg is True:  # the puck now rides in the jaws (position and orientation)
                P = np.eye(4); P[:3, :3], P[:3, 3] = puck_rot, puck_world
                held = np.linalg.inv(tcp(q)) @ P
            else:
                held = None
        else:  # clock
            emit(q, j, name, HZ)
        emit(q, j, name, HOLD)
    out = HERE / 'sis1_sim.yaml'
    yaml.safe_dump({'assume': a, 'frames': frames}, open(out, 'w'), default_flow_style=None)
    print(f'{len(frames)} frames ({len(frames) / HZ:.0f} s) -> {out}')


if __name__ == '__main__':
    main()
