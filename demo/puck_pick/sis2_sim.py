#!/usr/bin/env python3
"""Throwaway demo: the sis2 step list (execute.unload_steps, staged as a real run from the nominal pose) as frames
for show.py -> sis2_sim.yaml. The jog-taught clock poses (grasp_via, grasp_approach, grasp_ascend) as the URDF and
control.yaml offsets see them; the puck starts in the jaws' place at the grasp approach pose and rides along from
the close step. Same motion model as sis1_sim.py (joints move linearly at the profile speeds and arrive together).
Visualization only: never talks to control or hardware.
Usage: source ~/craft/install/setup.bash && python3 sis2_sim.py      (show.sh --sis2 runs it and RViz)
"""
from pathlib import Path

import numpy as np
import yaml

import execute as ex
import plan
import teach
from craft_interfaces.msg import TaskCommand
from sis1_sim import ARM_SPEED, HOLD, HZ, LM_SPEED, jaw, quat

HERE = Path(__file__).resolve().parent


def main():
    p = yaml.safe_load(open(HERE / 'plan.yaml'))
    a = dict(p['assume'])
    ctrl = yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']
    ch = plan.Chain(plan.URDF)
    tcp = lambda qv: ch.fk(dict(zip(ex.ARM, qv)))
    start = ex.nominal_pose()
    steps = ex.stage(ex.unload_steps(start[0], start), start[:5])

    # The stored puck: in the jaws at the grasp approach pose, its top grasp_depth (28 mm) above the fingertips,
    # its axis along the tool.
    T = tcp(teach.raw_to_target(ex.PRE_GRASP_RAW_DEG)[:5])
    x, d, z = T[:3, 0], T[:3, 1], T[:3, 2]
    tip = T[:3, 3] + a['finger_beyond_tcp'] * d
    puck_world = tip + (a['puck_height'] / 2 - 0.028) * d
    puck_rot = np.column_stack([x, -z, d])  # marker z (the puck axis) along the tool
    a['puck_size'] = [0.03, 0.03, a['puck_height']]

    q = np.array(start[:5], dtype=float)
    j = jaw(start[5], a, ctrl)
    held = None
    frames = []

    def emit(qv, jv, label, n=1):
        nonlocal puck_world, puck_rot
        for _ in range(n):
            if held is not None:
                Tp = tcp(qv) @ held
                puck_world, puck_rot = Tp[:3, 3], Tp[:3, :3]
            joints = {k: float(v) for k, v in zip(ex.ARM, qv)}
            joints['jaw_upper_joint'] = float(jv)
            frames.append({'joints': joints, 'puck': [float(v) for v in puck_world],
                           'puck_quat': quat(puck_rot), 'label': label})

    emit(q, j, 'start (nominal pose)', HOLD)
    for name, kind, arg in steps:
        if kind == TaskCommand.MANIPULATOR:
            for target in arg:  # stage(): the shoulder-held pose, then the pose
                target = np.array(target, dtype=float)
                j1 = jaw(target[5], a, ctrl) if len(target) > 5 else j
                target = target[:5]
                t = max(abs(target[0] - q[0]) / LM_SPEED, np.abs(target[1:] - q[1:]).max() / ARM_SPEED, 0.2)
                n = max(int(t * HZ), 2)
                for k in range(1, n + 1):
                    f = k / n
                    emit(q + (target - q) * f, j + (j1 - j) * f, name)
                q, j = target, j1
        elif kind == TaskCommand.GRIPPER:
            j1 = a['jaw_grip'] if arg is True else a['jaw_open'] if arg is False else jaw(arg, a, ctrl)
            for k in range(1, HZ + 1):
                emit(q, j + (j1 - j) * k / HZ, name)
            j = j1
            if arg is True:
                P = np.eye(4); P[:3, :3], P[:3, 3] = puck_rot, puck_world
                held = np.linalg.inv(tcp(q)) @ P
            else:
                held = None
        emit(q, j, name, HOLD)
    out = HERE / 'sis2_sim.yaml'
    yaml.safe_dump({'assume': a, 'frames': frames}, open(out, 'w'), default_flow_style=None)
    print(f'{len(frames)} frames ({len(frames) / HZ:.0f} s) -> {out}')
    print(f'puck at the grasp (URDF base_link): {np.round(puck_world if held is None else T[:3, 3], 3)}')


if __name__ == '__main__':
    main()
