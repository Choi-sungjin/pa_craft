#!/usr/bin/env python3
"""Throwaway demo: pick the planned puck on the REAL robot and load it into the clock, via control_master.
--unload does the reverse: take the puck out of clock slot LOAD_SLOT and put it down at the planned spot.

Sends craft_interfaces/TaskCommand on /planning/task_command one step at a time and waits for
/control/task_result. Keeps the /planning/hold heartbeat alive (control rejects tasks without it).
Press Enter before every step; Ctrl+C sends STOP and hold.
Usage:  source ~/craft/install/setup.bash && python3 ~/craft/demo/puck_pick/execute.py [--unload] [--dry-run]
Delete with: rm -rf ~/craft/demo
"""
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from craft_interfaces.msg import ControlStatus, TaskCommand, TaskResult
from sensor_msgs.msg import JointState
from rclpy.node import Node
from std_msgs.msg import Bool

import plan  # URDF joint limits

HERE = Path(__file__).resolve().parent
ARM = ['slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']
STATUS = {0: 'RUNNING', 1: 'SUCCEEDED', 2: 'FAILED', 3: 'STOPPING', 4: 'STOPPED', 5: 'REJECTED'}
# Loading pose measured with DYNAMIXEL Wizard (2026-10-26), raw motor degrees: shoulder, elbow,
# wrist_pitch, tool_roll, with the LM on the old (LiDAR-centre) grasp line, LOAD_SLIDE.
# Clock slot 12 sits at the loading guide.
LOAD_MOTOR_DEG = [90.0, 140.0, 333.0, 92.0]
LOAD_SLIDE = -0.00035  # [m] before LM_BACKLASH
LOAD_SLOT = 12
# The LM stops ~2 mm short of the puck line (backlash): aim this much further toward the LM motor
# (+slide; the motor end is about +0.063). Applies to every plan pose and to LOAD_SLIDE.
LM_BACKLASH = 0.002  # [m]
# Unload: the puck sits this far below where the loading pose lets it go.
UNLOAD_DEPTH = 0.03  # [m] straight down from the loading pose
# The plain swing to the loading pose ends by sliding in from behind ~2 cm over the loading height and the
# open jaws catch the stored puck. So: swing to UNLOAD_ABOVE over the grasp with the wrist already
# folded to its grasp angle (shoulder/elbow high), then straight down. Raising it leans the tool more
# (4 cm: 24 deg, 6 cm: 34 deg) and bows the descent more (2.4 / 5.0 mm).
UNLOAD_ABOVE = 0.06  # [m] above the unload grasp, i.e. 3 cm over the loading pose
# Put-down: open this far above the front grasp height. A puck taken from the clock sits deeper in the
# jaws than a front-grasped one, so lowering to the grasp height pressed it into the shelf (the arm's
# I gain kept pushing) and the arm dropped when the jaws let go. The puck now falls <= this far.
PUT_DOWN_ABOVE = 0.01  # [m]


def load_pose():
    """LOAD_MOTOR_DEG as URDF joint angles (control.yaml offsets/directions)."""
    c = yaml.safe_load(open(plan.CONTROL_YAML))['control_master']['ros__parameters']
    return [d * (math.radians(m) - o)
            for m, o, d in zip(LOAD_MOTOR_DEG, c['arm_joint_offsets'], c['arm_joint_directions'])]


def below(arm, dz):
    """arm (shoulder, elbow, wrist_pitch, tool_roll) moved so the TCP goes dz straight down and the
    tool keeps its angle: Newton on the three parallel pitch joints, URDF FK."""
    ch = plan.Chain(plan.URDF)
    names = ARM[1:4]
    def pose(q):
        T = ch.fk(dict(zip(names, q), tool_roll_joint=arm[3]))
        return np.array([T[1, 3], T[2, 3], math.atan2(T[2, 2], T[1, 2])])
    q = np.array(arm[:3], dtype=float)
    goal = pose(q) - np.array([0.0, dz, 0.0])
    for _ in range(50):
        e = goal - pose(q)
        e[2] = math.atan2(math.sin(e[2]), math.cos(e[2]))
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.column_stack([(pose(q + h) - pose(q)) / 1e-6 for h in np.eye(3) * 1e-6])
        q += np.linalg.solve(J, e)
    else:
        raise RuntimeError(f'no IK for {dz * 100:.1f} cm below the loading pose')
    for n, v in zip(names, q):
        lo, hi = ch.limits(n)
        if not lo <= v <= hi:
            raise RuntimeError(f'{n} {math.degrees(v):.1f} deg is outside the URDF limits '
                               f'{math.degrees(lo):.1f}..{math.degrees(hi):.1f} at {dz * 100:.1f} cm below the loading pose')
    return [float(v) for v in q] + [arm[3]]


def above(arm, dz):
    """arm with wrist_pitch and tool_roll kept, shoulder/elbow solved so the TCP sits dz straight above.
    The tool leans by however much shoulder+elbow change (the wrist limit forbids keeping it)."""
    ch = plan.Chain(plan.URDF)
    tcp = lambda q: ch.fk(dict(zip(ARM[1:], q)))[1:3, 3]
    q = np.array(arm, dtype=float)
    goal = tcp(q) + np.array([0.0, dz])
    for _ in range(50):
        e = goal - tcp(q)
        if np.linalg.norm(e) < 1e-7:
            break
        J = np.column_stack([(tcp(q + h) - tcp(q)) / 1e-6 for h in (np.r_[1e-6, 0, 0, 0], np.r_[0, 1e-6, 0, 0])])
        q[:2] += np.linalg.solve(J, e)
    else:
        raise RuntimeError(f'no IK for {dz * 100:.1f} cm above the unload grasp')
    for n, v in zip(ARM[1:3], q):
        lo, hi = ch.limits(n)
        if not lo <= v <= hi:
            raise RuntimeError(f'{n} {math.degrees(v):.1f} deg is outside the URDF limits at {dz * 100:.1f} cm above')
    return [float(v) for v in q]


def steps_from_plan(current_slide=None, start=None):
    f = yaml.safe_load(open(HERE / 'plan.yaml'))['frames']
    # plan.py layout at 20 Hz: 0-19 hold above, 20-59 descent, 60-79 close, rest hold (no lift).
    pose = lambda i: [f[i]['joints'][ARM[0]] + LM_BACKLASH] + [f[i]['joints'][n] for n in ARM[1:]]
    # The arm moves first: a rest pose outside the control limits (clamped, sagging) never settles,
    # and control completes no task, not even a gripper one, until the arm settles.
    s = []
    if current_slide is not None:  # arm first with the LM held, then the LM alone
        s.append(('arm above the puck, LM stays', TaskCommand.MANIPULATOR, [current_slide] + pose(0)[1:]))
    s.append(('open gripper', TaskCommand.GRIPPER, False))
    s.append(('LM to the puck line (move above the puck)', TaskCommand.MANIPULATOR, pose(0)))
    # One move per leg: the joint-space path bows the TCP a few mm off the vertical line,
    # inside the 14 mm open-jaw clearance per side, and avoids stop-and-go.
    s.append(('descend', TaskCommand.MANIPULATOR, pose(59)))
    s += [('close gripper (grasp by current >= 0.1 A)', TaskCommand.GRIPPER, True)]
    s.append(('lift the puck', TaskCommand.MANIPULATOR, pose(20)))
    load_slide = LOAD_SLIDE + LM_BACKLASH
    s.append(('LM to the loading line, arm stays', TaskCommand.MANIPULATOR, [load_slide] + pose(20)[1:]))
    s.append((f'clock to slot {LOAD_SLOT} (loading guide)', TaskCommand.CLOCK, LOAD_SLOT))
    # Front to back in one joint move: the wrist swings ~250 degrees with the puck held.
    s.append(('arm to the loading pose, LM stays', TaskCommand.MANIPULATOR, [load_slide] + load_pose()))
    s.append(('open gripper (drop the puck into the clock)', TaskCommand.GRIPPER, False))
    s.append(('arm back to the lift pose, LM stays', TaskCommand.MANIPULATOR, [load_slide] + pose(20)[1:]))
    if start is not None:  # reverse of the start: LM alone first, then the arm
        s.append(('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + pose(20)[1:]))
        s.append(('arm back to the start pose', TaskCommand.MANIPULATOR, start))
    return s


def unload_steps_from_plan(current_slide=None, start=None):
    """Reverse of steps_from_plan: come down on the puck from above (clock slot LOAD_SLOT), grip it
    UNLOAD_DEPTH below the loading pose, lift it straight back to the loading pose, swing it
    to the front and set it down where the plan grasps it."""
    f = yaml.safe_load(open(HERE / 'plan.yaml'))['frames']
    pose = lambda i: [f[i]['joints'][ARM[0]] + LM_BACKLASH] + [f[i]['joints'][n] for n in ARM[1:]]
    load_slide = LOAD_SLIDE + LM_BACKLASH
    s = []
    if current_slide is not None:  # arm first with the LM held, then the LM alone
        s.append(('arm to the lift pose, LM stays', TaskCommand.MANIPULATOR, [current_slide] + pose(20)[1:]))
    s.append(('LM to the loading line, arm stays', TaskCommand.MANIPULATOR, [load_slide] + pose(20)[1:]))
    s.append((f'clock to slot {LOAD_SLOT} (loading guide)', TaskCommand.CLOCK, LOAD_SLOT))
    s.append(('open gripper', TaskCommand.GRIPPER, False))
    grasp = below(load_pose(), UNLOAD_DEPTH)  # UNLOAD_DEPTH straight under the loading pose
    high = above(grasp, UNLOAD_ABOVE)
    s.append((f'arm over the slot, wrist folded, {UNLOAD_ABOVE * 100:.0f} cm above the grasp, LM stays',
              TaskCommand.MANIPULATOR, [load_slide] + high))
    s.append(('straight down to the grasp, LM stays', TaskCommand.MANIPULATOR, [load_slide] + grasp))
    s.append(('close gripper (grasp the puck in the clock)', TaskCommand.GRIPPER, True))
    # Out with the tool angle kept: the grasp and the loading pose share it (shoulder+elbow+wrist sum),
    # so the joint move between them lifts the puck straight up 3 cm (<= 1.3 mm bow) without tilting.
    # Going to `high` instead tilts the tool from the first centimetre and drags the puck on the slot.
    # The wrist limit allows no higher at this angle; from here the swing is the load run's in reverse.
    s.append((f'straight up {UNLOAD_DEPTH * 100:.0f} cm to the loading pose, tool angle kept, LM stays',
              TaskCommand.MANIPULATOR, [load_slide] + load_pose()))
    s.append(('arm to the lift pose (swing the puck to the front), LM stays', TaskCommand.MANIPULATOR,
              [load_slide] + pose(20)[1:]))
    s.append(('LM to the puck line', TaskCommand.MANIPULATOR, pose(20)))
    # plan.yaml frames 20..59: straight descent from approach_above to the grasp in 40 even steps.
    approach_above = yaml.safe_load(open(HERE / 'plan.yaml'))['assume']['approach_above']
    put = 20 + round(39 * (1 - PUT_DOWN_ABOVE / approach_above))
    s.append((f'put down ({PUT_DOWN_ABOVE * 100:.0f} cm above the grasp height)', TaskCommand.MANIPULATOR, pose(put)))
    s.append(('open gripper (release the puck)', TaskCommand.GRIPPER, False))
    s.append(('rise above the puck', TaskCommand.MANIPULATOR, pose(20)))
    if start is not None:  # reverse of the start: LM alone first, then the arm
        s.append(('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + pose(20)[1:]))
        s.append(('arm back to the start pose', TaskCommand.MANIPULATOR, start))
    return s


class Runner(Node):
    def __init__(self):
        super().__init__('puck_pick_execute')
        self.cmd = self.create_publisher(TaskCommand, '/planning/task_command', 10)
        self.hold = self.create_publisher(Bool, '/planning/hold', 10)
        self.result, self.status, self.grasped, self.joints = None, None, None, None
        self.create_subscription(JointState, '/control/arm_joints', lambda m: setattr(self, 'joints', m), 10)
        self.create_subscription(TaskResult, '/control/task_result', lambda m: setattr(self, 'result', m), 10)
        self.create_subscription(ControlStatus, '/control/status', lambda m: setattr(self, 'status', m), 10)
        self.create_subscription(Bool, '/control/gripper_grasped', lambda m: setattr(self, 'grasped', m.data), 10)
        self.hold_value = False
        self.create_timer(0.1, lambda: self.hold.publish(Bool(data=self.hold_value)))  # 10 Hz heartbeat
        self.task_id = int(time.time())  # monotonic across runs; control rejects retired (smaller) IDs

    def send(self, kind, arg):
        self.task_id += 1
        c = TaskCommand()
        c.header.stamp = self.get_clock().now().to_msg()
        c.task_id, c.revision, c.operation, c.kind = self.task_id, 1, TaskCommand.EXECUTE, kind
        if kind == TaskCommand.MANIPULATOR:
            c.joints.name, c.joints.position = list(ARM), [float(v) for v in arg]
        elif kind == TaskCommand.GRIPPER:
            c.close = bool(arg)
        elif kind == TaskCommand.CLOCK:
            c.clock_slot = int(arg)
        self.result = None
        self.cmd.publish(c)
        return self.task_id

    def wait(self, task_id, timeout=35.0):
        t = time.time()
        while time.time() - t < timeout:
            r = self.result
            if r is not None and r.task_id == task_id and r.status not in (TaskResult.RUNNING, TaskResult.STOPPING):
                return STATUS.get(r.status, str(r.status)), r.reason
            time.sleep(0.05)
        return 'TIMEOUT', 'no terminal result'

    def stop(self):
        c = TaskCommand(); c.operation = TaskCommand.STOP; c.task_id = 0
        self.cmd.publish(c); self.hold_value = True


def describe(kind, arg):
    if kind == TaskCommand.MANIPULATOR:
        return [round(v, 4) for v in arg]
    if kind == TaskCommand.CLOCK:
        return f'slot {arg}'
    return 'close' if arg else 'open'


def run_steps(node, steps):
    """Run steps one Enter at a time ('bro' runs the rest without asking); False if the user quit
    or a step did not succeed."""
    auto = False
    for name, kind, arg in steps:
        if auto:
            print(f'>> {name}')
        else:
            answer = input(f'>> {name}? ').strip().lower()
            if answer == 'q':
                return False
            auto = answer == 'bro'
        tid = node.send(kind, arg)
        status, reason = node.wait(tid)
        extra = f'  grasped={node.grasped}' if kind == TaskCommand.GRIPPER and arg else ''
        print(f'   -> {status} {reason}{extra}')
        if status != 'SUCCEEDED':
            print('step did not succeed')
            if kind == TaskCommand.MANIPULATOR and node.joints is not None:
                # A joint (LM: 0.5 motor deg = 0.06 mm) left more than the 0.5-degree tolerance away (e.g. sagging under load) keeps the
                # board "moving", and control completes nothing until it settles.
                now = dict(zip(node.joints.name, node.joints.position))
                for n, target in zip(ARM, arg):
                    err = now[n] - target
                    unit = f'{err * 1000:+.1f} mm' if n == 'slide_joint' else f'{math.degrees(err):+.1f} deg'
                    flag = '  <-- off' if (abs(err) > 0.00006 if n == 'slide_joint' else abs(err) > math.radians(0.5)) else ''
                    print(f'     {n:18s} target {target:+.4f} actual {now[n]:+.4f}  error {unit}{flag}')
            return False
    return True


def go_home(node, start):
    """After a failed step or q: rise above the puck where the LM is, then LM and arm back to the start."""
    time.sleep(0.5)
    s = node.status
    if s is not None and s.fault:
        print(f'control FAULT ("{s.reason}"): the board latched a fault and accepts nothing until it reboots.\n'
              'Support the arm, then resetarm; the LM turn count is lost, so push the LM to the motor end\n'
              'before the board starts again (same as a power-up).')
        node.stop()
        return
    if input('\nreturn to the start pose step by step? Enter = yes, n = no: ').strip().lower() == 'n':
        node.stop()
        return
    f = yaml.safe_load(open(HERE / 'plan.yaml'))['frames']
    above = [f[20]['joints'][n] for n in ARM[1:]]
    slide = dict(zip(node.joints.name, node.joints.position))['slide_joint']
    slide = min(max(slide, -0.0428), 0.0625)
    steps = [('open gripper (drops a held puck)', TaskCommand.GRIPPER, False),
             ('rise above the puck height, LM stays', TaskCommand.MANIPULATOR, [slide] + above),
             ('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + above),
             ('arm back to the start pose', TaskCommand.MANIPULATOR, start)]
    if not run_steps(node, steps):
        print('return stopped; sending STOP + hold'); node.stop()


def main():
    dry = '--dry-run' in sys.argv
    make_steps = unload_steps_from_plan if '--unload' in sys.argv else steps_from_plan
    steps = make_steps()
    for name, kind, arg in steps:
        print(f'  {name:45s}', describe(kind, arg))
    if dry:
        return 0
    rclpy.init()
    node = Runner()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    try:
        t = time.time()
        while node.status is None and time.time() - t < 3:
            time.sleep(0.1)
        if node.status is None:
            print('no /control/status: is control_master running?'); return 1
        time.sleep(1.0)  # heartbeat must be alive before control accepts tasks
        s = node.status
        print(f'control: ready={s.ready} fault={s.fault} held={s.held} reason="{s.reason}"')
        if not s.ready or s.fault:
            print('control is not ready; aborting'); return 1
        if node.joints is None:
            print('no /control/arm_joints; aborting'); return 1
        now = dict(zip(node.joints.name, node.joints.position))
        slide = now['slide_joint']
        print(f'LM now: slide {slide:+.4f} m (motor end after a correct start is about +0.063)')
        if not -0.045 <= slide <= 0.064:
            print('LM position is outside the calibrated range (wrong start position?). Run checkarm; aborting'); return 1
        slide = min(max(slide, -0.0428), 0.0625)
        limits = plan.Chain(plan.URDF).limits
        outside = [n for n in ARM[1:] if not limits(n)[0] <= now[n] <= limits(n)[1]]
        if outside:  # control clamps these into the limits, so the first move pulls them in
            print(f'note: arm joints outside the limits {outside}; control pulls them inside on the first move.')
        # Return pose: the start, but 3 degrees inside the limits so it can settle there.
        margin = math.radians(3.0)
        start = [slide] + [min(max(now[n], limits(n)[0] + margin), limits(n)[1] - margin) for n in ARM[1:]]
        steps = make_steps(slide, start)
        for name, kind, arg in steps:
            print(f'  {name:45s}', describe(kind, arg))
        print('\nSUPPORT THE ARM. Keep a hand on the 12 V power. Enter = run step, q + Enter = quit.\n')
        if not run_steps(node, steps):
            go_home(node, start)
        print('done. control keeps holding the last pose; Ctrl+C to exit this script.')
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print('\nCtrl+C: sending STOP + hold'); node.stop(); time.sleep(0.5)
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
