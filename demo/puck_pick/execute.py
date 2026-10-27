#!/usr/bin/env python3
"""Throwaway demo: pick the planned puck on the REAL robot and load it into the clock, via control_master.
--unload does the reverse: take the puck out of clock slot LOAD_SLOT and put it down at the planned spot.
--lying picks the lying puck of plan_lying.yaml (plan.py --lying) on its long axis and loads it the same way.

Sends craft_interfaces/TaskCommand on /planning/task_command one step at a time and waits for
/control/task_result. Keeps the /planning/hold heartbeat alive (control rejects tasks without it).
Press Enter before every step; Ctrl+C sends STOP and hold.
Usage:  source ~/craft/install/setup.bash && python3 ~/craft/demo/puck_pick/execute.py [--unload | --lying] [--dry-run]
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
# Elbow first, shoulder last (2026-10-27): every arm move runs elbow, wrist, roll and LM first with the shoulder
# held, then the shoulder alone, so the shoulder never moves with the rest of the arm still swinging (it carries
# the whole arm and ran out of torque). Not for taking the puck out of the clock (NO_STAGING).
# Vertical moves go in STAGE_SEG pieces so the staged path stays near the line.
NO_STAGING = ('arm over the slot', 'straight down to the grasp', 'straight up ')
STAGE_SEG = 4  # plan frames (1 mm each) per vertical piece
# --lying: the arm reaches the plan's tool angle through a pose LYING_HIGH over the grasp, then comes down the
# vertical line in 1 cm pieces. Going straight from the standing lift pose to the approach dipped the fingertips
# into the floor in simulation (2026-10-27, nearly happened on the robot).
LYING_HIGH = 0.10  # [m]
# The swings between the standing lift pose and LYING_HIGH move the TCP on a straight line in LYING_SWING staged
# pieces. Staged as one joint move, going out swept the fingertips to the floor (-1 mm) and coming back pressed
# the held puck 42 mm into the shelf; 20 pieces clear the shelf by >= 49 mm (simulated, 2026-10-27).
LYING_SWING = 20


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


def plan_pose(name='plan.yaml'):
    """pose(i): frame i of a plan.py output as [slide + LM_BACKLASH, shoulder, elbow, wrist_pitch, tool_roll]."""
    f = yaml.safe_load(open(HERE / name))['frames']
    return lambda i: [f[i]['joints'][ARM[0]] + LM_BACKLASH] + [f[i]['joints'][n] for n in ARM[1:]]


def frames_path(pose, i0, i1):
    """pose(i) for plan frames i0 -> i1 (either direction) every STAGE_SEG frames, both ends included."""
    step = STAGE_SEG if i1 > i0 else -STAGE_SEG
    return [pose(i) for i in list(range(i0, i1, step))[1:] + [i1]]


def lying_line(heights):
    """Arms (shoulder, elbow, wrist_pitch, tool_roll) with the plan_lying tool angle at each height over its grasp."""
    p = yaml.safe_load(open(HERE / 'plan_lying.yaml'))
    ch, roll = plan.Chain(plan.URDF), p['frames'][0]['joints']['tool_roll_joint']
    mg = math.radians(p['assume']['limit_margin_deg'])
    out = []
    for h in heights:
        for q in plan.ik_top_down(ch, np.array(p['puck_grasp_urdf']) + [0, 0, h], p['assume']['tool_tilt_deg']):
            if all(ch.limits(n)[0] + mg <= q[n] <= ch.limits(n)[1] - mg for n in ARM[1:4]):
                out.append([q[n] for n in ARM[1:4]] + [roll]); break
        else:
            raise RuntimeError(f'no pose {h * 100:.0f} cm over the lying grasp')
    return out


def tcp_path(arm0, arm1, n):
    """n arms (shoulder, elbow, wrist_pitch, tool_roll) moving the TCP on a straight line arm0 -> arm1 with the
    tool angle and roll turning evenly (IK per point, the branch nearest the previous point)."""
    ch = plan.Chain(plan.URDF)
    def pt(arm):
        T = ch.fk(dict(zip(ARM[1:], arm)))
        return T[:3, 3], (-90 - math.degrees(math.atan2(T[2, 1], T[1, 1]))) % 360
    (p0, t0), (p1, t1) = pt(arm0), pt(arm1)
    t1 = t0 + (t1 - t0 + 180) % 360 - 180
    out, prev = [], arm0
    for k in range(1, n + 1):
        f = k / n
        roll = arm0[3] + (arm1[3] - arm0[3]) * f
        sols = [[q[n_] for n_ in ARM[1:4]] + [roll] for q in plan.ik_top_down(ch, p0 + (p1 - p0) * f, t0 + (t1 - t0) * f)]
        sols = [q for q in sols if all(ch.limits(n_)[0] <= v <= ch.limits(n_)[1] for n_, v in zip(ARM[1:4], q))]
        if not sols:
            raise RuntimeError(f'no IK on the TCP line at {f:.0%}')
        prev = min(sols, key=lambda q: sum(abs(x - y) for x, y in zip(q, prev)))
        out.append(prev)
    out[-1] = list(arm1)
    return out


def stage(steps, prev):
    """Elbow first, shoulder last: each MANIPULATOR target (a pose or a path of poses) becomes the pose with the
    shoulder still at its previous angle, then the pose. prev: the arm pose before steps (None: first not staged)."""
    out = []
    for name, kind, arg in steps:
        if kind != TaskCommand.MANIPULATOR:
            out.append((name, kind, arg)); continue
        path = arg if isinstance(arg[0], list) else [arg]
        poses = []
        for q in path:
            mid = [q[0], prev[1]] + q[2:] if prev is not None and not name.startswith(NO_STAGING) else q
            if mid != q and mid != prev:
                poses.append(mid)
            poses.append(q); prev = q
        out.append((name, kind, poses))
    return out


def load_tail(lift, start):
    """Lift pose (puck held, LM on the puck line) -> clock slot LOAD_SLOT -> back to start."""
    load_slide = LOAD_SLIDE + LM_BACKLASH
    s = [('LM to the loading line, arm stays', TaskCommand.MANIPULATOR, [load_slide] + lift)]
    s.append((f'clock to slot {LOAD_SLOT} (loading guide)', TaskCommand.CLOCK, LOAD_SLOT))
    # Front to back in one joint move: the wrist swings ~250 degrees with the puck held.
    s.append(('arm to the loading pose, LM stays', TaskCommand.MANIPULATOR, [load_slide] + load_pose()))
    s.append(('open gripper (drop the puck into the clock)', TaskCommand.GRIPPER, False))
    s.append(('arm back to the lift pose, LM stays', TaskCommand.MANIPULATOR, [load_slide] + lift))
    if start is not None:  # reverse of the start: LM alone first, then the arm
        s.append(('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + lift))
        s.append(('arm back to the start pose', TaskCommand.MANIPULATOR, start))
    return s


def steps_from_plan(current_slide=None, start=None):
    # plan.py layout at 20 Hz: 0-19 hold above, 20-59 descent, 60-79 close, rest hold (no lift).
    pose = plan_pose()
    # The arm moves first: a rest pose outside the control limits (clamped, sagging) never settles,
    # and control completes no task, not even a gripper one, until the arm settles.
    s = []
    if current_slide is not None:  # arm first with the LM held, then the LM alone
        s.append(('arm above the puck, LM stays', TaskCommand.MANIPULATOR, [current_slide] + pose(0)[1:]))
    s.append(('open gripper', TaskCommand.GRIPPER, False))
    s.append(('LM to the puck line (move above the puck)', TaskCommand.MANIPULATOR, pose(0)))
    s.append(('descend', TaskCommand.MANIPULATOR, frames_path(pose, 20, 59)))
    s += [('close gripper (grasp by current >= 0.1 A)', TaskCommand.GRIPPER, True)]
    s.append(('lift the puck', TaskCommand.MANIPULATOR, frames_path(pose, 59, 20)))
    return s + load_tail(pose(20)[1:], start)


def lying_steps_from_plan(current_slide=None, start=None):
    """--lying: grip the lying puck's near end at the plan_lying tool angle, come down and lift it on the vertical
    line, swing it via the LYING_HIGH pose to the standing plan's lift pose (tool down, roll back to 0; the puck
    sits in the jaws as a standing-grasped one) and load it like steps_from_plan."""
    pose, lift = plan_pose('plan_lying.yaml'), plan_pose()(20)[1:]
    a = yaml.safe_load(open(HERE / 'plan_lying.yaml'))['assume']['approach_above']
    down = lying_line(np.arange(LYING_HIGH, a, -0.01))   # LYING_HIGH .. 1 cm above the approach
    high, slide = down[0], pose(0)[0]
    s = []
    if current_slide is not None:
        s.append(('arm to the standing lift pose, LM stays', TaskCommand.MANIPULATOR, [current_slide] + lift))
        s.append((f'tool out, {LYING_HIGH * 100:.0f} cm over the grasp height, LM stays', TaskCommand.MANIPULATOR,
                  [[current_slide] + q for q in tcp_path(lift, high, LYING_SWING)]))
    s.append(('open gripper', TaskCommand.GRIPPER, False))
    s.append(('LM to the puck line, arm stays', TaskCommand.MANIPULATOR, [slide] + high))
    s.append(('down to the approach (4 cm over the grasp)', TaskCommand.MANIPULATOR,
              [[slide] + q for q in down[1:]] + [pose(0)]))
    s.append(('descend', TaskCommand.MANIPULATOR, frames_path(pose, 20, 59)))
    s.append(('close gripper (grasp by current >= 0.1 A)', TaskCommand.GRIPPER, True))
    s.append(('lift the puck 4 cm', TaskCommand.MANIPULATOR, frames_path(pose, 59, 20)))
    s.append((f'up to {LYING_HIGH * 100:.0f} cm, LM stays', TaskCommand.MANIPULATOR, [[slide] + q for q in down[::-1]]))
    s.append(('arm to the standing lift pose (tool down, roll back), LM stays', TaskCommand.MANIPULATOR,
              [[slide] + q for q in tcp_path(high, lift, LYING_SWING)]))
    return s + load_tail(lift, start)


def unload_steps_from_plan(current_slide=None, start=None):
    """Reverse of steps_from_plan: come down on the puck from above (clock slot LOAD_SLOT), grip it
    UNLOAD_DEPTH below the loading pose, lift it straight back to the loading pose, swing it
    to the front and set it down where the plan grasps it."""
    pose = plan_pose()
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
    s.append((f'put down ({PUT_DOWN_ABOVE * 100:.0f} cm above the grasp height)', TaskCommand.MANIPULATOR,
              frames_path(pose, 20, put)))
    s.append(('open gripper (release the puck)', TaskCommand.GRIPPER, False))
    s.append(('rise above the puck', TaskCommand.MANIPULATOR, frames_path(pose, put, 20)))
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
    if kind == TaskCommand.MANIPULATOR:  # a path of poses after stage()
        return [round(v, 4) for v in arg[-1]] + ([f'{len(arg)} moves'] if len(arg) > 1 else [])
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
        for target in (arg if kind == TaskCommand.MANIPULATOR else [arg]):
            status, reason = node.wait(node.send(kind, target))
            if status != 'SUCCEEDED':
                break
        extra = f'  grasped={node.grasped}' if kind == TaskCommand.GRIPPER and arg else ''
        print(f'   -> {status} {reason}{extra}')
        if status != 'SUCCEEDED':
            print('step did not succeed')
            if kind == TaskCommand.MANIPULATOR and node.joints is not None:
                # A joint (LM: 0.5 motor deg = 0.06 mm) left more than the 0.5-degree tolerance away (e.g. sagging under load) keeps the
                # board "moving", and control completes nothing until it settles.
                now = dict(zip(node.joints.name, node.joints.position))
                for n, t in zip(ARM, target):
                    err = now[n] - t
                    unit = f'{err * 1000:+.1f} mm' if n == 'slide_joint' else f'{math.degrees(err):+.1f} deg'
                    flag = '  <-- off' if (abs(err) > 0.00006 if n == 'slide_joint' else abs(err) > math.radians(0.5)) else ''
                    print(f'     {n:18s} target {t:+.4f} actual {now[n]:+.4f}  error {unit}{flag}')
            return False
    return True


def go_home(node, start, lying):
    """After a failed step or q: rise above the puck where the LM is (lying: up its vertical line to LYING_HIGH
    first when the tool is out there), then LM and arm back to the start, elbow first."""
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
    now = dict(zip(node.joints.name, node.joints.position))
    slide = min(max(now['slide_joint'], -0.0428), 0.0625)
    rise = [plan_pose()(20)[1:]]
    if lying:
        g = yaml.safe_load(open(HERE / 'plan_lying.yaml'))['puck_grasp_urdf']
        tcp = plan.Chain(plan.URDF).fk(now)[:3, 3]
        if tcp[1] < (g[1] + plan.ASSUME['lidar_front_y']) / 2:  # tool out over the lying puck
            h0 = max(tcp[2] - g[2], 0) + STAGE_SEG / 1000
            up = lying_line(np.arange(h0, LYING_HIGH + 1e-6, STAGE_SEG / 1000))  # 1 mm per plan frame
            rise = up + tcp_path(up[-1], rise[0], LYING_SWING)
    steps = [('open gripper (drops a held puck)', TaskCommand.GRIPPER, False),
             ('rise above the puck height, LM stays', TaskCommand.MANIPULATOR, [[slide] + r for r in rise]),
             ('LM back to the start position', TaskCommand.MANIPULATOR, [start[0]] + rise[-1]),
             ('arm back to the start pose', TaskCommand.MANIPULATOR, start)]
    steps = stage(steps, [now[n] for n in ARM])
    if not run_steps(node, steps):
        print('return stopped; sending STOP + hold'); node.stop()


def main():
    dry = '--dry-run' in sys.argv
    lying = '--lying' in sys.argv
    make_steps = unload_steps_from_plan if '--unload' in sys.argv else lying_steps_from_plan if lying else steps_from_plan
    steps = stage(make_steps(), None)
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
        steps = stage(make_steps(slide, start), [slide] + [now[n] for n in ARM[1:]])
        for name, kind, arg in steps:
            print(f'  {name:45s}', describe(kind, arg))
        print('\nSUPPORT THE ARM. Keep a hand on the 12 V power. Enter = run step, q + Enter = quit.\n')
        if not run_steps(node, steps):
            go_home(node, start, lying)
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
