#!/usr/bin/env python3
"""Stream an arm trajectory to control_master point by point, with a breakaway kick against stiction (2026-11-18).

One MANIPULATOR task is opened with the first point, every later point is an UPDATE_PATH of it (control_master
rewrites the goals with a short travel time, the board restarts the timed move from where it is). The command is

    q_cmd(t) = q_ref(t) + sigma * s_j(t) * max(kick_j - lag_j(t), 0),     sigma = sign(qdot_ref_j(t))
    lag_j(t) = sigma * (q_ref_j(t) - q_meas_j(t))        (how far the joint trails the reference)

  kick_j   breakaway lead of joint j [rad] (arm_stream.yaml, from --measure-kick), + and - direction apart
  s_j(t)   1 while the reference moves, the joint stands still (|qdot_meas| < still) and trails it by more than
           lag_min, then fades with tau once it moves; 0 in the last end_off seconds
  So a stuck joint gets a lead of exactly kick_j from where it stands, never more; a joint that keeps up or runs
  ahead gets nothing, every cycle (lag <= 0: no kick, also while it fades). sigma is frozen when the kick starts;
  a reference that slows below `still` or turns around ends the kick at once (no sign flip mid-kick).
  Units per joint: slide [mm], the rest [deg] in arm_stream.yaml; every command is clamped inside the URDF limits
  (a point outside them would fail the whole task in control_master).

Usage (robot commands are run by the user):
  python3 arm_stream.py --from via --to stand_above_c5 --T 4            stream a joint-space move (quintic timing)
  python3 arm_stream.py --traj path.csv                                 t, slide, shoulder, elbow, wrist, roll rows
  python3 arm_stream.py --measure-kick shoulder --dir +1                creep one joint, print the breakaway lead
  ... --no-kick                                                         stream without the kick (compare logs)
  ... --dry-run                                                         no ROS: simulate a joint with stiction
Every run writes runs/stream_MMDD_HHMMSS/log.csv (t, ref, cmd, measured, shoulder current).
Run it with control only, not with planning_master: it sends /planning/hold False and its own task IDs like
puck_execute does without --planned, and a planning task arriving mid-stream would replace the arm task.
"""
import csv
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ARM = ['slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']
SHORT = {'slide': 0, 'shoulder': 1, 'elbow': 2, 'wrist': 3, 'wrist_pitch': 3, 'roll': 4, 'tool_roll': 4}
CFG = HERE / 'arm_stream.yaml'


def arg(flag, default=None, cast=str):
    return cast(sys.argv[sys.argv.index(flag) + 1]) if flag in sys.argv else default


UNIT = np.array([1e-3] + [math.pi / 180] * 4)  # yaml joint units -> URDF (slide mm -> m, the rest deg -> rad)
UNIT_NAME = ['mm'] + ['deg'] * 4


def load_cfg():
    """arm_stream.yaml with every per-joint list in URDF units (m for the slide, rad for the rest)."""
    c = yaml.safe_load(open(CFG))
    for k in ('kick_pos', 'kick_neg', 'still', 'lag_min', 'start_tol', 'abort', 'creep', 'breakaway', 'creep_max'):
        c[k] = np.array(c[k], float) * UNIT
    c['kick_pos'][0] = c['kick_neg'][0] = 0.0  # no kick on the LM
    import plan
    ch = plan.Chain(plan.URDF)
    lim = np.array([ch.limits(n) for n in ARM])
    margin = np.array(c['limit_margin'], float) * UNIT
    c['q_lo'], c['q_hi'] = lim[:, 0] + margin, lim[:, 1] - margin
    return c


# ---------------------------------------------------------------- trajectories

def named_pose(name):
    """'nominal' | 'via' | 'carry_via' | a teach.yaml name -> [slide, shoulder, elbow, wrist, roll] URDF."""
    import teach
    from executor_preview import preview
    if name in ('nominal', 'via', 'carry_via'):
        return np.array(preview('--goto', 'nominal')[name][:5], float)
    entries = {e['name']: e for e in yaml.safe_load(open(HERE / 'teach.yaml'))}
    return np.array(teach.raw_to_target(entries[name]['raw_deg'])[:5], float)


def quintic(qa, qb, T, dt):
    """Joint-space move qa -> qb in T s, rest-to-rest (zero speed and acceleration at both ends)."""
    t = np.arange(0.0, T + dt / 2, dt)
    s = t / T
    h = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
    return t, qa + np.outer(h, qb - qa)


def read_traj(path, dt):
    rows = np.array([[float(v) for v in r] for r in csv.reader(open(path)) if r and not r[0].startswith('#')])
    t = np.arange(0.0, rows[-1, 0] + dt / 2, dt)
    return t, np.stack([np.interp(t, rows[:, 0], rows[:, j]) for j in range(1, 6)], axis=1)


# ---------------------------------------------------------------- kick

class Kick:
    def __init__(self, cfg, T, on=True):
        self.c, self.T, self.on = cfg, T, on
        self.s = np.zeros(5)
        self.sigma = np.zeros(5)  # direction frozen when a kick starts (0: no kick running)

    def __call__(self, t, q_ref, qdot_ref, q_meas, qdot_meas, dt):
        """Extra command on top of q_ref [URDF units, 5]."""
        if not self.on:
            return np.zeros(5)
        c = self.c
        sign_ref = np.sign(qdot_ref)
        moving = np.abs(qdot_ref) > c['still']
        lag_now = sign_ref * (q_ref - q_meas)
        start = (self.sigma == 0) & moving & (np.abs(qdot_meas) < c['still']) & (lag_now > c['lag_min'])
        self.sigma = np.where(start, sign_ref, self.sigma)
        # a reference that stops or turns around ends the kick at once
        self.sigma = np.where(~moving | (sign_ref != self.sigma), 0.0, self.sigma)
        stuck = (self.sigma != 0) & (np.abs(qdot_meas) < c['still'])
        target = stuck.astype(float)
        if t > self.T - c['end_off_s']:
            target[:] = 0.0
            self.sigma[:] = 0.0
        self.s = np.where(target > self.s, target, self.s + (target - self.s) * min(1.0, dt / c['tau_s']))
        self.s = np.where(self.sigma == 0, 0.0, self.s)
        self.sigma = np.where(self.s < 1e-3, 0.0, self.sigma)
        lag = self.sigma * (q_ref - q_meas)
        k = np.where(self.sigma > 0, c['kick_pos'], c['kick_neg'])
        extra = np.where(lag > 0, np.maximum(k - lag, 0.0), 0.0)  # never on a joint that keeps up or runs ahead
        return self.sigma * self.s * extra


class SpeedEstimate:
    """Joint speed from /control/arm_joints over a window (one tick, 0.088 deg, per 50 ms is 1.8 deg/s: a
    point-to-point difference only says 'a tick changed or not'). Uses the message stamps, new messages only."""

    def __init__(self, window=0.15):
        self.window, self.hist = window, []

    def add(self, stamp, q):
        if self.hist and stamp <= self.hist[-1][0]:
            return
        self.hist.append((stamp, q.copy()))
        while len(self.hist) > 2 and self.hist[-1][0] - self.hist[1][0] >= self.window:
            self.hist.pop(0)

    def speed(self):
        if len(self.hist) < 2:
            return np.zeros(5)
        (t0, q0), (t1, q1) = self.hist[0], self.hist[-1]
        return (q1 - q0) / max(t1 - t0, 1e-3)


# ---------------------------------------------------------------- offline plant (dry run)

class StictionJoint:
    """One joint behind a position loop: it does not move until |cmd - q| > dead (static friction / stiffness), then
    follows with time constant tau until it stops again (|qdot| tiny)."""

    def __init__(self, q0, dead, tau=0.06):
        self.q, self.v, self.dead, self.tau, self.moving = q0, 0.0, dead, tau, False

    def step(self, cmd, dt):
        err = cmd - self.q
        if not self.moving and abs(err) > self.dead:
            self.moving = True
        if self.moving:
            self.v = err / self.tau
            self.q += self.v * dt
            if abs(err) < 0.25 * self.dead:  # kinetic friction is lower: it sticks again near the target
                self.moving, self.v = False, 0.0
        else:
            self.v = 0.0
        return self.q


# ---------------------------------------------------------------- ROS streaming

class Streamer:
    def __init__(self):
        import rclpy
        from craft_interfaces.msg import TaskCommand, TaskResult
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool
        self.rclpy, self.TaskCommand, self.TaskResult = rclpy, TaskCommand, TaskResult
        from rclpy.signals import SignalHandlerOptions
        # no rclpy SIGINT handler: Ctrl-C raises KeyboardInterrupt with the context alive, so STOP still goes out
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('arm_stream')
        self.pub = self.node.create_publisher(TaskCommand, '/planning/task_command', 10)
        self.hold = self.node.create_publisher(Bool, '/planning/hold', 10)
        self.node.create_timer(0.1, lambda: self.hold.publish(Bool(data=False)))
        self.joints = None
        self.result = None
        self.node.create_subscription(JointState, '/control/arm_joints', lambda m: setattr(self, 'joints', m), 10)
        self.node.create_subscription(TaskResult, '/control/task_result', lambda m: setattr(self, 'result', m), 10)
        self.task_id, self.rev = int(time.time()) * 10, 0

    def spin(self, s=0.0):
        end = time.monotonic() + s
        while True:
            self.rclpy.spin_once(self.node, timeout_sec=0.002)
            if time.monotonic() >= end:
                return

    def measured(self, joint='shoulder_joint'):
        """(q [5] URDF, current of `joint` [A] or nan, stamp [s]) from /control/arm_joints."""
        m = self.joints
        if m is None:
            return None, math.nan, None
        q = np.array([m.position[m.name.index(n)] for n in ARM])
        cur = m.effort[m.name.index(joint)] if len(m.effort) == len(m.name) else math.nan
        return q, cur, m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec

    def send(self, q, duration, first=False, lo=None, hi=None):
        c = self.TaskCommand()
        c.header.stamp = self.node.get_clock().now().to_msg()
        if first:
            self.task_id += 1
            self.rev = 0
        self.rev += 1
        c.task_id, c.revision, c.kind = self.task_id, self.rev, self.TaskCommand.MANIPULATOR
        c.operation = self.TaskCommand.EXECUTE if first else self.TaskCommand.UPDATE_PATH
        c.duration_s = float(duration)
        if lo is not None:
            q = np.clip(q, lo, hi)  # a point outside the limits would fail the task (and hold the arm)
        c.joints.name, c.joints.position = list(ARM), [float(v) for v in q]
        self.pub.publish(c)

    def state(self):
        r = self.result
        if r is None or r.task_id != self.task_id:
            return 'NONE', ''
        names = {self.TaskResult.RUNNING: 'RUNNING', self.TaskResult.SUCCEEDED: 'SUCCEEDED',
                 self.TaskResult.FAILED: 'FAILED', self.TaskResult.REJECTED: 'REJECTED',
                 self.TaskResult.STOPPED: 'STOPPED', self.TaskResult.STOPPING: 'STOPPING'}
        return names.get(r.status, str(r.status)), r.reason

    def stop(self):
        try:
            c = self.TaskCommand(); c.operation = self.TaskCommand.STOP; c.task_id = self.task_id
            for _ in range(3):  # no ACK wanted here: a few copies in case one is dropped
                self.pub.publish(c)
                self.spin(0.05)
            print('STOP sent')
        except Exception as e:  # the context is gone: say so instead of hiding it
            print(f'STOP could NOT be sent ({e}): stop the arm with the e-stop')

    def close(self):
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def control_stream_ms():
    """control.yaml arm_stream_direct_ms of this checkout (what control was started with if it was not edited since)."""
    c = yaml.safe_load(open(HERE.parents[1] / 'src/craft_control/config/control.yaml'))
    return int(c['control_master']['ros__parameters'].get('arm_stream_direct_ms', 0))


def log_dir():
    d = HERE / 'runs' / time.strftime('stream_%m%d_%H%M%S')
    d.mkdir(parents=True, exist_ok=True)
    return d


def run_stream(t, Q, cfg, kick_on=True, dry=False):
    """Stream Q (rows at times t). Returns the log rows."""
    dt = float(t[1] - t[0])
    qdot = np.gradient(Q, dt, axis=0)
    kick = Kick(cfg, t[-1], kick_on)
    lead = cfg['point_travel_s']
    rows = []
    if dry:
        joints = [StictionJoint(Q[0, j], math.radians(cfg['dry_dead_deg'][j])) for j in range(5)]
        meas = Q[0].copy()
        prev = meas.copy()
        for i, ti in enumerate(t):
            vm = (meas - prev) / dt
            cmd = Q[i] + kick(ti, Q[i], qdot[i], meas, vm, dt)
            prev = meas.copy()
            meas = np.array([jt.step(c, dt) for jt, c in zip(joints, cmd)])
            rows.append([ti, *Q[i], *cmd, *meas, math.nan])
        for _ in range(int(1.0 / dt)):  # settle after the last point
            prev = meas.copy()
            meas = np.array([jt.step(c, dt) for jt, c in zip(joints, Q[-1])])
        rows.append([t[-1] + 1.0, *Q[-1], *Q[-1], *meas, math.nan])
        return rows
    direct = control_stream_ms()
    period = 1000.0 / cfg['hz']
    if direct and not period <= direct <= 3 * period:
        print(f'WARNING: control.yaml arm_stream_direct_ms {direct} with {period:.0f} ms points here: below the period '
              'the arm stops between points, far above it lags; 1.5-2x the period is the aim')
    print('points: ' + (f'constant speed, {direct} ms each (firmware STREAM)' if direct else
                        f'timed moves of {lead:.2f} s with ramps (TIME)'))
    s = Streamer()
    lo, hi = cfg['q_lo'], cfg['q_hi']
    try:
        s.spin(0.5)
        q0, _, st0 = s.measured()
        if q0 is None:
            raise RuntimeError('no /control/arm_joints: is control running?')
        off = np.abs(q0 - Q[0])
        if np.any(off > cfg['start_tol']):
            j = int(np.argmax(off / cfg['start_tol']))
            raise RuntimeError(f'{ARM[j]} is {off[j] / UNIT[j]:.1f} {UNIT_NAME[j]} from the start of the trajectory: '
                               'move it there first')
        if np.any(Q < lo) or np.any(Q > hi):
            raise RuntimeError('the trajectory leaves the URDF limits (minus limit_margin)')
        speed = SpeedEstimate(cfg['speed_window_s'])
        speed.add(st0, q0)
        s.send(Q[0], lead, first=True, lo=lo, hi=hi)
        reopened = 0
        t0 = time.monotonic()
        for i, ti in enumerate(t):
            while time.monotonic() - t0 < ti:
                s.spin(0.001)
            q, cur, stamp = s.measured()
            speed.add(stamp, q)
            err = np.abs(q - Q[i])
            if np.any(err > cfg['abort']):  # before anything else goes out
                j = int(np.argmax(err / cfg['abort']))
                raise RuntimeError(f'{ARM[j]} trails by {err[j] / UNIT[j]:.1f} {UNIT_NAME[j]}: stopped')
            st, why = s.state()
            if st in ('FAILED', 'REJECTED', 'STOPPED', 'STOPPING'):
                raise RuntimeError(f'task {st}: {why}')
            cmd = Q[i] + kick(ti, Q[i], qdot[i], q, speed.speed(), dt)
            if st == 'SUCCEEDED':  # arrived between points (a stall over settle_time_s): open a new task
                reopened += 1
            s.send(cmd, lead, first=(st == 'SUCCEEDED'), lo=lo, hi=hi)
            rows.append([ti, *Q[i], *np.clip(cmd, lo, hi), *q, cur])
        s.spin(0.05)
        s.send(Q[-1], max(lead, 0.3), first=s.state()[0] != 'RUNNING', lo=lo, hi=hi)  # the end, also if it ended
        s.spin(0.1)
        end = time.monotonic() + 5.0
        while time.monotonic() < end and s.state()[0] == 'RUNNING':
            s.spin(0.02)
        q, cur, _ = s.measured()
        rows.append([time.monotonic() - t0, *Q[-1], *Q[-1], *q, cur])
        print(f'end: {s.state()[0]}' + (f'  (task re-opened {reopened}x after stalls)' if reopened else ''))
        return rows
    except BaseException:
        s.stop()
        raise
    finally:
        s.close()


def measure_kick(joint, direction, cfg, dry=False):
    """Creep one joint's target at `creep` from where it stands until it has moved `breakaway`: the lead
    (target - measured) just before that is printed. With the position I gain winding up while it creeps, this is
    'the lead needed at this creep rate', not a pure static-friction number: measure at the creep rate the moves use.
    Then back to the start. Not for the slide (LM)."""
    j = SHORT[joint]
    if j == 0:
        raise SystemExit('--measure-kick is for the rotary joints (shoulder, elbow, wrist, roll)')
    rate = cfg['creep'][j] * direction
    dt = 1.0 / cfg['hz']
    moved = cfg['breakaway'][j]
    limit = cfg['creep_max'][j]
    if dry:
        jt = StictionJoint(0.0, math.radians(cfg['dry_dead_deg'][j]))
        k, last_lead = 0, 0.0
        while True:
            k += 1
            cmd = rate * k * dt
            q_before = jt.q
            q = jt.step(cmd, dt)
            if abs(q) > moved or abs(cmd) > limit:
                print(f'(dry) {joint} {direction:+d}: breakaway lead {math.degrees(last_lead):+.2f} deg '
                      f'(simulated band {cfg["dry_dead_deg"][j]} deg)')
                return
            last_lead = cmd - q_before
    s = Streamer()
    lo, hi = cfg['q_lo'], cfg['q_hi']
    try:
        s.spin(0.5)
        q0, _, _ = s.measured()
        if q0 is None:
            raise RuntimeError('no /control/arm_joints: is control running?')
        if not lo[j] <= q0[j] + direction * limit <= hi[j]:
            raise RuntimeError(f'creeping {math.degrees(limit):.1f} deg from here would leave the limits')
        s.send(q0, cfg['point_travel_s'], first=True, lo=lo, hi=hi)
        t0 = time.monotonic()
        lead, last_lead, cur = None, 0.0, math.nan
        while True:
            s.spin(dt)
            el = time.monotonic() - t0
            cmd = q0.copy()
            cmd[j] += rate * el
            q, cur, _ = s.measured(ARM[j])
            st, why = s.state()
            if st in ('FAILED', 'REJECTED', 'STOPPED', 'STOPPING'):
                raise RuntimeError(f'task {st}: {why}')
            if abs(q[j] - q0[j]) > moved:
                lead = last_lead  # the lead it took, just before it moved
                break
            if abs(cmd[j] - q0[j]) > limit:
                break
            last_lead = cmd[j] - q[j]
            s.send(cmd, cfg['point_travel_s'], first=(st == 'SUCCEEDED'), lo=lo, hi=hi)
        print(f'{joint} {direction:+d}: ' + (f'breakaway lead {math.degrees(lead):+.2f} deg at '
                                            f'{math.degrees(abs(rate)):.2f} deg/s creep ({ARM[j]} current {cur:+.3f} A)'
                                            if lead is not None else
                                            f'no breakaway within {math.degrees(limit):.1f} deg'))
        s.spin(0.05)
        s.send(q0, 1.0, first=s.state()[0] != 'RUNNING', lo=lo, hi=hi)  # back where it started
        s.spin(0.1)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and s.state()[0] == 'RUNNING':
            s.spin(0.02)
    except BaseException:
        s.stop()
        raise
    finally:
        s.close()


def save(rows, kick_on, dry):
    d = log_dir()
    with open(d / 'log.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t'] + [f'ref_{n}' for n in ARM] + [f'cmd_{n}' for n in ARM] + [f'meas_{n}' for n in ARM]
                   + ['shoulder_current_a'])
        w.writerows(rows)
    R = np.array(rows)
    err = np.degrees(np.abs(R[:-1, 11:16] - R[:-1, 1:6]))
    final = np.degrees(np.abs(R[-1, 11:16] - R[-1, 1:6]))
    print(f"{'dry ' if dry else ''}{'kick' if kick_on else 'no kick'}: tracking error max/mean [deg] "
          + ' '.join(f'{n.split("_")[0]} {e.max():.2f}/{e.mean():.2f}' for n, e in zip(ARM, err.T))
          + '   final ' + ' '.join(f'{v:.2f}' for v in final))
    print(f'log: {d / "log.csv"}')


def main():
    cfg = load_cfg()
    dry = '--dry-run' in sys.argv
    if '--measure-kick' in sys.argv:
        return measure_kick(arg('--measure-kick'), arg('--dir', 1, int), cfg, dry)
    dt = 1.0 / cfg['hz']
    if '--traj' in sys.argv:
        t, Q = read_traj(arg('--traj'), dt)
    elif '--from' in sys.argv:
        t, Q = quintic(named_pose(arg('--from')), named_pose(arg('--to')), arg('--T', 4.0, float), dt)
    else:
        print(__doc__)
        return 1
    kick_on = '--no-kick' not in sys.argv
    if dry and '--compare' in sys.argv:
        if not np.any(cfg['kick_pos']) and not np.any(cfg['kick_neg']):  # nothing measured yet: kick = the sim band
            cfg['kick_pos'] = cfg['kick_neg'] = np.radians(np.array(cfg['dry_dead_deg'], float))
            print('(kick not measured yet: the dry run kicks by dry_dead_deg)')
        for on in (False, True):
            save(run_stream(t, Q, cfg, on, True), on, True)
        return 0
    save(run_stream(t, Q, cfg, kick_on, dry), kick_on, dry)
    return 0


if __name__ == '__main__':
    sys.exit(main())
