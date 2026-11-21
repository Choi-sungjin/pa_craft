#!/usr/bin/env python3
"""MANIPULATOR UPDATE_PATH (streamed arm points, 2026-11-18) against a real control_master and fake PTY boards.

CRAFT_CONTROL_BINARY=<built control_master> python3 test_arm_stream_integration.py
"""
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '174')
sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from craft_interfaces.msg import TaskCommand, TaskResult  # noqa: E402

import test_planning_watchdog_integration as base  # noqa: E402

NAMES = ['slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']


class RecordingBoard(base.FakeBoard):
    """FakeBoard that also keeps the TIME lines (the travel time of each written move)."""

    def __init__(self, feedback):
        self.times, self.streams = [], []
        super().__init__(feedback)

    def _run(self):
        import select
        pending = b''
        next_feedback = 0.0
        while self.running:
            if self.streaming and time.monotonic() >= next_feedback:
                try:
                    os.write(self.master, (','.join(str(v) for v in self.feedback) + '\n').encode())
                except BlockingIOError:
                    pass
                next_feedback = time.monotonic() + 0.02
            if select.select([self.master], [], [], 0.005)[0]:
                pending += os.read(self.master, 4096)
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    if line.startswith(b'TIME,'):
                        self.times.append(int(line.split(b',')[1]))
                        continue
                    if line.startswith(b'STREAM,'):
                        self.streams.append(int(line.split(b',')[1]))
                        continue
                    if line.startswith((b'IGAIN,', b'TOL,')):
                        continue
                    self.command = [float(v) for v in line.split(b',')]
                    self.last_command_at = time.monotonic()


def arm_cmd(task_id, revision, q, op=TaskCommand.EXECUTE, duration=0.1):
    c = TaskCommand(task_id=task_id, revision=revision, operation=op, kind=TaskCommand.MANIPULATOR, duration_s=duration)
    c.joints = JointState(name=NAMES, position=q)
    return c


def direct_mode(binary):
    """arm_stream_direct_ms 50: stream points send STREAM,50 once (no TIME), the next ordinary task STREAM,0 + TIME."""
    popen = base.subprocess.Popen

    def with_param(args, **kw):
        return popen(list(args) + ['-p', 'arm_stream_direct_ms:=50'], **kw)
    base.subprocess.Popen = with_param
    h = base.Harness(binary)
    base.subprocess.Popen = popen
    try:
        h.pump(lambda: h.status is not None and h.status.ready and h.status.stopped, hold=True)
        h.pump(lambda: not h.status.held, hold=False)
        arm = h.arm
        arm.feedback[6] = 1
        n_t, n_s = len(arm.times), len(arm.streams)
        opening = arm_cmd(301, 1, [.05, .2, .3, .4, .5], duration=0.1)
        opening.header.frame_id = 'arm_stream'
        h.task_pub.publish(opening)
        h.pump(lambda: 301 in h.results and len(arm.streams) > n_s and arm.streams[-1] == 50, hold=False)
        for rev, sh in enumerate([.21, .22, .23], start=2):
            h.task_pub.publish(arm_cmd(301, rev, [.05, sh, .3, .4, .5], TaskCommand.UPDATE_PATH, 0.1))
            h.run_for(.06, hold=False)
        h.pump(lambda: abs(arm.command[1] - .23) < 1e-4, hold=False)
        assert arm.streams[n_s:] and set(arm.streams[n_s:]) == {50}, arm.streams  # with every point
        assert len(arm.times) == n_t, arm.times              # no TIME for stream points
        print('PASS: direct stream points send STREAM,50 and no TIME', flush=True)
        arm.feedback[0:5] = [.05, .23, .3, .4, .5]
        arm.feedback[6] = 0
        h.pump(lambda: h.results[301].status == TaskResult.SUCCEEDED, hold=False, timeout=5)
        arm.feedback[6] = 1
        h.task_pub.publish(arm_cmd(302, 1, [.05, .3, .3, .4, .5], duration=0.7))
        h.pump(lambda: arm.times and arm.times[-1] == 700, hold=False)
        assert arm.streams[-1] == 0, arm.streams
        print('PASS: the next ordinary task turns the stream off (STREAM,0) and is timed again', flush=True)
    except Exception:
        print(h.close())
        raise
    else:
        h.close()


def main():
    binary = Path(os.environ['CRAFT_CONTROL_BINARY']).resolve()
    rclpy.init()
    base.FakeBoard_orig = base.FakeBoard
    h = None
    try:
        base.FakeBoard = RecordingBoard  # Harness builds its boards from base.FakeBoard
        popen = base.subprocess.Popen
        try:
            base.subprocess.Popen = lambda args, **kw: popen(
                list(args) + ['-p', 'arm_stream_direct_ms:=0'], **kw)
            h = base.Harness(binary)
        finally:
            base.subprocess.Popen = popen
        h.pump(lambda: h.status is not None and h.status.ready and h.status.stopped, hold=True)
        h.pump(lambda: not h.status.held, hold=False)
        arm = h.arm
        arm.feedback[6] = 1  # board busy: the task stays running while points stream

        q1 = [.05, .2, .3, .4, .5]
        h.task_pub.publish(arm_cmd(201, 1, q1, duration=0.1))
        h.pump(lambda: 201 in h.results and arm.command and abs(arm.command[1] - .2) < 1e-4, hold=False)
        assert h.results[201].status == TaskResult.RUNNING
        assert arm.times and arm.times[-1] == 100, arm.times

        # a stream point: new targets and travel time for the same task
        q2 = [.05, .25, .35, .45, .5]
        n_times = len(arm.times)
        h.task_pub.publish(arm_cmd(201, 2, q2, TaskCommand.UPDATE_PATH, duration=0.08))
        h.pump(lambda: abs(arm.command[1] - .25) < 1e-4 and len(arm.times) > n_times, hold=False)
        assert arm.times[-1] == 80, arm.times
        assert h.results[201].status == TaskResult.RUNNING
        print('PASS: stream point rewrites targets and TIME', flush=True)

        # an old revision is ignored
        h.task_pub.publish(arm_cmd(201, 1, [.05, .1, .1, .1, .5], TaskCommand.UPDATE_PATH))
        h.run_for(.2, hold=False)
        assert abs(arm.command[1] - .25) < 1e-4, arm.command
        print('PASS: old revision ignored', flush=True)

        # an update for another task id is rejected and does not touch the arm
        h.task_pub.publish(arm_cmd(299, 1, [.05, .1, .1, .1, .5], TaskCommand.UPDATE_PATH))
        h.pump(lambda: 299 in h.results, hold=False)
        assert h.results[299].status == TaskResult.REJECTED and abs(arm.command[1] - .25) < 1e-4
        print('PASS: foreign update rejected', flush=True)

        # an invalid point (NaN) fails the task
        h.task_pub.publish(arm_cmd(201, 3, [.05, float('nan'), .35, .45, .5], TaskCommand.UPDATE_PATH))
        h.pump(lambda: h.results[201].status == TaskResult.FAILED, hold=False)
        print('PASS: invalid stream point fails the task', flush=True)

        # a fresh task streams and completes once the board arrives
        arm.feedback[6] = 0  # the failed task held the arm: the board idles again
        h.pump(lambda: h.status.stopped, hold=True)
        arm.feedback[6] = 1
        h.pump(lambda: not h.status.held, hold=False)
        h.task_pub.publish(arm_cmd(202, 1, q1))
        h.pump(lambda: 202 in h.results and h.results[202].status == TaskResult.RUNNING, hold=False)
        for rev, s in enumerate([.21, .22, .23, .24], start=2):
            h.task_pub.publish(arm_cmd(202, rev, [.05, s, .3, .4, .5], TaskCommand.UPDATE_PATH, 0.06))
            h.run_for(.05, hold=False)
        h.pump(lambda: abs(arm.command[1] - .24) < 1e-4, hold=False)
        arm.feedback[0:5] = [.05, .24, .3, .4, .5]
        arm.feedback[6] = 0
        h.pump(lambda: h.results[202].status == TaskResult.SUCCEEDED, hold=False, timeout=5)
        assert h.results[202].revision == 5, h.results[202].revision
        print('PASS: streamed task succeeds at its last point (revision 5)', flush=True)
        assert arm.streams and set(arm.streams) == {0}, arm.streams  # timed mode: STREAM,0 with every move
        print('PASS: timed stream points send STREAM,0 + TIME only', flush=True)
        print('PASS: arm stream integration')
    except Exception:
        if h:
            print(h.close())
        raise
    else:
        h.close()
        direct_mode(binary)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
