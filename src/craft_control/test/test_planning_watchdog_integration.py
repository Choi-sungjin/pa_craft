#!/usr/bin/env python3
"""Exercise a real control_master against two disposable PTYs, never physical ports.

Source the ROS environment, set CRAFT_CONTROL_BINARY to the built control_master,
and run this script. All tests run with use_sim_time=true and no /clock publisher.
"""

import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import tempfile
import threading
import time
import tty

os.environ.setdefault('ROS_DOMAIN_ID', '173')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('ROS_LOG_DIR', str(Path(tempfile.gettempdir()) / 'craft-control-watchdog-log'))

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32


class FakeBoard:
    def __init__(self, feedback):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.port = os.ttyname(self.slave)
        self.feedback = list(feedback)
        self.command = None
        self.last_command_at = None
        self.running = True
        self.streaming = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        pending = b''
        next_feedback = 0.0
        while self.running:
            if self.streaming and time.monotonic() >= next_feedback:
                line = ','.join(str(value) for value in self.feedback) + '\n'
                try:
                    os.write(self.master, line.encode())
                except BlockingIOError:
                    pass
                next_feedback = time.monotonic() + 0.02
            if select.select([self.master], [], [], 0.005)[0]:
                pending += os.read(self.master, 4096)
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    self.command = [float(value) for value in line.split(b',')]
                    self.last_command_at = time.monotonic()

    def close(self):
        self.running = False
        self.thread.join(timeout=1.0)
        os.close(self.master)
        os.close(self.slave)


class Harness:
    def __init__(self, binary, calibrated=True):
        self.arm = FakeBoard([0.05, 0.1, 0.2, 0.3, 0.4, 0.6, 0, 0])
        self.drive = FakeBoard([0, 0, 0.7, 0, 0])
        self.log = tempfile.TemporaryFile(mode='w+t')
        config = Path(__file__).resolve().parents[1] / 'config' / 'control.yaml'
        arguments = [str(binary), '--ros-args', '--params-file', str(config)]
        overrides = {
            'arm_lm_port': self.arm.port,
            'clock_wheel_port': self.drive.port,
            'use_sim_time': 'true',
            'arm_ik_mapping_confirmed': str(calibrated).lower(),
            'lm_mapping_confirmed': str(calibrated).lower(),
            'lm_pulley_radius': '1.0',
            'lm_gear_ratio': '1.0',
            'gripper_mapping_confirmed': 'true',
            'gripper_open_angle': '0.2',
            'gripper_close_angle': '1.2',
        }
        for name, value in overrides.items():
            arguments.extend(['-p', f'{name}:={value}'])
        self.process = subprocess.Popen(arguments, stdout=self.log, stderr=subprocess.STDOUT)
        self.node = rclpy.create_node('planning_watchdog_integration', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.held = None
        self.board_states = {}
        self.joints = None
        self.joint_samples = 0
        self.node.create_subscription(Bool, '/control/planning_held', self._on_held, 10)
        for board in ['arm_lm', 'wheel', 'clock']:
            self.node.create_subscription(
                Int32, f'/control/{board}_state',
                lambda message, name=board: self.board_states.update({name: message.data}), 10)
        self.node.create_subscription(JointState, '/control/arm_joints', self._on_joints, 10)
        self.hold_pub = self.node.create_publisher(Bool, '/planning/hold', 10)
        self.path_pub = self.node.create_publisher(RosPath, '/planning/target_path', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/planning/odom', 10)
        self.speed_pub = self.node.create_publisher(Float64, '/planning/target_v', 10)
        self.arm_pub = self.node.create_publisher(JointState, '/planning/target_arm_joints', 10)
        self.gripper_pub = self.node.create_publisher(Bool, '/planning/gripper_command', 10)
        self.clock_pub = self.node.create_publisher(Int32, '/planning/target_clock_slot', 10)

    def _on_held(self, message):
        self.held = message.data

    def _on_joints(self, message):
        self.joints = message
        self.joint_samples += 1

    def pump(self, until, hold=None, commands=False, timeout=3.0):
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError('control_master exited unexpectedly')
            if time.monotonic() >= next_publish:
                if hold is not None:
                    self.hold_pub.publish(Bool(data=hold))
                if commands:
                    self.send_targets()
                next_publish = time.monotonic() + 0.05
            rclpy.spin_once(self.node, timeout_sec=0.005)
            if until():
                return
        raise AssertionError('timed out waiting for control/serial result')

    def run_for(self, seconds, hold=None, commands=False):
        finish = time.monotonic() + seconds
        self.pump(lambda: time.monotonic() >= finish, hold, commands, seconds + 0.5)

    def send_targets(self):
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        path = RosPath()
        path.header.frame_id = 'map'
        for x in [0.0, 2.0]:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            path.poses.append(pose)
        self.path_pub.publish(path)
        self.speed_pub.publish(Float64(data=0.1))
        self.arm_pub.publish(JointState(
            name=['slide_joint', 'shoulder_joint', 'elbow_joint',
                  'wrist_pitch_joint', 'tool_roll_joint'],
            position=[0.1, 0.2, 0.3, 0.4, 0.5]))
        self.gripper_pub.publish(Bool(data=True))
        self.clock_pub.publish(Int32(data=7))

    def board_holds(self, board):
        if board.command is None:
            return False
        expected = board.feedback[:6] if board is self.arm else [0.0, 0.0, board.feedback[2]]
        return len(board.command) == len(expected) and all(
            math.isclose(a, b, abs_tol=1e-4) for a, b in
            zip(board.command, expected))

    def all_axes_hold(self):
        return self.board_holds(self.arm) and self.board_holds(self.drive)

    def executing(self):
        return (self.arm.command is not None and self.drive.command is not None
                and self.drive.command[0] > 1.0 and self.arm.command[-1] > 1.0
                and abs(self.drive.command[2] - 0.7) > 0.1)

    def close(self):
        self.node.destroy_node()
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=1.0)
        self.arm.close()
        self.drive.close()
        self.log.seek(0)
        output = self.log.read()
        self.log.close()
        return output


def main():
    binary = Path(os.environ['CRAFT_CONTROL_BINARY']).resolve()
    if not binary.is_file():
        raise ValueError(f'control binary is missing: {binary}')
    rclpy.init()
    harness = Harness(binary)
    try:
        harness.pump(lambda: harness.held is True and harness.all_axes_hold())
        assert harness.node.get_clock().now().nanoseconds == 0, '/clock must stay paused'
        harness.pump(lambda: harness.joints is not None)
        assert harness.joints.name == [
            'slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']
        assert harness.joints.header.frame_id == 'base_link'
        assert all(math.isclose(a, b, abs_tol=1e-6) for a, b in
                   zip(harness.joints.position, [0.05, 0.1, 0.2, 0.3, 0.4]))
        harness.pump(lambda: harness.held is False, hold=False)
        harness.pump(harness.executing, hold=False, commands=True)
        harness.pump(lambda: harness.held is True and harness.all_axes_hold(), hold=True)
        harness.run_for(0.2, hold=True, commands=True)
        harness.pump(lambda: harness.held is False, hold=False)
        harness.run_for(0.2, hold=False)
        assert harness.all_axes_hold(), 'release revived a target received before/during hold'

        harness.pump(harness.executing, hold=False, commands=True)
        harness.pump(lambda: harness.held is True and harness.all_axes_hold(), timeout=1.2)
        harness.pump(lambda: harness.held is False, hold=False)
        harness.run_for(0.2, hold=False)
        assert harness.all_axes_hold(), 'heartbeat recovery revived an old target'

        harness.pump(harness.executing, hold=False, commands=True)
        harness.drive.feedback[3] = 2
        harness.pump(lambda: harness.held is True and harness.all_axes_hold(), hold=False)
        harness.drive.feedback[3] = 0
        harness.pump(lambda: harness.held is False, hold=False)
        harness.run_for(0.2, hold=False)
        assert harness.all_axes_hold(), 'fault recovery revived an old target'

        for board, state_name in [(harness.arm, 'arm_lm'), (harness.drive, 'wheel')]:
            harness.pump(harness.executing, hold=False, commands=True)
            board.streaming = False
            other = harness.drive if board is harness.arm else harness.arm
            harness.pump(lambda: harness.held is True and harness.board_holds(other)
                         and harness.board_states.get(state_name) == 2
                         and (board is not harness.drive
                              or harness.board_states.get('clock') == 2),
                         hold=False, timeout=1.2)
            harness.run_for(0.06, hold=False)  # Let previously queued PTY bytes drain.
            stale_tx_at = board.last_command_at
            healthy_tx_at = other.last_command_at
            samples = harness.joint_samples
            harness.run_for(0.15, hold=False)
            assert board.last_command_at == stale_tx_at, 'stale board still received cached targets'
            assert other.last_command_at > healthy_tx_at, 'healthy board stopped receiving hold'
            assert harness.board_holds(other), 'healthy board received a moving target during hold'
            if board is harness.arm:
                assert harness.joint_samples == samples, 'stale motor feedback was republished'
                board.feedback[0] += 0.02
            else:
                board.feedback[2] += 0.1
            board.streaming = True
            harness.pump(lambda: harness.held is False
                         and harness.board_states.get(state_name) == 0, hold=False)
            harness.run_for(0.2, hold=False)
            assert harness.all_axes_hold(), 'serial feedback recovery revived an old target'

        harness.close()
        harness = Harness(binary, calibrated=False)
        harness.pump(lambda: harness.held is True and harness.all_axes_hold())
        harness.run_for(0.3, hold=False)
        assert harness.joints is None, 'unconfirmed calibration published a fabricated joint state'
        print('PASS: startup, all-axis hold, resume, stale heartbeat, hardware fault, '
              'each serial feedback timeout, measured joints/calibration gate, paused ROS clock')
    except Exception:
        print(harness.close())
        raise
    else:
        harness.close()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
