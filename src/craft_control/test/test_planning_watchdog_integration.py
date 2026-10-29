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
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Int32
from craft_interfaces.msg import TaskCommand, TaskResult, ControlStatus


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
        # + imu_ok, gx, gy, gz [rad/s], ax, ay, az [g]: both boards still; lm_arm reads -1 g so
        # the relay is checked per board.
        self.arm = FakeBoard([0.05, 0.1, 0.2, 0.3, 0.4, 0.6, 0, 0, 1, 0, 0, 0, 0, 0, -1])
        self.drive = FakeBoard([0, 0, 0.7, 0, 0, 1, 0, 0, 0, 0, 0, 1])
        self.log = tempfile.TemporaryFile(mode='w+t')
        config = Path(__file__).resolve().parents[1] / 'config' / 'control.yaml'
        arguments = [str(binary), '--ros-args', '--params-file', str(config)]
        overrides = {
            'arm_lm_port': self.arm.port,
            'clock_wheel_port': self.drive.port,
            'use_sim_time': 'true',
            'arm_ik_mapping_confirmed': str(calibrated).lower(),
            'lm_mapping_confirmed': str(calibrated).lower(),
            'lm_meters_per_rad': '1.0',
            'gripper_mapping_confirmed': 'true',
            'gripper_open_angle': '0.2',
            'gripper_close_angle': '1.2',
            # Logic test, not hardware calibration: identity arm/LM mapping with wide limits,
            # so the fake feedback values stay valid whatever control.yaml measures.
            'arm_min_angle': '[-3.0,-3.0,-3.0,-3.0]',
            'arm_max_angle': '[3.0,3.0,3.0,3.0]',
            'arm_joint_offsets': '[0.0,0.0,0.0,0.0]',
            'arm_joint_directions': '[1.0,1.0,1.0,1.0]',
            'lm_motor_offset': '0.0',
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
        self.odom_pub = self.node.create_publisher(Odometry, '/planning/odom', 10)
        self.task_pub = self.node.create_publisher(TaskCommand, '/planning/task_command', 10)
        self.status = None
        self.results = {}
        self.node.create_subscription(ControlStatus, '/control/status', lambda m: setattr(self, 'status', m), 10)
        self.node.create_subscription(TaskResult, '/control/task_result',
                                      lambda m: self.results.update({m.task_id: m}), 10)
        self.imu = {}
        for board in ['wheel_clock', 'lm_arm']:
            self.node.create_subscription(Imu, f'/control/imu/{board}',
                                          lambda m, b=board: self.imu.update({b: m}), qos_profile_sensor_data)

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

    def nav(self, task_id, revision=1, operation=TaskCommand.EXECUTE):
        c = TaskCommand(task_id=task_id, revision=revision, operation=operation,
                        kind=TaskCommand.NAVIGATE, speed=.1)
        c.path.header.frame_id = 'map'
        for x in [0.,2.]:
            p=PoseStamped(); p.header.frame_id='map'; p.pose.position.x=x; p.pose.orientation.w=1.
            c.path.poses.append(p)
        self.task_pub.publish(c)
        return c

    def stop_task(self, task_id):
        self.task_pub.publish(TaskCommand(task_id=task_id, operation=TaskCommand.STOP))

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
    rclpy.init()
    h=Harness(binary)
    try:
        h.pump(lambda: h.status is not None and h.status.ready and h.status.stopped, hold=True)
        assert h.node.get_clock().now().nanoseconds == 0
        # Each board's IMU fields are relayed raw (board frame, accel g -> m/s^2) for perception.
        h.drive.feedback[8], h.arm.feedback[11] = 0.3, -0.3
        h.pump(lambda: len(h.imu) == 2 and h.imu['wheel_clock'].angular_velocity.z == 0.3 and
               h.imu['lm_arm'].angular_velocity.z == -0.3, hold=True)
        assert h.imu['wheel_clock'].header.frame_id == 'wheel_clock_imu'
        assert abs(h.imu['wheel_clock'].linear_acceleration.z - 9.80665) < 1e-9
        assert abs(h.imu['lm_arm'].linear_acceleration.z + 9.80665) < 1e-9
        h.drive.feedback[8] = h.arm.feedback[11] = 0
        h.pump(lambda: h.status.held is False,hold=False)
        h.run_for(.1,hold=False,commands=True)
        c=h.nav(101)
        h.pump(lambda: h.results.get(101) is not None and h.drive.command[0] > 0,
               hold=False,commands=True)
        assert list(h.results[101].controllers) == [0,0,1,0,0]
        h.nav(102)
        h.pump(lambda:h.results.get(102) is not None,hold=False,commands=True)
        assert h.results[102].status==TaskResult.REJECTED
        h.nav(101,2,TaskCommand.UPDATE_PATH)
        h.pump(lambda:h.results[101].revision==2,hold=False,commands=True)
        h.task_pub.publish(c)  # repeated EXECUTE does not reset revision
        h.run_for(.1,hold=False,commands=True)
        assert h.results[101].revision==2
        # A hold command is not a physical stop acknowledgement.
        h.drive.feedback[0]=h.drive.feedback[1]=.5
        h.stop_task(101)
        h.pump(lambda:h.results[101].status==TaskResult.STOPPING,hold=True)
        h.run_for(.15,hold=True)
        assert h.results[101].status==TaskResult.STOPPING and not h.status.stopped
        h.drive.feedback[0]=h.drive.feedback[1]=0
        h.pump(lambda:h.results[101].status==TaskResult.STOPPED and h.status.stopped,hold=True)
        h.pump(lambda:not h.status.held,hold=False)
        h.run_for(.1,hold=False,commands=True)
        h.task_pub.publish(c)  # retired ID cannot re-execute
        h.run_for(.15,hold=False,commands=True)
        assert h.all_axes_hold() and h.results[101].status==TaskResult.STOPPED

        arm=TaskCommand(task_id=103,revision=1,kind=TaskCommand.MANIPULATOR)
        arm.joints=JointState(name=['slide_joint','shoulder_joint','elbow_joint',
                             'wrist_pitch_joint','tool_roll_joint'],position=[.1,.2,.3,.4,.5])
        h.task_pub.publish(arm)
        h.pump(lambda:103 in h.results,hold=False)
        assert list(h.results[103].controllers)==[1,1,0,0,0]
        h.arm.feedback[1:5]=[.2,.3,.4,.5]
        h.pump(lambda:h.results[103].controllers[0]==TaskResult.DONE,hold=False)
        assert h.results[103].status==TaskResult.RUNNING  # LM still moving
        h.arm.feedback[0]=.1
        h.pump(lambda:h.results[103].status==TaskResult.SUCCEEDED,hold=False)
        # A serial dropout must fail the task and hold the other board.
        h.run_for(.1,hold=False,commands=True); h.nav(104)
        h.pump(lambda:104 in h.results,hold=False,commands=True)
        h.arm.streaming=False
        h.pump(lambda:h.results[104].status==TaskResult.FAILED and h.status.fault,
               hold=False,commands=True)
        h.run_for(.08,hold=True)
        tx=h.arm.last_command_at; h.run_for(.12,hold=True)
        assert h.arm.last_command_at==tx and h.board_holds(h.drive)
        h.arm.streaming=True
        h.pump(lambda:h.status.ready and h.status.stopped,hold=True)
        h.pump(lambda:not h.status.held,hold=False)
        h.run_for(.1,hold=False,commands=True); h.nav(105)
        h.pump(lambda:105 in h.results and h.drive.command[0]>0,hold=False,commands=True)
        h.pump(lambda:h.status.held and h.all_axes_hold() and
               h.results[105].status in (TaskResult.STOPPING,TaskResult.STOPPED),commands=True,timeout=2)
        assert h.results[105].status in (TaskResult.STOPPING,TaskResult.STOPPED)
        print('PASS: raw board IMU relay, task IDs, deduplication, path revisions, per-controller matrix, '
              'measured stop, serial timeout and planning watchdog with paused ROS clock')
    except Exception:
        print(h.close()); raise
    else:
        h.close()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
