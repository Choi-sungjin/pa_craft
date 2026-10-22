"""Run the installed C++ master against synthetic sensors and controller feedback."""

from copy import deepcopy
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Int32, String
import yaml


JOINT_NAMES = [
    'slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint',
]


class SyntheticController:
    """Keep feedback fresh while the test explicitly controls acknowledgements."""

    def __init__(self, node, executor, child, log):
        self.node, self.executor, self.child, self.log = node, executor, child, log
        self.state = None
        self.hold = None
        self.speed = None
        self.states, self.speeds = [], []
        self.path_commands, self.arm_commands = [], []
        self.gripper_commands, self.clock_commands = [], []
        self.active_path = None
        self.tracking_mode = None
        self.publish_odometry = True
        self.obstacle_frame = 'map'
        self.obstacle_positions = []
        self.position = (0.5, 0.5)
        self.joint_positions = [0.0] * 5
        self.observed_position = None
        self.clock_slot = 1
        self.gripper_reached = self.clock_reached = True
        # Cached true is deliberately present before each goal. A new false/true
        # cycle must be observed after the command, regardless of this old value.
        self.wheel_reached = self.arm_reached = self.lm_reached = True
        self.last_publish = -math.inf
        self.publishers = {}
        for topic, message in (
            ('/perception/state/odom', Odometry),
            ('/perception/lidar/obstacle', PoseArray),
            ('/arm/odom', JointState),
            ('/control/planning_held', Bool),
            ('/control/wheel_state', Int32),
            ('/control/arm_lm_state', Int32),
            ('/control/clock_state', Int32),
            ('/control/wheel_target_reached', Bool),
            ('/control/arm_target_reached', Bool),
            ('/control/lm_target_reached', Bool),
            ('/control/gripper_target_reached', Bool),
            ('/control/clock_target_reached', Bool),
            ('/control/clock_slot', Int32),
            ('/control/tracking_path', RosPath),
            ('/perception/vision/object_pose', PoseStamped),
            ('/planning/target_base_pose', PoseStamped),
            ('/planning/target_tcp_pose', PoseStamped),
            ('/planning/mission', String),
        ):
            self.publishers[topic] = node.create_publisher(message, topic, 10)
        self.subscriptions = [
            node.create_subscription(String, '/planning/state', self.on_state, 10),
            node.create_subscription(Bool, '/planning/hold', self.on_hold, 10),
            node.create_subscription(Float64, '/planning/target_v', self.on_speed, 10),
            node.create_subscription(RosPath, '/planning/target_path', self.on_path, 10),
            node.create_subscription(JointState, '/planning/target_arm_joints',
                                     self.arm_commands.append, 10),
            node.create_subscription(Bool, '/planning/gripper_command',
                                     self.gripper_commands.append, 10),
            node.create_subscription(Int32, '/planning/target_clock_slot',
                                     self.clock_commands.append, 10),
        ]

    def on_state(self, msg):
        self.state = msg.data
        self.states.append(msg.data)

    def on_hold(self, msg):
        self.hold = msg.data

    def on_speed(self, msg):
        self.speed = msg.data
        self.speeds.append(msg.data)

    def on_path(self, msg):
        if not msg.poses:
            self.active_path = None
            return
        self.active_path = msg
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if not self.path_commands or stamp != (
                self.path_commands[-1].header.stamp.sec,
                self.path_commands[-1].header.stamp.nanosec):
            self.path_commands.append(msg)

    def publish_feedback(self):
        now = time.monotonic()
        if now - self.last_publish < 0.02:
            return
        self.last_publish = now
        stamp = self.node.get_clock().now().to_msg()
        if self.publish_odometry:
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, 'map'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = self.position
            odom.pose.pose.orientation.w = 1.0
            self.publishers['/perception/state/odom'].publish(odom)
        points = PoseArray()
        points.header.stamp, points.header.frame_id = stamp, self.obstacle_frame
        for x, y, z in self.obstacle_positions:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, z
            pose.orientation.w = 1.0
            points.poses.append(pose)
        self.publishers['/perception/lidar/obstacle'].publish(points)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name, joints.position = JOINT_NAMES, self.joint_positions
        self.publishers['/arm/odom'].publish(joints)
        for topic in ('/control/wheel_state', '/control/arm_lm_state', '/control/clock_state'):
            self.publishers[topic].publish(Int32(data=0))
        self.publishers['/control/planning_held'].publish(Bool(data=self.hold is not False))
        for topic, value in (
            ('/control/wheel_target_reached', self.wheel_reached),
            ('/control/arm_target_reached', self.arm_reached),
            ('/control/lm_target_reached', self.lm_reached),
            ('/control/gripper_target_reached', self.gripper_reached),
            ('/control/clock_target_reached', self.clock_reached),
        ):
            self.publishers[topic].publish(Bool(data=value))
        self.publishers['/control/clock_slot'].publish(Int32(data=self.clock_slot))
        if self.observed_position is not None:
            observed = PoseStamped()
            observed.header.stamp, observed.header.frame_id = stamp, 'base_link'
            observed.pose.position.x, observed.pose.position.y, observed.pose.position.z = self.observed_position
            observed.pose.orientation.w = 1.0
            self.publishers['/perception/vision/object_pose'].publish(observed)
        if self.active_path is not None and self.tracking_mode is not None:
            tracking = deepcopy(self.active_path)
            if self.tracking_mode == 'mismatched':
                tracking.header.stamp.sec += 1
            self.publishers['/control/tracking_path'].publish(tracking)

    def check_process(self):
        if self.child.poll() is not None:
            self.log.flush()
            self.log.seek(0)
            raise AssertionError(
                f'planning_master exited with code {self.child.returncode}:\n' + self.log.read())

    def wait(self, predicate, description, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.check_process()
            self.publish_feedback()
            self.executor.spin_once(timeout_sec=0.005)
            if predicate():
                return
        self.log.flush()
        self.log.seek(0)
        raise AssertionError(
            f'{description}; state={self.state}, hold={self.hold}, speed={self.speed}, '
            f'paths={len(self.path_commands)}, arms={len(self.arm_commands)}\n{self.log.read()}')

    def observe_for(self, duration):
        deadline = time.monotonic() + duration
        self.wait(lambda: time.monotonic() >= deadline, 'observation interval', duration + 0.5)

    def goal(self, topic, frame='map', position=(1.5, 0.5, 0.0)):
        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = frame
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = position
        goal.pose.orientation.w = 1.0
        self.publishers[topic].publish(goal)

    def reset(self):
        self.publishers['/planning/mission'].publish(String(data='RESET'))
        self.wait(lambda: self.state == 'CANCELLED' and self.hold is True and self.speed == 0.0,
                  'reset clears the latched fault')


@pytest.fixture
def controller(tmp_path, monkeypatch):
    # The process and rclpy context share a private domain; no bringup or hardware
    # nodes are started, and the test cannot discover a normal robot domain.
    domain = 50 + os.getpid() % 150
    monkeypatch.setenv('ROS_DOMAIN_ID', str(domain))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    width = height = 40
    pixels = bytes(0 if x in (0, width - 1) or y in (0, height - 1) else 255
                   for y in range(height) for x in range(width))
    (tmp_path / 'map.pgm').write_bytes(b'P5\n40 40\n255\n' + pixels)
    map_file = tmp_path / 'map.yaml'
    map_file.write_text(yaml.safe_dump({
        'image': 'map.pgm', 'mode': 'trinary', 'resolution': 0.1,
        'origin': [0.0, 0.0, 0.0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }))
    mission_file = tmp_path / 'missions.yaml'
    mission_file.write_text(yaml.safe_dump({'missions': {'TEST_MISSION': [
        {'id': 'observed_tcp', 'type': 'tcp', 'source': 'object_pose',
         'pose': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]},
        {'id': 'grasp', 'type': 'gripper', 'close': True},
        {'id': 'store', 'type': 'clock', 'slot': 2},
        {'id': 'settle', 'type': 'wait', 'duration_s': 0.08},
    ]}}))
    overrides = tmp_path / 'overrides.yaml'
    overrides.write_text(yaml.safe_dump({'planning_master': {'ros__parameters': {
        'map_file': str(map_file), 'mission_file': str(mission_file), 'loop_hz': 40.0,
        'odom_timeout_s': 0.3, 'joint_timeout_s': 0.3,
        'obstacle_timeout_s': 0.3, 'observation_timeout_s': 0.3,
        'require_lidar': True, 'goal_heading_required': False,
        'joint_state_topic': '/arm/odom',
        'fsm.ready_timeout_s': 0.8, 'fsm.compute_timeout_s': 1.0,
        'fsm.execution_timeout_s': 2.0, 'fsm.feedback_timeout_s': 0.3,
        'fsm.completion_debounce_s': 0.05, 'fsm.max_compute_retries': 0,
    }}}))
    share = Path(get_package_share_directory('craft_planning'))
    executable = Path(get_package_prefix('craft_planning')) / 'lib/craft_planning/planning_master'
    assert executable.is_file(), f'build and source craft_planning first: {executable}'
    context = Context()
    node = executor = child = None
    with (tmp_path / 'planning_master.log').open('w+') as log:
        try:
            rclpy.init(context=context, domain_id=domain)
            node = rclpy.create_node('planning_runtime_regression', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            child = subprocess.Popen([
                str(executable), '--ros-args', '--params-file', str(share / 'config/planning.yaml'),
                '--params-file', str(overrides),
            ], stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
            fake = SyntheticController(node, executor, child, log)
            fake.wait(lambda: all(pub.get_subscription_count() > 0
                                  for pub in fake.publishers.values()) and
                      fake.state == 'IDLE' and fake.hold is True and fake.speed == 0.0,
                      'DDS discovery and idle hold acknowledgement', timeout=10.0)
            yield fake
        finally:
            if child is not None:
                if child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=3.0)
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if context.ok():
                context.shutdown()


def test_navigation_requires_matching_tracking_and_new_completion(controller):
    fake = controller
    assert not fake.path_commands
    assert all(speed == 0.0 for speed in fake.speeds)
    fake.goal('/planning/target_base_pose')
    fake.wait(lambda: fake.state == 'EXECUTING' and bool(fake.path_commands),
              'navigation publishes a path after fresh readiness inputs')
    assert fake.hold is False
    fake.tracking_mode = 'mismatched'
    fake.observe_for(0.08)
    assert fake.state == 'EXECUTING'
    assert all(speed == 0.0 for speed in fake.speeds), (
        'wheel speed must stay zero until a matching controller path is acknowledged')
    fake.tracking_mode = 'matching'
    fake.wait(lambda: fake.speed is not None and fake.speed > 0.0,
              'matching controller path authorizes cruise speed')
    fake.position = (1.5, 0.5)
    fake.observe_for(0.15)
    assert fake.state == 'EXECUTING', 'cached reached=true cannot complete a fresh goal'
    fake.wheel_reached = False
    fake.observe_for(0.08)
    fake.wheel_reached = True
    fake.wait(lambda: fake.state == 'SUCCEEDED' and fake.hold is True and fake.speed == 0.0,
              'new false/true completion and goal odometry finish navigation')


def test_empty_obstacles_need_no_transform_but_points_do(controller):
    fake = controller
    fake.tracking_mode = 'matching'
    fake.obstacle_frame = 'unmapped_lidar_frame'
    fake.observe_for(0.08)
    fake.goal('/planning/target_base_pose')
    fake.wait(lambda: fake.state == 'EXECUTING' and fake.speed is not None and fake.speed > 0.0,
              'an empty scan needs no coordinate transform')
    fake.obstacle_positions = [(0.5, 0.5, 0.0)]
    fake.wait(lambda: fake.state == 'FAULT' and fake.hold is True and fake.speed == 0.0,
              'actual obstacle points still require a coordinate transform', timeout=1.5)


def test_odometry_fault_reset_and_native_tcp_ik(controller):
    fake = controller
    fake.tracking_mode = 'matching'
    fake.goal('/planning/target_base_pose')
    fake.wait(lambda: fake.state == 'EXECUTING' and fake.speed is not None and fake.speed > 0.0,
              'navigation starts before the odometry dropout')
    fake.publish_odometry = False
    fake.wait(lambda: fake.state == 'FAULT' and fake.hold is True and fake.speed == 0.0,
              'stale odometry faults and stops navigation', timeout=1.5)
    commands = len(fake.path_commands)
    fake.publish_odometry = True
    fake.goal('/planning/target_base_pose')
    fake.observe_for(0.1)
    assert fake.state == 'FAULT'
    assert len(fake.path_commands) == commands, 'a new goal must not bypass a latched fault'
    fake.reset()
    fake.goal('/planning/target_base_pose', frame='unmapped_camera_frame')
    fake.wait(lambda: fake.state == 'FAULT' and fake.hold is True and fake.speed == 0.0,
              'a direct base goal in an invalid frame faults')
    fake.reset()
    # FK of all-zero joints in the real installed v75 URDF. No /ik_solution
    # publisher exists in this test: the C++ master must invoke its native IK.
    fake.goal('/planning/target_tcp_pose', frame='base_link',
              position=(0.149, 0.3694331033206, 0.3391808458987))
    fake.wait(lambda: fake.state == 'EXECUTING' and bool(fake.arm_commands),
              'native IK publishes the ordered five-joint control command')
    command = fake.arm_commands[-1]
    assert list(command.name) == JOINT_NAMES
    assert len(command.position) == 5 and all(math.isfinite(q) for q in command.position)
    assert all(abs(q) < 1e-6 for q in command.position)
    fake.observe_for(0.1)
    assert fake.state == 'EXECUTING', 'cached arm/LM true must not finish a new TCP request'
    fake.arm_reached = fake.lm_reached = False
    fake.observe_for(0.08)
    fake.joint_positions = list(command.position)
    fake.arm_reached = fake.lm_reached = True
    fake.wait(lambda: fake.state == 'SUCCEEDED' and fake.hold is True and fake.speed == 0.0,
              'fresh arm and LM completion gates finish the TCP request')


def test_current_robot_footprint_collision_stops_an_active_path(controller):
    fake = controller
    fake.tracking_mode = 'matching'
    fake.goal('/planning/target_base_pose')
    fake.wait(lambda: fake.state == 'EXECUTING' and fake.speed is not None and fake.speed > 0.0,
              'a free path is executing before the localization discontinuity')
    # This is fresh, valid map-frame odometry, but its center lies in the black
    # boundary wall. Checking only the previously accepted path would miss it.
    fake.position = (0.05, 0.5)
    fake.wait(lambda: fake.state == 'FAULT' and fake.hold is True and fake.speed == 0.0,
              'the actual current footprint in a wall faults and holds the robot', timeout=1.0)


def test_observed_mission_sequences_actions_and_deduplicates(controller):
    fake = controller
    fake.observed_position = (0.149, 0.3694331033206, 0.3391808458987)
    fake.publishers['/planning/mission'].publish(String(data='TEST_MISSION'))
    fake.wait(lambda: bool(fake.arm_commands), 'observed mission starts native TCP IK')
    fake.arm_reached = fake.lm_reached = False
    fake.observe_for(0.08)
    fake.arm_reached = fake.lm_reached = True
    fake.wait(lambda: bool(fake.gripper_commands), 'TCP completion advances to gripper')
    assert fake.gripper_commands[-1].data is True
    fake.observe_for(0.08)
    assert not fake.clock_commands, 'cached gripper true must not skip the grasp step'
    fake.gripper_reached = False
    fake.observe_for(0.08)
    fake.gripper_reached = True
    fake.wait(lambda: bool(fake.clock_commands), 'gripper completion advances to carousel')
    assert fake.clock_commands[-1].data == 2
    fake.clock_reached = False
    fake.observe_for(0.08)
    fake.clock_reached = True
    fake.observe_for(0.08)
    assert fake.hold is False, 'carousel must wait for the requested measured slot'
    fake.clock_slot = 2
    fake.wait(lambda: fake.state == 'SUCCEEDED' and fake.hold is True,
              'carousel and timed wait complete the configured mission')
    # SUCCEEDED is also briefly published between steps; allow the timed wait
    # to complete before asserting terminal deduplication.
    fake.observe_for(0.4)
    assert fake.state == 'SUCCEEDED'
    fake.publishers['/planning/mission'].publish(String(data='TEST_MISSION'))
    fake.observe_for(0.15)
    assert len(fake.arm_commands) == 1, 'duplicate mission names must not restart work'
    fake.publishers['/planning/mission'].publish(String(data='INIT'))
    fake.observe_for(0.05)
    fake.publishers['/planning/mission'].publish(String(data='TEST_MISSION'))
    fake.wait(lambda: len(fake.arm_commands) == 2, 'INIT permits a deliberate mission repeat')
    fake.publishers['/planning/mission'].publish(String(data='STOP'))
    fake.wait(lambda: fake.state == 'CANCELLED' and fake.hold is True and fake.speed == 0.0,
              'STOP cancels the mission queue and holds every axis')
