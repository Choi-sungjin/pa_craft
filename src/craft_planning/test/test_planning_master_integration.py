"""Real PlanningMaster, synthetic ControlMaster and sensors; isolated ROS domain."""
import math
import copy
import os
from pathlib import Path
import signal
import subprocess
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from craft_interfaces.msg import TaskCommand, TaskResult, ControlStatus, BoxObservation, BoxObservationRequest
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool, String, Int32MultiArray
import yaml

JOINTS = ['slide_joint', 'shoulder_joint', 'elbow_joint', 'wrist_pitch_joint', 'tool_roll_joint']

class Fake:
    def __init__(self, node, child, log):
        self.node, self.child, self.log = node, child, log
        self.executor = SingleThreadedExecutor(context=node.context)
        self.executor.add_node(node)
        self.state = None
        self.hold = True
        self.allow_stop = True
        self.feedback = True
        self.auto_complete = False
        self.commands = []
        self.rejected_while_held = []
        self.current = None
        self.near_previous_task_id = 0
        self.result_status = TaskResult.RUNNING
        self.result_id = None
        self.obstacles = []
        self.box_request = None
        self.box_requests = []
        self.box_sent = []
        self.box_observation_gap = .3
        self.box_observation_enabled = True
        self.box_hold_first_camera = False
        self.hold_verify_observations = False
        self.stale_stop_ack = False
        self.stale_ack_stamp = None
        self.box_observation_mode = 'valid'
        self.box_observation_fixed_stamp = None
        self.box_observation_valid = True
        self.box_observation_reason = ''
        self.force_control_unheld = False
        self.box_observation_extrapolated = False
        self.box_observation_uncertainty = .002
        self.box_observation_lateral = 0.
        self.box_observation_yaw = 0.
        self.box_camera_offset = 0.
        self.box_camera_pose_shift = 0.
        self.box_lidar_pose_shift = 0.
        self.camera_verify_gap_override = None
        self.position = (0.5, 0.5)
        self.localization_mode = "valid"
        self.fixed_stamp = None
        self.imu_mode = "valid"
        self.imu_fixed_stamp = None
        self.forwarded_odom = 0
        node.create_subscription(Odometry, "/planning/odom", self.on_forwarded_odom, 10)
        self.states = []
        self.pubs = {}
        for topic, cls in [('/control/status', ControlStatus), ('/control/task_result', TaskResult),
                           ('/perception/state/odom', Odometry), ('/perception/lidar/odom', Odometry),
                           ('/perception/imu/data', Imu), ('/control/arm_joints', JointState),
                           ('/perception/lidar/obstacle', PoseArray), ('/planning/command', String),
                           ('/planning/mission', String), ('/planning/target_base_pose', PoseStamped),
                           ('/planning/target_tcp_pose', PoseStamped), ('/perception/vision/object_pose', PoseStamped),
                           ('/perception/vision/d435/apriltag', Int32MultiArray),
                           ('/perception/localization/state', String), ('/planning/approach_box', String),
                           ('/planning/approach_box_near', String),
                           ('/perception/box/pose', PoseStamped),
                           ('/perception/box/observation', BoxObservation),
                           ('/perception/uppercam/observation', BoxObservation)]:
            self.pubs[topic] = node.create_publisher(cls, topic, 10)
        node.create_subscription(BoxObservationRequest, '/planning/box_observation_request', self.on_box_request, 10)
        node.create_subscription(String, '/planning/state', self.on_state, 10)
        node.create_subscription(Bool, '/planning/hold', lambda m: setattr(self, 'hold', m.data), 10)
        node.create_subscription(TaskCommand, '/planning/task_command', self.on_command, 10)

    def on_box_request(self, m):
        self.box_request = m
        self.box_requests.append(m)
        if m.source == 'uppercam' and sum(r.source == 'uppercam' for r in self.box_requests) == 2 and self.camera_verify_gap_override is not None:
            self.box_observation_gap = self.camera_verify_gap_override
        if m.source == 'uppercam' and sum(r.source == 'uppercam' for r in self.box_requests) == 2 and self.hold_verify_observations:
            self.box_observation_enabled = False
        if m.source == 'uppercam' and self.box_hold_first_camera:
            self.box_observation_enabled = False
            self.box_hold_first_camera = False

    def on_forwarded_odom(self, m):
        self.forwarded_odom += 1

    def on_state(self, m):
        self.state = m.data
        self.states.append(m.data)

    def on_command(self, m):
        self.commands.append(m)
        if m.operation == TaskCommand.EXECUTE:
            self.current = m
            if self.hold:
                self.rejected_while_held.append(m)
                self.result_status = TaskResult.REJECTED
                self.result_id = None
                return
            self.result_status = TaskResult.RUNNING
            self.result_id = None
        elif m.operation == TaskCommand.UPDATE_PATH:
            self.current = m
        elif m.operation == TaskCommand.STOP and self.allow_stop:
            self.result_status = TaskResult.STOPPED
            if self.stale_stop_ack:
                from builtin_interfaces.msg import Time
                ns = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec - 10_000_000
                self.stale_ack_stamp = Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        if self.feedback:
            c = ControlStatus(ready=True, held=self.hold and not self.force_control_unheld,
                              stopped=self.hold and self.allow_stop and not self.force_control_unheld)
            c.header.stamp = self.stale_ack_stamp if self.stale_stop_ack and self.stale_ack_stamp else stamp
            self.pubs['/control/status'].publish(c)
            if self.current:
                r = TaskResult(task_id=self.result_id or self.current.task_id,
                               revision=self.current.revision, status=self.result_status)
                if self.auto_complete: r.status = TaskResult.SUCCEEDED
                r.header.stamp = self.stale_ack_stamp if self.stale_stop_ack and self.stale_ack_stamp else stamp
                self.pubs['/control/task_result'].publish(r)
            elif self.near_previous_task_id:
                r = TaskResult(task_id=self.near_previous_task_id, status=self.result_status)
                r.header.stamp = stamp; self.pubs['/control/task_result'].publish(r)
        o = Odometry()
        o.header.stamp, o.header.frame_id = stamp, 'map'
        o.pose.pose.position.x, o.pose.pose.position.y = self.position
        yaw = getattr(self, 'yaw', 0.0)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pubs['/perception/state/odom'].publish(o)
        mode = self.localization_mode
        if mode != 'absent':
            if mode == 'repeated':
                if self.fixed_stamp is None: self.fixed_stamp = stamp
                o.header.stamp = self.fixed_stamp
            elif mode == 'future':
                ns = o.header.stamp.nanosec + 20_000_000
                o.header.stamp.sec += ns // 1_000_000_000
                o.header.stamp.nanosec = ns % 1_000_000_000
            elif mode == 'nan':
                o.twist.twist.linear.x = float('nan')
            elif mode == 'covariance_nan':
                o.twist.covariance[0] = float('nan')
            elif mode == 'covariance_negative':
                o.twist.covariance[0] = -1.0
            self.pubs['/perception/lidar/odom'].publish(o)
        imu = Imu(); imu.header.stamp = stamp; imu.header.frame_id = 'base_link'
        if self.imu_mode == 'repeated':
            if self.imu_fixed_stamp is None: self.imu_fixed_stamp = stamp
            imu.header.stamp = self.imu_fixed_stamp
        elif self.imu_mode == 'future':
            ns = imu.header.stamp.nanosec + 20_000_000
            imu.header.stamp.sec += ns // 1_000_000_000
            imu.header.stamp.nanosec = ns % 1_000_000_000
        elif self.imu_mode == 'nan':
            imu.angular_velocity.z = float('nan')
        elif self.imu_mode == 'covariance_nan':
            imu.angular_velocity_covariance[8] = float('nan')
        elif self.imu_mode == 'covariance_negative':
            imu.angular_velocity_covariance[8] = -1.0
        if self.imu_mode != 'absent':
            self.pubs['/perception/imu/data'].publish(imu)
        j = JointState(name=JOINTS, position=[0.0]*5)
        j.header.stamp = stamp
        self.pubs['/control/arm_joints'].publish(j)
        points = PoseArray()
        points.header.stamp, points.header.frame_id = stamp, 'map'
        for x, y in self.obstacles:
            from geometry_msgs.msg import Pose
            p = Pose(); p.position.x, p.position.y = x, y; p.orientation.w = 1.0
            points.poses.append(p)
        self.pubs['/perception/lidar/obstacle'].publish(points)
        observation = PoseStamped()
        observation.header.stamp, observation.header.frame_id = stamp, 'base_link'
        observation.pose.position.x, observation.pose.position.y, observation.pose.position.z = (
            0.149, 0.3694331033206, 0.3391808458987)
        observation.pose.orientation.w = 1.0
        self.pubs['/perception/vision/object_pose'].publish(observation)
        if self.box_request and self.box_request.source != 'off' and self.box_observation_enabled:
            msg = BoxObservation(); msg.header.frame_id = 'map'; msg.header.stamp = stamp
            if self.box_observation_mode == 'repeated':
                if self.box_observation_fixed_stamp is None: self.box_observation_fixed_stamp = stamp
                msg.header.stamp = self.box_observation_fixed_stamp
            elif self.box_observation_mode == 'at_request':
                msg.header.stamp = self.box_request.header.stamp
            elif self.box_observation_mode == 'future':
                ns = msg.header.stamp.nanosec + 20_000_000
                msg.header.stamp.sec += ns // 1_000_000_000
                msg.header.stamp.nanosec = ns % 1_000_000_000
            elif self.box_observation_mode == 'stale':
                ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec - 550_000_000
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            msg.request_id = self.box_request.request_id; msg.source = self.box_request.source
            msg.valid = self.box_observation_valid; msg.reason = self.box_observation_reason
            msg.box_pose = copy.deepcopy(self.box_request.expected_box)
            if msg.source == 'uppercam': msg.box_pose.position.y += self.box_camera_pose_shift
            if msg.source == 'lidar':
                msg.box_pose.position.x -= self.box_lidar_pose_shift * math.sin(yaw)
                msg.box_pose.position.y += self.box_lidar_pose_shift * math.cos(yaw)
            msg.gap_m = self.box_observation_gap + (self.box_camera_offset if msg.source == 'uppercam' else 0.)
            msg.gap_uncertainty_m = .010 if msg.source == 'lidar' else self.box_observation_uncertainty
            msg.extrapolated = self.box_observation_extrapolated and msg.source == 'uppercam'
            msg.lateral_m = self.box_observation_lateral; msg.yaw_rad = self.box_observation_yaw
            if msg.source == 'uppercam':  # planning reads the continuous camera here: base_link, request_id 0
                msg.header.frame_id = 'base_link'; msg.request_id = 0
            self.box_sent.append(copy.deepcopy(msg))
            self.pubs['/perception/uppercam/observation' if msg.source == 'uppercam' else '/perception/box/observation'].publish(msg)
        if getattr(self, 'box', None):  # (x, y, yaw): a box detection, fresh every cycle
            b = PoseStamped(); b.header.stamp, b.header.frame_id = stamp, 'map'
            b.pose.position.x, b.pose.position.y = self.box[0], self.box[1]
            b.pose.orientation.z, b.pose.orientation.w = math.sin(self.box[2] / 2), math.cos(self.box[2] / 2)
            self.pubs['/perception/box/pose'].publish(b)

    def wait(self, condition, timeout=5):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            if self.child.poll() is not None:
                self.log.seek(0); raise AssertionError(self.log.read())
            self.publish()
            self.executor.spin_once(timeout_sec=.02)
            if condition(): return
        self.log.seek(0)
        raise AssertionError(f'timeout; state={self.state}\n{self.log.read()}')

    def run_for(self, duration):
        end = time.monotonic()+duration
        self.wait(lambda: time.monotonic() >= end, duration+1)

    def op(self, name):
        self.pubs['/planning/command'].publish(String(data=name))

    def goal(self, x=1.5, frame='map'):
        p = PoseStamped(); p.header.stamp = self.node.get_clock().now().to_msg()
        p.header.frame_id = frame; p.pose.position.x = x; p.pose.position.y = .5
        p.pose.orientation.w = 1.0
        self.pubs['/planning/target_base_pose'].publish(p)
        return p

    def starts(self):
        return [c for c in self.commands if c.operation == TaskCommand.EXECUTE]

@pytest.fixture
def fake(tmp_path, monkeypatch, request):
    monkeypatch.setenv('ROS_DOMAIN_ID',os.environ.get('CAMERA_TEST_DOMAIN', str(50+os.getpid()%150)))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR',str(tmp_path/'logs'))
    (tmp_path/'map.pgm').write_bytes(b'P5\n40 40\n255\n'+bytes(
        0 if x in (0,39) or y in (0,39) else 255 for y in range(40) for x in range(40)))
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1,
        origin=[0.,0.,0.], negate=0, occupied_thresh=.65, free_thresh=.196)))
    (tmp_path/'missions.yaml').write_text(yaml.safe_dump({'missions': {'TEST': [
        {'id':'tcp', 'type':'tcp', 'source':'object_pose', 'pose':[0.,0.,0.,0.,0.,0.,1.]},
        {'id':'grasp','type':'gripper','close':True},
        {'id':'slot','type':'clock','slot':2}, {'id':'wait','type':'wait','duration_s':.1}]}}))
    (tmp_path/'tags.yaml').write_text(yaml.safe_dump({'tags': {7: {'mission': 'TEST'}}}))
    (tmp_path/'override.yaml').write_text(yaml.safe_dump({'planning_master': {'ros__parameters': {
        'map_file':str(tmp_path/'map.yaml'), 'mission_file':str(tmp_path/'missions.yaml'),
        'tag_registry_file':str(tmp_path/'tags.yaml'),
        'fsm.initialization_timeout_s':5., 'fsm.ready_timeout_s':5.,
        'fsm.stop_timeout_s':.6, 'fsm.feedback_timeout_s':.5,
        'fsm.execution_timeout_s':10., 'replan_period_s':.1,
        # The tests below were written for A*+arc (STOP_TURN segments) and padding 0.24 m (their corridors);
        # global_local has its own test. planning.yaml's robot size changes must not change these maps.
        'path_planner.r_robot':.21, 'path_planner.margin':.03,
        'path_planner.mode':'global_local' if getattr(request, 'param', None) in ('global_local', 'camera_segmented') else 'arc',
        'path_planner.local_ahead': .5 if getattr(request, 'param', None) == 'camera_segmented' else 4.0,
        'path_planner.local_update': .1 if getattr(request, 'param', None) == 'camera_segmented' else 2.0,
        'require_lidar_localization': getattr(request, 'param', None) in ('localization', 'localization_missing'),
        'localization_observation_timeout_s': 0.0 if getattr(request, 'param', None)=='bad_localization_timeout' else .3,
        # One box in the middle of the 4 x 4 m room (not in the map: only the approach matters here).
        'box_approach.camera_enabled': getattr(request, 'param', None) in ('camera', 'camera_500', 'camera_segmented', 'camera_only'),
        'box_approach.camera_only_longitudinal': getattr(request, 'param', None) == 'camera_only',
        'box_approach.dock_lookahead': 0.0 if getattr(request, 'param', None) == 'bad_dock_lookahead_zero' else (-.01 if getattr(request, 'param', None) == 'bad_dock_lookahead_negative' else .020),
        'box_approach.camera_calibrated_gap_min': .009 if getattr(request, 'param', None) == 'bad_camera_range_lower' else .010,
        'box_approach.camera_calibrated_gap_max': .081 if getattr(request, 'param', None) == 'bad_camera_range_upper' else .080,
        'box_approach.settle_s': .05 if getattr(request, 'param', None) in ('camera', 'camera_500', 'camera_segmented') else .5,
        'box_approach.max_corrections': 1 if getattr(request, 'param', None) == 'global_local' else 0,
        'box_approach.names':['L1', 'L2', 'L3', 'B'], 'box_approach.B':[2.5, 2.5, 90.0]}}}))
    context = Context(); rclpy.init(context=context)
    node = rclpy.create_node('fake_control',context=context)
    binary = Path(get_package_prefix('craft_planning'))/'lib/craft_planning/planning_master'
    config = Path(get_package_share_directory('craft_planning'))/'config/planning.yaml'
    rates = Path(__file__).resolve().parents[2]/'craft_bringup/config/rates.yaml'  # loop rates and timeouts
    with (tmp_path/'master.log').open('w+') as log:
        child = subprocess.Popen([str(binary),'--ros-args','--params-file',str(config),'--params-file',str(rates),
                                  '--params-file',str(tmp_path/'override.yaml')],stdout=log,stderr=log)
        f = Fake(node,child,log)
        if getattr(request, 'param', None) == 'localization_missing':
            f.localization_mode = 'absent'
        boards = []
        control_child = None
        try:
            if getattr(request, 'param', None) == 'real':
                import importlib.util
                helper = Path(__file__).resolve().parents[2] / 'craft_control/test/test_planning_watchdog_integration.py'
                spec = importlib.util.spec_from_file_location('pty_helpers', helper)
                module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
                # Firmware CSV: positions/states, then imu_ok, gx, gy, gz [rad/s], ax, ay, az [g] (still, upright).
                imu = [1, 0., 0., 0., 0., 0., 1.]
                boards = [module.FakeBoard([0.,0.,0.,0.,0.,.2,0,0] + imu), module.FakeBoard([0.,0.,0.,0,0] + imu)]
                control_binary = Path(get_package_prefix('craft_control'))/'lib/craft_control/control_master'
                control_config = Path(get_package_share_directory('craft_control'))/'config/control.yaml'
                args = [str(control_binary),'--ros-args','--params-file',str(control_config),'--params-file',str(rates)]
                for key, value in {'arm_lm_port':boards[0].port,'clock_wheel_port':boards[1].port,
                                   'arm_ik_mapping_confirmed':'true','lm_mapping_confirmed':'true',
                                   'gripper_mapping_confirmed':'true','lm_meters_per_rad':'1.0',
                                   'gripper_open_angle':'0.2',
                                   'gripper_close_angle':'1.2'}.items():
                    args.extend(['-p',f'{key}:={value}'])
                control_child = subprocess.Popen(args,stdout=log,stderr=log)
                f.feedback = False
            if getattr(request, 'param', None) not in ('localization_missing', 'bad_localization_timeout', 'bad_camera_range_lower', 'bad_camera_range_upper', 'bad_dock_lookahead_zero', 'bad_dock_lookahead_negative'):
                f.wait(lambda: f.state == 'Ready',10)
                f.op('START'); f.wait(lambda: f.state == 'Run.AwaitRequest')
            yield f
        finally:
            child.send_signal(signal.SIGINT)
            try: child.wait(timeout=5)
            except subprocess.TimeoutExpired: child.kill(); child.wait()
            if control_child:
                control_child.send_signal(signal.SIGINT)
                try: control_child.wait(timeout=3)
                except subprocess.TimeoutExpired: control_child.kill(); control_child.wait()
            for board in boards: board.close()
            f.executor.shutdown()
            node.destroy_node(); context.shutdown()

def test_queue_ids_replan_and_stale_completion(fake):
    f=fake; p=f.goal(); f.wait(lambda: len(f.starts())==1)
    first=f.starts()[0].task_id
    f.pubs['/planning/target_base_pose'].publish(p)  # duplicate request
    f.goal(2.)
    f.run_for(.3)  # no periodic replanning while the path is clear
    assert not any(c.operation==TaskCommand.UPDATE_PATH for c in f.commands)
    f.obstacles=[(1.,.5)]  # blocks the straight path: replan around it, same task
    f.wait(lambda: any(c.operation==TaskCommand.UPDATE_PATH for c in f.commands))
    f.obstacles=[]
    assert len(f.starts())==1
    assert all(c.task_id==first for c in f.commands)
    revisions=[c.revision for c in f.commands]; assert revisions==sorted(set(revisions))
    f.result_id=first-1; f.result_status=TaskResult.SUCCEEDED; f.run_for(.15)
    assert len(f.starts())==1 and f.state=='Run.Execute'
    f.result_id=first; f.wait(lambda: len(f.starts())==2)
    second=f.starts()[1].task_id; assert second>first
    f.result_id=first; f.result_status=TaskResult.SUCCEEDED; f.run_for(.15)
    assert f.state=='Run.Execute'
    f.result_id=second; f.wait(lambda:f.state=='Run.AwaitRequest')
    f.run_for(.15); assert len(f.starts())==2

def test_cancel_and_stop_wait_for_aggregate_ack(fake):
    f=fake; f.goal(); f.wait(lambda:f.state=='Run.Execute')
    f.allow_stop=False; f.op('CANCEL'); f.run_for(.15)
    assert f.state=='Run.Stopping'
    f.allow_stop=True; f.result_status=TaskResult.STOPPED
    f.wait(lambda:f.state=='Run.AwaitRequest')
    f.op('STOP'); f.wait(lambda:f.state=='Ready')
    f.goal(); f.run_for(.1); assert f.state=='Ready'

def test_stop_timeout_fault_and_reset_requires_stop(fake):
    f=fake; f.goal(); f.wait(lambda:f.state=='Run.Execute')
    f.allow_stop=False; f.op('CANCEL'); f.wait(lambda:f.state=='Fault')
    f.op('RESET'); f.run_for(.15); assert f.state=='Fault'
    f.allow_stop=True; f.wait(lambda:f.state=='Ready')
    f.run_for(.15); assert f.state=='Ready'  # reset never resumes

def test_replan_route_failure_retries_and_invalid_request_is_rejected(fake):
    f=fake; f.goal(frame='bad'); f.run_for(.15)
    assert f.state=='Run.AwaitRequest' and not f.starts()
    f.goal(); f.wait(lambda:f.state=='Run.Execute')
    updates_before = len([c for c in f.commands if c.operation == TaskCommand.UPDATE_PATH])
    f.obstacles=[(.5,.5)]; f.run_for(.4)
    assert f.state == 'Run.Execute' and not f.hold
    assert any(c.operation == TaskCommand.UPDATE_PATH and c.speed == 0. for c in f.commands)
    f.obstacles=[]
    f.wait(lambda:any(c.operation == TaskCommand.UPDATE_PATH and c.speed > 0.
                      for c in f.commands[updates_before:]))

def test_path_is_sent_one_stop_turn_segment_at_a_time(fake):
    # One-cell C-space corridor (padding 0.24 m, 0.1 m grid) east along y=0.35 that steps up one cell to
    # y=0.45 at x=1.55: two corners 0.1 m apart leave no room for a fillet, so (1.55, 0.35) is a STOP_TURN
    # and the path goes out as two control tasks.
    f=fake; f.position=(.35,.35)
    wall=lambda a, b, n: [(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(n+1)]
    f.obstacles=wall((0.,.65),(1.25,.65),25)+wall((1.85,.15),(3.9,.15),41)+wall((1.85,.75),(3.9,.75),41)
    p=PoseStamped(); p.header.stamp=f.node.get_clock().now().to_msg(); p.header.frame_id='map'
    p.pose.position.x, p.pose.position.y = 3.05, .45; p.pose.orientation.w=1.
    f.pubs['/planning/target_base_pose'].publish(p)
    f.wait(lambda: len(f.starts())==1)
    first=f.starts()[0]; end=first.path.poses[-1].pose.position
    assert abs(end.x-1.55)<1e-6 and abs(end.y-.35)<1e-6  # first segment ends at the STOP_TURN corner
    f.run_for(.3); assert len(f.starts())==1 and f.state=='Run.Execute'
    f.position=(end.x, end.y); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: len(f.starts())==2)
    second=f.starts()[1]; assert second.task_id>first.task_id and f.state=='Run.Execute'
    start, end=second.path.poses[0].pose.position, second.path.poses[-1].pose.position
    assert abs(start.x-1.55)<1e-6 and abs(start.y-.35)<1e-6  # planned again from the corner
    assert abs(end.x-3.05)<1e-6 and abs(end.y-.45)<1e-6      # the rest, up to the goal
    f.position=(3.05,.45); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: f.state=='Run.AwaitRequest')
    assert len(f.starts())==2

def test_blocked_path_waits_resumes_or_replans(fake):
    f=fake; f.goal(); f.wait(lambda:f.state=='Run.Execute' and f.starts())  # state may arrive before the command
    first=f.starts()[-1]; n=len(f.commands)
    # A passer-by on the path for less than block_wait_s: pause, then the same path resumes (no replan).
    f.obstacles=[(1.,.5)]
    f.wait(lambda:any(c.operation==TaskCommand.UPDATE_PATH and c.speed==0. for c in f.commands[n:]))
    f.run_for(.5); f.obstacles=[]
    f.wait(lambda:any(c.operation==TaskCommand.UPDATE_PATH and c.speed>0. for c in f.commands[n:]))
    resumed=[c for c in f.commands[n:] if c.operation==TaskCommand.UPDATE_PATH and c.speed>0.][0]
    assert [p.pose.position for p in resumed.path.poses]==[p.pose.position for p in first.path.poses]
    # Still blocked after block_wait_s (2 s): a new path around it.
    n=len(f.commands); f.obstacles=[(1.,.5)]
    f.run_for(1.); assert not any(c.operation==TaskCommand.UPDATE_PATH and c.speed>0. for c in f.commands[n:])
    f.wait(lambda:any(c.operation==TaskCommand.UPDATE_PATH and c.speed>0. for c in f.commands[n:]),4)
    detour=[c for c in f.commands[n:] if c.operation==TaskCommand.UPDATE_PATH and c.speed>0.][-1]
    assert max(p.pose.position.y for p in detour.path.poses)>.7 and f.state=='Run.Execute'

def test_segments_that_end_where_they_start_fault(fake):
    # The jog corridor of the segment test; control "arrives" at once while the robot never moves.
    f=fake; f.position=(.35,.35); f.auto_complete=True
    wall=lambda a, b, n: [(a[0]+(b[0]-a[0])*i/n, a[1]+(b[1]-a[1])*i/n) for i in range(n+1)]
    f.obstacles=wall((0.,.65),(1.25,.65),25)+wall((1.85,.15),(3.9,.15),41)+wall((1.85,.75),(3.9,.75),41)
    p=PoseStamped(); p.header.stamp=f.node.get_clock().now().to_msg(); p.header.frame_id='map'
    p.pose.position.x, p.pose.position.y = 3.05, .45; p.pose.orientation.w=1.
    f.pubs['/planning/target_base_pose'].publish(p)
    f.wait(lambda:f.state=='Fault',8)
    assert len(f.starts())==3

def test_control_feedback_timeout(fake):
    f=fake; f.goal(); f.wait(lambda:f.state=='Run.Execute')
    f.feedback=False; f.wait(lambda:f.state=='Fault',2)

def test_localization_lost_faults_but_dead_reckoning_does_not(fake):
    f=fake; f.goal(); f.wait(lambda:f.state=='Run.Execute')
    f.pubs['/perception/localization/state'].publish(String(data='dead_reckoning')); f.run_for(.15)
    assert f.state=='Run.Execute'
    f.pubs['/perception/localization/state'].publish(String(data='lost'))
    f.wait(lambda:f.state=='Fault',2)

def test_mission_native_ik_and_all_task_types(fake):
    f=fake; f.auto_complete=True
    f.pubs['/perception/vision/d435/apriltag'].publish(Int32MultiArray(data=[7]))
    f.wait(lambda:len(f.starts())==4 and f.state=='Run.AwaitRequest',10)
    tasks=f.starts(); assert [t.kind for t in tasks]==[1,2,3,4]
    assert list(tasks[0].joints.name)==JOINTS
    assert len({t.task_id for t in tasks})==4
    assert tasks[1].close and tasks[2].clock_slot==2
    f.pubs['/perception/vision/d435/apriltag'].publish(Int32MultiArray(data=[7])); f.run_for(.15)
    assert len(f.starts())==4

@pytest.mark.parametrize('fake',['real'],indirect=True)
def test_two_real_masters_queue_and_complete_over_pty(fake):
    f=fake
    f.goal(1.5); f.goal(2.)
    f.wait(lambda:len(f.starts())==1 and f.state=='Run.Execute')
    f.run_for(.2); assert len(f.starts())==1
    f.position=(1.5,.5)
    f.wait(lambda:len(f.starts())==2,5)
    f.position=(2.,.5)
    f.wait(lambda:f.state=='Run.AwaitRequest',5)
    assert f.starts()[0].task_id != f.starts()[1].task_id
    f.op('STOP'); f.wait(lambda:f.state=='Ready')


@pytest.mark.parametrize('fake',['global_local'],indirect=True)
def test_global_local_sends_windows_and_renews_after_progress(fake):
    # 4 x 4 m room: the global path (0.35, 0.35) -> (3.65, 3.65) is ~4.7 m, longer than the 4 m local window.
    f=fake; f.position=(.35,.35)
    p=PoseStamped(); p.header.stamp=f.node.get_clock().now().to_msg(); p.header.frame_id='map'
    p.pose.position.x, p.pose.position.y = 3.65, 3.65; p.pose.orientation.w=1.
    f.pubs['/planning/target_base_pose'].publish(p)
    f.wait(lambda: len(f.starts())==1)
    first=f.starts()[0]; pts=[(q.pose.position.x, q.pose.position.y) for q in first.path.poses]
    assert abs(pts[0][0]-.35)<1e-6 and abs(pts[0][1]-.35)<1e-6
    length=sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(pts, pts[1:]))
    assert 3.7 < length < 4.3 and math.hypot(pts[-1][0]-3.65, pts[-1][1]-3.65) > .3  # a window, not the goal
    f.run_for(.4)  # no renewal before local_update of progress
    assert not any(c.operation==TaskCommand.UPDATE_PATH for c in f.commands)
    # Drive 2.1 m along it: the next window goes out as UPDATE_PATH of the same task and reaches the goal.
    s=0.
    for a, b in zip(pts, pts[1:]):
        s+=math.hypot(b[0]-a[0], b[1]-a[1])
        if s>=2.1: f.position=b; break
    f.wait(lambda: any(c.operation==TaskCommand.UPDATE_PATH for c in f.commands))
    update=[c for c in f.commands if c.operation==TaskCommand.UPDATE_PATH][0]
    assert update.task_id==first.task_id and update.speed>0.
    end=update.path.poses[-1].pose.position
    assert abs(end.x-3.65)<1e-6 and abs(end.y-3.65)<1e-6
    near=min(math.hypot(q.pose.position.x-f.position[0], q.pose.position.y-f.position[1]) for q in update.path.poses)
    assert near<.05  # the new window passes through the robot
    f.position=(3.65,3.65); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: f.state=='Run.AwaitRequest')
    assert len(f.starts())==1


@pytest.mark.parametrize('fake',['global_local'],indirect=True)
def test_box_approach_stages_perceives_docks_and_corrects(fake):
    # Box B known at (2.5, 2.5). The step drives at once to the stage point of a face of the known box (no heading,
    # 3 cm arrival). There it waits for a detection taken standing still: the dock goes by the perceived box (3 cm
    # off, turned 5 deg), straight to its face goal at 0.05 m/s, 1 cm / 1 deg. The robot settles and is checked: 2 cm to
    # the side, it backs out (an obstacle behind shortens it), perceives, docks again, and aligned the step completes.
    f=fake; f.position=(.5,2.5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts())==1)
    stage=f.starts()[0]
    assert abs(stage.speed-.2)<1e-9 and not stage.require_goal_heading and abs(stage.goal_tolerance-.03)<1e-9
    known=(2.5, 2.5, math.pi/2)
    reach=.205/2+.165+.08
    def stage_face(c, b=known):
        e=c.path.poses[-1].pose.position
        for k in range(4):
            n=b[2]+k*math.pi/2
            for lead in (.20, .15, .10, .05, 0.):
                back=reach+.30+lead
                if math.hypot(e.x-(b[0]+back*math.cos(n)), e.y-(b[1]+back*math.sin(n)))<1e-6: return n
    known_normal=stage_face(stage)
    assert known_normal is not None  # the stage point of a face of the known box
    box=(2.5, 2.53, math.radians(95))
    normal=min((box[2]+k*math.pi/2 for k in range(4)), key=lambda n: abs(math.remainder(n-known_normal, 2*math.pi)))
    heading=normal+math.pi
    gx, gy = box[0]+reach*math.cos(normal), box[1]+reach*math.sin(normal)
    # At the stage point: nothing perceived yet, no dock.
    e=stage.path.poses[-1].pose.position
    f.yaw=heading+.8; f.position=(e.x, e.y); f.result_status=TaskResult.SUCCEEDED
    f.run_for(1.)
    assert not any(c.operation==TaskCommand.EXECUTE and abs(c.speed-.05)<1e-9 for c in f.commands)
    f.box=box
    # Perceived: the dock goes out, straight to the face goal of the perceived box.
    f.wait(lambda: any(c.operation==TaskCommand.EXECUTE and abs(c.speed-.05)<1e-9 for c in f.commands), 4)
    dock=[c for c in f.commands if c.operation==TaskCommand.EXECUTE and abs(c.speed-.05)<1e-9][0]
    assert dock.require_goal_heading and abs(dock.goal_tolerance-.01)<1e-9 and abs(dock.tolerance_rad-math.radians(1))<1e-9
    assert abs(stage.goal_tolerance-.03)<1e-9  # the stage drive stops near the normal
    d0, d1 = dock.path.poses[0].pose.position, dock.path.poses[-1].pose.position
    assert math.hypot(d0.x-e.x, d0.y-e.y)<.1 and math.hypot(d1.x-gx, d1.y-gy)<1e-6  # from by the robot to the goal
    off=max(abs(-math.sin(heading)*(q.pose.position.x-gx)+math.cos(heading)*(q.pose.position.y-gy)) for q in dock.path.poses)
    assert off<1e-6  # on the face normal
    # Arrive square to the face but 2 cm to the side, an obstacle 0.5 m behind (padding 0.24 m): the back-out is short.
    ax_, ay_ = gx-.02*math.sin(heading), gy+.02*math.cos(heading)
    f.obstacles=[(ax_-.5*math.cos(heading), ay_-.5*math.sin(heading))]
    f.yaw=heading; f.position=(ax_, ay_); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: any(c.operation==TaskCommand.EXECUTE and c.speed<0 for c in f.commands), 4)
    back=[c for c in f.commands if c.operation==TaskCommand.EXECUTE and c.speed<0][0]
    assert not back.require_goal_heading
    b0, b1 = back.path.poses[0].pose.position, back.path.poses[-1].pose.position
    back_len=math.hypot(b1.x-b0.x, b1.y-b0.y)  # the longest clear one in 5 cm steps (0.1 m test cells)
    assert .1-1e-6 <= back_len < .45 and abs(round(back_len/.05)*.05-back_len)<1e-6
    assert abs(math.remainder(math.atan2(b0.y-b1.y, b0.x-b1.x)-heading, 2*math.pi))<1e-6  # straight back, facing the box
    # Backed out: the dock starts where the robot is and leads away from the obstacle behind it.
    f.position=(b1.x, b1.y); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: len([c for c in f.commands if c.operation==TaskCommand.EXECUTE and abs(c.speed-.05)<1e-9])==2, 4)
    f.obstacles=[]
    again=[c for c in f.commands if c.operation==TaskCommand.EXECUTE and abs(c.speed-.05)<1e-9][-1]
    a1=again.path.poses[-1].pose.position
    assert math.hypot(a1.x-gx, a1.y-gy)<1e-6  # the same face goal
    f.position=(gx, gy); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: f.state=='Run.AwaitRequest', 4)

@pytest.mark.parametrize('fake', ['localization'], indirect=True)
@pytest.mark.parametrize('mode', ['absent', 'repeated', 'future', 'nan', 'covariance_nan', 'covariance_negative'])
def test_localization_loss_stops_and_latches(fake, mode):
    f = fake
    f.goal(); f.wait(lambda: len(f.starts()) == 1)
    f.localization_mode = mode
    f.wait(lambda: f.state == 'Fault', timeout=2)
    f.wait(lambda: f.hold and any(c.operation == TaskCommand.STOP for c in f.commands))
    f.run_for(.2)
    count = f.forwarded_odom
    f.run_for(.2)
    assert f.forwarded_odom == count
    starts = len(f.starts())
    f.localization_mode = 'valid'
    f.run_for(.5)
    assert f.state == 'Fault' and f.hold and len(f.starts()) == starts
    f.op('RESET'); f.wait(lambda: f.state == 'Ready')
    f.run_for(.2)
    assert len(f.starts()) == starts
    f.op('START'); f.wait(lambda: f.state == 'Run.AwaitRequest')

@pytest.mark.parametrize('fake', ['localization'], indirect=True)
def test_localization_continuous_observations_allow_execution(fake):
    f = fake
    f.goal(); f.wait(lambda: len(f.starts()) == 1)
    f.run_for(.7)
    assert f.state == 'Run.Execute' and not f.hold and f.forwarded_odom > 0

@pytest.mark.parametrize('fake', ['localization'], indirect=True)
@pytest.mark.parametrize('mode', ['absent', 'repeated', 'future', 'nan', 'covariance_nan', 'covariance_negative'])
def test_imu_loss_stops_and_latches(fake, mode):
    f = fake
    f.goal(); f.wait(lambda: len(f.starts()) == 1)
    f.imu_mode = mode
    f.wait(lambda: f.state == 'Fault', timeout=2)
    f.wait(lambda: f.hold and any(c.operation == TaskCommand.STOP for c in f.commands))
    f.run_for(.15)
    count = f.forwarded_odom
    f.run_for(.2)
    assert f.forwarded_odom == count
    f.imu_mode = 'valid'
    f.run_for(.4)
    assert f.state == 'Fault' and f.hold and len(f.starts()) == 1

@pytest.mark.parametrize('fake', ['localization_missing'], indirect=True)
def test_fresh_ekf_and_obstacles_do_not_make_missing_lidar_ready(fake):
    f = fake
    f.run_for(.8)
    assert f.state == 'Idle' and f.hold and f.forwarded_odom == 0
    f.op('START'); f.goal(); f.run_for(.3)
    assert not f.starts() and f.hold

@pytest.mark.parametrize('fake', ['bad_localization_timeout'], indirect=True)
def test_localization_timeout_must_be_positive(fake):
    deadline = time.monotonic() + 5
    while fake.child.poll() is None and time.monotonic() < deadline:
        time.sleep(.02)
    assert fake.child.poll() not in (None, 0)
    fake.log.seek(0)
    assert 'localization_observation_timeout_s must be finite and positive' in fake.log.read()

def camera_reach_lidar_dock(f):
    f.position=(.5,2.5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    stage = f.starts()[0]; end_pose = stage.path.poses[-1].pose; end = end_pose.position
    assert stage.tracking_reference_x == 0.0 and stage.lookahead_distance == 0.0
    assert not stage.camera_distance_control
    q = end_pose.orientation; f.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    f.position=(end.x,end.y); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: len(f.starts()) == 2)
    assert abs(f.starts()[1].tracking_reference_x - .165) < 1e-9
    assert abs(f.starts()[1].lookahead_distance - .020) < 1e-9
    assert not f.starts()[1].camera_distance_control
    assert f.box_request.source == 'lidar'
    assert any(c.operation == TaskCommand.STOP for c in f.commands)

def request_near_box(f, gap=.081):
    f.position = (2.5 - .1025 - .165 - gap, 2.5); f.yaw = 0.
    f.box_observation_gap = gap; f.near_previous_task_id = 42
    f.pubs['/planning/approach_box_near'].publish(String(data='B'))

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_near_entry_held_lidar_to_camera_without_stage_motion(fake):
    f = fake; request_near_box(f)
    f.near_previous_task_id = 0  # fresh control after restart has no TaskResult
    f.yaw = -math.radians(3.228)
    f.wait(lambda: len(f.starts()) == 1)
    c = f.starts()[0]
    assert c.camera_distance_control and abs(c.speed-.005) < 1e-9
    assert abs(c.camera_target_gap_m-.010) < 1e-9
    assert abs(c.tracking_reference_x-.165) < 1e-9 and abs(c.lookahead_distance-.020) < 1e-9
    assert [r.source for r in f.box_requests if r.source != 'off'] == ['lidar','uppercam']
    assert len([m for m in f.commands if m.operation == TaskCommand.STOP and m.task_id == 0]) >= 2

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_near_entry_far_gap_stops_without_motion(fake):
    f = fake; request_near_box(f,.111)
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert not f.starts() and all(r.source != 'uppercam' for r in f.box_requests)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_near_entry_requires_measured_held_stopped_status(fake):
    f = fake; f.allow_stop = False; request_near_box(f); f.near_previous_task_id = 0
    f.wait(lambda: f.state == 'Fault',timeout=7); f.wait(lambda: f.hold)
    assert not f.starts() and not f.box_requests

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_near_entry_rejects_pre_stop_control_status(fake):
    f = fake; f.stale_stop_ack = True; request_near_box(f)
    f.wait(lambda: f.state == 'Fault',timeout=3); f.wait(lambda: f.hold)
    assert not f.starts() and not f.box_requests

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_near_entry_sensor_disagreement_stops_without_motion(fake):
    f = fake; f.box_camera_offset = .021; request_near_box(f)
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert not f.starts()

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_near_entry_stale_lidar_stops_without_motion(fake):
    f = fake; f.box_observation_mode = 'stale'; request_near_box(f)
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert not f.starts() and all(r.source != 'uppercam' for r in f.box_requests)

def camera_start_handoff(f):
    camera_reach_lidar_dock(f)
    f.box_observation_gap = .099
    end=f.starts()[-1].path.poses[-1].pose.position; f.position=(end.x,end.y)
    f.wait(lambda: f.box_request and f.box_request.source == 'uppercam')

def camera_reach_dock(f):
    camera_start_handoff(f)
    f.wait(lambda: len(f.starts()) == 3)
    assert not f.rejected_while_held
    assert abs(f.starts()[-1].speed - .005) < 1e-9
    assert max(f.starts()[-1].speed_limit) <= .005 + 1e-9
    assert abs(f.starts()[-1].tracking_reference_x - .165) < 1e-9
    assert abs(f.starts()[-1].lookahead_distance - .020) < 1e-9
    assert f.starts()[-1].camera_distance_control
    assert abs(f.starts()[-1].camera_target_gap_m - .010) < 1e-9
    assert f.starts()[-1].camera_observation_stamp.sec > 0

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_handoff_uses_lidar_acquired_after_stop_and_settle(fake):
    f = fake; camera_reach_lidar_dock(f); f.box_observation_gap = .099
    before = len(f.commands); f.wait(lambda: any(c.operation == TaskCommand.STOP for c in f.commands[before:]))
    stop = next(c for c in f.commands[before:] if c.operation == TaskCommand.STOP)
    stop_ns = stop.header.stamp.sec * 1_000_000_000 + stop.header.stamp.nanosec
    f.wait(lambda: f.box_request and f.box_request.source == 'uppercam')
    req_ns = f.box_request.header.stamp.sec * 1_000_000_000 + f.box_request.header.stamp.nanosec
    lidar_times = [m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for m in f.box_sent
                   if m.source == 'lidar' and m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec < req_ns]
    assert req_ns >= stop_ns + 50_000_000
    assert any(t >= stop_ns + 50_000_000 for t in lidar_times)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_handoff_does_not_use_pre_settle_lidar(fake):
    f = fake; camera_reach_lidar_dock(f); f.box_observation_gap = .099
    before = len(f.commands); f.wait(lambda: any(c.operation == TaskCommand.STOP for c in f.commands[before:]))
    f.box_observation_enabled = False; f.run_for(.08)
    assert f.box_request.source == 'lidar' and f.hold
    f.box_observation_enabled = True
    f.wait(lambda: f.box_request and f.box_request.source == 'uppercam')

@pytest.mark.parametrize('fake', ['bad_dock_lookahead_zero', 'bad_dock_lookahead_negative'], indirect=True)
def test_dock_lookahead_must_be_positive(fake):
    deadline = time.monotonic() + 5
    while fake.child.poll() is None and time.monotonic() < deadline:
        time.sleep(.02)
    assert fake.child.poll() not in (None, 0)
    fake.log.seek(0)
    assert 'box_approach.dock_lookahead must be finite and positive' in fake.log.read()

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_dock_pause_resume_preserves_tracking_reference(fake):
    f = fake; camera_reach_dock(f)
    dock = f.starts()[-1]; point = dock.path.poses[len(dock.path.poses)//2].pose.position
    n = len(f.commands); f.obstacles = [(point.x, point.y)]
    f.wait(lambda: any(c.operation == TaskCommand.UPDATE_PATH and c.speed == 0. for c in f.commands[n:]))
    f.obstacles = []
    f.wait(lambda: any(c.operation == TaskCommand.UPDATE_PATH and c.speed > 0. for c in f.commands[n:]))
    updates = [c for c in f.commands[n:] if c.operation == TaskCommand.UPDATE_PATH]
    for c in updates:
        assert abs(c.tracking_reference_x - .165) < 1e-9
        assert abs(c.lookahead_distance - .020) < 1e-9
        assert c.camera_distance_control and abs(c.camera_target_gap_m - .010) < 1e-9

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_preserves_lidar_line_and_updates_gap_stamp(fake):
    f = fake; f.box_camera_pose_shift = .1; camera_reach_dock(f)
    dock = f.starts()[-1]
    lidar = f.starts()[1].path.poses
    start, end = lidar[0].pose.position, lidar[-1].pose.position
    dx, dy = end.x-start.x, end.y-start.y
    for pose in dock.path.poses:
        p = pose.pose.position
        assert abs(dx*(p.y-start.y)-dy*(p.x-start.x)) / math.hypot(dx,dy) < 1e-6
    n = len(f.commands); f.box_observation_gap = .085
    f.wait(lambda: any(c.operation == TaskCommand.UPDATE_PATH and c.camera_distance_control and
                       abs(c.camera_gap_m - .085) < 1e-9 for c in f.commands[n:]))
    update = next(c for c in f.commands[n:] if c.operation == TaskCommand.UPDATE_PATH and
                  c.camera_distance_control and abs(c.camera_gap_m - .085) < 1e-9)
    assert update.path.poses[-1].pose.position == dock.path.poses[-1].pose.position
    assert (update.camera_observation_stamp.sec, update.camera_observation_stamp.nanosec) > \
           (dock.camera_observation_stamp.sec, dock.camera_observation_stamp.nanosec)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_entry_heading_correctable_but_final_heading_strict(fake):
    f = fake; camera_reach_lidar_dock(f)
    lidar_path = f.starts()[1].path.poses; a = lidar_path[0].pose.position; b = lidar_path[-1].pose.position
    dx, dy = b.x-a.x, b.y-a.y
    f.box_lidar_pose_shift = .015; f.box_hold_first_camera = True; f.box_observation_gap = .099
    f.position = (b.x, b.y); f.wait(lambda: f.box_request and f.box_request.source == 'uppercam')
    f.position = (f.position[0] - .01675*math.sin(f.yaw), f.position[1] + .01675*math.cos(f.yaw))
    f.yaw -= math.radians(3.228); f.box_observation_enabled = True
    f.wait(lambda: len(f.starts()) == 3)
    dock = f.starts()[-1]
    for pose in dock.path.poses:
        p = pose.pose.position
        assert abs(dx*(p.y-a.y)-dy*(p.x-a.x))/math.hypot(dx,dy) < 1e-6
    assert dock.camera_distance_control
    f.box_observation_gap = .010
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    f.log.seek(0); assert 'target gap 10 mm' in f.log.read()

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_above_operational_range_faults(fake):
    f = fake; camera_reach_dock(f); f.box_observation_gap = .201
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_box_handoff_and_final_confirm(fake):
    f = fake; camera_reach_dock(f)
    f.box_observation_gap = .01
    f.wait(lambda: f.box_request and f.box_request.source == 'off')
    f.wait(lambda: f.state == 'Run.AwaitRequest')
    assert [r.source for r in f.box_requests if r.source != 'off'] == ['lidar', 'uppercam', 'uppercam']
    assert len([c for c in f.commands if c.operation == TaskCommand.STOP]) >= 3

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_dropout_stops_and_latches(fake):
    f = fake; camera_reach_dock(f)
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    f.box_observation_enabled = False
    f.wait(lambda: f.state == 'Fault')
    f.wait(lambda: f.hold)
    f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)
    starts = len(f.starts()); f.box_observation_enabled = True; f.run_for(.4)
    assert f.state == 'Fault' and f.hold and len(f.starts()) == starts

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_handoff_disagreement_faults_before_motion(fake):
    f = fake; f.box_camera_offset = -.021
    camera_start_handoff(f)
    f.wait(lambda: f.state == 'Fault')
    assert len(f.starts()) == 2 and f.hold

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('fault', ['invalid', 'uncertainty'])
def test_camera_bad_observation_stops(fake, fault):
    f = fake; camera_reach_dock(f)
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    if fault == 'invalid': f.box_observation_valid = False
    if fault == 'extrapolated': f.box_observation_extrapolated = True
    if fault == 'uncertainty': f.box_observation_uncertainty = .011
    if fault == 'lateral': f.box_observation_lateral = .03
    if fault == 'yaw': f.box_observation_yaw = .1
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_lateral_is_diagnostic_but_lidar_line_odom_drift_faults(fake):
    f = fake; camera_reach_dock(f)
    f.box_observation_lateral = .056; f.box_observation_yaw = .1
    f.run_for(.25); assert f.state == 'Run.Execute' and not f.hold
    f.position = (f.position[0] - .04*math.sin(f.yaw), f.position[1] + .04*math.cos(f.yaw))
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('mode', ['repeated', 'future', 'stale'])
def test_camera_bad_stamp_stops(fake, mode):
    f = fake; camera_reach_dock(f)
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    if mode == 'stale': f.run_for(.7)  # selected stamp remains after request but is older than 500 ms
    f.box_observation_mode = mode
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_initial_warmup_waits_while_held(fake):
    f = fake; f.box_hold_first_camera = True
    camera_start_handoff(f)
    f.run_for(.6)
    assert f.state == 'Run.Execute' and f.hold and len(f.starts()) == 2
    f.box_observation_enabled = True
    f.wait(lambda: len(f.starts()) == 3)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_initial_observation_deadline_faults(fake):
    f = fake; f.box_hold_first_camera = True
    camera_start_handoff(f)
    f.wait(lambda: f.state == 'Fault', timeout=6)
    assert f.hold and len(f.starts()) == 2

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('gap', [.006, .014])
def test_camera_final_gap_outside_tolerance_faults(fake, gap):
    f = fake; f.camera_verify_gap_override = gap
    camera_reach_dock(f); f.box_observation_gap = .01
    f.wait(lambda: f.state == 'Fault')
    assert f.hold and len(f.starts()) == 3

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('offset', [.009, .01])
def test_camera_upper_operating_range_allows_release(fake, offset):
    f = fake; f.box_camera_offset = offset
    camera_reach_dock(f)
    assert f.state == 'Run.Execute' and len(f.starts()) == 3

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_extrapolated_observation_can_start_fine_dock(fake):
    f = fake; f.box_camera_offset = .011; f.box_observation_extrapolated = True
    camera_reach_dock(f)
    assert f.state == 'Run.Execute' and len(f.starts()) == 3

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_stops_at_upper_tolerance_edge(fake):
    f = fake; camera_reach_dock(f)
    f.box_observation_gap = .013
    f.wait(lambda: f.box_request and f.box_request.source == 'off')
    f.wait(lambda: f.state == 'Run.AwaitRequest')

@pytest.mark.parametrize('fake', ['bad_camera_range_lower', 'bad_camera_range_upper'], indirect=True)
def test_camera_range_config_cannot_expand_calibration(fake):
    deadline = time.monotonic() + 5
    while fake.child.poll() is None and time.monotonic() < deadline:
        time.sleep(.02)
    assert fake.child.poll() not in (None, 0)
    fake.log.seek(0)
    assert 'box camera gap limits must stay within the calibrated' in fake.log.read()

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_handoff_20mm_disagreement_boundary_allowed(fake):
    f = fake; f.box_camera_offset = -.02
    camera_reach_dock(f)
    assert f.state == 'Run.Execute' and len(f.starts()) == 3

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('gap, uncertainty, should_pass', [
    (.031, .010, True), (.031, .0101, False),
    (.030, .005, True), (.030, .0051, False)])
def test_camera_near_far_uncertainty_boundary(fake, gap, uncertainty, should_pass):
    f = fake; camera_reach_dock(f)
    f.box_observation_gap = gap; f.box_observation_uncertainty = uncertainty
    if should_pass:
        f.run_for(.5)
        assert f.state == 'Run.Execute' and not f.hold
    else:
        stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
        f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
        f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_first_selected_camera_sample_older_than_500ms_faults(fake):
    f = fake; f.box_hold_first_camera = True
    camera_start_handoff(f)
    f.run_for(.7)  # now - 550 ms is after the request, but already stale
    assert f.hold and len(f.starts()) == 2
    f.box_observation_mode = 'stale'; f.box_observation_enabled = True
    f.wait(lambda: f.state == 'Fault')
    assert f.hold and len(f.starts()) == 2

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_lidar_success_at_99mm_stops_before_camera_handoff(fake):
    f = fake; camera_reach_lidar_dock(f)
    f.box_observation_gap = .099
    f.wait(lambda: f.box_request and f.box_request.source == 'uppercam')
    f.wait(lambda: len(f.starts()) == 3)
    assert not f.rejected_while_held

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_lidar_success_at_111mm_faults_before_camera_handoff(fake):
    f = fake; camera_reach_lidar_dock(f)
    f.box_observation_gap = .111; f.run_for(.15)
    f.result_status = TaskResult.SUCCEEDED
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert len(f.starts()) == 2
    assert all(r.source != 'uppercam' for r in f.box_requests)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_lidar_success_at_103mm_stops_for_camera_handoff(fake):
    f = fake; camera_reach_lidar_dock(f)
    f.box_observation_gap = .103; f.run_for(.15); f.result_status = TaskResult.SUCCEEDED
    f.wait(lambda: f.box_request and f.box_request.source == 'uppercam')
    f.wait(lambda: len(f.starts()) == 3)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_stop_rejects_delayed_pre_stop_ack(fake):
    f = fake; f.stale_stop_ack = True
    f.position = (.5, 2.5); f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    pose = f.starts()[0].path.poses[-1].pose; e = pose.position; q = pose.orientation
    f.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    f.position = (e.x, e.y); f.result_status = TaskResult.SUCCEEDED
    f.wait(lambda: f.stale_ack_stamp is not None)
    f.run_for(.2)
    assert f.hold and not f.box_requests and len(f.starts()) == 1
    f.stale_stop_ack = False
    f.wait(lambda: f.box_request and f.box_request.source == 'lidar')

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_verify_ignores_frames_acquired_before_settle(fake):
    f = fake; f.hold_verify_observations = True
    camera_reach_dock(f); f.box_observation_gap = .01
    f.wait(lambda: sum(r.source == 'uppercam' for r in f.box_requests) == 2)
    request = f.box_request
    f.run_for(.08)  # receive old frames after the 50 ms settle period
    for offset in (10_000_000, 20_000_000, 30_000_000):
        ns = request.header.stamp.sec * 1_000_000_000 + request.header.stamp.nanosec + offset
        m = BoxObservation(); m.header.frame_id = 'map'
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
        m.request_id = request.request_id; m.source = 'uppercam'; m.valid = True
        m.box_pose = request.expected_box; m.gap_m = .01; m.gap_uncertainty_m = .002
        f.pubs['/perception/box/observation'].publish(m)
        f.run_for(.025)
    assert f.state == 'Run.Execute' and f.hold
    f.box_observation_enabled = True
    f.wait(lambda: f.box_request and f.box_request.source == 'off')
    f.wait(lambda: f.state == 'Run.AwaitRequest')

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_lidar_observation_stale_while_waiting_stop_ack_faults(fake):
    f = fake; camera_reach_lidar_dock(f)
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    f.stale_stop_ack = True; f.box_observation_gap = .07
    f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)
    f.box_observation_enabled = False
    f.wait(lambda: f.state == 'Fault', timeout=1)
    f.log.seek(0)
    assert 'box observation stale while stopping' in f.log.read()

def publish_aged_camera_observation(f, age, valid=True):
    request = f.box_request
    now = f.node.get_clock().now().nanoseconds
    ns = now - round(age * 1_000_000_000)
    requested = request.header.stamp.sec * 1_000_000_000 + request.header.stamp.nanosec
    assert ns > requested
    m = BoxObservation(); m.header.frame_id = 'map'
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    m.request_id = request.request_id; m.source = 'uppercam'; m.valid = valid
    m.box_pose = request.expected_box; m.gap_m = .099; m.gap_uncertainty_m = .002
    f.pubs['/perception/box/observation'].publish(m)

@pytest.mark.parametrize('fake', ['camera_500'], indirect=True)
def test_camera_500_accepts_450ms_selected_sample(fake):
    f = fake; f.box_hold_first_camera = True
    camera_start_handoff(f); f.run_for(.65)
    assert f.hold and len(f.starts()) == 2
    publish_aged_camera_observation(f, .45)
    f.box_observation_enabled = True  # continuing new observations keep the stream fresh
    f.wait(lambda: len(f.starts()) == 3)
    assert f.state != 'Fault'

@pytest.mark.parametrize('fake', ['camera_500'], indirect=True)
def test_camera_500_rejects_550ms_selected_sample(fake):
    f = fake; f.box_hold_first_camera = True
    camera_start_handoff(f); f.run_for(.75)
    publish_aged_camera_observation(f, .55)
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert len(f.starts()) == 2

@pytest.mark.parametrize('fake', ['camera_500'], indirect=True)
def test_camera_500_invalid_sample_faults_without_timeout(fake):
    f = fake; f.box_hold_first_camera = True
    camera_start_handoff(f); f.run_for(.05)
    begin = time.monotonic(); publish_aged_camera_observation(f, .001, valid=False)
    f.wait(lambda: f.state == 'Fault', timeout=.4); f.wait(lambda: f.hold)
    assert time.monotonic() - begin < .5 and len(f.starts()) == 2

def goal_yaw(command):
    q = command.path.poses[-1].pose.orientation
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('position', [(.5, .5), (2.93, 2.5)])
def test_camera_stage_last_goal_faces_box_before_perception(fake, position):
    f = fake; f.position = position
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    c = f.starts()[0]; assert c.require_goal_heading and c.tolerance_rad > 0
    end = c.path.poses[-1].pose.position
    bearing = math.atan2(2.5 - end.y, 2.5 - end.x)
    assert abs(math.remainder(goal_yaw(c) - bearing, 2*math.pi)) < 1e-6
    f.position = (end.x, end.y); f.yaw = goal_yaw(c) + math.pi
    f.run_for(.2)  # target position reached but controller has not reported heading completion
    assert not f.box_requests and len(f.starts()) == 1
    f.yaw = goal_yaw(c); f.result_status = TaskResult.SUCCEEDED
    f.wait(lambda: f.box_request and f.box_request.source == 'lidar')

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_stage_opposite_heading_never_perceives(fake):
    f = fake; f.position = (2.93, 2.5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    c = f.starts()[0]; end = c.path.poses[-1].pose.position
    assert c.require_goal_heading
    f.position = (end.x, end.y); f.yaw = goal_yaw(c) + math.pi
    f.result_status = TaskResult.SUCCEEDED  # corrupted or mistaken success feedback
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert len(f.starts()) == 1 and not f.box_requests

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_stage_box_corners_outside_front_lidar_fov_faults(fake):
    f = fake; f.position = (.5, 2.5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    c = f.starts()[0]; f.yaw = goal_yaw(c); f.position = (2.5, 2.5)
    f.result_status = TaskResult.SUCCEEDED
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    assert not f.box_requests
    f.log.seek(0); assert 'box corners outside front lidar field of view' in f.log.read()

@pytest.mark.parametrize('fake', ['camera_segmented'], indirect=True)
def test_camera_stage_intermediate_segments_do_not_force_heading(fake):
    f = fake; f.position = (.5, .5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) >= 1)
    for _ in range(16):
        c = f.starts()[-1]
        if c.require_goal_heading:
            break
        assert not f.box_requests
        end = c.path.poses[-1].pose.position
        f.position = (end.x, end.y); f.result_status = TaskResult.SUCCEEDED
        count = len(f.starts())
        f.wait(lambda: len(f.starts()) > count, timeout=8)
    else:
        pytest.fail('final segment was not sent')
    c = f.starts()[-1]
    assert len(f.starts()) > 1 and c.require_goal_heading
    end = c.path.poses[-1].pose.position
    bearing = math.atan2(2.5 - end.y, 2.5 - end.x)
    assert abs(math.remainder(goal_yaw(c) - bearing, 2*math.pi)) < 1e-6

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_stage_pause_resume_keeps_face_heading(fake):
    f = fake; f.position = (.5, 2.5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    c = f.starts()[0]; assert c.require_goal_heading
    mid = c.path.poses[len(c.path.poses)//2].pose.position
    f.obstacles = [(mid.x, mid.y)]; n = len(f.commands)
    f.wait(lambda: any(x.operation == TaskCommand.UPDATE_PATH and x.speed == 0. for x in f.commands[n:]))
    f.obstacles = []
    f.wait(lambda: any(x.operation == TaskCommand.UPDATE_PATH and x.speed > 0. for x in f.commands[n:]))
    pause = next(x for x in f.commands[n:] if x.operation == TaskCommand.UPDATE_PATH and x.speed == 0.)
    resume = next(x for x in f.commands[n:] if x.operation == TaskCommand.UPDATE_PATH and x.speed > 0.)
    assert pause.require_goal_heading and resume.require_goal_heading
    assert pause.tolerance_rad == c.tolerance_rad == resume.tolerance_rad
    assert abs(math.remainder(goal_yaw(pause) - goal_yaw(c), 2*math.pi)) < 1e-9
    assert abs(math.remainder(goal_yaw(resume) - goal_yaw(c), 2*math.pi)) < 1e-9
    assert not f.box_requests

def camera_startup_miss(f):
    f.box_hold_first_camera = True
    camera_start_handoff(f)
    f.box_observation_valid = False
    f.box_observation_reason = 'uppercam_geometry_not_detected'
    f.box_observation_enabled = True

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_startup_geometry_miss_waits_then_valid_releases(fake):
    f = fake; camera_startup_miss(f)
    f.run_for(.7)
    assert f.hold and f.state == 'Run.Execute' and len(f.starts()) == 2
    f.box_observation_valid = True; f.box_observation_reason = ''
    f.wait(lambda: len(f.starts()) == 3)
    assert not f.rejected_while_held

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_startup_geometry_miss_fixed_deadline_faults(fake):
    f = fake; camera_startup_miss(f)
    f.run_for(4.4)
    assert f.hold and f.state == 'Run.Execute' and len(f.starts()) == 2
    f.wait(lambda: f.state == 'Fault', timeout=2); f.wait(lambda: f.hold)
    f.box_observation_valid = True; f.box_observation_reason = ''
    f.run_for(.25)
    assert len(f.starts()) == 2 and f.state == 'Fault'

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
@pytest.mark.parametrize('fault', ['other_reason', 'extrapolated', 'future', 'stale', 'unheld', 'at_request'])
def test_camera_startup_only_fresh_held_geometry_miss_waits(fake, fault):
    f = fake; camera_startup_miss(f)
    f.box_observation_enabled = False
    f.run_for(.15)
    if fault == 'other_reason': f.box_observation_reason = 'camera_exception'
    if fault == 'extrapolated': f.box_observation_extrapolated = True
    if fault == 'future': f.box_observation_mode = 'future'
    if fault == 'at_request': f.box_observation_mode = 'at_request'
    if fault == 'stale':
        f.run_for(.6)  # stale stamp remains newer than request
        f.box_observation_mode = 'stale'
    if fault == 'unheld':
        f.force_control_unheld = True; f.run_for(.12)
    f.box_observation_enabled = True
    f.wait(lambda: f.state == 'Fault', timeout=2)
    assert len(f.starts()) == 2

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_startup_geometry_miss_after_first_valid_faults(fake):
    f = fake; camera_reach_dock(f)
    f.box_observation_valid = False
    f.box_observation_reason = 'uppercam_geometry_not_detected'
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_verify_geometry_miss_never_gets_startup_grace(fake):
    f = fake; f.hold_verify_observations = True
    camera_reach_dock(f); f.box_observation_gap = .01
    f.wait(lambda: sum(r.source == 'uppercam' for r in f.box_requests) == 2)
    f.box_observation_valid = False
    f.box_observation_reason = 'uppercam_geometry_not_detected'
    f.box_observation_enabled = True
    f.wait(lambda: f.state == 'Fault', timeout=2)
    assert f.hold and len(f.starts()) == 3

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_startup_first_valid_near_deadline_still_requires_full_validation(fake):
    f = fake; camera_startup_miss(f)
    f.run_for(4.5)
    assert f.hold and f.state == 'Run.Execute' and len(f.starts()) == 2
    f.box_observation_valid = True; f.box_observation_reason = ''
    f.wait(lambda: len(f.starts()) == 3, timeout=1)
    assert not f.rejected_while_held

@pytest.mark.parametrize('fake', ['camera'], indirect=True)
def test_camera_startup_unrelated_request_cannot_unlock_or_fault(fake):
    f = fake; f.box_hold_first_camera = True; camera_start_handoff(f)
    m = BoxObservation(); m.header.frame_id = 'map'; m.header.stamp = f.node.get_clock().now().to_msg()
    m.request_id = f.box_request.request_id - 1; m.source = 'uppercam'
    m.valid = False; m.reason = 'uppercam_geometry_not_detected'
    f.pubs['/perception/box/observation'].publish(m)
    f.run_for(.25)
    assert f.hold and f.state == 'Run.Execute' and len(f.starts()) == 2


def camera_only_reach_dock(f):
    """camera_only_longitudinal: stage -> LiDAR line -> one dock along it, camera requested at the dock start."""
    f.position=(.5,2.5)
    f.pubs['/planning/approach_box'].publish(String(data='B'))
    f.wait(lambda: len(f.starts()) == 1)
    end_pose = f.starts()[0].path.poses[-1].pose; end = end_pose.position
    q = end_pose.orientation; f.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    f.box_hold_first_camera = True  # the camera sees nothing until the test says so (box out of its range)
    f.position=(end.x,end.y); f.result_status=TaskResult.SUCCEEDED
    f.wait(lambda: len(f.starts()) == 2)
    dock = f.starts()[1]
    assert not dock.camera_distance_control and abs(dock.tracking_reference_x - .165) < 1e-9
    assert max(dock.speed_limit) <= .02 + 1e-9
    assert [r.source for r in f.box_requests] == ['lidar', 'uppercam']
    return dock

@pytest.mark.parametrize('fake', ['camera_only'], indirect=True)
def test_camera_only_follows_lidar_line_and_upper_cam_ends(fake):
    f = fake; dock = camera_only_reach_dock(f)
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    f.run_for(.4)  # no camera gap yet, far from the box: keeps following, no stop, no fault
    assert f.state == 'Run.Execute' and not f.hold
    assert sum(c.operation == TaskCommand.STOP for c in f.commands) == stops
    f.box_observation_gap = .05; f.box_observation_enabled = True
    f.wait(lambda: any(c.operation == TaskCommand.UPDATE_PATH and c.camera_distance_control and c.task_id == dock.task_id
                       for c in f.commands))
    assert len(f.starts()) == 2  # same task: control switches to camera distance control mid-dock
    f.box_observation_gap = .01
    f.wait(lambda: f.box_request and f.box_request.source == 'off')
    f.wait(lambda: f.state == 'Run.AwaitRequest')
    assert [r.source for r in f.box_requests if r.source != 'off'] == ['lidar', 'uppercam', 'uppercam']

@pytest.mark.parametrize('fake', ['camera_only'], indirect=True)
def test_camera_only_no_camera_gap_by_handoff_stops(fake):
    f = fake; dock = camera_only_reach_dock(f)
    stops = sum(c.operation == TaskCommand.STOP for c in f.commands)
    end = dock.path.poses[-1].pose.position; f.position = (end.x, end.y)  # LiDAR line end: 10 mm, no camera gap
    f.wait(lambda: f.state == 'Fault'); f.wait(lambda: f.hold)
    f.log.seek(0); log = f.log.read()
    assert 'no upper cam gap by' in log, log[-2000:]
    f.wait(lambda: sum(c.operation == TaskCommand.STOP for c in f.commands) > stops)
    assert not any(c.camera_distance_control for c in f.commands)

@pytest.mark.parametrize('fake', ['camera_only'], indirect=True)
def test_camera_only_ignores_lidar_gap_at_the_stage(fake):
    f = fake; f.box_observation_gap = .09  # LiDAR says already inside handoff_gap: the old flow failed here
    camera_only_reach_dock(f)
    assert f.state == 'Run.Execute'
