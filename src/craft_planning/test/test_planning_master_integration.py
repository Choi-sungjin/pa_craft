"""Real PlanningMaster, synthetic ControlMaster and sensors; isolated ROS domain."""
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
from craft_interfaces.msg import TaskCommand, TaskResult, ControlStatus
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
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
        self.current = None
        self.result_status = TaskResult.RUNNING
        self.result_id = None
        self.obstacles = []
        self.position = (0.5, 0.5)
        self.states = []
        self.pubs = {}
        for topic, cls in [('/control/status', ControlStatus), ('/control/task_result', TaskResult),
                           ('/perception/state/odom', Odometry), ('/control/arm_joints', JointState),
                           ('/perception/lidar/obstacle', PoseArray), ('/planning/command', String),
                           ('/planning/mission', String), ('/planning/target_base_pose', PoseStamped),
                           ('/planning/target_tcp_pose', PoseStamped), ('/perception/vision/object_pose', PoseStamped),
                           ('/perception/vision/d435/apriltag', Int32MultiArray)]:
            self.pubs[topic] = node.create_publisher(cls, topic, 10)
        node.create_subscription(String, '/planning/state', self.on_state, 10)
        node.create_subscription(Bool, '/planning/hold', lambda m: setattr(self, 'hold', m.data), 10)
        node.create_subscription(TaskCommand, '/planning/task_command', self.on_command, 10)

    def on_state(self, m):
        self.state = m.data
        self.states.append(m.data)

    def on_command(self, m):
        self.commands.append(m)
        if m.operation == TaskCommand.EXECUTE:
            self.current = m
            self.result_status = TaskResult.RUNNING
            self.result_id = None
        elif m.operation == TaskCommand.UPDATE_PATH:
            self.current = m
        elif m.operation == TaskCommand.STOP and self.allow_stop:
            self.result_status = TaskResult.STOPPED

    def publish(self):
        stamp = self.node.get_clock().now().to_msg()
        if self.feedback:
            c = ControlStatus(ready=True, held=self.hold, stopped=self.hold and self.allow_stop)
            c.header.stamp = stamp
            self.pubs['/control/status'].publish(c)
            if self.current:
                r = TaskResult(task_id=self.result_id or self.current.task_id,
                               revision=self.current.revision, status=self.result_status)
                if self.auto_complete: r.status = TaskResult.SUCCEEDED
                r.header.stamp = stamp
                self.pubs['/control/task_result'].publish(r)
        o = Odometry()
        o.header.stamp, o.header.frame_id = stamp, 'map'
        o.pose.pose.position.x, o.pose.pose.position.y = self.position
        o.pose.pose.orientation.w = 1.0
        self.pubs['/perception/state/odom'].publish(o)
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
    monkeypatch.setenv('ROS_DOMAIN_ID',str(50+os.getpid()%150))
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
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
        'fsm.execution_timeout_s':10., 'replan_period_s':.1}}}))
    context = Context(); rclpy.init(context=context)
    node = rclpy.create_node('fake_control',context=context)
    binary = Path(get_package_prefix('craft_planning'))/'lib/craft_planning/planning_master'
    config = Path(get_package_share_directory('craft_planning'))/'config/planning.yaml'
    with (tmp_path/'master.log').open('w+') as log:
        child = subprocess.Popen([str(binary),'--ros-args','--params-file',str(config),
                                  '--params-file',str(tmp_path/'override.yaml')],stdout=log,stderr=log)
        f = Fake(node,child,log)
        boards = []
        control_child = None
        if getattr(request, 'param', None) == 'real':
            import importlib.util
            helper = Path(__file__).resolve().parents[2] / 'craft_control/test/test_planning_watchdog_integration.py'
            spec = importlib.util.spec_from_file_location('pty_helpers', helper)
            module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
            boards = [module.FakeBoard([0.,0.,0.,0.,0.,.2,0,0]), module.FakeBoard([0.,0.,0.,0,0,1,0])]
            control_binary = Path(get_package_prefix('craft_control'))/'lib/craft_control/control_master'
            control_config = Path(get_package_share_directory('craft_control'))/'config/control.yaml'
            args = [str(control_binary),'--ros-args','--params-file',str(control_config)]
            for key, value in {'arm_lm_port':boards[0].port,'clock_wheel_port':boards[1].port,
                               'arm_ik_mapping_confirmed':'true','lm_mapping_confirmed':'true',
                               'gripper_mapping_confirmed':'true','lm_pulley_radius':'1.0',
                               'lm_gear_ratio':'1.0','gripper_open_angle':'0.2',
                               'gripper_close_angle':'1.2'}.items():
                args.extend(['-p',f'{key}:={value}'])
            control_child = subprocess.Popen(args,stdout=log,stderr=log)
            f.feedback = False
        try:
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
    f.wait(lambda: any(c.operation==TaskCommand.UPDATE_PATH for c in f.commands))
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

def test_control_feedback_timeout(fake):
    f=fake; f.goal(); f.wait(lambda:f.state=='Run.Execute')
    f.feedback=False; f.wait(lambda:f.state=='Fault',2)

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
