#!/usr/bin/env python3
"""시나리오 하나를 실행한다: 가짜 보드·센서(sim_node) + 실제 control_master·planning_master 를 띄우고,
run_corner.py 와 같은 순서(RESET → START → 목표)로 요청을 보내며 모든 것을 기록한 뒤 지표 HTML 을 만든다.

환경(ROS_DOMAIN_ID 등)은 sim.sh 가 맞춘다. 직접 부를 때도 sim.sh 를 거치는 것을 권장.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String

from craft_interfaces.msg import ControlStatus, TaskCommand, TaskResult

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from craft_sim import report  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
TERMINAL = ('Fault',)


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class Recorder(Node):
    def __init__(self):
        super().__init__('craft_sim_runner')
        self.rec = {'truth': [], 'state': [], 'box_phase': [], 'box_status': [], 'box_alignment': [],
                    'paths': [], 'lookahead': [], 'commands': [], 'results': [], 'control': [], 'events': [],
                    'approach_goal': [], 'actions': []}
        self.state = None
        self.complete_seen = False
        self.last_control = None
        self.create_subscription(String, '/planning/state', self.on_state, LATCHED)
        self.create_subscription(String, '/planning/box_phase', self.on_phase, LATCHED)
        self.create_subscription(String, '/planning/box_status', lambda m: self.add('box_status', text=m.data), 10)
        self.create_subscription(Float64MultiArray, '/planning/box_alignment',
                                 lambda m: self.add('box_alignment', v=list(m.data)), 10)
        self.create_subscription(Odometry, '/sim/truth', self.on_truth, 50)
        self.create_subscription(PathMsg, '/control/tracking_path', self.on_path, 10)
        self.create_subscription(PointStamped, '/control/lookahead_point',
                                 lambda m: self.add('lookahead', x=m.point.x, y=m.point.y), 50)
        self.create_subscription(TaskCommand, '/planning/task_command', self.on_command, 50)
        self.create_subscription(TaskResult, '/control/task_result', self.on_result, 50)
        self.create_subscription(ControlStatus, '/control/status', self.on_control, 50)
        self.create_subscription(String, '/sim/events', self.on_event, 50)
        self.create_subscription(PoseStamped, '/planning/approach_goal', self.on_goal, LATCHED)
        self.pub_cmd = self.create_publisher(String, '/planning/command', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/planning/target_base_pose', 10)
        self.pub_box = self.create_publisher(String, '/planning/approach_box', 10)
        self.pub_box_near = self.create_publisher(String, '/planning/approach_box_near', 10)

    def add(self, key, **kw):
        kw.setdefault('t', time.time())
        self.rec[key].append(kw)

    def on_state(self, m):
        if m.data != self.state:
            self.add('state', v=m.data)
        self.state = m.data
        if m.data == 'Run.Complete':
            self.complete_seen = True

    def on_phase(self, m):
        if not self.rec['box_phase'] or self.rec['box_phase'][-1]['v'] != m.data:
            self.add('box_phase', v=m.data)

    def on_truth(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.rec['truth'].append({'t': stamp_s(m.header.stamp), 'x': m.pose.pose.position.x,
                                  'y': m.pose.pose.position.y, 'yaw': yaw,
                                  'v': m.twist.twist.linear.x, 'w': m.twist.twist.angular.z})

    def on_path(self, m):
        self.add('paths', pts=[(p.pose.position.x, p.pose.position.y) for p in m.poses])

    def on_command(self, m):
        op = {0: 'EXECUTE', 1: 'UPDATE_PATH', 2: 'STOP'}.get(m.operation, str(m.operation))
        self.add('commands', op=op, task_id=int(m.task_id), revision=int(m.revision), speed=m.speed,
                 n=len(m.path.poses), goal_tol=m.goal_tolerance, ref_x=m.tracking_reference_x,
                 lookahead=m.lookahead_distance, camera=m.camera_distance_control, camera_gap=m.camera_gap_m)

    def on_result(self, m):
        last = self.rec['results'][-1] if self.rec['results'] else None
        if not last or (last['task_id'], last['status'], last['reason']) != (int(m.task_id), int(m.status), m.reason):
            self.add('results', task_id=int(m.task_id), status=int(m.status), reason=m.reason)

    def on_control(self, m):
        cur = (m.ready, m.fault, m.held, m.stopped, m.reason)
        if self.last_control != cur:
            self.add('control', ready=m.ready, fault=m.fault, held=m.held, stopped=m.stopped, reason=m.reason)
        self.last_control = cur

    def on_event(self, m):
        try:
            self.rec['events'].append(json.loads(m.data))
        except ValueError:
            self.add('events', text=m.data)

    def on_goal(self, m):
        q = m.pose.orientation
        self.add('approach_goal', x=m.pose.position.x, y=m.pose.position.y,
                 yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def spin_for(self, seconds, until=None):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until and until():
                return True
        return bool(until and until())

    def command(self, text):
        self.pub_cmd.publish(String(data=text))
        self.add('actions', text=f'명령 {text}')


def start(cmd, log, env=None):
    with open(log, 'w') as f:  # 자식이 fd 를 물려받으므로 여기서는 닫아도 된다
        return subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT, env=env, start_new_session=True)


def stop_all(procs):
    for p in procs:
        if p and p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
    deadline = time.time() + 3
    for p in procs:
        while p and p.poll() is None and time.time() < deadline:
            time.sleep(0.05)
    for p in procs:
        if p and p.poll() is None:
            try:
                os.killpg(p.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def resolve(path, *bases: Path):
    """절대 경로면 그대로, 아니면 bases 순서로 찾아 처음 있는 것(없으면 첫 base 기준)."""
    p = Path(str(path)).expanduser()
    if p.is_absolute():
        return p
    for b in bases:
        if (b / p).exists():
            return (b / p).resolve()
    return (bases[0] / p).resolve()


def package_share() -> Path:
    """시나리오·맵·RViz 설정 위치: 설치된 share/craft_sim, 없으면 소스 패키지 폴더."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('craft_sim'))
    except Exception:
        return HERE.parent


def find_craft_src() -> Path:
    """로봇 yaml(planning.yaml, control.yaml, rates.yaml)이 있는 src 폴더: CRAFT_SIM_SRC, 아니면 이 파일 위쪽에서 찾음.
    symlink 설치면 저장소 src, 일반 설치면 작업공간 루트의 src 가 잡힌다."""
    env = os.environ.get('CRAFT_SIM_SRC')
    if env:
        return Path(env).expanduser().resolve()
    for d in [HERE.resolve()] + list(HERE.resolve().parents):
        for cand in (d, d / 'src'):
            if (cand / 'craft_planning/config/planning.yaml').exists():
                return cand
    sys.exit('[sim] craft 소스(src/craft_planning/config/planning.yaml)를 못 찾음: CRAFT_SIM_SRC=<저장소>/src 로 지정')


def node_binary(pkg: str, exe: str, ws: str | None) -> str:
    if ws:
        return str(Path(ws) / 'install' / pkg / 'lib' / pkg / exe)
    try:
        from ament_index_python.packages import get_package_prefix
        return str(Path(get_package_prefix(pkg)) / 'lib' / pkg / exe)
    except Exception:
        sys.exit(f'[sim] {pkg} 가 설치돼 있지 않음: 작업공간 install/setup.bash 를 source 했는지 확인')


def run_action(r: Recorder, act: dict, timeout: float):
    """요청 하나: Ready/AwaitRequest 확인 → 요청 → Run.Complete / Fault / 시간 초과."""
    t_prep = time.time()

    def early(result):
        now = time.time()
        return {'type': act['type'], 'result': result, 'state': r.state, 't_start': t_prep, 't_end': now,
                'elapsed_s': now - t_prep, 'action': act}
    if r.state == 'Fault':
        r.command('RESET')
        r.spin_for(3.0, lambda: r.state != 'Fault')
    if not r.spin_for(20, lambda: r.state in ('Ready', 'Run.AwaitRequest')):
        return early('not_ready')
    if r.state == 'Ready':
        r.command('START')
        if not r.spin_for(5, lambda: r.state == 'Run.AwaitRequest'):
            return early('not_started')
    r.complete_seen = False
    t0 = time.time()
    typ = act['type']
    if typ == 'goal':
        x, y = act['pose'][:2]
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = r.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = float(x), float(y)
        if len(act['pose']) > 2:
            yaw = math.radians(float(act['pose'][2]))
        else:
            tx = r.rec['truth'][-1] if r.rec['truth'] else {'x': 0.0, 'y': 0.0}
            yaw = math.atan2(y - tx['y'], x - tx['x'])  # run_corner 와 같이 진행 방향(요구 안 함)
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        r.pub_goal.publish(g)
        r.add('actions', text=f'목표 ({x:.3f}, {y:.3f})', type='goal', x=float(x), y=float(y), yaw=yaw)
    elif typ in ('box', 'box_near'):
        (r.pub_box if typ == 'box' else r.pub_box_near).publish(String(data=act['name']))
        r.add('actions', text=f'박스 {act["name"]} ({typ})', type=typ, name=act['name'])
    else:
        raise ValueError(f'알 수 없는 action type: {typ}')
    ok = r.spin_for(timeout, lambda: r.complete_seen or r.state in TERMINAL)
    out = {'type': typ, 't_start': t0, 't_end': time.time(), 'elapsed_s': time.time() - t0, 'state': r.state,
           'action': act}
    if r.complete_seen:
        out['result'] = 'complete'
    elif r.state == 'Fault':
        out['result'] = 'fault'
    else:
        out['result'] = 'timeout' if not ok else r.state
        r.command('STOP')
        r.spin_for(1.0)
    r.spin_for(0.5)  # 정지 뒤 자세 기록
    return out


def main():
    share = package_share()
    ap = argparse.ArgumentParser(description='craft 2D 시뮬레이터 시나리오 실행')
    ap.add_argument('scenario', help='시나리오 yaml 경로, 또는 share/craft_sim/scenarios 안의 파일 이름(예: 02_box_L1)')
    ap.add_argument('--craft-src', default=None, help='로봇 yaml 이 있는 src 폴더 (기본: CRAFT_SIM_SRC 또는 자동 탐색)')
    ap.add_argument('--ws', default=None, help='master 바이너리를 찾을 colcon 작업공간 (기본: 현재 source 된 환경)')
    ap.add_argument('--out', default=os.environ.get('CRAFT_SIM_OUT', str(Path.home() / 'craft_sim_runs')))
    ap.add_argument('--rviz', action='store_true', help='RViz 함께 띄우기')
    ap.add_argument('--keep', action='store_true', help='끝나도 노드를 내리지 않고 Enter 를 기다림')
    a = ap.parse_args()

    sc_path = Path(a.scenario).expanduser()
    if not sc_path.exists():
        name = a.scenario if a.scenario.endswith('.yaml') else a.scenario + '.yaml'
        sc_path = share / 'scenarios' / name
    if not sc_path.exists():
        sys.exit(f'[sim] 시나리오를 찾을 수 없음: {a.scenario}')
    sc_path = sc_path.resolve()
    sc = yaml.safe_load(sc_path.read_text())
    craft_src = Path(a.craft_src).resolve() if a.craft_src else find_craft_src()
    name = sc.get('name', sc_path.stem)
    run_dir = Path(a.out) / f'{name}_{time.strftime("%Y%m%d_%H%M%S")}'
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / 'scenario.yaml').write_text(sc_path.read_text())
    map_yaml = resolve(sc.get('map', 'maps/arena5_legend_opt_arena1cm.yaml'), sc_path.parent.parent, share, Path.cwd())

    # 노드 파라미터: 로봇과 같은 yaml 두 개 + 시뮬레이터 덮어쓰기 (launch 에서 주는 값과 같은 것)
    pty_file = run_dir / 'pty.txt'
    procs = []
    print(f'[sim] 실행 폴더: {run_dir}')
    sim = start([sys.executable, '-u', str(HERE / 'sim_node.py'), '--scenario', str(sc_path), '--craft-src',
                 str(craft_src), '--map', str(map_yaml), '--pty-out', str(pty_file)], run_dir / 'sim.log')
    procs.append(sim)
    for _ in range(100):
        if pty_file.exists() and pty_file.read_text().strip():
            break
        time.sleep(0.05)
    else:
        stop_all(procs)
        sys.exit('[sim] sim_node 가 PTY 를 만들지 못함: sim.log 확인')
    port = pty_file.read_text().strip()
    overrides = {
        'control_master': {'ros__parameters': {'clock_wheel_port': port, 'arm_enabled': False,
                                               **(sc.get('params', {}) or {}).get('control', {})}},
        'planning_master': {'ros__parameters': {'map_file': str(map_yaml), 'require_arm_joints': False,
                                                'require_lidar_localization': True,
                                                **(sc.get('params', {}) or {}).get('planning', {})}},
    }
    ov = run_dir / 'overrides.yaml'
    ov.write_text(yaml.safe_dump(overrides, allow_unicode=True, sort_keys=False))
    rates = craft_src / 'craft_bringup/config/rates.yaml'
    control_bin = node_binary('craft_control', 'control_master', a.ws)
    planning_bin = node_binary('craft_planning', 'planning_master', a.ws)
    procs.append(start([str(control_bin), '--ros-args', '--params-file', str(craft_src / 'craft_control/config/control.yaml'),
                        '--params-file', str(rates), '--params-file', str(ov)], run_dir / 'control.log'))
    procs.append(start([str(planning_bin), '--ros-args', '--params-file',
                        str(craft_src / 'craft_planning/config/planning.yaml'), '--params-file', str(rates),
                        '--params-file', str(ov)], run_dir / 'planning.log'))
    if a.rviz:
        procs.append(start(['rviz2', '-d', str(share / 'rviz' / 'sim.rviz')], run_dir / 'rviz.log'))

    rclpy.init()
    r = Recorder()
    t_begin = time.time()
    results = []
    interrupted = False

    def on_sig(*_):
        nonlocal interrupted
        interrupted = True
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_sig)
    try:
        r.spin_for(float(sc.get('settle_s', 2.0)))
        dead = [n for n, p in zip(('sim_node', 'control_master', 'planning_master'), procs) if p.poll() is not None]
        if dead:
            raise RuntimeError(f'{", ".join(dead)} 가 시작 직후 종료됨: {run_dir} 의 *.log 확인')
        # 같은 도메인에서 다른 프로그램(bag 재생 등)이 같은 topic 을 내면 결과가 오염된다.
        ours = {'craft_sim', 'craft_sim_runner', 'control_master', 'planning_master'}
        foreign = sorted({i.node_name for t in ('/tf', '/lidar/merged', '/perception/state/odom', '/control/status')
                          for i in r.get_publishers_info_by_topic(t) if i.node_name not in ours})
        if foreign:
            raise RuntimeError(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")} 에 다른 발행자가 있음: {foreign}. '
                               'CRAFT_SIM_DOMAIN=<다른 번호> 로 다시 실행')
        for i, act in enumerate(sc.get('actions', [])):
            print(f'[sim] 요청 {i + 1}: {act}')
            res = run_action(r, act, float(act.get('timeout_s', sc.get('timeout_s', 180))))
            res['expect'] = act.get('expect', 'complete')
            res['pass'] = res['result'] == res['expect']
            results.append(res)
            print(f'[sim]   → {res["result"]} ({res.get("elapsed_s", 0):.1f}s, 기대 {res["expect"]})')
            if not res['pass']:
                break
        if a.keep:
            input('[sim] Enter 를 누르면 종료합니다 ...')
    except KeyboardInterrupt:
        interrupted = True
        print('\n[sim] 중단: STOP 보내고 정리')
    except RuntimeError as e:
        print(f'[sim] {e}')
    finally:
        try:
            r.command('STOP')
            r.spin_for(0.3)
        except Exception:
            pass
        rec = r.rec
        r.destroy_node()
        rclpy.try_shutdown()
        stop_all(procs[::-1])

    summary = {'name': name, 'scenario': str(sc_path), 'run_dir': str(run_dir), 'map': str(map_yaml),
               'craft_src': str(craft_src), 't_begin': t_begin, 'interrupted': interrupted, 'actions': results}
    (run_dir / 'record.json').write_text(json.dumps(rec))
    try:
        metrics = report.build(run_dir, sc, summary, rec, craft_src, map_yaml)
    except Exception as e:  # 보고서 실패가 기록을 잃게 하지 않도록
        import traceback
        traceback.print_exc()
        metrics = {'report_error': repr(e)}
    summary['metrics'] = metrics
    summary['pass'] = bool(results) and all(x['pass'] for x in results) and len(results) == len(sc.get('actions', []))
    (run_dir / 'result.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1))
    print(f'[sim] 결과: {"통과" if summary["pass"] else "실패"}  보고서: {run_dir / "report.html"}')
    sys.exit(0 if summary['pass'] else 1)


if __name__ == '__main__':
    main()
