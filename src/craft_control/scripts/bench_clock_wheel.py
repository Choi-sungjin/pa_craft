#!/usr/bin/env python3
"""clock+wheel OpenCR bench test (no LiDAR, LM guide or arm).

Prerequisites (separate terminals):
  socat -d -d pty,raw,echo=0,link=/tmp/arm_dummy pty,raw,echo=0
  ros2 run craft_control control_master --ros-args \
      --params-file src/craft_control/config/control.yaml \
      -p arm_lm_port:=/tmp/arm_dummy -p clock_wheel_port:=/dev/ttyACM0
Wheels must be lifted off the ground. This script publishes a fake odometry fixed at the
origin, so the controller never arrives and wheels keep spinning until stopped.

Clock steps are judged from /control/* topics. Wheel commands and feedback are not
published on ROS, so wheel steps print the expected command and ask the operator (y/n).
Results are written to bench_clock_wheel_<time>.md in the current directory.
"""

import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32

WHEEL_RADIUS = 0.033
MAX_V = 0.26
MAX_WHEEL = 7.8
CLOCK_TIMEOUT = 20.0  # [s] per clock move


class Bench(Node):
    def __init__(self):
        super().__init__('bench_clock_wheel')
        self.lock = threading.Lock()
        self.clock_slot = None
        self.clock_state = None
        self.wheel_state = None
        self.wheel_reached = None
        self.slot_count = 0
        self.saw_moving = False
        self.reached_log = []  # (time, value)
        self.odom_xy = (0.0, 0.0)

        self.slot_pub = self.create_publisher(Int32, '/planning/target_clock_slot', 10)
        self.v_pub = self.create_publisher(Float64, '/planning/target_v', 10)
        self.path_pub = self.create_publisher(Path, '/planning/target_path', 10)
        self.odom_pub = self.create_publisher(Odometry, '/planning/odom', 10)
        self.create_subscription(Int32, '/control/clock_slot', self.on_slot, 50)
        self.create_subscription(Int32, '/control/clock_state', self.on_clock_state, 50)
        self.create_subscription(Int32, '/control/wheel_state', self.on_wheel_state, 50)
        self.create_subscription(Bool, '/control/clock_target_reached', self.on_clock_reached, 50)
        self.create_subscription(Bool, '/control/wheel_target_reached', self.on_wheel_reached, 50)
        self.create_timer(0.05, self.publish_odom)

    def on_slot(self, msg):
        with self.lock:
            self.clock_slot = msg.data
            self.slot_count += 1

    def on_clock_state(self, msg):
        with self.lock:
            self.clock_state = msg.data
            if msg.data == 1:
                self.saw_moving = True

    def on_wheel_state(self, msg):
        with self.lock:
            self.wheel_state = msg.data

    def on_clock_reached(self, msg):
        with self.lock:
            self.reached_log.append((time.monotonic(), msg.data))

    def on_wheel_reached(self, msg):
        with self.lock:
            self.wheel_reached = msg.data

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.odom_xy
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def send_slot(self, slot):
        with self.lock:
            self.saw_moving = False
            self.reached_log.clear()
            sent = time.monotonic()
        self.slot_pub.publish(Int32(data=slot))
        return sent

    def send_v(self, v):
        self.v_pub.publish(Float64(data=v))

    def send_path(self, points, frame='odom'):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        for x, y in points:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def snapshot(self):
        with self.lock:
            return dict(slot=self.clock_slot, clock_state=self.clock_state,
                        wheel_state=self.wheel_state, wheel_reached=self.wheel_reached,
                        saw_moving=self.saw_moving, reached=list(self.reached_log))


class Runner:
    def __init__(self, node, auto_yes):
        self.node = node
        self.auto_yes = auto_yes
        self.results = []

    def record(self, test_id, desc, ok, note=''):
        mark = 'PASS' if ok else 'FAIL'
        print(f'  -> [{mark}] {test_id} {desc} {note}')
        self.results.append((test_id, desc, mark, note))

    def ask(self, question):
        if self.auto_yes:
            print(f'  ? {question} [auto y]')
            return True, ''
        while True:
            ans = input(f'  ? {question} [y/n, 메모는 n 뒤에]: ').strip()
            if ans[:1] in ('y', 'Y'):
                return True, ans[1:].strip()
            if ans[:1] in ('n', 'N'):
                return False, ans[1:].strip()

    def pause(self, text):
        if self.auto_yes:
            print(f'  >> {text} [auto]')
            return
        input(f'  >> {text} (Enter) ')

    # ---------------- clock ----------------
    def wait_clock(self, target, sent):
        """Wait for reached=false after the send, then reached=true with slot==target, idle."""
        deadline = sent + CLOCK_TIMEOUT
        saw_false = False
        while time.monotonic() < deadline:
            s = self.node.snapshot()
            for t, value in s['reached']:
                if t < sent:
                    continue
                if not value:
                    saw_false = True
                elif saw_false and s['slot'] == target and s['clock_state'] == 0:
                    return True, time.monotonic() - sent, s
            time.sleep(0.02)
        return False, CLOCK_TIMEOUT, self.node.snapshot()

    def clock_move(self, test_id, start, target, direction, deg, check_direction=True):
        desc = f'{start}->{target} ({direction} {deg}°)'
        print(f'\n[{test_id}] clock {desc}')
        sent = self.node.send_slot(target)
        ok, elapsed, s = self.wait_clock(target, sent)
        note = f'{elapsed:.2f}s slot={s["slot"]} state={s["clock_state"]} moving_seen={s["saw_moving"]}'
        if not ok:
            self.record(test_id, desc, False, 'timeout/불일치 ' + note)
            return False
        if not s['saw_moving']:
            note += ' (clock_state=1 미관측)'
        if check_direction:
            dir_ok, memo = self.ask(f'{direction} 방향으로 약 {deg}° 회전했고 슬롯 정렬 OK?')
            self.record(test_id, desc, dir_ok, note + (f' 메모:{memo}' if memo else ''))
            return dir_ok
        self.record(test_id, desc, True, note)
        return True

    def setup_slot(self, slot):
        print(f'\n[setup] clock -> {slot}')
        sent = self.node.send_slot(slot)
        ok, _, s = self.wait_clock(slot, sent)
        if not ok and s['slot'] == slot and s['clock_state'] == 0:
            ok = True  # already there: reached may stay true without movement
        if not ok:
            print(f'  !! setup 실패 slot={s["slot"]} state={s["clock_state"]}')
        return ok

    def clock_tests(self):
        print('\n===== 2단계: Clock =====')
        if not self.setup_slot(12):
            self.record('C-setup', '-> 12', False)
            return
        self.clock_move('C1', 12, 3, 'CCW', 90)
        self.clock_move('C2', 3, 7, 'CCW', 120)
        self.clock_move('C3', 7, 12, 'CCW', 150)
        self.clock_move('C4', 12, 7, 'CW', 150)
        if self.setup_slot(11):
            self.clock_move('C5', 11, 1, 'CCW', 60)

        # C6: re-send the same slot mid-motion, target must not change.
        print('\n[C6] 이동 중 같은 슬롯 재전송 (1->5)')
        sent = self.node.send_slot(5)
        time.sleep(0.3)
        self.node.slot_pub.publish(Int32(data=5))
        time.sleep(0.3)
        self.node.slot_pub.publish(Int32(data=5))
        ok, elapsed, s = self.wait_clock(5, sent)
        if ok:
            ok, memo = self.ask('멈칫/역회전 없이 한 번에 5번 슬롯으로 갔나?')
        else:
            memo = ''
        self.record('C6', '1->5 + 재전송 2회', ok, f'{elapsed:.2f}s slot={s["slot"]} {memo}')

        # C7: invalid slots hold position.
        for test_id, bad in (('C7a', 0), ('C7b', 13)):
            print(f'\n[{test_id}] invalid slot {bad}')
            sent = self.node.send_slot(bad)
            time.sleep(2.0)
            s = self.node.snapshot()
            reached_after = [v for t, v in s['reached'] if t >= sent]
            ok = s['slot'] == 5 and s['clock_state'] == 0 and not any(reached_after)
            self.record(test_id, f'slot {bad} -> 정지 유지', ok,
                        f'slot={s["slot"]} state={s["clock_state"]} reached={reached_after[-1:] }')

    # ---------------- wheel ----------------
    def wheel_step(self, test_id, desc, expected, question):
        print(f'\n[{test_id}] {desc}\n  기대: {expected}')
        time.sleep(1.5)
        ok, memo = self.ask(question)
        s = self.node.snapshot()
        note = f'wheel_state={s["wheel_state"]}' + (f' 메모:{memo}' if memo else '')
        if s['wheel_state'] == 2:
            ok = False
            note += ' FAULT'
        self.record(test_id, desc, ok, note)
        return ok

    def drive(self, v, points, frame='odom'):
        self.node.send_v(v)
        time.sleep(0.1)  # path uses the most recent target_v
        self.node.send_path(points, frame)

    def stop(self):
        self.node.send_v(0.0)
        time.sleep(0.1)
        self.node.send_path([])
        time.sleep(1.0)

    def wheel_tests(self):
        print('\n===== 3단계: Wheel (바퀴 띄운 상태) =====')
        self.pause('바퀴가 바닥에서 떠 있는지 확인')
        self.node.odom_xy = (0.0, 0.0)
        straight = [(0.0, 0.0), (1.0, 0.0)]
        w = 0.1 / WHEEL_RADIUS

        self.drive(0.1, straight)
        self.wheel_step('W1', 'v=0.1 직진', f'좌우 모두 전진 {w:.2f} rad/s',
                        '양쪽 바퀴가 같은 속도로 전진 방향으로 도나?')
        self.stop()

        self.drive(-0.1, [(0.0, 0.0), (-1.0, 0.0)])
        self.wheel_step('W2', 'v=-0.1 후진', f'좌우 모두 후진 {-w:.2f} rad/s',
                        '양쪽 바퀴가 같은 속도로 후진 방향으로 도나?')
        self.stop()

        self.drive(0.1, [(0.0, 0.0), (0.5, 0.3)])
        self.wheel_step('W3', 'v=0.1 좌회전 경로', '둘 다 전진, 오른쪽 > 왼쪽',
                        '오른쪽 바퀴가 왼쪽보다 빠르게 전진하나?')
        self.stop()

        wmax = min(MAX_V / WHEEL_RADIUS, MAX_WHEEL)
        self.drive(0.5, straight)
        self.wheel_step('W4', 'v=0.5 (max_v 초과)', f'좌우 모두 {wmax:.2f} rad/s로 제한 (W1의 약 {wmax / w:.1f}배)',
                        'W1보다 약 2.6배 빠르고 좌우 같은 속도인가?')

        print('\n[W5] 회전 중 v=0')
        self.node.send_v(0.0)  # target_v applies on the next control tick, path unchanged
        self.wheel_step('W5', '회전 중 v=0', '부드럽게 감속 후 정지', '부드럽게 감속하며 멈췄나?')

        self.drive(0.1, straight)
        time.sleep(1.0)
        self.node.send_path([])
        self.wheel_step('W6a', '빈 경로', '즉시 정지', '바퀴가 멈췄나?')
        self.drive(0.1, straight)
        time.sleep(1.0)
        self.node.send_path(straight, frame='map')
        self.wheel_step('W6b', 'frame 불일치(map) 경로', '즉시 정지', '바퀴가 멈췄나?')

        print('\n[W7] odom을 경로 끝 (1,0)으로 이동')
        self.drive(0.1, straight)
        time.sleep(1.0)
        self.node.odom_xy = (1.0, 0.0)
        time.sleep(1.0)
        s = self.node.snapshot()
        ok, memo = self.ask('바퀴가 멈췄나?')
        ok = ok and s['wheel_reached'] is True
        self.record('W7', '경로 끝 도착 -> 정지', ok,
                    f'wheel_target_reached={s["wheel_reached"]}' + (f' 메모:{memo}' if memo else ''))
        self.node.odom_xy = (0.0, 0.0)
        self.stop()

    # ---------------- concurrent ----------------
    def concurrent_tests(self):
        print('\n===== 4단계: Clock + Wheel 동시 =====')
        if not self.setup_slot(12):
            self.record('D-setup', '-> 12', False)
            return
        self.drive(0.1, [(0.0, 0.0), (1.0, 0.0)])
        time.sleep(1.0)
        rate = measure_rate(self.node, 3.0)
        print(f'  바퀴 회전 중 피드백 rate {rate:.1f} Hz')
        for test_id, start, target, d, deg in (('D1', 12, 3, 'CCW', 90), ('D2', 3, 7, 'CCW', 120),
                                               ('D3', 7, 12, 'CCW', 150), ('D4', 12, 7, 'CW', 150)):
            self.clock_move(test_id, start, target, d, deg)
        ok, memo = self.ask('clock 이동 내내 바퀴 속도가 일정했나?')
        s = self.node.snapshot()
        ok = ok and s['wheel_state'] != 2 and s['clock_state'] != 2
        self.record('D5', '동시 동작 중 바퀴 일정 / fault 없음', ok,
                    f'rate={rate:.1f}Hz wheel_state={s["wheel_state"]} clock_state={s["clock_state"]} {memo}')
        self.stop()

    def w8(self):
        print('\n===== W8: 통신 끊김 시 정지 (마지막) =====')
        self.drive(0.1, [(0.0, 0.0), (1.0, 0.0)])
        time.sleep(1.0)
        self.pause('바퀴가 도는 중. control_master 터미널에서 Ctrl-C 후 정지 여부 관찰')
        ok, memo = self.ask('control_master 종료 후 바퀴가 스스로 멈췄나? (메모에 대략 정지 시간)')
        self.record('W8', 'host 종료 시 펌웨어 정지', ok, memo)


def measure_rate(node, seconds):
    with node.lock:
        start = node.slot_count
    time.sleep(seconds)
    with node.lock:
        return (node.slot_count - start) / seconds


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--only', choices=['clock', 'wheel', 'concurrent', 'w8'], action='append',
                        help='run only these stages (repeatable); default: all')
    parser.add_argument('--skip-w8', action='store_true', help='skip the control_master shutdown test')
    parser.add_argument('--yes', action='store_true', help='answer y to every prompt (dry run on a simulator)')
    args = parser.parse_args()
    stages = args.only or ['clock', 'wheel', 'concurrent', 'w8']
    if args.skip_w8 and 'w8' in stages:
        stages.remove('w8')

    rclpy.init()
    node = Bench()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    runner = Runner(node, args.yes)

    print('===== 1단계: control_master 연결 확인 =====')
    rate = measure_rate(node, 3.0)
    s = node.snapshot()
    print(f'  피드백 rate {rate:.1f} Hz, slot={s["slot"]}, clock_state={s["clock_state"]}, '
          f'wheel_state={s["wheel_state"]}')
    ok = rate > 10.0 and s['clock_state'] == 0 and s['wheel_state'] == 0
    runner.record('S1', '피드백 수신 / idle', ok, f'{rate:.1f}Hz')
    if rate == 0.0:
        print('  !! /control/clock_slot 수신 없음: control_master/OpenCR 연결 확인 후 다시 실행')
        rclpy.shutdown()
        return

    try:
        if 'clock' in stages:
            runner.clock_tests()
        if 'wheel' in stages:
            runner.wheel_tests()
        if 'concurrent' in stages:
            runner.concurrent_tests()
        if 'w8' in stages:
            runner.w8()
    except KeyboardInterrupt:
        print('\n중단됨')
    finally:
        node.send_v(0.0)
        node.send_path([])
        time.sleep(0.3)

    name = time.strftime('bench_clock_wheel_%Y%m%d_%H%M%S.md')
    with open(name, 'w') as f:
        f.write('| ID | 시험 | 결과 | 비고 |\n|---|---|---|---|\n')
        for row in runner.results:
            f.write('| ' + ' | '.join(str(c).replace('|', '/') for c in row) + ' |\n')
    passed = sum(r[2] == 'PASS' for r in runner.results)
    print(f'\n결과 {passed}/{len(runner.results)} PASS -> {name}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
