#!/usr/bin/env python3
"""Mission adapter for the calibrated executor. No direct control/hold publisher.
Request: '<token> <box> <slot> <collect|find|lie2|lie5>'; result: '<token> <OK|FAILED> <reason>'.
Only one child runs; STOP cancels it and stops planning. Output is retained per request.
"""
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from std_msgs.msg import String


class Picker(Node):
    def __init__(self):
        super().__init__('puck_picker')
        executor = Path(get_package_prefix('craft_planning')) / 'lib/craft_planning/puck_execute'
        self.executor_path = Path(self.declare_parameter('executor_path', str(executor)).value)
        self.log_dir = Path(self.declare_parameter('log_dir', str(Path.home() / 'craft_mission_logs/picks')).value)
        self.timeout = float(self.declare_parameter('timeout_s', 600.0).value)
        self.child = None
        self.token = None
        self.log = None
        self.cancelled = False
        self.stop_at = None
        self.results = self.create_publisher(String, '/mission/pick_result', 10)
        self.command = self.create_publisher(String, '/planning/command', 10)
        self.create_subscription(String, '/mission/pick_request', self.request, 10)
        self.create_subscription(String, '/mission/pick_cancel', lambda _: self.cancel(), 10)
        self.create_timer(0.1, self.poll)

    def result(self, token, ok, reason):
        self.results.publish(String(data=f'{token} {"OK" if ok else "FAILED"} {reason}'))

    def request(self, msg):
        fields = msg.data.split()
        token = fields[0] if fields else 'invalid'
        try:
            if len(fields) not in (4,5):
                raise ValueError('expected token box slot mode [remaining]')
            token, box, slot, mode = fields[:4]
            remaining = int(fields[4]) if len(fields) == 5 else None
            if remaining is not None and remaining not in (1,2):
                raise ValueError("remaining puck count must be 1 or 2")
            if not token.isdigit() or not box.isalnum() or not 1 <= int(slot) <= 12 or mode not in ('collect', 'find', 'lie2', 'lie5'):
                raise ValueError('invalid pick request')
            if self.child:
                raise ValueError('picker is busy')
            if not self.executor_path.is_file():
                raise ValueError(f'executor missing: {self.executor_path}')
            args = [str(self.executor_path), '--planning', '--box', box, '--slot', slot, '--auto']
            args += ['--collect'] if mode == 'collect' else ['--find'] if mode == 'find' else ['--lie', '--cell', mode[-1], '--pick']
            if mode == 'collect' and remaining is not None:
                args += ['--expected-pucks', str(remaining)]
            self.log_dir.mkdir(parents=True, exist_ok=True)
            path = self.log_dir / f'pick_{token}.log'
            self.log = path.open('w')
            self.child = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=self.log, stderr=subprocess.STDOUT,
                                          start_new_session=True)
            self.token, self.started = token, time.monotonic()
            self.cancelled, self.stop_at = False, None
            self.get_logger().info(f'{msg.data}: {path}')
        except (ValueError, OSError) as e:
            if not self.child and self.log:
                self.log.close(); self.log = None
            self.result(token, False, str(e))

    def cancel(self):
        if not self.child or self.cancelled:
            return
        self.cancelled = True
        self.stop_at = time.monotonic()
        if self.child.poll() is None:
            try:
                os.killpg(self.child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        if rclpy.ok():
            self.command.publish(String(data='STOP'))

    def poll(self):
        if not self.child:
            return
        if time.monotonic() - self.started > self.timeout:
            self.cancel()
        if self.stop_at and time.monotonic() - self.stop_at > 3 and self.child.poll() is None:
            os.killpg(self.child.pid, signal.SIGKILL)
        code = self.child.poll()
        if code is None:
            return
        self.result(self.token, code == 0 and not self.cancelled,
                    f'exit={code}, cancelled={self.cancelled}, log={self.log.name}')
        self.log.close()
        self.log, self.child = None, None


def main():
    rclpy.init()
    node = Picker()
    try:
        rclpy.spin(node)
    finally:
        node.cancel()
        if node.child:
            try:
                node.child.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(node.child.pid, signal.SIGKILL)
                node.child.wait()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
