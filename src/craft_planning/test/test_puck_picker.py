"""Adapter process failures/cancellation, with a fake executor and an isolated ROS domain."""
import importlib.util
import signal
import time
from pathlib import Path

import pytest
import rclpy
from std_msgs.msg import String


@pytest.fixture
def picker(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '219')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.setenv('ROS_STATIC_PEERS', '')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros'))
    source = Path(__file__).resolve().parents[1] / 'scripts/puck_picker.py'
    spec = importlib.util.spec_from_file_location('puck_picker', source)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    rclpy.init()
    node = module.Picker()
    node.log_dir = tmp_path
    node.executor_path = tmp_path / "executor"
    node.executor_path.touch(mode=0o755)
    results = []
    node.result = lambda *args: results.append(args)
    try:
        yield node, results
    finally:
        node.cancel()
        if node.child:
            node.child.wait(timeout=5)
        if node.log:
            node.log.close()
        node.destroy_node(); rclpy.shutdown()


def finish(node):
    deadline = time.monotonic() + 5
    while node.child and time.monotonic() < deadline:
        node.poll(); time.sleep(.02)
    assert node.child is None


def test_executor_failure_is_not_success_and_slot_mode_pass_through(picker):
    node, results = picker
    node.executor_path.write_text('#!/usr/bin/python3\nimport sys\nprint(sys.argv)\nsys.exit(7)\n')
    node.request(String(data='123 S1 8 lie5'))
    finish(node)
    assert results[0][:2] == ('123', False)
    log = (node.log_dir / 'pick_123.log').read_text()
    assert "'--planning'" in log and "'--slot', '8'" in log
    assert "'--lie', '--cell', '5', '--pick'" in log


def test_cancel_fails_even_if_child_exits_zero(picker):
    node, results = picker
    node.executor_path.write_text('#!/usr/bin/python3\nimport signal,time,sys\nsignal.signal(signal.SIGINT, lambda *_: sys.exit(0))\nprint("ready", flush=True)\ntime.sleep(30)\n')
    node.request(String(data='124 S2 12 find'))
    deadline = time.monotonic() + 3
    while 'ready' not in (node.log_dir / 'pick_124.log').read_text():
        assert time.monotonic() < deadline
        time.sleep(.02)
    node.cancel(); finish(node)
    assert results[0][:2] == ('124', False)
    assert 'cancelled=True' in results[0][2]


def test_invalid_mode_does_not_launch(picker):
    node, results = picker
    node.request(String(data='125 S1 12 lie8'))
    assert node.child is None and results[0][:2] == ('125', False)


@pytest.mark.parametrize('remaining',(1,2))
def test_collect_passes_remaining_count_for_fresh_survey(picker,remaining):
    node,results=picker
    node.executor_path.write_text('#!/usr/bin/python3\nimport sys\nprint(sys.argv)\n')
    node.request(String(data=f'126 S1 1 collect {remaining}'))
    finish(node)
    assert results[0][:2]==('126',True)
    log=(node.log_dir/'pick_126.log').read_text()
    assert "'--collect'" in log and f"'--expected-pucks', '{remaining}'" in log
