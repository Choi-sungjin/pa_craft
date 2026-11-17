#!/usr/bin/env python3
"""Pure offline S1 regressions: actual held_check/run_steps with injected observations/transport.

No ROS node, camera, serial device or hardware publisher is created.
"""
import ast
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[3]


class HeldCriterion(unittest.TestCase):
    def test_both_shapes_use_prism_reference_and_all_prism_limits(self):
        source = ROOT / 'tools/realsense_check/grip_locate.py'
        tree = ast.parse(source.read_text())
        # Execute the production function without importing camera/detector dependencies.
        body = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'held_check']
        with tempfile.TemporaryDirectory() as directory:
            ref = Path(directory) / 'held.json'
            ref.write_text(json.dumps({'prism': {'cx': 853., 'bottom': 313.},
                                       'cylinder': {'cx': 100., 'bottom': 100.}}))
            env = dict(json=json, os=__import__('os'), sys=SimpleNamespace(argv=[]),
                       HELD=str(ref), HELD_X_PX=10., HELD_Y_PX=10., HELD_DEEP_PX=25.,
                       shape=lambda cls: cls.split('_')[-1])
            exec(compile(ast.Module(body=body, type_ignores=[]), str(source), 'exec'), env)
            for sh in ('prism', 'cylinder'):
                for dx, dy, score, expected in [(0, 0, .95, 'OK'), (10, 10, .85, 'OK'),
                                                (-10, -25, .95, 'OK'), (11, 0, .95, 'BAD'),
                                                (0, 11, .95, 'BAD'), (0, -26, .95, 'BAD'),
                                                (0, 0, .84, 'BAD')]:
                    with self.subTest(shape=sh, dx=dx, dy=dy, score=score):
                        observation = {'cls': 'red_' + sh, 'score': score,
                                       'box': [843. + dx, 293. + dy, 20., 20.]}
                        env['locate'] = lambda prefix: ([observation], None)
                        out = io.StringIO()
                        with contextlib.redirect_stdout(out):
                            env['held_check']('offline')
                        self.assertIn('HELD ' + expected, out.getvalue())


class CloseOrdering(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(ROOT / 'demo/puck_pick'))
        prior = sys.argv
        sys.argv = ['execute.py', '--find', '--auto', '--dry-run']
        try:
            spec = importlib.util.spec_from_file_location('s1_execute', ROOT / 'demo/puck_pick/execute.py')
            cls.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(cls.module)
        finally:
            sys.argv = prior

    def test_delayed_true_and_missing_grasp(self):
        e = self.module
        for delay, result, expected in [(0., 'SUCCEEDED', True), (.15, 'SUCCEEDED', True),
                                         (None, 'SUCCEEDED', False), (0., 'FAILED', False)]:
            with self.subTest(delay=delay, result=result):
                clock = SimpleNamespace(now=0.)
                fake_time = SimpleNamespace(monotonic=lambda: clock.now,
                                            sleep=lambda seconds: setattr(clock, 'now', clock.now + seconds))

                class OfflineRunner:
                    planned = True
                    joints = None

                    @property
                    def grasped(self):
                        return delay is not None and clock.now >= delay

                    def send(self, *args):
                        return 1

                    def wait(self, task_id):
                        return result, 'offline'

                prior_time = e.time
                e.time = fake_time
                try:
                    with contextlib.redirect_stdout(io.StringIO()):
                        actual = e.run_steps(OfflineRunner(), [('stand pick: close gripper (grasp)',
                                                               e.TaskCommand.GRIPPER, True)])
                    self.assertEqual(actual, expected)
                    self.assertLessEqual(clock.now, .51)
                finally:
                    e.time = prior_time


if __name__ == '__main__':
    unittest.main()
