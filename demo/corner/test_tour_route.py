"""Offline route publication regression tests; no ROS node or motor connection."""
import sys
import unittest
from types import SimpleNamespace
from pathlib import Path

sys.path.insert(0, str(Path.home() / 'craft/demo/corner'))
import run_tour as tour
from builtin_interfaces.msg import Time


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Demo:
    def __init__(self):
        self.state = 'Ready'
        self.slam = (3.90, 0.45, 3.10)
        self.goal, self.viz_goal = Publisher(), Publisher()
        self.completed = False
        self.odom = None
        self.commands = []

    def cmd(self, command):
        self.commands.append(command)
        if command == 'START':
            self.state = 'Run.AwaitRequest'

    def spin(self, seconds, condition=None):
        if seconds == 1.0:
            self.completed = True
        return True if condition is None else condition()


class RoutePublication(unittest.TestCase):
    def test_route_is_one_request_with_ordered_via_points_and_goal(self):
        demo, pub = Demo(), Publisher()
        clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1)))
        node = SimpleNamespace(get_clock=lambda: clock)
        via = [(2.75, 0.50), (2.28, 0.95)]
        self.assertTrue(tour.drive(demo, node, 1.88, 3.20, 1, 20, via, pub))
        self.assertEqual(len(pub.messages), 1)
        self.assertEqual(len(demo.goal.messages), 0)
        route = pub.messages[0]
        self.assertEqual(route.header.frame_id, 'map')
        self.assertEqual([(p.pose.position.x, p.pose.position.y) for p in route.poses], via + [(1.88, 3.20)])
        self.assertEqual(demo.commands, ['START'])

    def test_leaving_right_side_requests_stop(self):
        demo, pub = Demo(), Publisher()
        demo.odom = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=1.10, y=3.2))))
        clock = SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=1)))
        node = SimpleNamespace(get_clock=lambda: clock)
        self.assertFalse(tour.drive(demo, node, 1.88, 3.20, 1, 20, [(2.75, 0.50)], pub))
        self.assertEqual(demo.commands[-1], 'STOP')


class ClosedRoute(unittest.TestCase):
    def test_route_closes_at_actual_start_and_keeps_every_vertex(self):
        x,y=tour.ROUTE[0]
        points,distance=tour.closed_route(x,y)
        self.assertLess(distance,1e-8)
        self.assertEqual(points[-1],(x,y))
        for vertex in tour.ROUTE[:-1]:
            self.assertIn(vertex,points)
        self.assertGreater(len(points),100)
        self.assertGreaterEqual(min(px for px,py in points),tour.MIN_X)

    def test_entry_in_middle_keeps_whole_loop_and_actual_goal(self):
        x,y=tour.ROUTE[70]
        points,distance=tour.closed_route(x+.005,y)
        self.assertEqual(points[-1],(x+.005,y))
        self.assertLessEqual(distance,.006)
        self.assertIn(tour.ROUTE[0],points)
        self.assertIn(tour.ROUTE[-20],points)


class AutomaticEntry(unittest.TestCase):
    def test_entry_uses_segments_and_needs_no_exact_start(self):
        # Halfway along a long bottom straight, 0.30 m off it: previously the 0.15 m gate rejected this.
        points,distance=tour.closed_route(3.0,.05)
        self.assertAlmostEqual(distance,.30,places=5)
        self.assertEqual(points[-1],(3.0,.05))
        self.assertAlmostEqual(points[0][0],3.0,places=5)
        self.assertAlmostEqual(points[0][1],.35,places=5)
        self.assertIn(tour.ROUTE[70],points)


if __name__ == '__main__':
    unittest.main()
