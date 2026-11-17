#!/usr/bin/env python3
"""Fake-board control test; run only in an isolated ROS domain."""
import math
import os
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '87')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('ROS_STATIC_PEERS', '')
import rclpy
from geometry_msgs.msg import PointStamped
from craft_interfaces.msg import TaskCommand, TaskResult
from test_planning_watchdog_integration import Harness


def main():
    rclpy.init()
    h = Harness(Path(os.environ['CRAFT_CONTROL_BINARY']))
    points = []
    h.node.create_subscription(PointStamped, '/control/lookahead_point', lambda m: points.append(m.point), 10)
    try:
        h.pump(lambda: h.status is not None and h.status.ready and h.status.stopped, hold=True)
        h.pump(lambda: not h.status.held, hold=False)
        h.run_for(.2, hold=False, commands=True)
        # Invalid new tasks are rejected before any target is stored.
        for task_id, ref, ahead in [(201, -1., .02), (202, math.nan, .02), (203, math.inf, .02),
                                     (204, .2, -1.), (205, .2, math.nan), (206, .2, math.inf),
                                     (207, .2, 0.), (210, 0., .02)]:
            c = TaskCommand(task_id=task_id, revision=1, kind=TaskCommand.NAVIGATE, speed=.1,
                            tracking_reference_x=ref, lookahead_distance=ahead)
            from geometry_msgs.msg import PoseStamped
            c.path.header.frame_id = 'map'
            for x in (0., 2.):
                p = PoseStamped(); p.pose.position.x = x; p.pose.orientation.w = 1.; c.path.poses.append(p)
            h.task_pub.publish(c)
            h.pump(lambda: task_id in h.results, hold=False, commands=True)
            assert h.results[task_id].status == TaskResult.REJECTED
        c = h.nav(208)
        c.tracking_reference_x = .165; c.lookahead_distance = .02
        # h.nav already sends legacy task, so immediately update its accepted task.
        h.pump(lambda: 208 in h.results, hold=False, commands=True)
        c.operation = TaskCommand.UPDATE_PATH; c.revision = 2; h.task_pub.publish(c)
        h.pump(lambda: h.results[208].revision == 2, hold=False, commands=True)
        points.clear()
        h.pump(lambda: bool(points), hold=False, commands=True)
        assert math.isclose(points[-1].x, .185, abs_tol=1e-8), points[-1]
        c.revision = 3; c.tracking_reference_x = 0.; c.lookahead_distance = 0.; h.task_pub.publish(c)
        h.pump(lambda: h.results[208].revision == 3, hold=False, commands=True)
        points.clear(); h.pump(lambda: bool(points), hold=False, commands=True)
        assert math.isclose(points[-1].x, .18, abs_tol=1e-8), points[-1]
        c.revision = 4; c.tracking_reference_x = .2097; c.lookahead_distance = .02; h.task_pub.publish(c)
        h.pump(lambda: h.results[208].revision == 4, hold=False, commands=True)
        h.stop_task(208)
        h.pump(lambda: h.results[208].status == TaskResult.STOPPED, hold=True, commands=True)
        h.pump(lambda: not h.status.held, hold=False, commands=True)
        h.nav(209)
        h.pump(lambda: 209 in h.results, hold=False, commands=True)
        points.clear(); h.pump(lambda: bool(points), hold=False, commands=True)
        assert math.isclose(points[-1].x, .18, abs_tol=1e-8), points[-1]
        print('FRONT_REFERENCE_FAKE_BOARD_PASS invalid=8 front=0.185 legacy=0.18')
    finally:
        h.close(); rclpy.shutdown()

if __name__ == '__main__':
    main()
