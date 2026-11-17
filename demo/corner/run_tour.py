#!/usr/bin/env python3
"""Tour for path tracking bags.

Default: one CLOSED continuous tour of the navigable corridors at map x >= 1.2 m, about 30 m.
The robot returns to its actual starting XY after following all via points in order. The nearest route segment selects the entry automatically; the robot
aligns itself before moving off. One request, no goal stop at intermediate vertices.
Route tasks use continuous tracking: bends are driven while translating, after moving off, severe heading loss holds still.
Obstacle holds and watchdogs remain enabled; a blocked route waits rather than skipping remaining corridors.

--all: the old tour of the whole arena, the waypoints of TOUR one after another, each its own planning goal
(/planning/target_base_pose, like run_corner.py: the robot stops at each and turns on the spot). A closed loop of about
44 m: it starts at the waypoint nearest the robot and goes once round.

Needs the stack up with the saved map (cornerup arm:=false), the robot standing still. Ctrl+C sends STOP.
RViz "corner planned path" (/corner/path) keeps the planned path of every leg driven so far, not only the current one.
  run_tour.py [--all | --short | --s4-l1] [--near X Y] [--skip-localize] [--from N] [--legs N] [--timeout S] [--dry-run]
--short: a 10 m loop (left column up and down, bottom wall right and left) for quick comparisons, about 2 min.
--s4-l1: from below box S4 (0.56 4.63) to below right of box L1 (3.91 0.38), about 6.5 m, one way per run: put the
  robot at one end (--near 0.56 4.63), it drives to the other; run again (--skip-localize) to come back.
--near X Y: where the robot was put [m, map], as in run_corner.py (the marked start is 0.56 0.98).
--from N: start at waypoint N (1-based) instead of the nearest; --legs N: stop after N waypoints.
A waypoint the robot cannot reach (standing still for --stuck s, a fault, the timeout) is given up (STOP) and the tour
goes on with the next one; two in a row end the tour. Rerun: it goes on from the nearest waypoint.

Waypoints: map ~/maps/arena5_legend_opt_arena1cm (2026-11-11), each >= 0.31 m from the walls and boxes of that map.
Another map or moved walls: set them again.
"""
import argparse
import math
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_corner as rc  # noqa: E402
from map_tools import MapGrid, ScanMatcher  # noqa: E402

TOUR = [  # [m, map]
    (0.56, 0.98), (0.56, 3.73), (0.56, 2.00), (0.56, 0.38), (2.21, 0.38), (3.91, 0.38), (2.91, 0.81), (2.26, 1.13),  # left column, bottom
    (2.26, 1.83), (3.91, 1.81), (3.91, 2.93), (3.91, 1.81), (2.26, 1.83),                              # right pocket, in and out
    (2.26, 2.83), (3.01, 3.63), (3.01, 4.58),                                                          # centre
    # In front of box L3, up the corridor right of it (0.28 m from the wall and the box: if planning refuses it, it goes
    # round by the top corridor instead) and back down (added 2026-11-11: tour_1111_1656 never came near L3).
    (3.95, 4.75), (4.09, 5.99), (3.95, 4.75),
    (1.61, 4.58), (0.56, 4.63), (0.56, 6.03),                                                          # top-left room
    (4.09, 5.99), (1.56, 6.00), (1.53, 4.98),                                                          # top corridor, in and out
    # No waypoint beside box L2 or halfway along the top corridor (2026-11-11): LiDAR obstacles 0.2-0.5 m off the map's
    # box / wall put the goals (1.71, 3.73) and (2.21, 6.00) inside the padding and planning waited in front of them
    # until the timeout (tour_1111_1754, tour_1111_1727). The legs still pass both places.
    (1.71, 2.83), (2.26, 1.83), (2.26, 1.03), (2.21, 0.38), (0.56, 0.38),                              # back down the centre
]

# --short: a 10 m closed loop in the left column and along the bottom wall for comparing settings quickly (about 2 min
# instead of 6). Legs up 2.75 m, down 3.35 m, right 1.65 m, left 1.65 m: all four directions (the pose heading is off by
# -4.5 deg driving up the map and -0.7 driving down), each long enough to show the part after the first 1.5 m.
SHORT = [(0.56, 0.98), (0.56, 3.73), (0.56, 0.38), (2.21, 0.38), (0.56, 0.38)]

# --s4-l1: the one run from box S4 to box L1 (below S4 -> below right of L1, about 6.5 m across the arena), the route of
# the box missions. Two waypoints: started at one, the robot drives to the other (the one it stands on is skipped), so
# each run is one way; run it again for the way back.
S4_L1 = [(0.56, 4.63), (3.91, 0.38)]


# Closed coverage route, 2026-11-12. Coordinates in map metres; all planned poses must satisfy MIN_X.
# Dense arc points define moving U-turns in branches. The global planner keeps their order at repeated corridors.
MIN_X = 1.20
ROUTE = [
    (2.300000, 0.350000), (4.000000, 0.350000), (4.046587, 0.356133), (4.090000, 0.374115),
    (4.127279, 0.402721), (4.155885, 0.440000), (4.173867, 0.483413), (4.180000, 0.530000),
    (4.173867, 0.576587), (4.155885, 0.620000), (4.127279, 0.657279), (4.090000, 0.685885),
    (4.046587, 0.703867), (4.000000, 0.710000), (2.000000, 0.710000), (1.600000, 0.710000),
    (1.553413, 0.703867), (1.510000, 0.685885), (1.472721, 0.657279), (1.444115, 0.620000),
    (1.426133, 0.576587), (1.420000, 0.530000), (1.426133, 0.483413), (1.444115, 0.440000),
    (1.472721, 0.402721), (1.510000, 0.374115), (1.553413, 0.356133), (1.600000, 0.350000),
    (2.000000, 0.350000), (2.064705, 0.358519), (2.125000, 0.383494), (2.176777, 0.423223),
    (2.216506, 0.475000), (2.241481, 0.535295), (2.250000, 0.600000), (2.250000, 1.500000),
    (2.258519, 1.564705), (2.283494, 1.625000), (2.323223, 1.676777), (2.375000, 1.716506),
    (2.435295, 1.741481), (2.500000, 1.750000), (3.860000, 1.750000), (3.924705, 1.758519),
    (3.985000, 1.783494), (4.036777, 1.823223), (4.076506, 1.875000), (4.101481, 1.935295),
    (4.110000, 2.000000), (4.110000, 2.900000), (4.103867, 2.946587), (4.085885, 2.990000),
    (4.057279, 3.027279), (4.020000, 3.055885), (3.976587, 3.073867), (3.930000, 3.080000),
    (3.883413, 3.073867), (3.840000, 3.055885), (3.802721, 3.027279), (3.774115, 2.990000),
    (3.756133, 2.946587), (3.750000, 2.900000), (3.750000, 2.100000), (3.741481, 2.035295),
    (3.716506, 1.975000), (3.676777, 1.923223), (3.625000, 1.883494), (3.564705, 1.858519),
    (3.500000, 1.850000), (2.550000, 1.850000), (2.485295, 1.858519), (2.425000, 1.883494),
    (2.373223, 1.923223), (2.333494, 1.975000), (2.308519, 2.035295), (2.300000, 2.100000),
    (2.300000, 2.250000), (2.291481, 2.314705), (2.266506, 2.375000), (2.226777, 2.426777),
    (2.175000, 2.466506), (2.114705, 2.491481), (2.050000, 2.500000), (1.750000, 2.500000),
    (1.685295, 2.508519), (1.625000, 2.533494), (1.573223, 2.573223), (1.533494, 2.625000),
    (1.508519, 2.685295), (1.500000, 2.750000), (1.500000, 4.100000), (1.510222, 4.177646),
    (1.540192, 4.250000), (1.587868, 4.312132), (1.650000, 4.359808), (1.722354, 4.389778),
    (1.800000, 4.400000), (3.000000, 4.400000), (3.064705, 4.408519), (3.125000, 4.433494),
    (3.176777, 4.473223), (3.216506, 4.525000), (3.241481, 4.585295), (3.250000, 4.650000),
    (3.250000, 4.720000), (3.243867, 4.766587), (3.225885, 4.810000), (3.197279, 4.847279),
    (3.160000, 4.875885), (3.116587, 4.893867), (3.070000, 4.900000), (3.023413, 4.893867),
    (2.980000, 4.875885), (2.942721, 4.847279), (2.914115, 4.810000), (2.896133, 4.766587),
    (2.890000, 4.720000), (2.890000, 4.600000), (2.881481, 4.535295), (2.856506, 4.475000),
    (2.816777, 4.423223), (2.765000, 4.383494), (2.704705, 4.358519), (2.640000, 4.350000),
    (1.750000, 4.350000), (1.685295, 4.358519), (1.625000, 4.383494), (1.573223, 4.423223),
    (1.533494, 4.475000), (1.508519, 4.535295), (1.500000, 4.600000), (1.500000, 5.750000),
    (1.510222, 5.827646), (1.540192, 5.900000), (1.587868, 5.962132), (1.650000, 6.009808),
    (1.722354, 6.039778), (1.800000, 6.050000), (2.700000, 6.050000), (2.738823, 6.044889),
    (2.775000, 6.029904), (2.806066, 6.006066), (2.829904, 5.975000), (2.844889, 5.938823),
    (2.850000, 5.900000), (2.856474, 5.850824), (2.875455, 5.805000), (2.905650, 5.765650),
    (2.945000, 5.735455), (2.990824, 5.716474), (3.040000, 5.710000), (3.089176, 5.716474),
    (3.135000, 5.735455), (3.174350, 5.765650), (3.204545, 5.805000), (3.223526, 5.850824),
    (3.230000, 5.900000), (3.235111, 5.938823), (3.250096, 5.975000), (3.273934, 6.006066),
    (3.305000, 6.029904), (3.341177, 6.044889), (3.380000, 6.050000), (3.900000, 6.050000),
    (3.964705, 6.041481), (4.025000, 6.016506), (4.076777, 5.976777), (4.116506, 5.925000),
    (4.141481, 5.864705), (4.150000, 5.800000), (4.150000, 4.800000), (4.141481, 4.735295),
    (4.116506, 4.675000), (4.076777, 4.623223), (4.025000, 4.583494), (3.964705, 4.558519),
    (3.900000, 4.550000), (3.300000, 4.550000), (3.235295, 4.541481), (3.175000, 4.516506),
    (3.123223, 4.476777), (3.083494, 4.425000), (3.058519, 4.364705), (3.050000, 4.300000),
    (3.050000, 3.150000), (2.300000, 2.600000), (2.300000, 1.200000), (2.050000, 0.850000),
    (2.050000, 0.600000), (2.058519, 0.535295), (2.083494, 0.475000), (2.123223, 0.423223),
    (2.175000, 0.383494), (2.235295, 0.358519), (2.300000, 0.350000),
]


def closed_route(x, y):
    """Connect to the nearest route segment, traverse the full loop, and return to the actual start."""
    vertices = ROUTE[:-1]
    best = None
    for i, (ax, ay) in enumerate(vertices):
        bx, by = vertices[(i + 1) % len(vertices)]
        dx, dy = bx-ax, by-ay
        length2 = dx*dx + dy*dy
        t = max(0.0, min(1.0, ((x-ax)*dx + (y-ay)*dy)/length2)) if length2 > 0 else 0.0
        entry = (ax+t*dx, ay+t*dy)
        distance = math.hypot(entry[0]-x, entry[1]-y)
        if best is None or distance < best[0]:
            best = (distance, i, entry)
    distance, i, entry = best
    following = (i + 1) % len(vertices)
    candidates = [entry] + vertices[following:] + vertices[:following] + [entry, (x, y)]
    points, previous = [], (x, y)
    for point in candidates:
        if math.hypot(point[0]-previous[0], point[1]-previous[1]) > 1e-7:
            points.append(point)
            previous = point
    return points, distance


class WholePath:
    """Stands in for Demo.viz_path (/corner/path, latched): the planned path of every leg so far stays in RViz instead of
    only the current one. One piece per leg: a replan toward the same goal replaces that leg's piece."""

    def __init__(self, pub):
        self.pub, self.legs = pub, []  # legs: [(goal x, goal y), poses]

    def publish(self, m):
        end = m.poses[-1].pose.position
        if self.legs and math.hypot(end.x - self.legs[-1][0][0], end.y - self.legs[-1][0][1]) < 0.05:
            self.legs[-1] = ((end.x, end.y), list(m.poses))
        else:
            self.legs.append(((end.x, end.y), list(m.poses)))
        whole = PathMsg()
        whole.header = m.header
        whole.poses = [p for _, poses in self.legs for p in poses]
        self.pub.publish(whole)


def drive(d, node, gx, gy, timeout, stuck, via=(), route_pub=None):
    """One planning goal, as run_corner.py sends it; with via points, one route through them. True on Run.Complete."""
    if d.state == 'Fault':
        d.cmd('RESET')
        d.spin(3.0, lambda: d.state != 'Fault')
    if not d.spin(20, lambda: d.state in ('Ready', 'Run.AwaitRequest')):
        print(f'[tour] planning not ready (state {d.state}): see /planning/diagnostics')
        return False
    d.obstacle_cleared_at = None
    was_ready = d.state == 'Ready'
    d.cmd('START')
    if was_ready and not d.spin(5, lambda: d.state == 'Run.AwaitRequest'):
        print(f'[tour] planning did not start (state {d.state})')
        return False
    d.spin(3.0, lambda: d.obstacle_cleared_at is not None)  # START clears the obstacle memory
    d.spin(0.5)
    d.completed = False
    x, y, _ = d.slam
    heading = math.atan2(gy - y, gx - x)  # not required (goal_heading_required: false)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x, goal.pose.position.y = gx, gy
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(heading / 2), math.cos(heading / 2)
    d.viz_goal.publish(goal)
    if via:
        route = PathMsg()
        route.header = goal.header
        for px, py in list(via) + [(gx, gy)]:
            p = PoseStamped()
            p.header = goal.header
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = px, py, 1.0
            route.poses.append(p)
        route_pub.publish(route)
    else:
        d.goal.publish(goal)
    start = moved_at = time.time()
    seen = None
    while time.time() - start < timeout:
        d.spin(1.0)
        if d.odom is not None:
            p = d.odom.pose.pose.position
            if via and p.x < MIN_X:
                print(f'\n[tour] outside the right-side route (x={p.x:.2f} m): STOP')
                d.cmd('STOP')
                return False
            if seen is None or math.hypot(p.x - seen[0], p.y - seen[1]) > 0.05:
                seen, moved_at = (p.x, p.y), time.time()
            elif time.time() - moved_at > stuck:  # planning waits in front of an obstacle (or turns on the spot for good)
                print(f'\n[tour] standing at ({p.x:+.2f}, {p.y:+.2f}) for {stuck:.0f} s, {math.hypot(gx - p.x, gy - p.y):.2f} m short')
                return False
            print(f'[tour]   at ({p.x:+.2f}, {p.y:+.2f}), {math.hypot(gx - p.x, gy - p.y):.2f} m to go   ', end='\r')
        if d.completed:
            return True
        if d.state == 'Fault':
            d.spin(1.1)
            print(f'\n[tour] planning fault: {d.last_diag or "(see planning_master log)"}')
            return False
    print('\n[tour] timeout')
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default=str(Path.home() / 'maps/arena5_legend_opt_arena1cm.yaml'))
    ap.add_argument('--near', type=float, nargs=2, metavar=('X', 'Y'), help='robot placed roughly here [m, map]')
    ap.add_argument('--skip-localize', action='store_true', help='trust slam_toolbox as it is')
    ap.add_argument('--from', dest='first', type=int, help='start at this waypoint (1-based); default: the nearest')
    ap.add_argument('--legs', type=int, default=len(TOUR), help='number of waypoints to drive')
    ap.add_argument('--timeout', type=float, default=600.0, help='[s] per route or waypoint')
    ap.add_argument('--stuck', type=float, default=20.0, help='[s] standing within 5 cm this long: give the waypoint up')
    modes = ap.add_mutually_exclusive_group()
    modes.add_argument('--all', action='store_true', help='the old tour of the whole arena, a goal per waypoint (TOUR)')
    modes.add_argument('--short', action='store_true', help='the 10 m loop (SHORT) instead of the whole arena')
    modes.add_argument('--s4-l1', action='store_true', help='box S4 to box L1 only (S4_L1), one way per run')
    ap.add_argument('--dry-run', action='store_true', help='localize and print the order, do not drive')
    args = ap.parse_args()
    if args.first is not None and args.first < 1:
        ap.error('--from must be at least 1')
    if not (args.all or args.short or args.s4_l1) and (args.first is not None or args.legs != len(TOUR)):
        ap.error('--from/--legs require --all, --short or --s4-l1')
    tour = S4_L1 if args.s4_l1 else SHORT if args.short else TOUR
    if args.first is not None and args.first > len(tour):
        ap.error('--from is outside this tour')
    args.legs = min(args.legs, len(tour))

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda *_: setattr(rc, 'INTERRUPTED', True))  # Demo.spin() raises KeyboardInterrupt
    node = rclpy.create_node('tour_demo')
    d = rc.Demo(node)
    route_pub = node.create_publisher(PathMsg, '/planning/target_base_route', 10)
    continuous = not (args.all or args.short or args.s4_l1)
    def check_route_bounds(msg):
        if continuous and any(p.pose.position.x < MIN_X for p in msg.poses):
            print('[tour] planned route crosses x minimum: STOP')
            d.cmd('STOP')
            setattr(rc, 'INTERRUPTED', True)
    node.create_subscription(PathMsg, '/control/tracking_path', check_route_bounds, 10)
    try:
        print('[tour] waiting for /lidar/merged and slam_toolbox ...')
        if not d.spin(20, lambda: d.scans and d.slam is not None):
            print('[tour] no /lidar/merged or no map->base_link TF: is corner_demo.launch.py up?')
            return 1
        if not args.skip_localize:
            print('[tour] global localization: keep the robot still ...')
            pts = d.scan_points()
            res = ScanMatcher(MapGrid(args.map)).localize(pts, near=args.near) if pts is not None and len(pts) >= 60 else None
            if res is None or res['pose'] is None or res['ambiguous'] or res['score'] < 0.85:
                print(f'[tour] localization failed ({"no fit" if res is None or res["pose"] is None else res}): '
                      'the scan/map fit is unreliable; keep still, check the live scan against the map and rerun')
                return 1
            pose = res['pose']
            print(f'[tour] found ({pose[0]:+.3f}, {pose[1]:+.3f}, {math.degrees(pose[2]):+.1f} deg) score {res["score"]:.2f}')
            if not any(d.send_initial(pose) or d.spin(5.0, lambda: d.agrees(pose)) for _ in range(3)):
                print(f'[tour] slam_toolbox did not settle on it (its map->base_link {d.slam}); rerun')
                return 1
        if not (args.all or args.short or args.s4_l1):
            x, y, yaw = d.slam
            pts, distance = closed_route(x, y)
            if x < MIN_X:
                print(f'[tour] robot centre must start at x >= {MIN_X:.2f} m')
                return 1
            off = math.degrees(math.remainder(math.atan2(pts[0][1] - y, pts[0][0] - x) - yaw, 2 * math.pi))
            print(f'[tour] closed route: {len(pts)} via/goal points, return to ({x:+.2f}, {y:+.2f}), x >= {MIN_X:.2f} m')
            print(f'[tour] entry {distance:.2f} m from the loop; automatic initial alignment ({off:+.0f} deg)')
            if args.dry_run:
                return 0
            d.start_trail()
            if drive(d, node, *pts[-1], args.timeout, args.stuck, via=pts[:-1], route_pub=route_pub):
                print('\n[tour] done')
                return 0
            d.cmd('STOP')
            return 1
        x, y, _ = d.slam
        first = args.first - 1 if args.first else min(range(len(tour)), key=lambda i: math.hypot(tour[i][0] - x, tour[i][1] - y))
        order = [(first + k) % len(tour) for k in range(args.legs)]
        print(f'[tour] robot at ({x:+.2f}, {y:+.2f}); waypoints {" ".join(str(i + 1) for i in order)}')
        if args.dry_run:
            return 0
        d.start_trail()
        d.viz_path = WholePath(d.viz_path)  # Demo republishes /control/tracking_path through it
        failed, skipped = 0, []
        for n, i in enumerate(order):
            gx, gy = tour[i]
            x, y, _ = d.slam
            if math.hypot(gx - x, gy - y) < 0.15:  # already there (planning goal_tolerance 0.10)
                continue
            print(f'\n[tour] {n + 1}/{len(order)}: waypoint {i + 1} ({gx:+.2f}, {gy:+.2f})')
            if drive(d, node, gx, gy, args.timeout, args.stuck):
                failed = 0
                continue
            d.cmd('STOP')
            d.spin(3.0, lambda: d.state in ('Ready', 'Fault'))
            skipped.append(i + 1)
            failed += 1
            if failed == 2:
                print(f'[tour] two waypoints in a row not reached: stopping (given up: {skipped})')
                return 1
        print(f'\n[tour] done' + (f', waypoints given up: {skipped}' if skipped else ''))
        return 0
    except BaseException as ex:  # Ctrl+C, a dropped ssh session, any error: never leave planning driving
        d.cmd('STOP')
        setattr(rc, 'INTERRUPTED', False)  # let the STOP go out
        d.spin(0.3)
        if isinstance(ex, KeyboardInterrupt):
            print('\n[tour] interrupted: STOP')
            return 1
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
