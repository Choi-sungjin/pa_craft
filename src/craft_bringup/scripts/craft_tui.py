#!/usr/bin/env python3
"""craft terminal dashboard: what the robot is doing, in one terminal (curses, no GUI; fine over SSH).

  ros2 run craft_bringup craft_tui.py            (q quits, p pauses the log)

Panels (every value shows how old it is; grey = stale, nothing = never received):
  STATE        mission / planning / box phase / control layers, how long each has held, and one line of what is
               going on right now ("planning path...", "waiting for obstacle to pass...", ...).
  LOCALIZATION map pose and speed, EKF sigma (/odometry/global covariance), scan fit = share of the LiDAR points
               lying on a wall of /planning/map at the current pose (a live check of the pose), global localization
               and perception's localization state.
  PATH         remaining length of the path control tracks, planning time, queue, LiDAR obstacles (count,
               nearest and where), what planning waits for.
  MISSION      traffic light counts, barcode, puck survey.
  LOG          the last mission / planning / box events.
  TOPICS       age of every input.
Read-only: it subscribes, never publishes.
"""
import collections
import curses
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

try:
    from craft_interfaces.msg import ControlStatus
except ImportError:
    ControlStatus = None
try:
    from craft_perception.msg import PuckSurvey, TrafficLight
except ImportError:
    PuckSurvey = TrafficLight = None
try:
    import cv2
except ImportError:
    cv2 = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
BODY_CENTER_X, BODY_R = 0.01275, 0.27  # merged scan is in base_link: drop the robot itself (run_corner.py)
FIT_TOL = 0.05                          # [m] a scan point within this of a wall counts as fitting

BOX_PHASE_TEXT = {
    'stage': 'driving to the stage point in front of the box',
    'stage_stop': 'stopping at the stage point',
    'lidar_perceive': 'measuring the box face with the LiDAR',
    'lidar_release': 'releasing the hold before the dock',
    'lidar_dock': 'docking along the LiDAR face line',
    'handoff_stop': 'stopping for the camera handoff',
    'camera_perceive': 'measuring the gap with the upper cam',
    'camera_release': 'releasing the hold before the camera dock',
    'camera_dock': 'final approach, upper cam stops it',
    'final_stop': 'stopped at the face, settling',
    'camera_verify': 'checking the final alignment',
}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Latest:
    """A value and when it came."""
    def __init__(self):
        self.value, self.at, self.since, self.count = None, None, None, 0

    def set(self, value, track_change=False):
        now = time.time()
        if track_change and value != self.value:
            self.since = now
        self.value, self.at = value, now
        self.count += 1

    def age(self):
        return None if self.at is None else time.time() - self.at


class Board:
    def __init__(self, node):
        self.n = node
        self.v = collections.defaultdict(Latest)
        self.log = collections.deque(maxlen=200)
        self.map = None  # (dist [m] per cell, res, ox, oy, w, h)
        self.map_source = None
        self.fit = None
        self.fit_at = 0.0
        sub = node.create_subscription
        sub(String, '/mission/state', lambda m: self.changed('mission', m.data, 'mission'), LATCHED)
        sub(String, '/planning/state', lambda m: self.changed('planning', m.data, 'planning'), LATCHED)
        sub(String, '/planning/box_phase', lambda m: self.changed('box_phase', m.data, 'box'), LATCHED)
        sub(String, '/planning/box_status', lambda m: self.event('box', m.data), 20)
        sub(DiagnosticArray, '/planning/diagnostics', self.on_diag, 10)
        sub(Odometry, '/perception/state/odom', lambda m: self.v['odom'].set(m), 10)
        sub(Odometry, '/odometry/global', lambda m: self.v['ekf'].set(m), 10)
        sub(String, '/perception/localization/state', lambda m: self.changed('loc_state', m.data, 'loc'), 10)
        sub(String, '/perception/localization/global', lambda m: self.changed('global', m.data, 'loc'), LATCHED)
        sub(PoseArray, '/perception/lidar/obstacle', lambda m: self.v['obstacles'].set(m), 10)
        sub(Path, '/control/tracking_path', lambda m: self.v['path'].set(m), 10)
        sub(LaserScan, '/lidar/merged', lambda m: self.v['scan'].set(m), qos_profile_sensor_data)
        sub(OccupancyGrid, '/planning/map', lambda m: self.on_map(m, 'planning'), LATCHED)
        # the robot's planning_master does not publish /planning/map (the simulator does): slam_toolbox's map then
        sub(OccupancyGrid, '/perception/lidar/map', lambda m: self.on_map(m, 'slam'), LATCHED)
        # what planning asks of control (true = hold) and what control does: a hold that never lifts shows here
        sub(Bool, '/planning/hold', lambda m: self.flag('hold_req', m.data, 'planning hold request'), 50)
        sub(Bool, '/control/planning_held', lambda m: self.flag('held', m.data, 'control held'), 50)
        sub(String, '/perception/vision/uppercam/barcode', lambda m: self.changed('barcode', m.data, 'mission'), LATCHED)
        if ControlStatus:
            sub(ControlStatus, '/control/status', self.on_control, 10)
        if TrafficLight:
            sub(TrafficLight, '/perception/vision/uppercam/traffic_light', lambda m: self.v['light'].set(m), LATCHED)
        if PuckSurvey:
            sub(PuckSurvey, '/perception/vision/d435/puck_survey', lambda m: self.v['survey'].set(m), LATCHED)

    # ---------------------------------------------------------------- inputs
    def event(self, source, text):
        self.log.append((time.time(), source, text))

    def changed(self, key, value, source):
        old = self.v[key].value
        self.v[key].set(value, track_change=True)
        if value != old:
            self.event(source, f'{key}: {value}')

    def on_diag(self, m):
        for s in m.status:
            if s.name == 'planning/state_machine':
                kv = {x.key: x.value for x in s.values}
                kv['_message'] = s.message
                old = (self.v['diag'].value or {}).get('waiting_for')
                self.v['diag'].set(kv)
                if kv.get('waiting_for') and kv.get('waiting_for') != old:
                    self.event('planning', 'waiting: ' + kv['waiting_for'])

    def on_control(self, m):
        value = (m.ready, m.fault, m.held, m.stopped, m.reason)
        if value != (self.v['control'].value or ()):
            self.event('control', f'ready={int(m.ready)} fault={int(m.fault)} held={int(m.held)} '
                                  f'stopped={int(m.stopped)} {m.reason}')
        self.v['control'].set(value, track_change=True)

    def flag(self, key, value, label):
        old = self.v[key].value
        self.v[key].set(value, track_change=True)
        if value != old:
            self.event('hold', f'{label}: {value}')

    def on_map(self, m, source):
        if cv2 is None or (source == 'slam' and self.map_source == 'planning'):
            return
        self.map_source = source
        w, h = m.info.width, m.info.height
        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        occupied = grid >= 65
        dist = cv2.distanceTransform((~occupied).astype(np.uint8), cv2.DIST_L2, 3) * m.info.resolution
        self.map = (dist, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y, w, h)
        self.event('map', f'{source} map {w}x{h} @ {m.info.resolution * 100:.0f} cm')

    # ---------------------------------------------------------------- derived
    def pose(self):
        o = self.v['odom'].value
        if o is None:
            return None
        p = o.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation), o.twist.twist.linear.x, o.twist.twist.angular.z

    def scan_fit(self):
        """Share of LiDAR points (0.3..6 m) within FIT_TOL of a map wall at the current pose; every 0.5 s."""
        if time.time() - self.fit_at < 0.5:
            return self.fit
        self.fit_at = time.time()
        scan, pose = self.v['scan'].value, self.pose()
        if self.map is None or scan is None or pose is None or (self.v['scan'].age() or 9) > 1.0:
            self.fit = None
            return None
        dist, res, ox, oy, w, h = self.map
        r = np.asarray(scan.ranges, float)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > 0.3) & (r < 6.0)
        x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        keep = np.hypot(x - BODY_CENTER_X, y) > BODY_R
        x, y = x[keep], y[keep]
        if len(x) < 30:
            self.fit = None
            return None
        c, s = math.cos(pose[2]), math.sin(pose[2])
        mx, my = pose[0] + c * x - s * y, pose[1] + s * x + c * y
        col, row = ((mx - ox) / res).astype(int), ((my - oy) / res).astype(int)
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        d = np.full(len(mx), np.inf)
        d[inside] = dist[row[inside], col[inside]]
        self.fit = (float(np.mean(d < FIT_TOL)), int(len(d)))
        return self.fit

    def path_left(self, pose):
        path = self.v['path'].value
        if path is None or not path.poses or pose is None:
            return None
        pts = np.array([(p.pose.position.x, p.pose.position.y) for p in path.poses])
        i = int(np.argmin(np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])))
        seg = np.hypot(np.diff(pts[i:, 0]), np.diff(pts[i:, 1]))
        return float(seg.sum()), len(path.poses)

    def nearest_obstacle(self, pose):
        obs = self.v['obstacles'].value
        if obs is None:
            return None
        if not obs.poses or pose is None:
            return len(obs.poses), None, None
        pts = np.array([(p.position.x, p.position.y) for p in obs.poses])
        d = np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
        i = int(np.argmin(d))
        bearing = math.degrees(math.remainder(math.atan2(pts[i, 1] - pose[1], pts[i, 0] - pose[0]) - pose[2],
                                              2 * math.pi))
        return len(obs.poses), float(d[i]), bearing

    def detail(self):
        """One line: what is going on right now, and its level (ok / wait / bad)."""
        planning = self.v['planning'].value or ''
        mission = self.v['mission'].value or ''
        diag = self.v['diag'].value or {}
        waiting = diag.get('waiting_for', '')
        phase = self.v['box_phase'].value or 'off'
        glob = self.v['global'].value or ''
        if self.v['planning'].at is None:
            return 'no /planning/state: is planning_master up (same ROS_DOMAIN_ID)?', 'bad'
        if planning == 'Fault':
            return 'PLANNING FAULT: ' + diag.get('_message', '').split(': ', 1)[-1], 'bad'
        if mission.startswith('Fault'):
            return 'MISSION FAULT: ' + mission.split(': ', 1)[-1], 'bad'
        if 'blocked' in waiting:
            return 'waiting for the obstacle to pass... (' + waiting + ')', 'wait'
        if 'route unavailable' in waiting:
            return 'no route, retrying... (' + waiting.split('(', 1)[-1].rstrip(')') + ')', 'wait'
        if mission.startswith('WatchSignal'):
            light = self.v['light'].value
            seen = f' (R{light.red} Y{light.yellow} G{light.green} so far)' if light is not None else ''
            return 'waiting for the traffic light to finish' + seen + '...', 'wait'
        if mission.startswith('ReadBarcode'):
            return 'reading the barcode...', 'wait'
        if mission.startswith(('Collect', 'Deliver', 'Verify', 'Return')) and glob and not glob.startswith('ok') \
                and planning in ('Ready', 'Run.AwaitRequest'):
            return 'waiting for global localization (' + glob + ')...', 'wait'
        if planning == 'Run.Compute':
            return 'planning path...', 'ok'
        if planning == 'Run.PrepareTask':
            return (waiting or 'preparing the task') + '...', 'wait'
        if planning.startswith('Run.Execute'):
            # Run.Execute[.Box.<Phase> | .Leave][.Blocked | .Replan | .DockBlocked | .NoRoute] (planning_master)
            tail = planning[len('Run.Execute'):]
            where = (f' (box {phase})' if '.Box.' in tail and phase else '')
            if tail.endswith('.Blocked'):
                return 'waiting for the obstacle to pass (replan after block_wait_s)' + where + '...', 'wait'
            if tail.endswith('.Replan'):
                return 'still blocked: planning a way around' + where + '...', 'wait'
            if tail.endswith('.DockBlocked'):
                return 'dock / back-out line blocked: waiting for it to clear (dock_block_wait_s)...', 'wait'
            if tail.endswith('.NoRoute'):
                return 'no route around yet: retrying' + where + '...', 'wait'
            if tail == '.Leave':
                return 'backing out of the box...', 'ok'
            if phase in BOX_PHASE_TEXT:
                return BOX_PHASE_TEXT[phase] + '...', 'ok'
            left = self.path_left(self.pose())
            return 'driving the path' + (f', {left[0]:.2f} m left' if left else '') + '...', 'ok'
        if planning == 'Run.Stopping':
            return 'stopping...', 'wait'
        if planning == 'Run.AwaitRequest':
            return 'planning running, waiting for a request', 'ok'
        if planning == 'Ready':
            return 'ready: waiting for START' + (' (mission ' + mission + ')' if mission else ''), 'ok'
        if planning == 'Idle':
            return 'starting up: ' + (waiting or 'waiting for control / odom / LiDAR'), 'wait'
        return planning, 'ok'


# -------------------------------------------------------------------- drawing
class TextWindow:
    """--print: the same screen into a character grid instead of curses."""
    def __init__(self, rows, cols):
        self.rows, self.cols = rows, cols
        self.erase()

    def erase(self):
        self.grid = [[' '] * self.cols for _ in range(self.rows)]

    def getmaxyx(self):
        return self.rows, self.cols

    def addstr(self, y, x, text, _attr=0):
        for i, ch in enumerate(text):
            if 0 <= y < self.rows and 0 <= x + i < self.cols:
                self.grid[y][x + i] = ch

    def refresh(self):
        pass

    def getch(self):
        return -1

    def text(self):
        return '\n'.join(''.join(r).rstrip() for r in self.grid)


class Screen:
    def __init__(self, win, board):
        self.w, self.b = win, board
        if isinstance(win, TextWindow):
            self.C = collections.defaultdict(int)
        else:
            curses.curs_set(0)
            win.nodelay(True)
            curses.start_color()
            curses.use_default_colors()
            for i, c in enumerate((curses.COLOR_GREEN, curses.COLOR_YELLOW, curses.COLOR_RED, curses.COLOR_CYAN,
                                   curses.COLOR_WHITE), 1):
                curses.init_pair(i, c, -1)
            self.C = {'ok': curses.color_pair(1), 'wait': curses.color_pair(2), 'bad': curses.color_pair(3),
                      'head': curses.color_pair(4) | curses.A_BOLD, 'dim': curses.A_DIM, 'bold': curses.A_BOLD,
                      '': curses.A_NORMAL}
        self.paused = False
        self.row = 0

    def put(self, col, text, style=''):
        h, w = self.w.getmaxyx()
        if self.row >= h or col >= w - 1:
            return col
        text = str(text)[:max(0, w - 1 - col)]
        try:
            self.w.addstr(self.row, col, text, self.C[style] if isinstance(style, str) else style)
        except curses.error:
            pass
        return col + len(text)

    def line(self, *parts):
        col = 0
        for part in parts:
            text, style = (part, '') if isinstance(part, str) else part
            col = self.put(col, text, style)
        self.row += 1

    def head(self, title):
        _, w = self.w.getmaxyx()
        self.line((('── ' + title + ' ').ljust(w - 1, '─'), 'head'))

    @staticmethod
    def age(latest, stale=2.0):
        a = latest.age()
        if a is None:
            return ('never', 'bad')
        return (f'{a:4.1f}s' if a < 100 else f'{a / 60:.0f}m', 'dim' if a < stale else 'wait')

    @staticmethod
    def held(latest):
        return '' if latest.since is None else f'{time.time() - latest.since:6.1f}s'

    def draw(self):
        b, v = self.b, self.b.v
        self.w.erase()
        self.row = 0
        self.line((' CRAFT ', 'head'), time.strftime('%H:%M:%S'), ('   q quit  p pause log', 'dim'))
        # ---------------------------------------------------------- state
        self.head('STATE')
        text, level = b.detail()
        self.line(('  ▶ ', self.C[level] | curses.A_BOLD), (text, self.C[level] | curses.A_BOLD))
        mission = v['mission'].value
        self.line('  mission   ', ((mission or '-').ljust(46), 'bad' if (mission or '').startswith('Fault') else 'bold'),
                  (self.held(v['mission']), 'dim'))
        planning = v['planning'].value
        diag = v['diag'].value or {}
        step = diag.get('step', '')
        self.line('  planning  ', ((planning or '-').ljust(18), 'bad' if planning == 'Fault' else 'bold'),
                  f'step {step or "-":<14} task …{(diag.get("task_id") or "-")[-4:]:<5} queue {diag.get("queued", "-"):<3}',
                  (self.held(v['planning']), 'dim'))
        phase = v['box_phase'].value
        self.line('  box       ', ((phase or '-').ljust(46), '' if phase in (None, 'off') else 'bold'),
                  (self.held(v['box_phase']), 'dim'))
        c = v['control'].value
        if c is None:
            self.line('  control   ', ('no /control/status', 'bad'))
        else:
            ready, fault, held, stopped, reason = c
            req = v['hold_req'].value
            self.line('  control   ', ('ready ' if ready else 'NOT READY ', 'ok' if ready else 'bad'),
                      ('FAULT ' if fault else '', 'bad'), ('held ' if held else 'released ', 'wait' if held else 'ok'),
                      ('stopped ' if stopped else 'moving ', ''),
                      ('  planning asks: ', 'dim'),
                      ('-' if req is None else 'HOLD' if req else 'release', 'dim' if req is None else 'wait' if req else 'ok'),
                      ('   ' + reason, 'dim'))
        # ---------------------------------------------------------- localization
        self.head('LOCALIZATION')
        pose = b.pose()
        if pose:
            self.line(f'  pose      x {pose[0]:+7.3f}  y {pose[1]:+7.3f}  yaw {math.degrees(pose[2]):+7.1f}°    '
                      f'v {pose[3]:+5.2f} m/s  w {math.degrees(pose[4]):+6.1f}°/s  ', self.age(v['odom'], 0.5))
        else:
            self.line('  pose      ', ('no /perception/state/odom', 'bad'))
        ekf = v['ekf'].value
        if ekf:
            cov = ekf.pose.covariance
            sx, sy, syaw = (math.sqrt(max(cov[i], 0.0)) for i in (0, 7, 35))
            worst = max(sx, sy)
            level = 'ok' if worst < 0.05 and syaw < math.radians(3) else 'wait' if worst < 0.15 else 'bad'
            grade = {'ok': 'good', 'wait': 'loose', 'bad': 'poor'}[level]
            self.line(f'  EKF σ     x {sx * 1000:5.0f} mm  y {sy * 1000:5.0f} mm  yaw {math.degrees(syaw):4.1f}°    ',
                      (f'[{grade}]', level), '  ', self.age(v['ekf'], 1.0))
        else:
            self.line('  EKF σ     ', ('no /odometry/global', 'wait'))
        fit = b.scan_fit()
        if fit is None:
            why = 'no map (/planning/map or /perception/lidar/map)' if b.map is None else 'no fresh scan / pose' if cv2 else 'no cv2'
            self.line('  scan fit  ', (why, 'wait'))
        else:
            share, n = fit
            level = 'ok' if share >= 0.7 else 'wait' if share >= 0.45 else 'bad'
            bar = '█' * int(share * 20) + '░' * (20 - int(share * 20))
            self.line(f'  scan fit  ', (f'{share * 100:5.1f}% ', level), (bar, level),
                      (f'  {n} pts within {FIT_TOL * 100:.0f} cm of a wall (pose check)', 'dim'))
        glob, state = v['global'].value, v['loc_state'].value
        self.line('  global    ', ((glob or '-').ljust(36), 'ok' if (glob or '').startswith('ok') else 'wait'),
                  ' perception ', (state or '-', 'ok' if state in ('ok', None) else 'bad'))
        # ---------------------------------------------------------- path / obstacles
        self.head('PATH / OBSTACLES')
        left = b.path_left(pose)
        plan_ms, plan_max = diag.get('plan_ms', ''), diag.get('plan_ms_max', '')
        try:
            plan = f'plan {float(plan_ms):5.1f} ms (max {float(plan_max):5.1f})'
        except ValueError:
            plan = 'plan -'
        self.line('  path      ', (f'{left[0]:5.2f} m left of {left[1]} poses' if left else 'no tracking path').ljust(30),
                  plan, '  rev ', diag.get('path_revision', '-'), '  ', self.age(v['path'], 9e9))
        obs = b.nearest_obstacle(pose)
        if obs is None:
            self.line('  obstacles ', ('no /perception/lidar/obstacle', 'wait'))
        else:
            count, near, bearing = obs
            if near is None:
                self.line('  obstacles ', (f'{count} pts  clear', 'ok'), '  ', self.age(v['obstacles'], 1.0))
            else:
                side = 'ahead' if abs(bearing) < 30 else 'behind' if abs(bearing) > 150 else \
                    ('left' if bearing > 0 else 'right')
                level = 'bad' if near < 0.30 else 'wait' if near < 0.60 else 'ok'
                self.line('  obstacles ', f'{count:4d} pts  nearest ', (f'{near:.2f} m {side} ({bearing:+.0f}°)', level),
                          '  ', self.age(v['obstacles'], 1.0))
        waiting = diag.get('waiting_for', '')
        self.line('  waiting   ', (waiting or '-', 'wait' if waiting else 'dim'))
        # ---------------------------------------------------------- mission inputs
        self.head('MISSION INPUTS')
        light = v['light'].value
        if light is not None:
            flag = ('complete ' if light.complete else 'counting ') + ('valid' if light.valid else '')
            self.line('  signal    ', f'R{light.red} Y{light.yellow} G{light.green}  ',
                      (flag, 'ok' if light.complete and light.valid else 'wait'),
                      ('   ' + ''.join('RYG'[s] if s < 3 else '?' for s in light.sequence), 'dim'))
        else:
            self.line('  signal    ', ('-', 'dim'))
        survey = v['survey'].value
        self.line('  barcode   ', (v['barcode'].value or '-').ljust(12),
                  '  survey ', (f'{len(survey.ids)}/12' if survey is not None else '-',
                               'ok' if survey is not None and survey.complete else ''))
        # ---------------------------------------------------------- log
        h, _ = self.w.getmaxyx()
        rows = max(0, h - self.row - 4)
        self.head('LOG' + ('  (paused)' if self.paused else ''))
        if not self.paused:
            self.shown = list(b.log)[-rows:] if rows else []
        for t, source, text in getattr(self, 'shown', []):
            style = 'bad' if 'FAULT' in text.upper() or 'Fault' in text else 'wait' if 'waiting' in text else ''
            self.line((time.strftime('  %H:%M:%S ', time.localtime(t)), 'dim'), (f'{source:<9}', 'head'), (text, style))
        # ---------------------------------------------------------- topics
        self.row = h - 2
        self.head('TOPICS')
        parts = []
        for key, name in (('odom', 'odom'), ('scan', 'lidar'), ('control', 'control'), ('planning', 'planning'),
                          ('diag', 'diag'), ('obstacles', 'obstacles'), ('ekf', 'ekf')):
            a = v[key].age()
            parts += [f' {name} ', ('-' if a is None else f'{a:.1f}s', 'bad' if a is None or a > 2 else 'dim')]
        self.row = h - 1
        col = 0
        for p in parts:
            text, style = (p, '') if isinstance(p, str) else p
            col = self.put(col, text, style)
        self.w.refresh()

    def keys(self):
        while True:
            k = self.w.getch()
            if k == -1:
                return True
            if k in (ord('q'), ord('Q'), 27):
                return False
            if k in (ord('p'), ord('P')):
                self.paused = not self.paused


def run(win):
    node = rclpy.create_node('craft_tui')
    board = Board(node)
    screen = Screen(win, board)
    next_draw = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.time() >= next_draw:
                if not screen.keys():
                    break
                screen.draw()
                next_draw = time.time() + 0.25
    except ExternalShutdownException:  # Ctrl+C / SIGTERM: rclpy shut down under the spin
        pass
    node.destroy_node()


def print_once(seconds, rows, cols):
    """--print S: listen S seconds, print one screen as text (logs, no terminal needed)."""
    node = rclpy.create_node('craft_tui')
    board = Board(node)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    win = TextWindow(rows, cols)
    Screen(win, board).draw()
    print(win.text())
    node.destroy_node()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if args and args[0] == '--print':
        rclpy.init()
        try:
            print_once(float(args[1]) if len(args) > 1 else 3.0, 46, 110)
        finally:
            rclpy.try_shutdown()
        return
    if not sys.stdout.isatty():
        sys.exit('craft_tui needs a terminal (or: craft_tui.py --print [seconds])')
    rclpy.init()
    try:
        curses.wrapper(run)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
