"""Further robot signals besides the state machine: what control does with a task, what was commanded, the
barcode, the gripper, the nearest LiDAR obstacle, the box observation.

Each is kept with the time it arrived and turned into
  - a line of the GPT facts block (lines), so "왜 멈췄어?", "앞에 뭐 있어?" are answered from what was received;
  - for a few of them a remark said at once in fixed words (the returned key: 'grasped', 'barcode_ascending', ...).
Nothing here is inferred: a signal that did not arrive, or is old, is left out.
"""

from __future__ import annotations

import math
import statistics
from collections import deque

from .variety import direction, metres
from .traffic import observation as traffic_observation, facts as traffic_facts
from .flow import BoxEvidence

# craft_interfaces/TaskResult
RUNNING, SUCCEEDED, FAILED, STOPPING, STOPPED, REJECTED = range(6)
RESULT_KO = {RUNNING: '실행 중', SUCCEEDED: '완료', FAILED: '실패', STOPPING: '멈추는 중', STOPPED: '멈춤',
             REJECTED: '거부됨'}
ENDED_KO = {FAILED: '실패로', REJECTED: '거부하고'}
CONTROLLERS_KO = ('팔', '리니어(lm)', '바퀴', '회전판(clock)', '그리퍼')   # TaskTracker columns
ACTIVE = 1
# craft_interfaces/TaskCommand
EXECUTE, UPDATE_PATH, STOP = range(3)
NAVIGATE, MANIPULATOR, GRIPPER, CLOCK, WAIT = range(5)
# perception: P2-01 / P2-03 ascending, P2-02 / P2-04 descending (parse_barcode)
BARCODE_ORDER = {'P2-01': 'ascending', 'P2-03': 'ascending', 'P2-02': 'descending', 'P2-04': 'descending'}
ORDER_KO = {'ascending': '오름차순', 'descending': '내림차순'}
# perception /perception/vision/uppercam/traffic_light (craft_perception/TrafficLight): 12 blinks at the start
TRAFFIC_BLINKS = 12
# Remarks said whatever the planning state is (perception and the wheels, not the state machine).
STATE_FREE_REMARKS = ('barcode_ascending', 'barcode_descending', 'barcode_unknown', 'traffic_start', 'traffic_done',
                      'traffic_recount', 'puck_axis', 'clock_unload', 'heartbeat_lost', 'firmware_done',
                      'approach_line', 'gripcam_measured', 'gripcam_none', 'puck_loaded', 'pick_failed')
# Remarks another program asks for on /voice/event: the firmware upload (resetwheel / resetarm on the Jetson) and the
# puck pick (demo/puck_pick/execute.py): "gripcam_measured top=61 yaw=-22.9" after the gripper cam read the puck,
# "gripcam_none" when it saw none (the numbers are kept in the memory, not said), "puck_loaded slot=7" after the release in a carousel slot. key=value words are
# values for the sentence (numbers; anything else is ignored).
EVENT_REMARKS = ('firmware_done', 'gripcam_measured', 'gripcam_none', 'puck_loaded', 'pick_failed')
EVENT_GAP_S = {'firmware_done': 10.0, 'gripcam_measured': 3.0, 'gripcam_none': 3.0, 'puck_loaded': 3.0, 'pick_failed': 3.0}
# A puck axis found again only after this long without one is news ("벡터 계산 완료"); not again within PUCK_AXIS_GAP_S.
PUCK_AXIS_ABSENT_S = 5.0
PUCK_AXIS_GAP_S = 60.0
# A CLOCK turn with an empty gripper arms "taking a stored puck out of the clock" for this long: the next gripper
# close is the grasp in the slot (unloading: dice, sis2). Loading turns the clock while holding a puck: not armed.
CLOCK_UNLOAD_ARM_S = 120.0
# /planning/hold is planning's 20 Hz heartbeat to control (control holds every axis after 0.5 s without it).
HEARTBEAT_TIMEOUT_S = 1.0
# Motor supply: control_master republishes each board's firmware diagnostic line about once a second,
#   /control/wheel_clock_diag (wheels + clock), /control/arm_diag (arm + LM):
#   "D,reason,motor id,code,lowest V since the last line,V per motor,...,-1,deg C per motor,-1,peak A per motor".
# A drive sags the supply (2026-11-15 mission: 11.5 V standing, 10.4-11.2 V while driving), so the battery is
# judged by its resting level: the highest per-motor median of the last SUPPLY_WINDOW_S (a mission stops often
# enough: the longest stretch without a calm line was 13 s). Dips alone are never said.
SUPPLY_BOARDS = {'wheel': '바퀴 모터', 'arm': '팔 모터'}
SUPPLY_WINDOW_S = 60.0
SUPPLY_MIN_SPAN_S = 40.0      # this much of the window received before judging (no warning at startup)
SUPPLY_RECOVER_V = 0.3        # charged / replaced: back above low_v + this, said again when it drops later
SUPPLY_REPEAT_S = {'battery_low': 600.0, 'battery_critical': 180.0}
SUPPLY_REMARKS = tuple(f'{level}_{board}' for level in SUPPLY_REPEAT_S for board in SUPPLY_BOARDS)
STATE_FREE_REMARKS = STATE_FREE_REMARKS + SUPPLY_REMARKS
REMARKS = ('grasped', 'box_found', 'depart') + STATE_FREE_REMARKS


def parse_supply(line: str) -> tuple[float, float] | None:
    """(resting candidate: median of the motors' V, lowest V since the last line) of one diagnostic line."""
    try:
        values = [float(x) for x in line.strip().split(',')[1:]]
        end = values.index(-1.0) if -1.0 in values else len(values)
    except ValueError:
        return None
    motors = [v for v in values[4:end] if math.isfinite(v) and v > 0]
    if line.strip()[:2] != 'D,' or len(values) < 4 or not motors:
        return None
    return statistics.median(motors), values[3]


def command_ko(operation: int, kind: int, duration_s: float = 0.0, clock_slot: int = 0, close: bool = False) -> str:
    if operation == STOP:
        return '정지'
    if kind == NAVIGATE:
        return '주행 경로 갱신' if operation == UPDATE_PATH else '주행'
    if kind == MANIPULATOR:
        return '팔 이동'
    if kind == GRIPPER:
        return '그리퍼 닫기' if close else '그리퍼 열기'
    if kind == CLOCK:
        return f'회전판(clock) {clock_slot}번 칸으로 돌리기'
    if kind == WAIT:
        return f'{duration_s:.1f}초 대기' if math.isfinite(duration_s) and duration_s > 0 else '대기'
    return '알 수 없는 작업'


class Signals:
    def __init__(self, stale_s: float = 3.0, keep_s: float = 1800.0, obstacle_range_m: float = 3.0,
                 remark_gap_s: float = 30.0, supply_low_v: float = 11.0, supply_critical_v: float = 10.7):
        self.stale_s, self.keep_s = stale_s, keep_s
        self.obstacle_range_m = obstacle_range_m
        self.remark_gap_s = remark_gap_s          # the same remark again only after this
        self.result: dict | None = None           # {'task_id', 'status', 'controllers', 'reason', 'at'}
        self.failure: dict | None = None          # the last failed / rejected task, kept keep_s
        self.command: dict | None = None          # {'task_id', 'text', 'at'}
        self.barcode: tuple[str, float] | None = None
        self.grasped: tuple[bool, float] | None = None
        self.obstacle: dict | None = None         # {'distance', 'bearing', 'count', 'at'} (count 0 = none in range)
        self.box: dict | None = None
        self.box_valid_at = -math.inf             # the last valid box observation
        self.box_evidence = BoxEvidence()
        self.traffic: dict | None = None          # {'red', 'yellow', 'green', 'blinks', 'complete', 'valid', 'discarded', 'at'}
        self._traffic_notices: set[str] = set()
        self._remarked: dict[str, float] = {}
        self._puck_axis_at = -math.inf            # the last observation with a valid axis
        self._gripper_closed: bool | None = None  # the last gripper command (None = none seen yet)
        self._clock_unload_armed_at = -math.inf
        self.heartbeat_at = -math.inf             # the last /planning/hold message
        self.heartbeat_lost = False               # said once per outage
        self.event_values: dict[str, dict] = {}   # /voice/event name -> the numbers that came with it
        self.supply_low_v, self.supply_critical_v = supply_low_v, supply_critical_v
        self.supply: dict[str, dict] = {}         # board -> {'samples', 'rest', 'lowest', 'at', 'said', 'said_at'}

    def _remark(self, key: str, now: float, gap: float | None = None) -> str:
        if now - self._remarked.get(key, -math.inf) < (self.remark_gap_s if gap is None else gap):
            return ''
        self._remarked[key] = now
        return key

    # ---- control: the task it runs -----------------------------------------------------------
    def task_result(self, task_id: int, status: int, controllers, reason: str, now: float) -> str:
        """Returns what to remember ('' = nothing new): a task that failed or was rejected, once."""
        previous = self.result
        self.result = {'task_id': int(task_id), 'status': int(status), 'controllers': tuple(controllers),
                       'reason': reason.strip(), 'at': now}
        if status not in (FAILED, REJECTED):
            return ''
        if previous and previous['task_id'] == task_id and previous['status'] == status:
            return ''
        self.failure = dict(self.result)
        return f'제어가 task {task_id}을 {ENDED_KO[status]} 끝냄' + (f': {reason.strip()}' if reason.strip() else '')

    def task_command(self, task_id: int, operation: int, kind: int, duration_s: float, clock_slot: int,
                     close: bool, now: float) -> str:
        """Returns what to remember ('' for a path update of the same drive)."""
        text = command_ko(operation, kind, duration_s, clock_slot, close)
        new = operation != UPDATE_PATH
        self.command = {'task_id': int(task_id), 'text': text, 'at': now}
        return f'제어에 내린 명령: {text}' if new and kind != NAVIGATE else ''

    def clock_unload(self, operation: int, kind: int, close: bool, now: float) -> str:
        """The same TaskCommand: 'clock_unload' when the gripper closes on a puck stored in the clock (a clock turn
        with an empty gripper, then the first gripper close), else ''."""
        if operation != EXECUTE:
            return ''
        if kind == CLOCK:
            self._clock_unload_armed_at = now if not self._gripper_closed else -math.inf
        elif kind == GRIPPER:
            self._gripper_closed = bool(close)
            if close and now - self._clock_unload_armed_at <= CLOCK_UNLOAD_ARM_S:
                self._clock_unload_armed_at = -math.inf
                return self._remark('clock_unload', now, gap=5.0)
        return ''

    # ---- planning heartbeat --------------------------------------------------------------------
    def heartbeat(self, now: float):
        """One /planning/hold message (any value: it is the arrival that counts)."""
        self.heartbeat_at = now
        self.heartbeat_lost = False

    def heartbeat_check(self, now: float) -> str:
        """'heartbeat_lost' once when the heartbeat that was arriving stops for HEARTBEAT_TIMEOUT_S."""
        if self.heartbeat_lost or self.heartbeat_at == -math.inf or now - self.heartbeat_at < HEARTBEAT_TIMEOUT_S:
            return ''
        self.heartbeat_lost = True
        return self._remark('heartbeat_lost', now, gap=0.0)

    def event(self, text: str, now: float) -> str:
        """A remark asked for by name on /voice/event ('' = not one of EVENT_REMARKS, or said just now).
        "name key=value ...": the numbers are kept for the sentence (event_values)."""
        name, *words = text.split() or ['']
        if name not in EVENT_REMARKS:
            return ''
        values = {}
        for word in words:
            key, _, value = word.partition('=')
            try:
                number = float(value)
            except ValueError:
                continue
            if key and math.isfinite(number):
                values[key] = number
        key = self._remark(name, now, gap=EVENT_GAP_S.get(name, 10.0))
        if key:
            self.event_values[name] = values
        return key

    def event_words(self, key: str) -> tuple[str, dict]:
        """(phrase key, words) for an event remark (the carousel slot is said; gripper-cam numbers are not)."""
        v = self.event_values.get(key, {})
        if key == 'puck_loaded' and 'slot' in v:
            return 'notice.puck_loaded.slot', {'slot': f"{v['slot']:.0f}"}
        return 'notice.' + key, {}

    # ---- motor supply (battery) -------------------------------------------------------------
    def motor_supply(self, board: str, line: str, now: float) -> str:
        """One diagnostic line of board 'wheel' / 'arm'. Returns 'battery_low_wheel', 'battery_critical_arm', ...
        when the resting level falls below a threshold (again after SUPPLY_REPEAT_S while it stays there)."""
        parsed = parse_supply(line) if board in SUPPLY_BOARDS else None
        if parsed is None or self.supply_low_v <= 0:
            return ''
        level, lowest = parsed
        s = self.supply.setdefault(board, {'samples': deque(), 'rest': None, 'lowest': lowest, 'at': now,
                                           'said': 0, 'said_at': -math.inf})
        samples = s['samples']
        samples.append((now, level))
        while samples and now - samples[0][0] > SUPPLY_WINDOW_S:
            samples.popleft()
        s['lowest'], s['at'] = lowest, now
        if now - samples[0][0] < SUPPLY_MIN_SPAN_S:
            return ''
        rest = s['rest'] = max(v for _, v in samples)
        if rest >= self.supply_low_v + SUPPLY_RECOVER_V:
            s['said'] = 0  # charged: a later drop is news again
        rank = 2 if rest < self.supply_critical_v else 1 if rest < self.supply_low_v else 0
        if not rank:
            return ''
        key = ('battery_critical' if rank == 2 else 'battery_low')
        if rank > s['said'] or (rank == s['said'] and now - s['said_at'] >= SUPPLY_REPEAT_S[key]):
            s['said'], s['said_at'] = rank, now
            return f'{key}_{board}'
        return ''

    def supply_values(self, key: str) -> dict:
        """Words for a battery remark: {'board': '바퀴 모터', 'volts': '10.9'}."""
        board = key.rsplit('_', 1)[-1]
        s = self.supply.get(board)
        rest = s['rest'] if s and s['rest'] is not None else 0.0
        return {'board': SUPPLY_BOARDS.get(board, board), 'volts': f'{rest:.1f}'}

    # ---- perception --------------------------------------------------------------------------
    def puck_axis(self, found: bool, now: float) -> str:
        """One puck observation message: 'puck_axis' when an axis is found after PUCK_AXIS_ABSENT_S without one."""
        if not found:
            return ''
        previous, self._puck_axis_at = self._puck_axis_at, now
        return self._remark('puck_axis', now, gap=PUCK_AXIS_GAP_S) if now - previous >= PUCK_AXIS_ABSENT_S else ''

    def read_barcode(self, text: str, now: float, quiet: bool = False) -> str:
        """Returns the remark key for a barcode read just now ('' = empty, the same again, or quiet)."""
        text = text.strip()
        previous = self.barcode[0] if self.barcode else ''
        self.barcode = (text, now) if text else None
        if not text or text == previous or quiet:
            return ''
        return self._remark('barcode_' + BARCODE_ORDER.get(text, 'unknown'), now, gap=0.0)

    def traffic_light(self, red: int, yellow: int, green: int, blinks: int, complete: bool, valid: bool,
                      discarded: int, now: float, quiet: bool = False, sequence=None) -> str:
        """One TrafficLight message (one per accepted blink, and on completion / reset). Returns the remark key:
        'traffic_start' at the first blink of a count, 'traffic_recount' when a wrong count was thrown away,
        'traffic_done' when 12 valid blinks are complete ('' = nothing new, or quiet)."""
        previous = self.traffic
        self.traffic = traffic_observation(red, yellow, green, blinks, complete, valid, discarded, now, sequence)
        if self.traffic['error']:
            return ''
        # An explicit reset/new cycle re-arms notices; a discarded count is the SAME session.
        if previous and (discarded < previous['discarded'] or
                         (not complete and previous['complete']) or
                         (blinks == 0 and discarded == 0 and previous['blinks'] > 0)):
            self._traffic_notices.clear()
        key = ''
        if complete and valid:
            key = 'traffic_done'
        elif previous and discarded > previous['discarded']:
            key = 'traffic_recount'
        elif blinks == 1 and not complete and discarded == 0:
            key = 'traffic_start'
        seen = key in self._traffic_notices
        if blinks > 0 or discarded > 0:
            self._traffic_notices.add('traffic_start')
        if key:
            self._traffic_notices.add(key)
        return key if key and not seen and not quiet else ''

    def traffic_values(self) -> dict:
        t = self.traffic
        return {'red': t['red'], 'yellow': t['yellow'], 'green': t['green']} if t else {}

    def traffic_notice_current(self, key: str) -> bool:
        t = self.traffic
        if not t or t['error'] or t['complete']:
            return False
        if key == 'traffic_start':
            return t['blinks'] > 0 and t['discarded'] == 0
        return key == 'traffic_recount' and t['discarded'] > 0

    def gripper(self, grasped: bool, now: float) -> str:
        previous = self.grasped
        self.grasped = (bool(grasped), now)
        if grasped and previous is not None and not previous[0]:
            return self._remark('grasped', now, gap=5.0)
        return ''

    def obstacles(self, points, now: float, pose=None):
        """points: [(x, y)]; pose (x, y, yaw) of the robot in the same frame, None when the points are already
        seen from the robot. Keeps the nearest one within range."""
        best = None
        count = 0
        for x, y in points:
            if pose is not None:
                dx, dy = x - pose[0], y - pose[1]
                cos, sin = math.cos(-pose[2]), math.sin(-pose[2])
                x, y = dx * cos - dy * sin, dx * sin + dy * cos
            distance = math.hypot(x, y)
            if not math.isfinite(distance) or distance > self.obstacle_range_m:
                continue
            count += 1
            if best is None or distance < best[0]:
                best = (distance, math.atan2(y, x))
        self.obstacle = {'distance': best[0] if best else None, 'bearing': best[1] if best else None,
                         'count': count, 'at': now}

    def nearest_obstacle(self, now: float) -> tuple[float, float] | None:
        """(distance m, bearing rad) of the nearest obstacle, while fresh."""
        o = self.obstacle
        if o is None or now - o['at'] > self.stale_s or o['distance'] is None:
            return None
        return o['distance'], o['bearing']

    def box_request(self, request_id: int, source: str, now: float, box_id: str = '',
                    box_side: float = math.nan):
        if self.box_evidence.request(request_id, source, now, box_id, box_side):
            self.box = None
            self.box_valid_at = -math.inf

    def box_observation(self, valid: bool, source: str, reason: str, gap_m: float, lateral_m: float,
                        yaw_rad: float, now: float, **quality) -> str:
        confirmed, notice = self.box_evidence.observe(
            valid, source, (gap_m, lateral_m, yaw_rad), now, reason=reason, **quality)
        self.box = {'valid': confirmed, 'raw_valid': bool(valid), 'source': source.strip(),
                    'reason': reason.strip() if isinstance(reason, str) else '', 'gap': gap_m,
                    'lateral': lateral_m, 'yaw': yaw_rad,
                    'at': now - quality['age_s'] if confirmed else now}
        if confirmed:
            self.box_valid_at = self.box['at']
        else:
            self.box_valid_at = -math.inf
        return 'box_found' if notice else ''

    # ---- sentences ---------------------------------------------------------------------------
    def obstacle_words(self, now: float) -> tuple[str, int] | None:
        """(side, whole metres) of the nearest obstacle for a spoken sentence."""
        nearest = self.nearest_obstacle(now)
        if nearest is None:
            return None
        d, side = metres(nearest[0]), direction(nearest[1])
        return (side, d) if side and d is not None else None

    def lines(self, now: float, ago=lambda s: f'{s:.0f}초 전') -> list[str]:
        """Facts for the GPT prompt, only what was received."""
        out = []
        c = self.command
        if c and now - c['at'] <= self.keep_s:
            out.append(f"- 제어에 마지막으로 내린 명령(/planning/task_command): {c['text']} (task {c['task_id']}), "
                       f"{ago(now - c['at'])}")
        r = self.result
        if r and now - r['at'] <= self.stale_s:
            line = f"- 제어의 작업 결과(/control/task_result): task {r['task_id']} {RESULT_KO.get(r['status'], '알 수 없음')}"
            moving = [name for name, state in zip(CONTROLLERS_KO, r['controllers']) if state == ACTIVE]
            if r['status'] == RUNNING and moving:
                line += ', 아직 움직이는 축: ' + ', '.join(moving)
            if r['reason']:
                line += f", 이유: {r['reason']}"
            out.append(line)
        f = self.failure
        if f and now - f['at'] <= self.keep_s and not (r and now - r['at'] <= self.stale_s and
                                                      r['task_id'] == f['task_id'] and r['status'] == f['status']):
            out.append(f"- 제어가 {ENDED_KO[f['status']]} 끝낸 작업: task {f['task_id']}"
                       + (f", 이유: {f['reason']}" if f['reason'] else '') + f", {ago(now - f['at'])}")
        if self.barcode and now - self.barcode[1] <= self.keep_s:
            code = self.barcode[0]
            order = ORDER_KO.get(BARCODE_ORDER.get(code, ''), '등록되지 않은 코드')
            out.append(f'- 읽은 바코드: {code} ({order}), {ago(now - self.barcode[1])}')
        t = self.traffic
        if t and now - t['at'] <= self.keep_s:
            if t.get('error'):
                out.append('- ' + traffic_facts(t))
            elif t['complete']:
                state = '12번 다 셈' if t['valid'] else '끝났지만 횟수가 맞지 않음'
            else:
                state = f"세는 중 {t['blinks']}/{TRAFFIC_BLINKS}"
            if not t.get('error'):
                out.append(f"- 신호등 깜빡임(/perception/vision/uppercam/traffic_light): 빨강 {t['red']}번, "
                           f"노랑 {t['yellow']}번, 초록 {t['green']}번 ({state}"
                           + (f", 다시 센 횟수 {t['discarded']}" if t['discarded'] else '') + f"), {ago(now - t['at'])}")
                out.append('- ' + traffic_facts(t))
        for board, s in self.supply.items():
            if s['rest'] is not None and now - s['at'] <= max(self.stale_s, 5.0):
                state = ('거의 바닥, 바로 충전 필요' if s['rest'] < self.supply_critical_v else
                         '부족, 곧 충전 필요' if s['rest'] < self.supply_low_v else '충분')
                out.append(f"- {SUPPLY_BOARDS[board]} 배터리 전압: 쉬는 동안 {s['rest']:.1f} V ({state}, "
                           f"부족 기준 {self.supply_low_v:.1f} V), 방금 순간 최저 {s['lowest']:.1f} V "
                           '(움직일 때 잠깐 떨어지는 것은 정상)')
        o = self.obstacle
        if o and now - o['at'] <= self.stale_s:
            if o['distance'] is None:
                out.append(f'- 라이다 장애물: {self.obstacle_range_m:.0f} m 안에 없음')
            else:
                out.append(f"- 가장 가까운 라이다 장애물: {direction(o['bearing'])} {o['distance']:.1f} m "
                           f"(장애물 점 {o['count']}개)")
        b = self.box
        if b and now - b['at'] <= self.stale_s:
            source = f"({b['source']})" if b['source'] else ''
            if b['valid'] and all(math.isfinite(v) for v in (b['gap'], b['lateral'], b['yaw'])):
                out.append(f"- 상자 면 양끝 연속 관측{source}: 앞 간격 {b['gap']:.2f} m, 좌우 어긋남 {b['lateral']:+.2f} m, "
                           f"기울기 {math.degrees(b['yaw']):+.0f}도. 접근 완료나 물체 식별 성공을 뜻하지 않음")
            elif b['raw_valid']:
                out.append(f'- 상자 확인 미완료{source}: 면 후보만 있거나 관측 근거가 부족함. 상자를 찾았다고 말하지 말 것')
            elif not b['valid']:
                out.append(f'- 상자 관측{source}: 실패' + (f", 이유: {b['reason']}" if b['reason'] else ''))
        return out
