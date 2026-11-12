"""Further robot signals besides the state machine: what control does with a task, what was commanded, the
barcode, the gripper, the nearest LiDAR obstacle, the box observation.

Each is kept with the time it arrived and turned into
  - a line of the GPT facts block (lines), so "왜 멈췄어?", "앞에 뭐 있어?" are answered from what was received;
  - for a few of them a remark said at once in fixed words (the returned key: 'grasped', 'barcode_ascending', ...).
Nothing here is inferred: a signal that did not arrive, or is old, is left out.
"""

from __future__ import annotations

import math

from .variety import direction, metres

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
                      'traffic_recount')
REMARKS = ('grasped', 'box_found', 'depart') + STATE_FREE_REMARKS


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
                 remark_gap_s: float = 30.0):
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
        self.traffic: dict | None = None          # {'red', 'yellow', 'green', 'blinks', 'complete', 'valid', 'discarded', 'at'}
        self._remarked: dict[str, float] = {}

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

    # ---- perception --------------------------------------------------------------------------
    def read_barcode(self, text: str, now: float, quiet: bool = False) -> str:
        """Returns the remark key for a barcode read just now ('' = empty, the same again, or quiet)."""
        text = text.strip()
        previous = self.barcode[0] if self.barcode else ''
        self.barcode = (text, now) if text else None
        if not text or text == previous or quiet:
            return ''
        return self._remark('barcode_' + BARCODE_ORDER.get(text, 'unknown'), now, gap=0.0)

    def traffic_light(self, red: int, yellow: int, green: int, blinks: int, complete: bool, valid: bool,
                      discarded: int, now: float, quiet: bool = False) -> str:
        """One TrafficLight message (one per accepted blink, and on completion / reset). Returns the remark key:
        'traffic_start' at the first blink of a count, 'traffic_recount' when a wrong count was thrown away,
        'traffic_done' when 12 valid blinks are complete ('' = nothing new, or quiet)."""
        previous = self.traffic
        self.traffic = {'red': int(red), 'yellow': int(yellow), 'green': int(green), 'blinks': int(blinks),
                        'complete': bool(complete), 'valid': bool(valid), 'discarded': int(discarded), 'at': now}
        if quiet:
            return ''
        if complete and valid:
            if previous and previous['complete'] and previous['valid'] and \
                    (previous['red'], previous['yellow'], previous['green']) == (red, yellow, green):
                return ''  # the same result again (latched message after a restart, a repeat)
            return self._remark('traffic_done', now, gap=0.0)
        if previous and discarded > previous['discarded']:
            return self._remark('traffic_recount', now, gap=5.0)
        if blinks == 1 and not complete:
            return self._remark('traffic_start', now, gap=10.0)
        return ''

    def traffic_values(self) -> dict:
        t = self.traffic
        return {'red': t['red'], 'yellow': t['yellow'], 'green': t['green']} if t else {}

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

    def box_observation(self, valid: bool, source: str, reason: str, gap_m: float, lateral_m: float,
                        yaw_rad: float, now: float) -> str:
        """Returns 'box_found' when a valid observation comes after none (or after an invalid one)."""
        previous = self.box
        self.box = {'valid': bool(valid), 'source': source.strip(), 'reason': reason.strip(), 'gap': gap_m,
                    'lateral': lateral_m, 'yaw': yaw_rad, 'at': now}
        was_valid = previous is not None and previous['valid'] and now - previous['at'] <= self.stale_s
        return self._remark('box_found', now) if valid and not was_valid else ''

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
            if t['complete']:
                state = '12번 다 셈' if t['valid'] else '끝났지만 횟수가 맞지 않음'
            else:
                state = f"세는 중 {t['blinks']}/{TRAFFIC_BLINKS}"
            out.append(f"- 신호등 깜빡임(/perception/vision/uppercam/traffic_light): 빨강 {t['red']}번, "
                       f"노랑 {t['yellow']}번, 초록 {t['green']}번 ({state}"
                       + (f", 다시 센 횟수 {t['discarded']}" if t['discarded'] else '') + f"), {ago(now - t['at'])}")
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
                out.append(f"- 상자 관측{source}: 앞 간격 {b['gap']:.2f} m, 좌우 어긋남 {b['lateral']:+.2f} m, "
                           f"기울기 {math.degrees(b['yaw']):+.0f}도")
            elif not b['valid']:
                out.append(f'- 상자 관측{source}: 실패' + (f", 이유: {b['reason']}" if b['reason'] else ''))
        return out
