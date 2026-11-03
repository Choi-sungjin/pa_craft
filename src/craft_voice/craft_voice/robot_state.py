"""What the robot is doing, from the planning state machine and friends.

planning_master publishes its StateMachine label on /planning/state every loop
(20 Hz): Idle, Ready, Run.AwaitRequest, Run.PrepareTask, Run.Compute,
Run.Execute, Run.Complete (one loop only), Run.Stopping, Fault. The fault
reason and current step come about once a second on /planning/diagnostics
(status 'planning/state_machine', message 'Fault: <phase>: <reason>').

StateTracker turns that stream, plus what perception sees (motion, YOLO
detections, pucks, AprilTags), into "what I am doing and what is around me":
facts for the GPT prompt, and announcement events on label changes. Spoken
sentences describe actions and situations, never state labels or error codes.
A Fault event waits up to fault_reason_wait_s for the reason so it can say why.
"""

from __future__ import annotations

import math
from collections import Counter, deque
from dataclasses import dataclass, field

STATE_KO = {
    'Idle': '시스템 초기화 중(센서·제어 준비 대기)',
    'Ready': '준비 완료, 미션 시작 대기',
    'Run.AwaitRequest': '다음 작업 대기',
    'Run.PrepareTask': '다음 작업 준비',
    'Run.Compute': '경로·팔 자세 계산',
    'Run.Execute': '작업 실행',
    'Run.Complete': '작업 하나 완료',
    'Run.Stopping': '정지 중',
    'Fault': '고장으로 정지',
}

STEP_KO = {'base_goal': '목표 지점 이동', 'tcp_goal': '팔 목표 자세 이동'}

# Substrings of planning fault reasons (state_machine.cpp, planning_master.cpp) ->
# (short Korean for the GPT facts, spoken situation: what happened and what the robot is doing now).
REASON_KO = (
    ('initialization timeout', '초기화 시간 초과', '준비가 끝나지 않아서 아직 출발하지 못하고 있어요.'),
    ('task preparation timeout', '작업 준비 시간 초과', '작업 준비가 늦어져서 멈춰 있어요.'),
    ('compute timeout', '계산 시간 초과', '갈 길을 찾지 못해서 멈춰 있어요.'),
    ('calculation failed', '경로나 자세 계산 실패', '갈 길을 찾지 못해서 멈춰 있어요.'),
    ('execution timeout', '실행 시간 초과', '목표까지 너무 오래 걸려서 멈춰 있어요.'),
    ('stop acknowledgement timeout', '정지 확인 시간 초과', '멈췄는지 확인이 늦어져서 그 자리에 서 있어요.'),
    ('localization lost', '위치 추정 실패', '제 위치를 놓쳐서 멈춰 있어요.'),
    ('control unavailable', '제어기 응답 없음', '바퀴와 팔 제어 연결이 끊겨서 멈춰 있어요.'),
    ('stale odom/lidar input', '오도메트리나 라이다 입력 끊김', '센서 정보가 끊겨서 멈춰 있어요.'),
    ('stale control task result', '제어 결과 응답 끊김', '제어 쪽 응답이 끊겨서 멈춰 있어요.'),
    ('no progress', '진행 없음', '앞으로 나아가지 못해서 멈춰 있어요.'),
    ('module/feedback fault', '모듈 피드백 이상', '장치 응답이 이상해서 멈춰 있어요.'),
)

# craft_perception YOLO labels (perception.yaml class_names) -> Korean.
OBJECT_KO = {'knob': '손잡이', 'sign_red': '빨간 신호', 'sign_yellow': '노란 신호', 'sign_green': '초록 신호',
             'sign_off': '꺼진 신호', 'puck': '퍽', 'lever': '레버'}

DIAGNOSTIC_NAME = 'planning/state_machine'
ACTIVE_PHASES = ('Run.PrepareTask', 'Run.Compute', 'Run.Execute')  # one step, start to finish


def parse_list(text: str) -> list[str]:
    return [item.strip() for item in text.split(',') if item.strip()]


def parse_mapping(text: str) -> dict[str, str]:
    """'approach:접근, grasp:잡기' -> {'approach': '접근', 'grasp': '잡기'}."""
    result = {}
    for item in parse_list(text):
        key, sep, value = item.partition(':')
        if sep and key.strip() and value.strip():
            result[key.strip()] = value.strip()
    return result


def reason_ko(reason: str) -> str:
    lowered = reason.lower()
    for needle, short, _ in REASON_KO:
        if needle in lowered:
            return short
    return reason.strip()


def copula(word: str) -> str:
    """'이에요' after a final consonant (받침), '예요' otherwise: '3번이에요', '2개예요'."""
    last = word.rstrip()[-1:] or '가'
    has_batchim = '가' <= last <= '힣' and (ord(last) - 0xAC00) % 28 != 0
    return '이에요' if has_batchim or last.isdigit() else '예요'


def fault_situation(reason: str) -> str:
    """Spoken sentence for a fault: the situation, never the raw error text."""
    lowered = reason.lower()
    for needle, _, situation in REASON_KO:
        if needle in lowered:
            return situation
    return '지금은 멈춰 있어요. 다시 움직이려면 확인이 필요해요.' if reason else '지금은 멈춰 있어요.'


@dataclass(frozen=True)
class StateEvent:
    label: str
    previous: str | None
    stamp: float
    step: str
    reason: str
    generation: int

    @property
    def fault(self) -> bool:
        return self.label == 'Fault'


@dataclass
class ControlSnapshot:
    ready: bool = False
    fault: bool = False
    held: bool = False
    stopped: bool = False
    reason: str = ''
    at: float = -math.inf


@dataclass
class StateTracker:
    announce: tuple = ('Ready', 'Run.Execute', 'Run.Complete', 'Run.Stopping', 'Fault')
    cooldown_s: float = 10.0
    fault_reason_wait_s: float = 1.2
    stale_s: float = 3.0
    perception_stale_s: float = 2.0
    step_names: dict = field(default_factory=dict)

    def __post_init__(self):
        self.label: str | None = None
        self.label_since = -math.inf
        self.label_at = -math.inf
        self.diag: dict[str, str] = {}
        self.diag_message = ''
        self.diag_level = 0
        self.diag_at = -math.inf
        self.step_started_at = -math.inf  # current step began: diagnostics older than this name the previous step
        self.fault_reason = ''
        self.fault_reason_at = -math.inf
        self.control = ControlSnapshot()
        self.context: dict[str, tuple[str, float]] = {}
        self.motion: tuple[float, float, float] | None = None       # (v m/s, w rad/s, at)
        self.objects: tuple[Counter, float] | None = None           # YOLO labels seen in the last frame
        self.pucks: tuple[list, float] | None = None                # [(standing, distance m)]
        self.tags: tuple[list, float] | None = None                 # AprilTag ids
        self.history: deque = deque(maxlen=8)
        self.generation = 0
        self._announced_at: dict[str, float] = {}
        self._pending_fault: StateEvent | None = None

    # ---- inputs -------------------------------------------------------------------------
    def update_planning(self, label: str, now: float) -> StateEvent | None:
        """Feed one /planning/state message. Returns an event to announce now, if any."""
        label = label.strip()
        if not label:
            return None
        self.label_at = now
        if label == self.label:
            return None
        previous, self.label, self.label_since = self.label, label, now
        if previous is not None and (label == 'Run.PrepareTask' or
                                     (label in ACTIVE_PHASES and previous not in ACTIVE_PHASES)):
            # A new step begins: PrepareTask only ever starts one, and entering Compute/Execute from
            # outside covers a missed PrepareTask sample. The first label after startup is not a step
            # start: diagnostics already received describe that same step.
            self.step_started_at = now
        self.history.append((now, label))
        self.generation += 1
        if label != 'Fault':
            self._pending_fault = None
        if label not in self.announce:
            return None
        event = StateEvent(label, previous, now, self.step_name(), '', self.generation)
        if event.fault:
            # Speak once the reason is in (or after fault_reason_wait_s), never rate limited.
            self._pending_fault = event
            return self.poll(now)
        if now - self._announced_at.get(label, -math.inf) < self.cooldown_s:
            return None
        self._announced_at[label] = now
        return event

    def update_diagnostics(self, name: str, level: int, message: str, values: dict, now: float):
        if name != DIAGNOSTIC_NAME:
            return
        self.diag = dict(values)
        self.diag_message = message
        self.diag_level = int(level)
        self.diag_at = now
        if message.startswith('Fault:'):
            reason = message[len('Fault:'):].strip()
            if reason:
                self.fault_reason, self.fault_reason_at = reason, now

    def update_control(self, ready: bool, fault: bool, held: bool, stopped: bool, reason: str, now: float):
        self.control = ControlSnapshot(bool(ready), bool(fault), bool(held), bool(stopped), reason, now)

    def update_context(self, topic: str, value: str, now: float):
        self.context[topic] = (value.strip(), now)

    # ---- what perception sees ------------------------------------------------------------
    def update_motion(self, linear_x: float, angular_z: float, now: float):
        self.motion = (float(linear_x), float(angular_z), now)

    def update_objects(self, labels, now: float):
        self.objects = (Counter(label for label in labels if label), now)

    def update_pucks(self, pucks, now: float):
        """pucks: [(standing: bool, distance from the camera in m)]."""
        self.pucks = (sorted(((bool(st), float(d)) for st, d in pucks), key=lambda p: p[1]), now)

    def update_tags(self, ids, now: float):
        self.tags = (sorted({int(i) for i in ids}), now)

    def poll(self, now: float) -> StateEvent | None:
        """Pending Fault event, once its reason arrived or the wait ran out."""
        event = self._pending_fault
        if event is None or self.label != 'Fault' or event.generation != self.generation:
            self._pending_fault = None
            return None
        has_reason = self.fault_reason_at >= event.stamp - 0.05
        if not has_reason and now - event.stamp < self.fault_reason_wait_s:
            return None
        self._pending_fault = None
        reason = self.fault_reason if has_reason else ''
        return StateEvent(event.label, event.previous, event.stamp, event.step, reason, event.generation)

    # ---- outputs ------------------------------------------------------------------------
    def step_known(self) -> bool:
        """Diagnostics (1 Hz) arrived after the current step began, so 'step' is this step."""
        return self.diag_at >= self.step_started_at

    def step_name(self) -> str:
        step = self.diag.get('step', '') if self.step_known() else ''
        return self.step_names.get(step, STEP_KO.get(step, step))

    def planning_fresh(self, now: float) -> bool:
        return now - self.label_at <= self.stale_s

    def template(self, event: StateEvent) -> str:
        """Spoken sentence for a change (no GPT): the action that starts or ends, or the situation."""
        step = event.step
        if event.label == 'Fault':
            return fault_situation(event.reason)
        return {
            'Idle': '시작할 준비를 하고 있어요.',
            'Ready': '준비됐어요. 출발 신호를 기다리고 있어요.',
            'Run.AwaitRequest': '다음 할 일을 기다리고 있어요.',
            'Run.PrepareTask': f'{step} 준비를 하고 있어요.' if step else '다음 작업을 준비하고 있어요.',
            'Run.Compute': '가는 길을 계산하고 있어요.',
            'Run.Execute': f'{step}, 시작할게요.' if step else '작업을 시작할게요.',
            'Run.Complete': f'{step}, 끝났어요.' if step else '작업 하나를 끝냈어요.',
            'Run.Stopping': '멈추는 중이에요.',
        }.get(event.label, '하는 일이 바뀌었어요.')

    def doing(self, now: float) -> str:
        """What the robot is doing right now, as a spoken sentence."""
        if self.label is None or not self.planning_fresh(now):
            return '아직 제 작업 상태를 받지 못했어요.'
        step = self.step_name()
        if self.label == 'Fault':
            return fault_situation(self.fault_reason)
        return {
            'Idle': '시작할 준비를 하고 있어요.',
            'Ready': '출발 신호를 기다리고 있어요.',
            'Run.AwaitRequest': '다음 할 일을 기다리고 있어요.',
            'Run.PrepareTask': f'{step} 준비를 하고 있어요.' if step else '다음 작업을 준비하고 있어요.',
            'Run.Compute': '가는 길을 계산하고 있어요.',
            'Run.Execute': f'{step} 중이에요.' if step else '작업하고 있어요.',
            'Run.Complete': f'{step}, 방금 끝났어요.' if step else '방금 작업 하나를 끝냈어요.',
            'Run.Stopping': '멈추는 중이에요.',
        }.get(self.label, '작업하고 있어요.')

    def current_template(self, now: float) -> str:
        """What I am doing, plus what my sensors see (no GPT)."""
        seen = self.seen_sentence(now)
        return self.doing(now) + (' ' + seen if seen else '')

    # ---- perception summaries -------------------------------------------------------------
    def _fresh(self, item, now: float):
        return item if item is not None and now - item[-1] <= self.perception_stale_s else None

    def moving_text(self, now: float) -> str:
        motion = self._fresh(self.motion, now)
        if motion is None:
            return ''
        v, w, _ = motion
        if abs(v) >= 0.02:
            return f'{"앞으로" if v > 0 else "뒤로"} 초속 {abs(v):.2f} m로 이동 중' + (', 방향 바꾸는 중' if abs(w) >= 0.15 else '')
        if abs(w) >= 0.15:
            return '제자리에서 방향을 바꾸는 중'
        return '멈춰 있음'

    def object_items(self, now: float) -> list[str]:
        items = []
        objects = self._fresh(self.objects, now)
        if objects is not None:
            items += [f'{OBJECT_KO.get(label, label)} {count}개' for label, count in sorted(objects[0].items())]
        pucks = self._fresh(self.pucks, now)
        if pucks is not None and pucks[0]:
            standing = sum(1 for st, _ in pucks[0] if st)
            lying = len(pucks[0]) - standing
            kinds = ', '.join(t for t in (f'선 것 {standing}' if standing else '', f'누운 것 {lying}' if lying else '')
                              if t)
            items.append(f'퍽 {len(pucks[0])}개({kinds}, 가장 가까운 것 약 {pucks[0][0][1]:.1f} m)')
        tags = self._fresh(self.tags, now)
        if tags is not None and tags[0]:
            items.append('AprilTag ' + ', '.join(str(i) for i in tags[0]) + '번')
        return items

    def seen_sentence(self, now: float) -> str:
        """Short spoken summary of what perception reports ('' if nothing fresh)."""
        items = []
        objects = self._fresh(self.objects, now)
        if objects is not None:
            items += [f'{OBJECT_KO.get(label, label)} {count}개' for label, count in sorted(objects[0].items())]
        pucks = self._fresh(self.pucks, now)
        if pucks is not None and pucks[0]:
            items.append(f'퍽 {len(pucks[0])}개')
        tags = self._fresh(self.tags, now)
        if tags is not None and tags[0]:
            items.append('태그 ' + ', '.join(str(i) for i in tags[0]) + '번')
        if not items:
            return ''
        joined = ', '.join(items)
        return f'지금 보이는 건 {joined}{copula(joined)}.'

    def perception_text(self, now: float) -> str:
        lines = []
        moving = self.moving_text(now)
        if moving:
            lines.append(f'- 움직임(odom): {moving}')
        objects = self._fresh(self.objects, now)
        items = self.object_items(now)
        if items:
            lines.append('- 인식된 것: ' + ', '.join(items))
        elif objects is not None:
            lines.append('- 인식된 것: 없음 (카메라 인식이 아무것도 찾지 못함)')
        return '\n'.join(lines)

    def context_text(self, now: float) -> str:
        """Facts block for the GPT prompt. Only what was received, with ages."""
        lines = [f'- 지금 하는 일: {self.doing(now)}']
        if self.label is None or not self.planning_fresh(now):
            lines.append('- 상태머신(/planning/state): 데이터 없음 (planning_master가 안 떠 있을 수 있음)')
        else:
            lines.append(f'- 상태머신: {self.label} ({STATE_KO.get(self.label, "알 수 없는 상태")}), '
                         f'{max(0.0, now - self.label_since):.0f}초째')
            if self.label == 'Fault' and self.fault_reason:
                lines.append(f'- 고장 원인: {self.fault_reason} ({reason_ko(self.fault_reason)})')
        if now - self.diag_at <= self.stale_s:
            step = self.diag.get('step', '') if self.step_known() else ''
            if step:
                lines.append(f'- 현재 단계: {step} ({self.step_name()}), task {self.diag.get("task_id", "?")}, '
                             f'남은 단계 {self.diag.get("queued", "?")}개')
            waiting = self.diag.get('waiting_for', '')
            if waiting and self.label in ('Idle', 'Run.PrepareTask'):
                lines.append(f'- 기다리는 입력: {waiting}')
        if len(self.history) > 1:
            lines.append('- 최근 상태 변화: ' + ' -> '.join(label for _, label in self.history))
        if now - self.control.at <= self.stale_s:
            c = self.control
            pairs = (('ready', c.ready), ('fault', c.fault), ('held', c.held), ('stopped', c.stopped))
            flags = ', '.join(name for name, on in pairs if on) or '없음'
            lines.append(f'- 제어(control) 상태 플래그: {flags}' + (f', 이유: {c.reason}' if c.reason else ''))
        for topic, (value, at) in sorted(self.context.items()):
            if now - at <= max(self.stale_s, 10.0) and value:
                lines.append(f'- {topic}: {value}')
        return '\n'.join(lines)
