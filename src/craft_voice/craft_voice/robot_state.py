"""What the robot is doing, from the planning state machine and friends.

planning_master publishes its StateMachine label on /planning/state every loop
(20 Hz): Idle, Ready, Run.AwaitRequest, Run.PrepareTask, Run.Compute,
Run.Execute, Run.Complete (one loop only), Run.Stopping, Fault. The fault
reason and current step come about once a second on /planning/diagnostics
(status 'planning/state_machine', message 'Fault: <phase>: <reason>').

StateTracker turns that stream into a snapshot for the GPT prompt and into
announcement events on label changes. A Fault event waits up to
fault_reason_wait_s for the diagnostics reason so it can say why.
"""

from __future__ import annotations

import math
from collections import deque
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

# Substrings of planning fault reasons (state_machine.cpp, planning_master.cpp) -> spoken Korean.
REASON_KO = (
    ('initialization timeout', '초기화 시간 초과'),
    ('task preparation timeout', '작업 준비 시간 초과'),
    ('compute timeout', '계산 시간 초과'),
    ('calculation failed', '경로나 자세 계산 실패'),
    ('execution timeout', '실행 시간 초과'),
    ('stop acknowledgement timeout', '정지 확인 시간 초과'),
    ('localization lost', '위치 추정 실패'),
    ('control unavailable', '제어기 응답 없음'),
    ('stale odom/lidar input', '오도메트리나 라이다 입력 끊김'),
    ('stale control task result', '제어 결과 응답 끊김'),
    ('no progress', '진행 없음'),
    ('module/feedback fault', '모듈 피드백 이상'),
)

DIAGNOSTIC_NAME = 'planning/state_machine'


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
    for needle, spoken in REASON_KO:
        if needle in lowered:
            return spoken
    return reason.strip()


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
    step_names: dict = field(default_factory=dict)

    def __post_init__(self):
        self.label: str | None = None
        self.label_since = -math.inf
        self.label_at = -math.inf
        self.diag: dict[str, str] = {}
        self.diag_message = ''
        self.diag_level = 0
        self.diag_at = -math.inf
        self.step_started_at = -math.inf  # last Run.PrepareTask: diagnostics older than this name the previous step
        self.fault_reason = ''
        self.fault_reason_at = -math.inf
        self.control = ControlSnapshot()
        self.context: dict[str, tuple[str, float]] = {}
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
        if label == 'Run.PrepareTask':
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
        """Deterministic spoken sentence for an event (no GPT)."""
        step = f'{event.step} ' if event.step else ''
        if event.label == 'Fault':
            if event.reason:
                return f'문제가 생겨서 멈췄어요. 원인: {reason_ko(event.reason)}.'
            return '문제가 생겨서 멈췄어요.'
        return {
            'Idle': '시스템을 초기화하고 있어요.',
            'Ready': '준비 완료. 미션 시작을 기다리고 있어요.',
            'Run.AwaitRequest': '다음 작업을 기다리고 있어요.',
            'Run.PrepareTask': f'{step}작업을 준비하고 있어요.',
            'Run.Compute': '경로를 계산하고 있어요.',
            'Run.Execute': f'{step}작업을 시작할게요.',
            'Run.Complete': f'{step}작업을 마쳤어요.',
            'Run.Stopping': '정지하는 중이에요.',
        }.get(event.label, f'상태가 바뀌었어요: {event.label}.')

    def current_template(self, now: float) -> str:
        if self.label is None or not self.planning_fresh(now):
            return '상태머신 데이터가 들어오지 않고 있어요.'
        reason = self.fault_reason if self.label == 'Fault' else ''
        return self.template(StateEvent(self.label, None, self.label_since, self.step_name(), reason,
                                        self.generation))

    def context_text(self, now: float) -> str:
        """Facts block for the GPT prompt. Only what was received, with ages."""
        lines = []
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
