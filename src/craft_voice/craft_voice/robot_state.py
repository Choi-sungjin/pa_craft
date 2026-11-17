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

STEP_KO = {'base_goal': '목표 지점 이동', 'tcp_goal': '팔 목표 자세 이동',
           'box_approach': '상자 접근', 'box_near_entry': '상자 앞 정렬'}

# Substrings of planning fault reasons (state_machine.cpp, planning_master.cpp) ->
# (short Korean for the GPT facts, spoken situation: what happened and what the robot is doing now).
REASON_KO = (
    ('initialization timeout', '초기화 시간 초과', '준비가 끝나지 않아서 아직 출발하지 못하고 있어요.'),
    ('task preparation timeout', '작업 준비 시간 초과', '작업 준비가 늦어져서 멈춰 있어요.'),
    ('compute timeout', '계산 시간 초과', '다음 동작을 정하지 못해서 멈춰 있어요.'),
    ('calculation failed', '경로나 자세 계산 실패', '다음 동작을 정하지 못해서 멈춰 있어요.'),
    ('execution timeout', '실행 시간 초과', '목표까지 너무 오래 걸려서 멈춰 있어요.'),
    ('stop acknowledgement timeout', '정지 확인 시간 초과', '멈췄는지 확인이 늦어져서 그 자리에 서 있어요.'),
    ('localization lost', '위치 추정 실패', '제 위치를 놓쳐서 멈춰 있어요.'),
    ('control unavailable', '제어기 응답 없음', '바퀴와 팔 제어 연결이 끊겨서 멈춰 있어요.'),
    ('stale odom/lidar input', '오도메트리나 라이다 입력 끊김', '센서 정보가 끊겨서 멈춰 있어요.'),
    ('stale control task result', '제어 결과 응답 끊김', '제어 쪽 응답이 끊겨서 멈춰 있어요.'),
    ('no progress', '진행 없음', '앞으로 나아가지 못해서 멈춰 있어요.'),
    ('module/feedback fault', '모듈 피드백 이상', '장치 응답이 이상해서 멈춰 있어요.'),
    # box approach (planning_master.cpp box stages): specific needles before the general ones
    ('path blocked', '상자 앞 경로 막힘', '상자 앞 길이 막혀서 멈춰 있어요.'),
    ('stays blocked', '상자 앞 경로가 계속 막힘', '상자 앞 길이 계속 막혀서 멈춰 있어요.'),
    ('replan', '경로 다시 찾기 실패', '다른 길을 찾지 못해서 멈춰 있어요.'),
    ('invalid lidar odom', '라이다·IMU 값 이상', '센서 값이 이상해서 멈춰 있어요.'),
    ('box observation', '상자 관측 실패', '상자를 제대로 보지 못해서 멈춰 있어요.'),
    ('box corners outside', '상자가 라이다 시야 밖', '상자가 시야를 벗어나서 멈춰 있어요.'),
    ('stage task failed', '이동 단계 실패', '가던 길에서 문제가 생겨서 멈춰 있어요.'),
    ('camera', '카메라 접근 실패', '카메라로 간격을 맞추지 못해서 멈춰 있어요.'),
    ('lidar', '라이다 접근 실패', '상자 앞에 정확히 서지 못해서 멈춰 있어요.'),
    ('box', '상자 접근 실패', '상자 앞에서 문제가 생겨서 멈춰 있어요.'),
)

# The same situations in the short, friendly persona (style 'cute'), by REASON_KO needle.
FAULT_CUTE = {
    'initialization timeout': '준비가 안 끝나서 아직 못 출발했어.',
    'task preparation timeout': '작업 준비가 늦어져서 멈췄어.',
    'compute timeout': '다음 동작을 못 정해서 멈췄어.',
    'calculation failed': '다음 동작을 못 정해서 멈췄어.',
    'execution timeout': '목표까지 너무 오래 걸려서 멈췄어.',
    'stop acknowledgement timeout': '멈췄는지 확인이 늦어서 서 있어.',
    'localization lost': '내 위치를 놓쳐서 멈췄어.',
    'control unavailable': '바퀴랑 팔 연결이 끊겨서 멈췄어.',
    'stale odom/lidar input': '센서 정보가 끊겨서 멈췄어.',
    'stale control task result': '제어 쪽 응답이 끊겨서 멈췄어.',
    'no progress': '앞으로 못 나가서 멈췄어.',
    'module/feedback fault': '장치 응답이 이상해서 멈췄어.',
    'path blocked': '상자 앞 길이 막혀서 멈췄어.',
    'stays blocked': '상자 앞 길이 계속 막혀서 멈췄어.',
    'replan': '다른 길을 못 찾아서 멈췄어.',
    'invalid lidar odom': '센서 값이 이상해서 멈췄어.',
    'box observation': '상자를 제대로 못 봐서 멈췄어.',
    'box corners outside': '상자가 시야에서 벗어나서 멈췄어.',
    'stage task failed': '가던 길에서 문제가 생겨서 멈췄어.',
    'camera': '카메라로 간격을 못 맞춰서 멈췄어.',
    'lidar': '상자 앞에 딱 못 서서 멈췄어.',
    'box': '상자 앞에서 문제가 생겨서 멈췄어.',
}

# Substrings of the diagnostics 'waiting_for' value (planning_master.cpp readiness_reason) ->
# (situation key, short Korean for the GPT facts). The first match wins.
WAITING_KO = (
    ('path still blocked; replanning', 'replan', '길이 계속 막혀서 경로를 다시 계산하는 중'),
    ('path blocked; waiting before replanning', 'blocked', '길이 막혀서 잠깐 기다리는 중(안 비키면 다시 계산)'),
    ('route unavailable', 'no_route', '갈 수 있는 길을 찾지 못함'),
    ('box: perceiving', 'box_perceive', '상자 위치를 센서로 확인하는 중'),
    ('box: settling', 'box_settle', '상자 앞에서 멈춰 흔들림이 가라앉기를 기다리는 중'),
    ('waiting for target observation', 'target', '목표물이 보이기를 기다리는 중'),
    ('no fresh matching target observation', 'target', '목표물이 최근에 관측되지 않음'),
    ('waiting for control release', 'control', '제어기 응답을 기다리는 중'),
    ('waiting for fresh odom', 'sensors', '센서 값(위치·라이다·관절)을 기다리는 중'),
    ('waiting for previous calculation', 'compute', '이전 계산이 끝나기를 기다리는 중'),
    ('local path:', 'local_path', '가까운 경로 계산 문제'),
)

# Spoken sentences. 'polite' is the original wording; 'cute' is a short, friendly 반말 persona.
# A phrases file (JSON {key: sentence}) replaces single sentences; '{step}' is the spoken step name.
PHRASES = {
    'polite': {
        'change.Idle': '시작할 준비를 하고 있어요.',
        'change.Ready': '준비됐어요. 출발 신호를 기다리고 있어요.',
        'change.Run.AwaitRequest': '다음 할 일을 기다리고 있어요.',
        'change.Run.PrepareTask': '다음 작업을 준비하고 있어요.',
        'change.Run.PrepareTask.step': '{step} 준비를 하고 있어요.',
        'change.Run.Compute': '다음 동작을 계산하고 있어요.',
        'change.Run.Execute': '작업을 시작할게요.',
        'change.Run.Execute.step': '{step}, 시작할게요.',
        'change.Run.Complete': '작업 하나를 끝냈어요.',
        'change.Run.Complete.step': '{step}, 끝났어요.',
        'change.Run.Stopping': '멈추는 중이에요.',
        'change.other': '하는 일이 바뀌었어요.',
        'doing.none': '아직 제 작업 상태를 받지 못했어요. 주행 프로그램이 꺼져 있는 것 같아요.',
        'doing.lost': '지금은 작업 상태 신호가 들어오지 않아요. 주행 프로그램이 꺼진 것 같아요.',
        'doing.Idle': '시작할 준비를 하고 있어요.',
        'doing.Ready': '출발 신호를 기다리고 있어요.',
        'doing.Run.AwaitRequest': '다음 할 일을 기다리고 있어요.',
        'doing.Run.PrepareTask': '다음 작업을 준비하고 있어요.',
        'doing.Run.PrepareTask.step': '{step} 준비를 하고 있어요.',
        'doing.Run.Compute': '다음 동작을 계산하고 있어요.',
        'doing.Run.Execute': '작업하고 있어요.',
        'doing.Run.Execute.step': '{step} 중이에요.',
        'doing.Run.Complete': '방금 작업 하나를 끝냈어요.',
        'doing.Run.Complete.step': '{step}, 방금 끝났어요.',
        'doing.Run.Stopping': '멈추는 중이에요.',
        'doing.other': '작업하고 있어요.',
        'fault.unknown': '지금은 멈춰 있어요. 다시 움직이려면 확인이 필요해요.',
        'fault.none': '지금은 멈춰 있어요.',
        'situation.replan': '길이 계속 막혀서 다른 길을 찾고 있어요.',
        'situation.blocked': '길이 막혀서 잠깐 기다리고 있어요.',
        'situation.no_route': '갈 수 있는 길을 찾지 못하고 있어요.',
        'situation.box_perceive': '상자 위치를 확인하고 있어요.',
        'situation.box_settle': '상자 앞에서 잠깐 멈춰 있어요.',
        'situation.target': '목표물이 보이기를 기다리고 있어요.',
        'situation.clear': '길이 다시 열려서 계속 갈게요.',
        'seen': '지금 보이는 건 {items}{copula}.',
        'cannot_answer': '지금은 그 질문에 답하기 어려워요.',
        'wake': '네!',
        'ack': '잠깐만요.',
        'unheard': '잘 못 들었어요.',
        'notice.long_compute': '계산이 평소보다 오래 걸리고 있어요.',
        'notice.long_blocked': '길이 계속 막혀 있어요. 비켜 주시면 지나갈게요.',
        'notice.long_execute': '이번 작업이 꽤 오래 걸리고 있어요.',
        'notice.many_replans': '경로를 여러 번 다시 찾고 있어요.',
        'notice.state_lost': '작업 상태 신호가 끊겼어요.',
        'notice.state_back': '작업 상태가 다시 들어오고 있어요.',
        'notice.idle': '한동안 할 일이 없었어요.',
    },
    'cute': {
        'change.Idle': '지금은 대기 중이야!',
        'change.Ready': '준비 끝! 출발 신호 기다리는 중이야.',
        'change.Run.AwaitRequest': '다음 작업 기다리는 중!',
        'change.Run.PrepareTask': '다음 작업 준비 중이야!',
        'change.Run.PrepareTask.step': '{step} 준비 중이야!',
        'change.Run.Compute': '계획을 계산하는 중이야!',
        'change.Run.Execute': '다음 경로 찾아갈게!',
        'change.Run.Execute.step': '{step} 시작할게!',
        'change.Run.Complete': '이번 단계 끝났어!',
        'change.Run.Complete.step': '{step} 끝났어!',
        'change.Run.Stopping': '멈추는 중이야.',
        'change.other': '하는 일이 바뀌었어!',
        'doing.none': '아직 상태 신호를 못 받았어. 주행 프로그램이 꺼져 있는 것 같아.',
        'doing.lost': '지금은 상태 신호가 안 들어와. 주행 프로그램이 꺼진 것 같아.',
        'doing.Idle': '지금은 대기 중이야!',
        'doing.Ready': '준비 끝! 출발 신호 기다리는 중이야.',
        'doing.Run.AwaitRequest': '다음 작업 기다리는 중!',
        'doing.Run.PrepareTask': '다음 작업 준비 중이야!',
        'doing.Run.PrepareTask.step': '{step} 준비 중이야!',
        'doing.Run.Compute': '계획을 계산하는 중이야!',
        'doing.Run.Execute': '지금 작업하는 중이야!',
        'doing.Run.Execute.step': '지금 {step} 중이야!',
        'doing.Run.Complete': '방금 한 단계 끝냈어!',
        'doing.Run.Complete.step': '방금 {step} 끝냈어!',
        'doing.Run.Stopping': '멈추는 중이야.',
        'doing.other': '지금 작업하는 중이야!',
        'fault.unknown': '지금은 멈춰 있어. 확인이 필요해.',
        'fault.none': '지금은 멈춰 있어.',
        'situation.replan': '예? 새로운 경로를 찾아볼게!',
        'situation.blocked': '길이 막혀서 잠깐 기다리는 중이야.',
        'situation.no_route': '갈 수 있는 길을 못 찾고 있어.',
        'situation.box_perceive': '상자 위치를 확인하는 중이야!',
        'situation.box_settle': '상자 앞에서 잠깐 멈췄어.',
        'situation.target': '목표물이 보이길 기다리는 중이야.',
        'situation.clear': '길 열렸다! 계속 갈게!',
        'seen': '지금 {items} 보여!',
        'cannot_answer': '그건 지금 잘 모르겠어.',
        'wake': '짜스!',
        'ack': '어디 보자!',
        'unheard': '앗, 잘 안 들렸어!',
        'notice.long_compute': '계산이 좀 오래 걸리네, 조금만 기다려 줘!',
        'notice.long_blocked': '길이 계속 막혀 있어. 좀 비켜 줄래?',
        'notice.long_execute': '이번 작업은 꽤 오래 걸리네!',
        'notice.many_replans': '길을 자꾸 다시 찾고 있어. 주변이 복잡한가 봐.',
        'notice.state_lost': '어? 상태 신호가 끊겼어.',
        'notice.state_back': '상태 신호 다시 들어온다!',
        'notice.idle': '한참 동안 할 일이 없었어.',
    },
}

# Things that happen without a label change (a diagnostics 'waiting_for' value appears or clears).
ANNOUNCED_SITUATIONS = ('replan', 'blocked')

# craft_perception YOLO labels (perception.yaml class_names) -> Korean.
OBJECT_KO = {'knob': '손잡이', 'sign_red': '빨간 신호', 'sign_yellow': '노란 신호', 'sign_green': '초록 신호',
             'sign_off': '꺼진 신호', 'puck': '퍽', 'lever': '레버'}

DIAGNOSTIC_NAME = 'planning/state_machine'

# What the other planning / control topics mean, for the GPT facts block.
TOPIC_KO = {
    '/planning/box_phase': '박스 접근 단계',
    '/planning/box_status': '박스 접근 상황',
    '/perception/localization/state': '위치 추정 상태',
    '/planning/command': '마지막으로 받은 명령',
    '/planning/mission': '받은 임무',
    '/planning/approach_box': '접근하라고 받은 박스',
    '/planning/approach_box_near': '가까이 접근하라고 받은 박스',
    '/planning/hold': '플래닝이 로봇을 붙잡아 두는 중(hold)',
    '/control/gripper_grasped': '그리퍼가 물체를 잡고 있음',
}
# planning/state_machine diagnostics values worth telling GPT (key -> meaning); '' and 'false' are left out.
DIAG_KO = (
    ('path_revision', '경로를 다시 만든 횟수'),
    ('more_segments', '남은 경로 구간 있음'),
    ('puck_graspable', '퍽을 잡을 수 있는 위치'),
    ('puck_grasp_reason', '퍽 잡기 판단 이유'),
    ('puck_grasp_confirmed', '퍽 잡은 것 확인됨'),
)

_STATUS_ASK = ('뭐해', '뭐하', '뭘하', '무얼하', '무엇을하', '무슨상태', '어떤상태', '상태야', '상태니', '상태어때',
               '상태알려', '상태말해', '상태보고', '지금상태', '현재상태', '무슨일하')


def is_status_question(text: str) -> bool:
    """"뭐 하는 상태야?", "지금 뭐 해?": answered straight from the state machine, not by GPT.
    "왜 ..." is a reasoning question and goes to GPT."""
    squeezed = ''.join(ch for ch in text if ch.isalnum())
    return bool(squeezed) and '왜' not in squeezed and any(word in squeezed for word in _STATUS_ASK)
ACTIVE_PHASES = ('Run.PrepareTask', 'Run.Compute', 'Run.Execute')  # one step, start to finish


def parse_list(text: str) -> list[str]:
    return [item.strip() for item in text.split(',') if item.strip()]


def parse_mapping(text: str, separator: str = ':') -> dict[str, str]:
    """'approach:접근, grasp:잡기' -> {'approach': '접근', 'grasp': '잡기'} ('=' for values with ':')."""
    result = {}
    for item in parse_list(text):
        key, sep, value = item.partition(separator)
        if sep and key.strip() and value.strip():
            result[key.strip()] = value.strip()
    return result


def name_step(step: str, names: dict) -> str:
    """Spoken name of a step id: exact match, then patterns where '*' captures part of the id
    ('load_S*': '시작 박스 * 퍽 싣기' turns load_S3 into '시작 박스 3 퍽 싣기')."""
    if step in names:
        return names[step]
    for key, value in names.items():
        if '*' in key:
            head, _, tail = key.partition('*')
            if step.startswith(head) and step.endswith(tail) and len(step) > len(head) + len(tail):
                return value.replace('*', step[len(head):len(step) - len(tail)])
    return ''


def reason_ko(reason: str) -> str:
    lowered = reason.lower()
    for needle, short, _ in REASON_KO:
        if needle in lowered:
            return short
    return reason.strip()


def copula(word: str) -> str:
    """'이에요' after a final consonant (받침), '예요' otherwise: '3번이에요', '2개예요'."""
    last = word.rstrip()[-1:] or '가'
    if last.isdigit():
        return '이에요' if last in '013678' else '예요'  # 영 일 삼 육 칠 팔 end in a consonant; 이 사 오 구 do not
    has_batchim = '가' <= last <= '힣' and (ord(last) - 0xAC00) % 28 != 0
    return '이에요' if has_batchim else '예요'


def fault_situation(reason: str, style: str = 'polite', lines: dict | None = None) -> str:
    """Spoken sentence for a fault: the situation, never the raw error text."""
    lowered = reason.lower()
    lines = lines or {}
    for needle, _, situation in REASON_KO:
        if needle in lowered:
            if 'fault.' + needle in lines:
                return lines['fault.' + needle]
            return FAULT_CUTE.get(needle, situation) if style == 'cute' else situation
    key = 'fault.unknown' if reason else 'fault.none'
    return lines.get(key) or PHRASES.get(style, PHRASES['polite'])[key]


def waiting_situation(waiting_for: str) -> tuple[str, str]:
    """diagnostics 'waiting_for' -> (situation key, short Korean); ('', '') if it is none we know."""
    lowered = waiting_for.lower()
    for needle, key, short in WAITING_KO:
        if needle in lowered:
            return key, short
    return '', ''


def load_phrases(path: str) -> dict:
    """A phrases file: JSON object {key: sentence}. Unknown keys are kept (reaction lines etc.)."""
    import json
    from pathlib import Path
    data = json.loads(Path(path).expanduser().read_text(encoding='utf-8'))
    if not isinstance(data, dict) or any(not isinstance(k, str) or not isinstance(v, str) for k, v in data.items()):
        raise ValueError(f'{path}: expected a JSON object of key: sentence')
    return {key: value.strip() for key, value in data.items() if value.strip()}


def ago(seconds: float) -> str:
    """'방금', '40초 전', '3분 전', '2시간 전' for the memory lines."""
    seconds = max(0.0, seconds)
    if seconds < 5:
        return '방금'
    if seconds < 90:
        return f'{seconds:.0f}초 전'
    if seconds < 5400:
        return f'{seconds / 60:.0f}분 전'
    return f'{seconds / 3600:.0f}시간 전'


@dataclass(frozen=True)
class StateEvent:
    label: str
    previous: str | None
    stamp: float
    step: str
    reason: str
    generation: int
    situation: str = ''   # 'replan', 'blocked', ...: something happened without a label change

    @property
    def fault(self) -> bool:
        return self.label == 'Fault'


@dataclass(frozen=True)
class Notice:
    """Something the robot noticed by itself (no state change): a state that lasts unusually long, ..."""
    key: str      # 'long_compute', 'long_blocked', 'long_execute', 'many_replans', 'state_lost', 'state_back', 'idle'
    facts: str    # Korean, for the GPT prompt
    generation: int


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
    step_name_wait_s: float = 1.2    # diagnostics (step name) are 1 Hz: wait for them before "X, 시작할게요"
    stale_s: float = 3.0
    perception_stale_s: float = 2.0
    step_names: dict = field(default_factory=dict)
    style: str = 'polite'                       # 'polite' | 'cute' (PHRASES)
    lines: dict = field(default_factory=dict)   # phrases file: replaces single sentences
    announce_situations: tuple = ANNOUNCED_SITUATIONS
    situation_cooldown_s: float = 15.0          # the same situation is announced at most once per cooldown
    # Noticing (0 = never): a state that lasts this long is worth a remark of its own.
    notice_compute_s: float = 8.0
    notice_blocked_s: float = 20.0
    notice_execute_s: float = 180.0
    notice_replans: int = 3
    notice_idle_s: float = 1800.0
    notice_lost_s: float = 5.0
    event_topics: tuple = ()                    # context topics that are events (a command): kept event_keep_s
    event_keep_s: float = 1800.0

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
        self.situation = ''                 # waiting situation from diagnostics ('' = none): 'replan', 'blocked', ...
        self.situation_text = ''
        self.situation_since = -math.inf
        self.journal: deque = deque(maxlen=40)   # (time, what happened): the robot's short memory
        self.counts: Counter = Counter()         # done / replan / blocked / fault since the run began
        self.step_counts: Counter = Counter()    # the same, for the current step only
        self.last_done: dict | None = None       # the step that just completed: {'step', 'queued', 'at'}
        self._noticed: set = set()               # notices already given in this episode
        self._had_planning = False
        self._lost = False
        self._idle_since = -math.inf
        self._idle_noticed_at = -math.inf
        self._pending_step: StateEvent | None = None

    # ---- sentences ----------------------------------------------------------------------
    def phrase(self, key: str, **values) -> str:
        """Sentence for a key: phrases file, then the style's table, then the polite wording."""
        text = (self.lines.get(key) or PHRASES.get(self.style, PHRASES['polite']).get(key)
                or PHRASES['polite'].get(key, ''))
        if values and '{' in text:
            try:
                return text.format(**values)
            except (KeyError, IndexError, ValueError):
                return text
        return text

    def remember(self, now: float, text: str):
        if text:
            self.journal.append((now, text))

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
        self._noticed -= {'long_compute', 'long_execute'}  # a new stretch of this state may be noticed again
        if label == 'Run.Complete':
            self.counts['done'] += 1
            queued = self.diag.get('queued', '') if self.step_known() else ''
            self.last_done = {'step': self.diag.get('step', '') if self.step_known() else '',
                              'queued': int(queued) if queued.isdigit() else None, 'at': now}
        elif label == 'Fault':
            self.counts['fault'] += 1
        elif label == 'Ready' and previous in (None, 'Idle', 'Fault'):
            self.counts.clear()  # a fresh start: what happened before is history, not this run
        if label in ('Idle', 'Ready', 'Run.AwaitRequest'):
            if previous not in ('Idle', 'Ready', 'Run.AwaitRequest'):
                self._idle_since = self._idle_noticed_at = now
        else:
            self._idle_since = -math.inf
        step = self.step_name() if label in ('Run.Execute', 'Run.Complete') else ''
        self.remember(now, STATE_KO.get(label, label) + (f'({step})' if step else ''))
        if previous == 'Fault':
            self.fault_reason, self.fault_reason_at = '', -math.inf  # a later fault must not reuse this cause
        if previous is not None and (label == 'Run.PrepareTask' or
                                     (label in ACTIVE_PHASES and previous not in ACTIVE_PHASES)):
            # A new step begins: PrepareTask only ever starts one, and entering Compute/Execute from
            # outside covers a missed PrepareTask sample. The first label after startup is not a step
            # start: diagnostics already received describe that same step.
            self.step_started_at = now
            self.step_counts.clear()
            self._noticed -= {'many_replans', 'long_blocked'}
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
        if (label in ('Run.PrepareTask', 'Run.Execute') and not event.step and self.step_name_wait_s > 0 and
                self.step_started_at > -math.inf):
            # A new step began but its name is not in yet: say it once the name is known (or after the wait).
            self._pending_step = event
            return self.poll(now)
        return event

    def update_diagnostics(self, name: str, level: int, message: str, values: dict,
                           now: float) -> StateEvent | None:
        """Feed one diagnostics status. Returns an event when a situation worth announcing began
        (the path got blocked, a new path is being searched): things that happen without a label change."""
        if name != DIAGNOSTIC_NAME:
            return None
        self.diag = dict(values)
        self.diag_message = message
        self.diag_level = int(level)
        self.diag_at = now
        if message.startswith('Fault:'):
            reason = message[len('Fault:'):].strip()
            if reason:
                if reason != self.fault_reason:
                    self.remember(now, f'멈춘 이유: {reason_ko(reason)}')
                self.fault_reason, self.fault_reason_at = reason, now
        return self._update_situation(now)

    def _update_situation(self, now: float) -> StateEvent | None:
        key, text = waiting_situation(self.diag.get('waiting_for', '')) if self.step_known() else ('', '')
        if self.label == 'Fault':
            key, text = '', ''  # the fault sentence says it; a leftover waiting value is not a situation
        if key == self.situation:
            return None
        previous, self.situation, self.situation_text, self.situation_since = self.situation, key, text, now
        self._noticed.discard('long_blocked')
        if key:
            self.counts[key] += 1
            self.step_counts[key] += 1
            self.remember(now, text)
        elif previous in ('blocked', 'replan'):
            self.remember(now, '길이 다시 열림')
        announced = self.announce_situations
        if previous in announced or key in announced or (not key and 'clear' in announced):
            self.generation += 1  # what the robot is doing changed, although the label did not
        if key not in announced and not (not key and previous in ('blocked', 'replan') and 'clear' in announced):
            return None
        name = key or 'clear'
        if self.label is None or not self.label.startswith('Run.'):
            return None
        if now - self._announced_at.get('situation.' + name, -math.inf) < self.situation_cooldown_s:
            return None
        self._announced_at['situation.' + name] = now
        return StateEvent(self.label, self.label, now, self.step_name(), '', self.generation, situation=name)

    def current_situation(self, now: float) -> str:
        """The waiting situation, while diagnostics are fresh and describe the current step."""
        if not self.situation or now - self.diag_at > self.stale_s or not self.step_known():
            return ''
        return self.situation

    def notice(self, now: float) -> Notice | None:
        """Call periodically: something worth a remark of its own, at most once per episode."""
        fresh = self.label is not None and self.planning_fresh(now)
        if fresh:
            self._had_planning = True
            if self._lost:
                self._lost = False
                return Notice('state_back', '끊겼던 작업 상태 신호가 다시 들어옴', self.generation)
        elif self._had_planning and not self._lost and self.notice_lost_s > 0 and \
                now - self.label_at >= max(self.notice_lost_s, self.stale_s):
            self._lost = True
            self.remember(now, '작업 상태 신호가 끊김')
            return Notice('state_lost', f'작업 상태 신호가 {now - self.label_at:.0f}초째 안 들어옴 '
                          '(플래닝 프로그램이 꺼졌을 수 있음)', self.generation)
        if not fresh:
            return None
        held = now - self.label_since

        def once(key: str, limit: float, lasted: float, facts: str) -> Notice | None:
            if limit <= 0 or lasted < limit or key in self._noticed:
                return None
            self._noticed.add(key)
            self.remember(now, facts)
            return Notice(key, facts, self.generation)

        situation = self.current_situation(now)
        result = None
        if self.label == 'Run.Compute':
            result = once('long_compute', self.notice_compute_s, held, f'계산이 {held:.0f}초째 이어짐(평소보다 김)')
        if result is None and situation in ('blocked', 'replan', 'no_route'):
            lasted = now - self.situation_since
            result = once('long_blocked', self.notice_blocked_s, lasted, f'{self.situation_text}: {lasted:.0f}초째')
        if result is None and self.notice_replans > 0 and self.step_counts['replan'] >= self.notice_replans:
            result = once('many_replans', 1e-9, 1.0, f'이번 작업에서 경로를 {self.step_counts["replan"]}번 다시 계산함')
        if result is None and self.label == 'Run.Execute' and not situation:
            result = once('long_execute', self.notice_execute_s, held, f'같은 작업을 {held:.0f}초째 실행 중')
        if result is None and self.notice_idle_s > 0 and self._idle_since > -math.inf and \
                now - self._idle_noticed_at >= self.notice_idle_s:
            self._idle_noticed_at = now  # again every notice_idle_s while nothing happens
            return Notice('idle', f'{(now - self._idle_since) / 60:.0f}분째 할 일 없이 대기 중', self.generation)
        return result

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
        """pucks: [(standing, distance from the camera in m)]; None where perception has no depth/axis."""
        items = [(None if st is None else bool(st), None if d is None else float(d)) for st, d in pucks]
        self.pucks = (sorted(items, key=lambda p: math.inf if p[1] is None else p[1]), now)

    def update_tags(self, ids, now: float):
        self.tags = (sorted({int(i) for i in ids}), now)

    def poll(self, now: float) -> StateEvent | None:
        """A pending event that is ready: a Fault once its reason arrived, a step start once its name did
        (or when the wait ran out). Dropped if the state changed meanwhile."""
        fault = self._poll_fault(now)
        if fault is not None:
            return fault
        event = self._pending_step
        if event is None:
            return None
        if self.label != event.label or event.generation != self.generation:
            self._pending_step = None
            return None
        step = self.step_name()
        if not step and now - event.stamp < self.step_name_wait_s:
            return None
        self._pending_step = None
        return StateEvent(event.label, event.previous, event.stamp, step, '', event.generation)

    def _poll_fault(self, now: float) -> StateEvent | None:
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
        """Spoken name of the current step; '' for ids without a name (ids are never read out)."""
        step = self.diag.get('step', '') if self.step_known() else ''
        return name_step(step, self.step_names) or STEP_KO.get(step, '')

    def planning_fresh(self, now: float) -> bool:
        return now - self.label_at <= self.stale_s

    def template(self, event: StateEvent) -> str:
        """Spoken sentence for a change (no GPT): the action that starts or ends, or the situation."""
        if event.label == 'Fault':
            return fault_situation(event.reason, self.style, self.lines)
        if event.situation:
            return self.phrase('situation.' + event.situation) or self.phrase('change.other')
        return self._state_phrase('change', event.label, event.step)

    def _state_phrase(self, group: str, label: str, step: str) -> str:
        key = f'{group}.{label}'
        if step:
            text = self.phrase(key + '.step', step=step)
            if text:
                return text
        return self.phrase(key) or self.phrase(group + '.other')

    def doing(self, now: float) -> str:
        """What the robot is doing right now, as a spoken sentence."""
        if self.label is None:
            return self.phrase('doing.none')
        if not self.planning_fresh(now):
            return self.phrase('doing.lost') or self.phrase('doing.none')
        if self.label == 'Fault':
            return fault_situation(self.fault_reason, self.style, self.lines)
        situation = self.current_situation(now)
        if situation and self.label.startswith('Run.'):
            text = self.phrase('situation.' + situation)
            if text:
                return text
        return self._state_phrase('doing', self.label, self.step_name())

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
            standing = sum(1 for st, _ in pucks[0] if st is True)
            lying = sum(1 for st, _ in pucks[0] if st is False)
            unknown = len(pucks[0]) - standing - lying
            parts = [f'선 것 {standing}' if standing else '', f'누운 것 {lying}' if lying else '',
                     f'자세 모름 {unknown}' if unknown else '']
            nearest = pucks[0][0][1]
            if nearest is not None:
                parts.append(f'가장 가까운 것 약 {nearest:.1f} m')
            items.append(f'퍽 {len(pucks[0])}개(' + ', '.join(p for p in parts if p) + ')')
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
        return self.phrase('seen', items=joined, copula=copula(joined))

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
            lines.append('- 상태머신(/planning/state): 지금 신호 없음 (주행 프로그램 planning_master가 꺼져 있거나 '
                         '연결이 끊김). 지금 무엇을 하는지는 알 수 없음')
            if self.label is not None:
                lines.append(f'- 마지막으로 받은 상태: {self.label} ({STATE_KO.get(self.label, "알 수 없는 상태")}), '
                             f'{ago(now - self.label_at)}')
        else:
            lines.append(f'- 상태머신: {self.label} ({STATE_KO.get(self.label, "알 수 없는 상태")}), '
                         f'{max(0.0, now - self.label_since):.0f}초째')
            if self.label == 'Fault' and self.fault_reason:
                lines.append(f'- 고장 원인: {self.fault_reason} ({reason_ko(self.fault_reason)})')
        if now - self.diag_at <= self.stale_s:
            step = self.diag.get('step', '') if self.step_known() else ''
            if step:
                lines.append(f'- 현재 단계: {step} ({self.step_name() or "이름 없음"}), '
                             f'task {self.diag.get("task_id", "?")}, '
                             f'남은 단계 {self.diag.get("queued", "?")}개')
            waiting = self.diag.get('waiting_for', '')
            situation = self.current_situation(now)
            if situation:
                lines.append(f'- 지금 상황: {self.situation_text}, {max(0.0, now - self.situation_since):.0f}초째')
            elif waiting and self.label in ('Idle', 'Run.PrepareTask'):
                lines.append(f'- 기다리는 입력: {waiting}')
            detail = self.diag_message.partition(':')[2].strip()
            if detail and self.label != 'Fault':
                lines.append(f'- 상태머신이 밝힌 사유: {detail}')
            for key, meaning in DIAG_KO:
                value = str(self.diag.get(key, '')).strip()
                if value and value not in ('false', '0'):
                    lines.append(f'- {meaning}: {"예" if value == "true" else value}')
        if len(self.history) > 1:
            lines.append('- 최근 상태 변화: ' + ' -> '.join(label for _, label in self.history))
        if now - self.control.at <= self.stale_s:
            c = self.control
            pairs = (('ready', c.ready), ('fault', c.fault), ('held', c.held), ('stopped', c.stopped))
            flags = ', '.join(name for name, on in pairs if on) or '없음'
            lines.append(f'- 제어(control) 상태 플래그: {flags}' + (f', 이유: {c.reason}' if c.reason else ''))
        for topic, (value, at) in sorted(self.context.items()):
            event = topic in self.event_topics
            if not value or value == 'false' or now - at > (self.event_keep_s if event else max(self.stale_s, 10.0)):
                continue
            name = f'{TOPIC_KO[topic]} ({topic})' if topic in TOPIC_KO else topic
            shown = '예' if value == 'true' else value
            lines.append(f'- {name}: {shown}' + (f', {ago(now - at)}' if event else ''))
        return '\n'.join(lines)

    def memory_text(self, now: float, limit: int = 12) -> str:
        """What happened recently, oldest first, with how long ago: the facts the robot may reason from."""
        lines = [f'- {ago(now - at)}: {text}' for at, text in list(self.journal)[-limit:]]
        counts = [f'{name} {self.counts[key]}번' for key, name in
                  (('done', '끝낸 작업'), ('blocked', '길 막힘'), ('replan', '경로 다시 계산'), ('fault', '멈춤'))
                  if self.counts[key]]
        if counts:
            lines.append('- 이번 실행에서: ' + ', '.join(counts))
        return '\n'.join(lines)
