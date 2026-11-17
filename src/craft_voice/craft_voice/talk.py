"""What a person says to the robot that is not a question for GPT.

Two kinds, both answered at once in fixed words:

- Requests the voice must actually carry out, or honestly refuse. Answering them through GPT made it promise
  things nothing then did ("응, 10분 동안 조용히 있을게" while it kept talking), and "전원 꺼" waited 12 s for GPT.
      voice_command('10분 동안 조용히 해')  -> ('quiet', 600.0)
      voice_command('이제 말해도 돼')        -> ('speak', None)
      voice_command('그만')                 -> ('hush', None)      the sentence being said stops
      voice_command('전원 꺼')              -> ('control', 'power')  the voice cannot: says so, nothing moves
      voice_command('소리 키워')            -> ('volume', None)
- Facts the voice knows better than GPT: the time and the date (GPT does not know them), who it is.
      fact_answer('지금 몇 시야', now)      -> '지금 오전 8시 5분이야.'

Anything else returns None and goes on to the menu, the weather, the state machine and GPT.
"""

from __future__ import annotations

import datetime as dt
import re

from .menu import _final_consonant

WEEKDAYS = '월화수목금토일'
_COUNT = {'한': 1, '두': 2, '세': 3, '네': 4, '다섯': 5, '여섯': 6, '일곱': 7, '여덟': 8, '아홉': 9, '열': 10,
          '십': 10, '이십': 20, '삼십': 30, '사십': 40, '오십': 50, '육십': 60, '일': 1, '이': 2, '삼': 3, '오': 5}
_DURATION = re.compile(r'(\d+|이십|삼십|사십|오십|육십|다섯|여섯|일곱|여덟|아홉|한|두|세|네|열|십|일|이|삼|오)(분|시간)(반)?')
MAX_QUIET_S = 7200.0

_QUIET = ('조용히', '말하지마', '말하지말', '말을하지마', '말을하지말', '말하지않', '말을하지않', '말좀그만', '그만말해', '그만떠들', '떠들지마', '입다물', '닥쳐', '쉿', '음소거')
_SPEAK = ('말해도돼', '말해도되', '말해도된다', '다시말해', '이제말해', '조용히안해도', '조용히하지않아도', '음소거해제',
          '음소거풀어', '다시떠들어', '말해도좋아')
_HUSH = re.compile(r'(?:그만|그만해|그만해요|됐어|됐어요|됐다|스톱말|알았어그만)')
_POWER = re.compile(r'(?:전원|파워|컴퓨터|젯슨|로봇)?(?:좀)?(?:꺼|꺼줘|꺼요|꺼라|끄세요|끄자|꺼버려|종료해|종료해줘|셧다운)'
                    r'|(?:전원|파워)(?:좀)?(?:꺼|꺼줘|꺼요|꺼라|끄세요|끄자|꺼버려)')
_STOP = re.compile(r'(?:지금|당장|빨리|일단|얼른)?(?:멈춰|멈춰줘|멈춰요|멈춰라|멈추세요|멈춰봐|정지|정지해|정지해줘|스톱|서|서라|'
                   r'세워|세워줘|비상정지)(?:요)?')
_MOVE = re.compile(r'(?:지금|당장|빨리|일단|얼른|다시)?(?:움직여|움직여줘|출발|출발해|출발해줘|가|가자|가줘|앞으로가|뒤로가|'
                   r'후진해|전진해|돌아|돌아줘|이리와|일로와|따라와|팔올려|팔내려|팔움직여|집어|잡아|놔|놓아)(?:요|봐)?')
_VOLUME = ('소리', '음량', '볼륨')
_LOUDER_SOFTER = ('키워', '줄여', '크게', '작게', '높여', '낮춰', '맞춰', '올려', '내려', '조절')
# 발표 시연(user 11-16): "크래프트, 달릴 준비 됐어?" -> 상태와 상관없이 이 대답
_READY = ('달릴준비', '달릴쭌비', '다릴준비', '달릴수있', '달려갈준비', '출발할준비', '출발준비', '뛸준비')
READY_LINES = {'cute': '달릴 준비 완료! 언제든 맡겨 줘!', 'polite': '달릴 준비 완료예요! 언제든 맡겨 주세요!'}
_WEIGHT = ('몸무게', '무게', '몇킬로', '몇kg', '체중', '무거워', '무겁', '가벼워', '가볍')
_NOT_ME = ('퍽', '상자', '박스', '물건', '이거', '그거', '저거')   # the weight of something else
_TIME = ('몇시', '지금시간', '현재시간', '시간알려', '시간몇', '몇시몇분')
_DATE = ('며칠', '몇일', '날짜', '무슨요일', '몇요일', '몇월')
_WHO = re.compile(r'(?:너|넌|너는|당신은?)?(?:누구|누구야|누구니|누구세요|누구예요|정체가뭐야|이름이뭐야|이름뭐야|이름이뭐니|'
                  r'자기소개(?:해|해줘|해봐|좀)?)(?:야|니|예요|세요|요)?')
_CAN = ('뭐할수있', '뭘할수있', '무엇을할수있', '뭐할줄알', '뭘할줄알', '할수있는게뭐', '할수있는거뭐', '무슨기능', '뭘물어',
        '뭐물어', '뭐물어봐도', '뭘물어봐도')


def squeeze(text: str) -> str:
    return ''.join(ch for ch in text if ch.isalnum())


def quiet_seconds(text: str, default_s: float) -> float:
    """'10분 동안' -> 600, '한 시간 반' -> 5400; none named: default_s. At most MAX_QUIET_S."""
    match = _DURATION.search(squeeze(text))
    if not match:
        return default_s
    count = int(match.group(1)) if match.group(1).isdigit() else _COUNT[match.group(1)]
    seconds = count * (60.0 if match.group(2) == '분' else 3600.0)
    if match.group(3):
        seconds += 30.0 if match.group(2) == '분' else 1800.0
    return max(60.0, min(MAX_QUIET_S, seconds))


def voice_command(text: str, quiet_default_s: float = 600.0) -> tuple[str, object] | None:
    s = squeeze(text)
    if not s:
        return None
    if any(word in s for word in _SPEAK):
        return 'speak', None
    if any(word in s for word in _QUIET) and '조용히안' not in s:
        return 'quiet', quiet_seconds(text, quiet_default_s)
    if _HUSH.fullmatch(s):
        return 'hush', None
    if _POWER.fullmatch(s):
        return 'control', 'power'
    if _STOP.fullmatch(s):
        return 'control', 'stop'
    if _MOVE.fullmatch(s):
        return 'control', 'move'
    if any(word in s for word in _VOLUME) and any(word in s for word in _LOUDER_SOFTER):
        return 'volume', None
    return None


def say_duration(seconds: float) -> str:
    minutes = int(round(seconds / 60.0))
    hours, minutes = divmod(minutes, 60)
    if hours and minutes:
        return f'{hours}시간 {minutes}분'
    return f'{hours}시간' if hours else f'{minutes}분'


def command_reply(kind: str, arg, cute: bool) -> str:
    """The fixed words for a voice_command result."""
    if kind == 'quiet':
        span = say_duration(arg)
        return (f'알았어, {span} 동안 조용히 있을게. 고장 나면 그때는 말할게.' if cute else
                f'알겠어요, {span} 동안 조용히 있을게요. 고장이 나면 그때는 말씀드릴게요.')
    if kind == 'speak':
        return '응, 다시 말할게!' if cute else '네, 다시 말할게요.'
    if kind == 'control':
        if arg == 'power':
            return ('전원은 내가 못 꺼. 직접 꺼 줘.' if cute else '전원은 제가 끌 수 없어요. 직접 꺼 주세요.')
        if arg == 'stop':
            return ('나는 말만 해서 직접 못 멈춰. 급하면 비상정지 버튼 눌러 줘!' if cute else
                    '저는 말만 해서 직접 멈출 수 없어요. 급하면 비상정지 버튼을 눌러 주세요.')
        return ('나는 말만 해서 직접 못 움직여. 움직이는 건 미션 프로그램이 해.' if cute else
                '저는 말만 해서 직접 움직일 수 없어요. 움직이는 건 미션 프로그램이 해요.')
    if kind == 'volume':
        return ('소리 크기는 말로는 아직 못 바꿔. 스피커에서 직접 맞춰 줘.' if cute else
                '소리 크기는 말로는 아직 바꿀 수 없어요. 스피커에서 직접 맞춰 주세요.')
    return ''


def _clock(now: dt.datetime) -> str:
    half = '오전' if now.hour < 12 else '오후'
    hour = now.hour % 12 or 12
    return f'{half} {hour}시' + (f' {now.minute}분' if now.minute else '')


def fact_answer(text: str, now: dt.datetime, cute: bool, clock_ok: bool = True) -> str | None:
    """The time, the date, who I am, what to ask me. None: not that kind of question."""
    s = squeeze(text)
    if not s:
        return None
    if any(word in s for word in _READY):
        return READY_LINES['cute' if cute else 'polite']
    s = s.replace('몇시간', '')   # "몇 시간 걸려?" asks for a duration, not the time

    def end(said: str) -> str:
        if cute:
            return said + ('이야.' if _final_consonant(said) else '야.')
        return said + ('이에요.' if _final_consonant(said) else '예요.')
    asks_time = any(word in s for word in _TIME)
    asks_date = any(word in s for word in _DATE) and '날씨' not in s
    if asks_time or asks_date:
        if not clock_ok:
            return '시계가 아직 안 맞춰져서 몰라.' if cute else '시계가 아직 맞춰지지 않아서 모르겠어요.'
        day = f'{now.month}월 {now.day}일 {WEEKDAYS[now.weekday()]}요일'
        if asks_time and asks_date:
            return end(f'오늘은 {day}, 지금은 {_clock(now)}')
        if asks_time:
            return end(f'지금 {_clock(now)}')
        return end(f'오늘은 {day}')
    if any(word in s for word in _WEIGHT) and not any(word in s for word in _NOT_ME):
        from .chatter import weight_line
        return weight_line('cute' if cute else 'polite')
    if _WHO.fullmatch(s):
        return ('나는 크래프트야! 경기장을 다니면서 퍽을 집어 옮기는 로봇이야.' if cute else
                '저는 크래프트예요. 경기장을 다니면서 퍽을 집어 옮기는 로봇이에요.')
    if any(word in s for word in _CAN):
        return ('지금 뭐 하는지, 왜 멈췄는지, 날씨, 식단, 시간, 배터리를 물어봐. 조용히 해 달라고 해도 돼!' if cute else
                '지금 뭐 하는지, 왜 멈췄는지, 날씨, 식단, 시간, 배터리를 물어보세요. 조용히 해 달라고 하셔도 돼요.')
    return None


_BATTERY = ('배터리', '전압', '충전')
_BATTERY_ASK = ('얼마', '몇', '남았', '남아', '어때', '괜찮', '확인', '알려', '해야', '필요', '상태', '있어', '부족')


def is_battery_question(text: str) -> bool:
    s = squeeze(text)
    return any(word in s for word in _BATTERY) and any(word in s for word in _BATTERY_ASK)


def battery_answer(supplies: dict, now: float, cute: bool, low_v: float, critical_v: float,
                   fresh_s: float = 10.0) -> str:
    """supplies: {'바퀴 모터': (volts or None, at)}. Only what was received recently is said."""
    parts, worst = [], ''
    for name, (volts, at) in supplies.items():
        if volts is None or now - at > fresh_s:
            continue
        parts.append(f'{name} {volts:.1f}볼트')
        if critical_v > 0 and volts < critical_v:
            worst = 'critical'
        elif low_v > 0 and volts < low_v and worst != 'critical':
            worst = 'low'
    if not parts:
        return ('배터리 신호를 아직 못 받았어. 모터 전원이 꺼져 있는 것 같아.' if cute else
                '배터리 신호를 아직 받지 못했어요. 모터 전원이 꺼져 있는 것 같아요.')
    head = ', '.join(parts)
    if worst == 'critical':
        return f'{head}. 거의 바닥이야, 지금 충전해 줘!' if cute else f'{head}예요. 거의 바닥이라 지금 충전해 주세요.'
    if worst == 'low':
        return f'{head}. 조금 부족해, 곧 충전해 줘.' if cute else f'{head}예요. 조금 부족하니 곧 충전해 주세요.'
    return f'{head}. 아직 괜찮아!' if cute else f'{head}예요. 아직 괜찮아요.'


def fixed_lines(style: str) -> list[str]:
    """Answers here that never change, synthesized beforehand so they play at once."""
    return [READY_LINES.get(style, READY_LINES['polite'])]
