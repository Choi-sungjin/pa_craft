"""혼잣말: 할 일이 없을 때 가끔 스스로 하는 한마디(게임 NPC처럼).

정해 둔 문장을 돌아가며 말한다(다 한 번씩 말하기 전에는 같은 문장을 다시 고르지 않는다). 시계가 맞을 때는
지금 시간대(아침·점심·오후·저녁·밤)의 문장도 섞는다. 간격은 period_s 를 중심으로 jitter 만큼 들쭉날쭉하다.

로봇 상태를 지어내지 않는다: 배터리, 주변 상황처럼 받은 신호가 있어야 알 수 있는 말은 넣지 않는다.
문장은 모두 고정 문장이라 미리 합성해 둔다(prepare.fixed_sentences 가 all_lines 를 쓴다).
"""
from __future__ import annotations

import random
import time

from .clock import clock_unset

LINES = {
    'cute': {
        'any': [
            '심심하다. 크래프트, 하고 불러 주면 바로 대답할게.',
            '대기 중, 대기 중! 출발 신호만 기다리고 있어.',
            '퍽 옮기는 거라면 자신 있어!',
            '미로 길, 머릿속으로 한 번 더 그려 보는 중이야.',
            '나 불렀어? 아니구나. 그냥 혼잣말이야.',
            '오늘 신호등은 몇 번씩 깜빡일까? 궁금하다.',
            '바코드야, 오름차순이 나올까 내림차순이 나올까?',
            '바퀴가 근질근질해. 얼른 달리고 싶다!',
            '나는 반듯한 직선 길이 제일 좋더라.',
            '궁금한 거 있으면 크래프트, 하고 물어봐!',
            '삐빅, 대기 모드 계속 유지 중!',
            '오늘도 실수 없이 잘해 보자!',
        ],
        'morning': ['좋은 아침! 오늘도 잘 부탁해.', '아침엔 역시 천천히 몸풀기부터!'],
        'lunch': ['슬슬 점심시간이다! 메뉴 궁금하면 크래프트, 점심 메뉴 뭐야? 하고 물어봐.',
                  '밥 맛있게 먹고 와! 나는 여기서 기다릴게.'],
        'afternoon': ['오후엔 괜히 나른하네. 로봇도 나른할 수 있다고!', '조금만 더 힘내자, 거의 다 왔어!'],
        'evening': ['오늘 하루도 고생 많았어!', '저녁이다. 마무리까지 잘 해 보자!'],
        'night': ['이 시간까지 다들 대단하다. 나도 같이 버틸게!'],
    },
    'polite': {
        'any': [
            '부르시면 바로 대답할게요. 크래프트라고 불러 주세요.',
            '출발 신호를 기다리고 있어요.',
            '퍽 옮기는 일이라면 자신 있어요.',
            '궁금한 게 있으면 언제든 물어봐 주세요.',
            '대기 모드를 유지하고 있어요.',
        ],
        'morning': ['좋은 아침이에요. 오늘도 잘 부탁드려요.'],
        'lunch': ['점심시간이네요. 메뉴가 궁금하면 물어봐 주세요.'],
        'afternoon': ['오후도 힘내세요.'],
        'evening': ['오늘 하루도 수고 많으셨어요.'],
        'night': ['늦은 시간까지 수고 많으세요.'],
    },
}


def time_slot(hour: float) -> str:
    """하루 중 시간대(시, 소수 포함): morning 5~11:30, lunch 11:30~13:30, afternoon ~18, evening ~24, night 0~5."""
    if 5 <= hour < 11.5:
        return 'morning'
    if 11.5 <= hour < 13.5:
        return 'lunch'
    if 13.5 <= hour < 18:
        return 'afternoon'
    if hour >= 18:
        return 'evening'
    return 'night'


def all_lines(style: str) -> list[str]:
    table = LINES.get(style, LINES['polite'])
    return sorted({line for lines in table.values() for line in lines})


def local_hour() -> float | None:
    if clock_unset():
        return None
    t = time.localtime()
    return t.tm_hour + t.tm_min / 60.0


class Chatter:
    def __init__(self, period_s: float, style: str = 'cute', jitter: float = 0.4, seed: int | None = None,
                 hour=local_hour):
        self.period_s = float(period_s)
        self.jitter = min(max(float(jitter), 0.0), 0.9)
        self.style = style
        self.hour = hour
        self._random = random.Random(seed)
        self._said: set[str] = set()
        self._next_at: float | None = None

    @property
    def enabled(self) -> bool:
        return self.period_s > 0

    def _gap(self) -> float:
        return self.period_s * (1.0 + self._random.uniform(-self.jitter, self.jitter))

    def postpone(self, now: float):
        """Something was said or happened: the next remark comes a whole gap later."""
        if self.enabled:
            self._next_at = now + self._gap()

    def due(self, now: float) -> bool:
        if not self.enabled:
            return False
        if self._next_at is None:
            self._next_at = now + self._gap()
        return now >= self._next_at

    def next_line(self, now: float) -> str:
        """The next sentence (and the time of the one after it)."""
        table = LINES.get(self.style, LINES['polite'])
        hour = self.hour()
        pool = list(table['any'])
        if hour is not None:
            slot = table.get(time_slot(hour), [])
            pool += slot * 2  # the time of day comes up a little more often
        fresh = [line for line in pool if line not in self._said]
        if not fresh:
            self._said.clear()
            fresh = pool
        line = self._random.choice(fresh)
        self._said.add(line)
        self._next_at = now + self._gap()
        return line
