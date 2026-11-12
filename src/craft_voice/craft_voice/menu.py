"""The cafeteria menu: "크래프트, 오늘 점심 메뉴 뭐야?" is answered from config/menu.yaml, not by GPT.

    Menu(path).answer('오늘 점심 메뉴 뭐야')      -> '오늘 점심 메뉴는 진국 차슈라멘, 언양식 파채불고기, 새우튀김, 김치비빔국수야!'
    Menu(path).answer('왜 멈췄어')                -> None  (not about the menu: someone else answers)

The day is taken from the question (오늘, 내일, 모레, 어제, 금요일, 다음 주 월요일, 13일, 11월 13일; today when it
names none) and the meal too (점심, 저녁; by the time of day when it names none). A day or a meal the file does
not have is said as not there: nothing is made up. The file is read again when it changes.
"""

from __future__ import annotations

import datetime as dt
import os
import re

WEEKDAYS = '월화수목금토일'
_RELATIVE = (('그저께', -2), ('그제', -2), ('어제', -1), ('내일모레', 2), ('모레', 2), ('내일', 1), ('오늘', 0))
_MEALS = (('lunch', ('점심', '중식', '런치')), ('dinner', ('저녁', '석식', '디너')), ('breakfast', ('아침', '조식')))
MEAL_KO = {'lunch': '점심', 'dinner': '저녁', 'breakfast': '아침'}
_ABOUT_FOOD = ('메뉴', '식단', '반찬', '급식', '밥뭐', '뭐먹', '머먹', '뭐나와', '뭐나오', '뭐야', '뭐지', '뭐예요', '뭐에요', '뭐였', '나왔',
               '알려', '말해', '어때', '있어', '나와', '나오')
_ALWAYS_FOOD = ('메뉴', '식단', '반찬', '급식', '밥뭐', '뭐먹', '머먹')
_ALL = ('전부', '전체', '모두', '다알려', '다말해', '자세히', '빠짐없이')
_DATE = re.compile(r'(?:(\d{1,2})월)?(\d{1,2})일')


def _squeeze(text: str) -> str:
    return ''.join(ch for ch in text if ch.isalnum())


def _final_consonant(word: str) -> bool:
    """The last syllable ends in a consonant (받침): '국수' False, '덮밥' True. Not Korean: False."""
    for ch in reversed(word):
        if '가' <= ch <= '힣':
            return (ord(ch) - ord('가')) % 28 != 0
        if ch.isalnum():
            return False
    return False


def which_meal(text: str) -> str:
    """'lunch' | 'dinner' | 'breakfast' | '' (the question names none)."""
    squeezed = _squeeze(text)
    for meal, words in _MEALS:
        if any(word in squeezed for word in words):
            return meal
    return ''


def which_day(text: str, today: dt.date) -> tuple[dt.date, str] | None:
    """(the day the question is about, how to call it) or None when it names no day."""
    squeezed = _squeeze(text)
    for word, offset in _RELATIVE:
        if word in squeezed:
            return today + dt.timedelta(days=offset), '모레' if offset == 2 else '그저께' if offset == -2 else word
    match = _DATE.search(squeezed)
    if match:
        month, day = int(match.group(1) or today.month), int(match.group(2))
        try:
            date = dt.date(today.year, month, day)
        except ValueError:
            return None
        return date, f'{day}일 {WEEKDAYS[date.weekday()]}요일'
    for index, letter in enumerate(WEEKDAYS):
        if letter + '요일' in squeezed:
            monday = today - dt.timedelta(days=today.weekday())
            week = ''
            if '다음주' in squeezed:
                monday, week = monday + dt.timedelta(days=7), '다음 주 '
            elif '지난주' in squeezed or '저번주' in squeezed:
                monday, week = monday - dt.timedelta(days=7), '지난주 '
            return monday + dt.timedelta(days=index), week + letter + '요일'
    return None


def is_menu_question(text: str, follow: bool = False) -> bool:
    """About the cafeteria menu. follow: said right after a menu answer ("내일은?", "저녁은?")."""
    squeezed = _squeeze(text)
    if not squeezed:
        return False
    if any(word in squeezed for word in _ALWAYS_FOOD):
        return True
    meal = bool(which_meal(text))
    if meal and any(word in squeezed for word in _ABOUT_FOOD):
        return True
    if follow:  # "내일은?", "금요일은?", "그럼 저녁은요?": a day or a meal and nothing else ("오늘 뭐 해?" is not)
        rest = _DATE.sub('', squeezed)
        for word in [w for _, words in _MEALS for w in words] + [w for w, _ in _RELATIVE] + \
                [letter + '요일' for letter in WEEKDAYS] + ['다음주', '지난주', '저번주', '그럼', '그러면']:
            rest = rest.replace(word, '')
        return rest != squeezed and rest in ('', '은', '는', '은요', '는요', '도', '도요', '엔', '에는')
    return False


class Menu:
    def __init__(self, path: str, style: str = 'polite', items: int = 4, now=dt.datetime.now, logger=None):
        self.path = os.path.expanduser(path)
        self.style = style
        self.items = max(1, int(items))
        self.now = now
        self.log = logger
        self._stamp = None
        self._days: dict[dt.date, dict] = {}
        self._last: tuple[dt.date, str, str] | None = None  # what the last answer was about, for "저녁은?"

    # ---- the file -----------------------------------------------------------------------
    def days(self) -> dict:
        """{date: {'lunch': [...], 'dinner': [...]}}; read again when the file changed, {} when it cannot be read."""
        try:
            stamp = os.path.getmtime(self.path)
        except OSError:
            self._stamp, self._days = None, {}
            return self._days
        if stamp != self._stamp:
            self._stamp = stamp
            try:
                import yaml
                with open(self.path, encoding='utf-8') as handle:
                    data = yaml.safe_load(handle) or {}
                self._days = {}
                for key, meals in (data.get('days') or {}).items():
                    date = key if isinstance(key, dt.date) else dt.date.fromisoformat(str(key))
                    self._days[date] = {meal: [str(dish).strip() for dish in dishes or [] if str(dish).strip()]
                                        for meal, dishes in (meals or {}).items()}
            except Exception as error:  # a broken file must not take the voice down
                self._days = {}
                if self.log:
                    self.log.error(f'menu file {self.path!r} not used: {type(error).__name__}: {error}')
        return self._days

    # ---- the answer ---------------------------------------------------------------------
    def answer(self, text: str, follow: bool = False) -> str | None:
        """The spoken answer, or None when the question is not about the menu."""
        if not is_menu_question(text, follow and self._last is not None):
            return None
        now = self.now()
        today = now.date()
        named = which_day(text, today)
        meal = which_meal(text)
        if follow and self._last is not None:  # "저녁은?" keeps the day, "내일은?" keeps the meal
            if named is None:
                named = (self._last[0], self._last[1])
            if not meal:
                meal = self._last[2]
        date, name = named or (today, '오늘')
        if not meal:
            meal = 'dinner' if date == today and now.hour >= 14 else 'lunch'
        self._last = (date, name, meal)
        cute = self.style == 'cute'
        days = self.days()
        if not days:
            return '식단표를 못 읽었어.' if cute else '식단표를 읽지 못했어요.'
        meal_ko = MEAL_KO[meal]
        if date not in days:
            return f'{name} 식단은 식단표에 없어.' if cute else f'{name} 식단은 식단표에 없어요.'
        dishes = days[date].get(meal) or []
        if not dishes:
            return f'{name} {meal_ko}은 식단표에 없어.' if cute else f'{name} {meal_ko}은 식단표에 없어요.'
        everything = any(word in _squeeze(text) for word in _ALL)
        said = dishes if everything else dishes[:self.items]
        listed = ', '.join(said)
        if cute:
            return f"{name} {meal_ko} 메뉴는 {listed}{'이야' if _final_consonant(said[-1]) else '야'}!"
        return f"{name} {meal_ko} 메뉴는 {listed}{'이에요' if _final_consonant(said[-1]) else '예요'}."
