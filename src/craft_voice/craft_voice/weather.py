"""Weather: "크래프트, 오늘 날씨 뭐야?" is answered from the forecast for one place (Open-Meteo, no key), not by GPT.

    Weather('구로구', 37.4954, 126.8874).answer('오늘 날씨 뭐야')
        -> '지금 구로구는 구름 조금, 6도야. 오늘 최고 16도, 최저 6도, 비 올 확률 2퍼센트야.'
    Weather(...).answer('왜 멈췄어')  -> None  (not about the weather: someone else answers)

The day comes from the question (오늘, 내일, 모레, 금요일, 13일; today when it names none). "내일은?" right after a
weather answer asks about that day. The forecast is kept for a while; when it cannot be fetched the answer says so
instead of making something up.
"""

from __future__ import annotations

import datetime as dt
import json
import threading
import time
import urllib.parse
import urllib.request

from .menu import _DATE, _RELATIVE, WEEKDAYS, _final_consonant, _squeeze, which_day

URL = 'https://api.open-meteo.com/v1/forecast'
FORECAST_DAYS = 7

# WMO weather codes as said aloud
_SKY = {0: '맑음', 1: '대체로 맑음', 2: '구름 조금', 3: '흐림', 45: '안개', 48: '안개',
        51: '이슬비', 53: '이슬비', 55: '이슬비', 56: '어는 비', 57: '어는 비',
        61: '비', 63: '비', 65: '강한 비', 66: '어는 비', 67: '어는 비',
        71: '눈', 73: '눈', 75: '많은 눈', 77: '싸락눈', 80: '소나기', 81: '소나기', 82: '강한 소나기',
        85: '눈', 86: '많은 눈', 95: '천둥 번개', 96: '우박과 천둥 번개', 99: '우박과 천둥 번개'}
_SNOW = {71, 73, 75, 77, 85, 86}

_ALWAYS = ('날씨', '기온', '우산', '일기예보', '예보')
_RAIN = ('비와', '비오', '비올', '비가와', '비가오', '비가올', '비내리', '눈와', '눈오', '눈올', '눈이와', '눈이오', '눈이올',
         '눈내리')
_WARMTH = ('몇도', '추워', '추울', '추운', '더워', '더울', '더운', '쌀쌀')
_WHEN_WHERE = ('밖', '바깥', '오늘', '내일', '모레', '요일', '서울', '구로')


def is_weather_question(text: str, follow: bool = False) -> bool:
    """About the weather. follow: said right after a weather answer ("내일은?", "모레는?")."""
    squeezed = _squeeze(text)
    if not squeezed:
        return False
    if any(word in squeezed for word in _ALWAYS + _RAIN):
        return True
    if any(word in squeezed for word in _WARMTH) and any(word in squeezed for word in _WHEN_WHERE):
        return True   # "밖에 추워?", "내일 몇 도야?"; "몇 도 돌았어?" alone is not the weather
    if follow:        # a day and nothing else
        rest = _DATE.sub('', squeezed)
        for word in [w for w, _ in _RELATIVE] + [letter + '요일' for letter in WEEKDAYS] + \
                ['다음주', '그럼', '그러면']:
            rest = rest.replace(word, '')
        return rest != squeezed and rest in ('', '은', '는', '은요', '는요', '도', '도요')
    return False


def _degrees(value: float) -> str:
    whole = round(float(value))
    return f'영하 {-whole}도' if whole < 0 else f'{whole}도'


def fetch_forecast(latitude: float, longitude: float, timeout_s: float = 4.0) -> dict:
    query = urllib.parse.urlencode({
        'latitude': latitude, 'longitude': longitude, 'timezone': 'Asia/Seoul', 'forecast_days': FORECAST_DAYS,
        'current': 'temperature_2m,apparent_temperature,weather_code',
        'daily': 'weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max'})
    with urllib.request.urlopen(f'{URL}?{query}', timeout=timeout_s) as response:
        return json.loads(response.read().decode('utf-8'))


class Weather:
    def __init__(self, place: str, latitude: float, longitude: float, style: str = 'polite',
                 now=dt.datetime.now, fetch=fetch_forecast, keep_s: float = 600.0, stale_s: float = 10800.0,
                 timeout_s: float = 4.0, monotonic=time.monotonic, logger=None):
        self.place = place
        self.latitude, self.longitude = float(latitude), float(longitude)
        self.style = style
        self.now = now
        self.fetch = fetch
        self.keep_s = keep_s       # fetched again after this
        self.stale_s = stale_s     # an older forecast is still said when a new one cannot be fetched, up to this
        self.timeout_s = timeout_s
        self.monotonic = monotonic
        self.log = logger
        self._topic = place + ('은' if _final_consonant(place) else '는')   # '구로구는', '구로동은'
        self._lock = threading.Lock()
        self._data: dict | None = None
        self._fetched_at = -float('inf')
        self._last: tuple[dt.date, str] | None = None   # the day the last answer was about, for "내일은?"

    # ---- the forecast -------------------------------------------------------------------
    def forecast(self) -> dict | None:
        with self._lock:
            age = self.monotonic() - self._fetched_at
            if self._data is not None and age < self.keep_s:
                return self._data
            try:
                data = self.fetch(self.latitude, self.longitude, self.timeout_s)
                data['daily']['time']   # the shape the answer needs
                self._data, self._fetched_at = data, self.monotonic()
            except Exception as error:  # no network must not take the voice down
                if self.log:
                    self.log.warning(f'weather not fetched: {type(error).__name__}: {error}')
                if age >= self.stale_s:
                    self._data = None
            return self._data

    def warm(self):
        """Fetch once in the background so the first question is answered at once."""
        threading.Thread(target=self.forecast, name='weather', daemon=True).start()

    # ---- the answer ---------------------------------------------------------------------
    def _end(self, sentence: str) -> str:
        """'… 6도' -> '… 6도야.' (cute) or '… 6도예요.' (polite)."""
        if self.style == 'cute':
            return sentence + ('이야.' if _final_consonant(sentence) else '야.')
        return sentence + ('이에요.' if _final_consonant(sentence) else '예요.')

    def answer(self, text: str, follow: bool = False) -> str | None:
        """The spoken answer, or None when the question is not about the weather."""
        if not is_weather_question(text, follow and self._last is not None):
            return None
        today = self.now().date()
        named = which_day(text, today)
        if named is None and follow and self._last is not None:
            named = self._last
        date, name = named or (today, '오늘')
        self._last = (date, name)
        cute = self.style == 'cute'
        data = self.forecast()
        if data is None:
            return '지금은 날씨 정보를 못 가져왔어.' if cute else '지금은 날씨 정보를 가져오지 못했어요.'
        daily = data['daily']
        days = [dt.date.fromisoformat(day) for day in daily['time']]
        if date not in days:
            if date < today:
                return '지난 날씨는 몰라.' if cute else '지난 날씨는 몰라요.'
            return f'{name} 날씨는 아직 예보가 없어.' if cute else f'{name} 날씨는 아직 예보가 없어요.'
        i = days.index(date)
        code = int(daily['weather_code'][i])
        chance = daily['precipitation_probability_max'][i]
        wet = '눈' if code in _SNOW else '비'
        span = f"최고 {_degrees(daily['temperature_2m_max'][i])}, 최저 {_degrees(daily['temperature_2m_min'][i])}"
        if chance is not None:
            span += f', {wet} 올 확률 {round(chance)}퍼센트'
        parts = []
        current = data.get('current') or {}
        if date == today and current.get('temperature_2m') is not None:
            now_sky = _SKY.get(int(current.get('weather_code', -1)), '')
            said = f"지금 {self._topic} " + (f'{now_sky}, ' if now_sky else '') + _degrees(current['temperature_2m'])
            feels = current.get('apparent_temperature')
            if feels is not None and abs(round(feels) - round(current['temperature_2m'])) >= 3:
                said += f', 체감 {_degrees(feels)}'
            parts.append(self._end(said))
            parts.append(self._end(f'오늘 {span}'))
        else:
            sky = _SKY.get(code, '')
            parts.append(self._end(f'{name} {self._topic} ' + (f'{sky}, ' if sky else '') + span))
        if chance is not None and chance >= 60:
            parts.append('우산 챙겨!' if cute else '우산 챙기세요.')
        return ' '.join(parts)


def chain(*answerers):
    """Several quick answerers as one: the first that answers. A follow-up ("내일은?") goes to the one that
    answered last; if it does not take it, the others hear it as an ordinary question."""
    answerers = [answer for answer in answerers if answer]
    if not answerers:
        return None
    last = []

    def answer(question: str, follow: bool = False):
        tries = [(answerers[0] if not last else last[0], True)] if follow else []
        tries += [(a, False) for a in answerers]
        for answerer, as_follow in tries:
            text = answerer(question, as_follow)
            if text:
                last[:] = [answerer]
                return text
        return None
    return answer
