"""Weather questions are answered from the forecast for one place, with the day taken from the question."""

import datetime as dt

from craft_voice.menu import Menu
from craft_voice.weather import Weather, chain, is_weather_question
from test_menu import FILE
from test_narrator import Backend, make

MONDAY = dt.datetime(2026, 11, 16, 7, 45)
FORECAST = {
    'current': {'temperature_2m': 5.7, 'apparent_temperature': 3.5, 'weather_code': 2},
    'daily': {'time': ['2026-11-16', '2026-11-17', '2026-11-18'], 'weather_code': [3, 61, 73],
              'temperature_2m_max': [16.1, 13.6, 0.4], 'temperature_2m_min': [5.5, 9.0, -3.2],
              'precipitation_probability_max': [2, 85, 70]}}


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def weather(fetch=None, style='cute', clock=None, **kw) -> Weather:
    calls = []

    def fake(lat, lon, timeout):
        calls.append((lat, lon))
        if fetch is not None:
            return fetch()
        return FORECAST
    w = Weather('구로구', 37.4954, 126.8874, style=style, now=lambda: MONDAY, fetch=fake,
                monotonic=clock or Clock(), **kw)
    w.calls = calls
    return w


def test_weather_questions():
    for text in ('오늘 날씨 뭐야?', '그래프트 오늘 날씨 뭐야?', '내일 비 와?', '우산 챙겨야 돼?', '밖에 추워?', '내일 몇 도야?'):
        assert is_weather_question(text), text
    for text in ('왜 멈췄어', '몇 도 돌았어?', '오늘 점심 메뉴 뭐야', '눈 앞에 뭐 있어?', '내일은?'):
        assert not is_weather_question(text), text
    assert is_weather_question('내일은?', follow=True) and is_weather_question('모레는요?', follow=True)
    assert not is_weather_question('내일 뭐 해?', follow=True)


def test_today_now_and_the_day():
    assert weather().answer('오늘 날씨 뭐야') == \
        '지금 구로구는 구름 조금, 6도야. 오늘 최고 16도, 최저 6도, 비 올 확률 2퍼센트야.'
    assert weather(style='polite').answer('날씨 어때요?') == \
        '지금 구로구는 구름 조금, 6도예요. 오늘 최고 16도, 최저 6도, 비 올 확률 2퍼센트예요.'


def test_feels_like_said_when_far_from_the_air():
    data = {**FORECAST, 'current': {'temperature_2m': 1.0, 'apparent_temperature': -3.4, 'weather_code': 0}}
    assert weather(fetch=lambda: data).answer('날씨').startswith('지금 구로구는 맑음, 1도, 체감 영하 3도야.')


def test_other_days_rain_snow_and_umbrella():
    w = weather()
    assert w.answer('내일 비 와?') == '내일 구로구는 비, 최고 14도, 최저 9도, 비 올 확률 85퍼센트야. 우산 챙겨!'
    assert w.answer('모레는?', follow=True) == \
        '모레 구로구는 눈, 최고 0도, 최저 영하 3도, 눈 올 확률 70퍼센트야. 우산 챙겨!'
    assert w.answer('금요일 날씨') == '금요일 날씨는 아직 예보가 없어.'
    assert w.answer('어제 날씨') == '지난 날씨는 몰라.'


def test_kept_for_a_while_and_said_so_when_it_cannot_be_fetched():
    clock = Clock()
    w = weather(clock=clock)
    w.answer('날씨')
    w.answer('내일 날씨')
    assert len(w.calls) == 1                         # kept, not fetched for every question
    clock.t = 700.0
    broken = weather(fetch=lambda: (_ for _ in ()).throw(OSError('no network')), clock=clock)
    assert broken.answer('오늘 날씨') == '지금은 날씨 정보를 못 가져왔어.'


def test_the_place_takes_the_right_particle():
    w = Weather('구로동', 0, 0, style='cute', now=lambda: MONDAY, fetch=lambda *a: FORECAST)
    assert w.answer('날씨').startswith('지금 구로동은 ')


def test_menu_and_weather_together_and_follow_ups_go_to_the_last_one():
    menu = Menu(FILE, style='cute', now=lambda: dt.datetime(2026, 11, 12, 11, 30))
    answer = chain(menu.answer, weather().answer)
    assert answer('오늘 점심 메뉴 뭐야').startswith('오늘 점심 메뉴는')
    assert answer('저녁은?', True).startswith('오늘 저녁 메뉴는')
    assert answer('내일 날씨 어때?').startswith('내일 구로구는 비')
    assert answer('모레는?', True).startswith('모레 구로구는 눈')    # the weather answered last
    assert answer('왜 멈췄어') is None
    assert chain(None, None) is None


def test_the_robot_answers_the_weather_without_gpt():
    backend = Backend(reply='나는 밖 날씨를 볼 수 없어서 지금은 모르겠어.')
    narrator, texts = make(backend=backend)
    narrator.quick_answer = chain(weather().answer)
    narrator.planning_state('Ready')
    narrator._jobs.clear()
    narrator._utterances.clear()
    narrator.wake('오늘 날씨 뭐야?')
    assert not narrator._jobs                         # nothing for GPT to do
    assert narrator.speak_once() is True
    assert texts == [('ask', '지금 구로구는 구름 조금, 6도야. 오늘 최고 16도, 최저 6도, 비 올 확률 2퍼센트야.')]
