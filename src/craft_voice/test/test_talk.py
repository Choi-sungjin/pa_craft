"""Requests the voice carries out or refuses, and facts it answers itself (talk.py), without GPT."""

import datetime as dt

from craft_voice import talk
from craft_voice.talk import fact_answer, quiet_seconds, voice_command
from test_narrator import Backend, make

MORNING = dt.datetime(2026, 11, 16, 8, 5)


def test_requests_from_the_robot_log():
    assert voice_command('10분 동안 조용히 해.') == ('quiet', 600.0)
    assert voice_command('그냥 아예 말을 하지 말어.') == ('quiet', 600.0)
    assert voice_command('지금 상태 신호 들어오는 거 없으니까 조용히 있어.') == ('quiet', 600.0)
    assert voice_command('전원 꺼.') == ('control', 'power')
    assert voice_command('음량 맞춰.') == ('volume', None)
    assert voice_command('이제 말해도 돼') == ('speak', None)
    assert voice_command('그만') == ('hush', None)
    assert voice_command('멈춰!') == ('control', 'stop') and voice_command('출발해') == ('control', 'move')


def test_questions_are_not_requests():
    for text in ('왜 멈췄어?', '지금 뭐 해?', '사과 세 개에 두 개를 더하면 몇 개야?', '오늘 날씨 뭐야?', '어디 가?',
                 '소리가 왜 이렇게 커?', '조용히 안 해도 돼'):
        found = voice_command(text)
        assert found is None or found[0] == 'speak', (text, found)


def test_how_long_to_be_quiet():
    assert quiet_seconds('30분만 조용히', 600) == 1800
    assert quiet_seconds('한 시간 반 동안 조용히 해', 600) == 5400
    assert quiet_seconds('삼십분 조용히', 600) == 1800
    assert quiet_seconds('열 시간 조용히', 600) == talk.MAX_QUIET_S
    assert quiet_seconds('조용히 해', 600) == 600


def test_time_date_and_who_i_am():
    assert fact_answer('지금 몇 시야?', MORNING, cute=True) == '지금 오전 8시 5분이야.'
    assert fact_answer('오늘 며칠이야', MORNING, cute=True) == '오늘은 11월 16일 월요일이야.'
    assert fact_answer('오늘 무슨 요일이에요?', MORNING, cute=False) == '오늘은 11월 16일 월요일이에요.'
    assert fact_answer('몇 시야', MORNING.replace(hour=15, minute=0), cute=True) == '지금 오후 3시야.'
    assert fact_answer('몇 시야', MORNING, cute=True, clock_ok=False) == '시계가 아직 안 맞춰져서 몰라.'
    assert fact_answer('너 누구야?', MORNING, cute=True).startswith('나는 크래프트야!')
    assert '날씨' in fact_answer('뭐 물어봐도 돼?', MORNING, cute=True)
    from craft_voice.chatter import WEIGHT_LINES
    assert fact_answer('너 몸무게 몇이야?', MORNING, cute=True) in WEIGHT_LINES['cute']
    assert fact_answer('크래프트 무거워?', MORNING, cute=False) in WEIGHT_LINES['polite']
    assert fact_answer('퍽 무게는 얼마야?', MORNING, cute=True) is None
    for text in ('몇 시간 걸려?', '오늘 날씨 뭐야?', '왜 멈췄어', '이거 누구 거야?'):
        assert fact_answer(text, MORNING, cute=True) is None, text


def test_battery_says_only_what_was_received():
    now = 100.0
    ok = talk.battery_answer({'바퀴 모터': (12.1, 99.0), '팔 모터': (11.8, 99.5)}, now, True, 11.0, 10.7)
    assert ok == '바퀴 모터 12.1볼트, 팔 모터 11.8볼트. 아직 괜찮아!'
    low = talk.battery_answer({'팔 모터': (10.5, 99.0)}, now, True, 11.0, 10.7)
    assert low.endswith('거의 바닥이야, 지금 충전해 줘!')
    assert talk.battery_answer({'팔 모터': (12.0, 10.0)}, now, True, 11.0, 10.7).startswith('배터리 신호를 아직')
    assert talk.is_battery_question('배터리 얼마나 남았어?') and not talk.is_battery_question('배터리 교체했어')


def quiet_robot():
    backend = Backend(reply='응, 10분 동안 조용히 있을게.')
    narrator, texts = make(backend=backend)
    narrator.tracker.style = 'cute'
    narrator.planning_state('Ready')
    narrator._jobs.clear()
    narrator._utterances.clear()
    return narrator, texts


def test_quiet_is_kept_and_ends_by_itself_or_when_asked():
    narrator, texts = quiet_robot()
    narrator.wake('10분 동안 조용히 해')
    assert not narrator._jobs                                   # GPT is not asked
    assert narrator.speak_once() is True
    assert texts[-1] == ('ask', '알았어, 10분 동안 조용히 있을게. 고장 나면 그때는 말할게.')
    assert narrator.muted
    narrator.wake('지금 뭐 해?')                                 # quiet: nothing said, nothing asked
    narrator.wake('')
    assert not narrator._jobs and not narrator._utterances
    narrator.wake('이제 말해도 돼')
    assert not narrator.muted and narrator.speak_once() is True and texts[-1] == ('ask', '응, 다시 말할게!')

    narrator.wake('5분만 조용히 있어')
    narrator.speak_once()
    assert narrator.muted
    narrator.clock.now += 299.0
    narrator.tick()
    assert narrator.muted
    narrator.clock.now += 2.0
    narrator.tick()
    assert not narrator.muted and narrator.speak_once() is True and texts[-1] == ('ack', '응, 다시 말할게!')


def test_control_requests_are_refused_at_once():
    narrator, texts = quiet_robot()
    narrator.wake('전원 꺼')
    assert not narrator._jobs and narrator.speak_once() is True and texts[-1][1] == '전원은 내가 못 꺼. 직접 꺼 줘.'
    narrator.wake('왜 멈췄어?')                                  # a question still goes to GPT
    assert any(job.text == '왜 멈췄어?' for job in narrator._jobs)


def test_time_answer_and_the_prompt_knows_the_time():
    narrator, texts = quiet_robot()
    narrator.wall_clock = lambda: MORNING
    narrator.wake('지금 몇 시야?')
    assert not narrator._jobs and narrator.speak_once() is True and texts[-1] == ('ask', '지금 오전 8시 5분이야.')
    narrator.wake('왜 멈췄어?')
    job = narrator._jobs[-1]
    instructions, prompt = narrator._prompt(job, has_image=False)
    assert '- 지금 시각: 2026년 11월 16일 월요일 8시 5분 (서울)' in prompt
    assert '추측해서 말하지 말고 모른다고 해' in prompt and '약속하지 마' in prompt


def test_ready_to_run_for_the_demo():
    from craft_voice.prepare import fixed_sentences
    for text in ('달릴 준비 됐어?', '크래프트 달릴 준비 됐어', '달릴 준비 완료됐어?', '출발할 준비 됐니?'):
        assert fact_answer(text, MORNING, cute=True) == '달릴 준비 완료! 언제든 맡겨 줘!', text
    assert fact_answer('달릴 준비 됐어요?', MORNING, cute=False) == '달릴 준비 완료예요! 언제든 맡겨 주세요!'
    assert '달릴 준비 완료! 언제든 맡겨 줘!' in fixed_sentences('cute', {}, {})     # plays at once
    narrator, texts = quiet_robot()
    narrator.wake('달릴 준비 됐어?')
    assert not narrator._jobs and not any(u.kind == 'ack' for u in narrator._utterances)   # no "어디 보자!" first
    assert narrator.speak_once() is True and texts[-1] == ('ask', '달릴 준비 완료! 언제든 맡겨 줘!')
