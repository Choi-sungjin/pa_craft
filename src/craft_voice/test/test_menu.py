"""Menu questions are answered from the menu file, with the day and the meal taken from the question."""

import datetime as dt
from pathlib import Path

from craft_voice.menu import Menu, is_menu_question, which_day, which_meal
from test_narrator import Backend, make

FILE = str(Path(__file__).resolve().parents[1] / 'config' / 'menu.yaml')
THURSDAY = dt.datetime(2026, 11, 12, 11, 30)


def menu(now=THURSDAY, **kw) -> Menu:
    return Menu(FILE, style='cute', now=lambda: now, **kw)


def test_today_lunch_from_the_file():
    assert menu().answer('오늘 점심 메뉴 뭐야') == '오늘 점심 메뉴는 진국 차슈라멘, 언양식 파채불고기, 새우튀김, 김치비빔국수야!'
    assert menu(items=2).answer('오늘 점심 메뉴 뭐야?') == '오늘 점심 메뉴는 진국 차슈라멘, 언양식 파채불고기야!'
    assert menu().answer('오늘 점심 전부 알려 줘').count(',') == 10          # all eleven dishes


def test_the_day_is_taken_from_the_question():
    today = THURSDAY.date()
    assert which_day('내일 점심', today) == (dt.date(2026, 11, 13), '내일')
    assert which_day('내일모레 메뉴', today) == (dt.date(2026, 11, 14), '모레')
    assert which_day('어제 저녁', today) == (dt.date(2026, 11, 11), '어제')
    assert which_day('월요일 점심', today) == (dt.date(2026, 11, 9), '월요일')
    assert which_day('다음 주 화요일', today) == (dt.date(2026, 11, 17), '다음 주 화요일')
    assert which_day('13일 메뉴', today) == (dt.date(2026, 11, 13), '13일 금요일')
    assert which_day('11월 10일 저녁', today) == (dt.date(2026, 11, 10), '10일 화요일')
    assert which_day('메뉴 뭐야', today) is None and which_day('31일 메뉴', today) is None   # no 31st in November
    assert menu().answer('내일 점심 뭐야').startswith('내일 점심 메뉴는 눈꽃치즈 불닭볶음,')
    assert menu().answer('월요일 점심 뭐였어').startswith('월요일 점심 메뉴는 수제순살 파닭과 머스터드소스,')
    assert menu().answer('11월 10일 저녁 식단').startswith('10일 화요일 저녁 메뉴는 콩나물밥과 양념장,')


def test_the_meal_is_taken_from_the_question_or_the_time_of_day():
    assert [which_meal(t) for t in ('점심 뭐야', '석식 메뉴', '아침은?', '메뉴 뭐야')] == ['lunch', 'dinner', 'breakfast', '']
    assert menu().answer('메뉴 뭐야').startswith('오늘 점심 메뉴는')                      # 11:30
    assert menu(dt.datetime(2026, 11, 12, 15, 0)).answer('메뉴 뭐야').startswith('오늘 저녁 메뉴는 새우볶음밥')
    assert menu(dt.datetime(2026, 11, 12, 15, 0)).answer('내일 메뉴 뭐야').startswith('내일 점심 메뉴는')


def test_what_is_not_in_the_file_is_said_as_not_there():
    assert menu().answer('금요일 저녁 메뉴 알려 줘') == '금요일 저녁은 식단표에 없어.'
    assert menu().answer('다음 주 월요일 메뉴') == '다음 주 월요일 식단은 식단표에 없어.'
    assert menu().answer('아침 메뉴 뭐야') == '오늘 아침은 식단표에 없어.'
    assert Menu('/nonexistent/menu.yaml', style='cute', now=lambda: THURSDAY).answer('메뉴 뭐야') == '식단표를 못 읽었어.'
    assert Menu(FILE, now=lambda: THURSDAY).answer('금요일 저녁 메뉴') == '금요일 저녁은 식단표에 없어요.'      # polite


def test_other_questions_are_left_alone():
    for text in ('왜 멈췄어', '뭐 하는 상태야', '오늘 점심 먹었어?', '저녁에 뭐 해?', '', '고마워'):
        assert not is_menu_question(text), text
        assert menu().answer(text) is None


def test_a_follow_up_keeps_the_day_or_the_meal():
    m = menu()
    assert m.answer('내일은?', follow=True) is None                              # nothing to follow
    m.answer('오늘 점심 메뉴 뭐야')
    assert m.answer('저녁은?', follow=True).startswith('오늘 저녁 메뉴는 새우볶음밥')
    assert m.answer('수요일은?', follow=True).startswith('수요일 저녁 메뉴는 고깃집 마무리 볶음밥')
    assert m.answer('그럼 다음 주 월요일 점심은요?', follow=True) == '다음 주 월요일 식단은 식단표에 없어.'
    for text in ('고마워', '오늘 뭐 해?', '내일 어디 가?'):                        # not about the menu
        assert m.answer(text, follow=True) is None, text


def test_the_file_is_read_again_when_it_changes(tmp_path):
    path = tmp_path / 'menu.yaml'
    path.write_text('days:\n  "2026-11-12":\n    lunch: [김밥]\n', encoding='utf-8')
    m = Menu(str(path), style='cute', now=lambda: THURSDAY)
    assert m.answer('오늘 점심 메뉴') == '오늘 점심 메뉴는 김밥이야!'
    path.write_text('days:\n  "2026-11-12":\n    lunch: [국수, 만두]\n', encoding='utf-8')
    import os
    os.utime(path, (1, 1))
    assert m.answer('오늘 점심 메뉴') == '오늘 점심 메뉴는 국수, 만두야!'
    path.write_text('days: [broken', encoding='utf-8')
    os.utime(path, (2, 2))
    assert m.answer('오늘 점심 메뉴') == '식단표를 못 읽었어.'


def test_the_robot_answers_the_menu_without_gpt():
    backend = Backend(reply='GPT가 지어낸 메뉴')
    narrator, texts = make(backend=backend)
    narrator.quick_answer = menu().answer
    narrator.planning_state('Ready')
    narrator._jobs.clear()
    narrator._utterances.clear()
    narrator.wake('오늘 점심 메뉴 뭐야')
    assert not narrator._jobs                                                    # nothing for GPT to do
    assert narrator.speak_once() is True
    assert texts == [('ask', '오늘 점심 메뉴는 진국 차슈라멘, 언양식 파채불고기, 새우튀김, 김치비빔국수야!')]
    narrator.planning_state('Run.Execute')
    narrator._jobs.clear()
    narrator._utterances.clear()
    narrator.followup('저녁은?')
    assert not narrator._jobs and narrator.speak_once() is True and texts[-1][1].startswith('오늘 저녁 메뉴는')
    narrator.ask('금요일 저녁 메뉴')                                              # /voice/ask goes the same way
    assert narrator.speak_once() is True and texts[-1] == ('ask', '금요일 저녁은 식단표에 없어.')
    narrator.clock.now += 60.0
    narrator.followup('내일은?')                                                  # too long after: an ordinary follow-up
    assert narrator._jobs and narrator._jobs[0].followup
    narrator.wake('왜 멈췄어')                                                    # other questions still go to GPT
    assert any(job.text == '왜 멈췄어' for job in narrator._jobs)
