"""모터 배터리: 쉬는 동안의 전압이 기준 아래면 바퀴 모터인지 팔 모터인지 말한다. 주행 중 순간 강하는 말하지 않는다."""

from craft_voice.robot_state import StateTracker
from craft_voice.signals import parse_supply
from test_narrator import Backend, Clock, make, run


def line(volts, lowest=None, amps=0.02):
    """control_master's republished firmware line: 5 wheel+clock motors at `volts`."""
    lowest = volts if lowest is None else lowest
    v = ','.join(f'{volts:.1f}' for _ in range(5))
    a = ','.join(f'{amps:.2f}' for _ in range(5))
    return f'D,0,0,0,{lowest:.1f},{v},-1,31,28,30,28,29,-1,{a}'


def feed(tracker, board, volts_at, start, seconds, lowest=None):
    said = []
    for i in range(seconds):
        now = start + i
        v = volts_at(i) if callable(volts_at) else volts_at
        tracker.update_motor_supply(board, line(v, lowest), now)
        while (n := tracker.notice(now + 0.01)) is not None:
            said.append(n)
    return said


def test_parse_the_recorded_line():
    assert parse_supply('D,0,0,0,11.4,11.6,11.5,11.5,11.6,11.4,-1,31,28,30,28,29,-1,0.02,0.00,0.00,0.00,0.00') \
        == (11.5, 11.4)
    assert parse_supply('D,0,0,0') is None and parse_supply('1,2,3') is None and parse_supply('D,x,y') is None


def test_a_mission_with_dips_but_a_full_battery_says_nothing():
    tracker = StateTracker()
    # the 01:09 mission: 11.5 V standing, 10.4-11.2 V while driving, standing again every ~13 s
    said = feed(tracker, 'wheel', lambda i: 11.5 if i % 13 == 0 else 10.9, 0, 600, lowest=10.4)
    assert said == []


def test_a_low_wheel_battery_is_said_once_with_its_board_and_volts():
    tracker = StateTracker()
    assert feed(tracker, 'wheel', 11.5, 0, 60) == []
    said = feed(tracker, 'wheel', 10.9, 60, 120)
    assert [n.key for n in said] == ['battery_low_wheel']
    assert said[0].facts == '바퀴 모터 배터리가 부족해요. 지금 10.9볼트예요. 곧 충전해 주세요.'
    assert feed(tracker, 'wheel', 10.9, 180, 300) == []                    # not again within 10 minutes
    assert [n.key for n in feed(tracker, 'wheel', 10.9, 480, 300)] == ['battery_low_wheel']   # 10 min after


def test_the_arm_board_is_named_and_critical_repeats_sooner():
    tracker = StateTracker(style='cute')
    said = feed(tracker, 'arm', 10.5, 0, 400)
    assert [n.key for n in said] == ['battery_critical_arm'] * 2              # at 40 s, again 3 minutes later
    assert said[0].facts == '팔 모터 배터리 거의 바닥이야! 10.5볼트야. 지금 바로 충전해 줘!'


def test_dropping_further_is_news_and_charging_resets():
    tracker = StateTracker()
    assert [n.key for n in feed(tracker, 'wheel', 10.9, 0, 100)] == ['battery_low_wheel']
    assert [n.key for n in feed(tracker, 'wheel', 10.6, 100, 100)] == ['battery_critical_wheel']
    assert feed(tracker, 'wheel', 12.3, 200, 100) == []                    # replaced / charged
    assert [n.key for n in feed(tracker, 'wheel', 10.9, 300, 100)] == ['battery_low_wheel']


def test_no_warning_before_the_window_is_filled_and_zero_disables():
    tracker = StateTracker()
    assert feed(tracker, 'wheel', 10.0, 0, 39) == []
    off = StateTracker(battery_low_v=0.0)
    assert feed(off, 'wheel', 10.0, 0, 200) == []


def test_the_facts_tell_the_resting_and_the_lowest_voltage():
    tracker = StateTracker()
    feed(tracker, 'wheel', 11.4, 0, 50, lowest=10.6)
    text = tracker.signals.lines(50.0)
    assert any('바퀴 모터 배터리 전압: 쉬는 동안 11.4 V (충분' in x and '순간 최저 10.6 V' in x for x in text)


def test_the_narrator_speaks_the_battery_line():
    clock = Clock()
    narrator, texts = make(Backend(), clock=clock, notice=True)
    for _ in range(45):                        # 45 s of a 10.8 V resting supply on the wheel board
        narrator.motor_supply('wheel', line(10.8))
        narrator.tick()
        run(narrator)
        clock.now += 1.0
    assert texts == [('thought', '바퀴 모터 배터리가 부족해요. 지금 10.8볼트예요. 곧 충전해 주세요.')]


def off_line(motors=5):
    """The firmware line with the motor power switched off: no motor answers, every V and deg C is 0."""
    return 'D,5,1,0,0.0,' + ','.join('0.0' for _ in range(motors)) + ',-1,' + ','.join('0' for _ in range(motors)) \
        + ',-1,' + ','.join('0.00' for _ in range(motors))


def feed_lines(tracker, board, lines, start):
    said = []
    for i, text in enumerate(lines):
        tracker.update_motor_supply(board, text, start + i)
        while (n := tracker.notice(start + i + 0.01)) is not None:
            said.append(n)
    return said


def test_zero_volts_on_every_motor_asks_to_switch_the_power_on():
    tracker = StateTracker()
    said = feed_lines(tracker, 'arm', [off_line(6)] * 3, 0)
    assert [n.key for n in said] == ['power_off_arm']                       # after two lines in a row, once
    assert said[0].facts == '팔 모터 전원이 0볼트예요. 전원을 올려 주세요.'
    assert feed_lines(tracker, 'arm', [off_line(6)] * 200, 3) == []         # not again within 5 minutes
    assert [n.key for n in feed_lines(tracker, 'arm', [off_line(6)] * 120, 203)] == ['power_off_arm']
    assert any('팔 모터 전원: 모든 모터가 0 V' in x for x in tracker.signals.lines(322.0))


def test_one_zero_line_or_one_dead_motor_is_not_power_off_and_power_on_resets():
    tracker = StateTracker()
    assert feed_lines(tracker, 'wheel', [off_line(), line(11.4)], 0) == []  # one line at start-up
    assert feed_lines(tracker, 'wheel', ['D,3,2,0,11.3,11.4,0.0,11.4,11.3,11.4,-1,30,0,30,30,30,-1,0,0,0,0,0'] * 5,
                      2) == []                                              # one motor not answering
    assert [n.key for n in feed_lines(tracker, 'wheel', [off_line()] * 2, 10)] == ['power_off_wheel']
    feed_lines(tracker, 'wheel', [line(11.4)] * 3, 12)                      # switched on
    said = feed_lines(tracker, 'wheel', [off_line()] * 2, 50)               # off again: news at once
    assert [n.key for n in said] == ['power_off_wheel']


def test_the_firmware_upload_reports_a_board_without_motor_power():
    tracker = StateTracker(style='cute')
    tracker.update_event('firmware_done', 100.0)
    tracker.update_event('power_off_wheel', 100.5)
    said = []
    while (n := tracker.notice(101.0)) is not None:
        said.append(n)
    assert [(n.key, n.facts) for n in said] == [('firmware_done', '업그레이드 완료.'),
                                              ('power_off_wheel', '바퀴 모터 전원이 0볼트야! 전원 올려 줘.')]
    # the control board then reports the same 0 V: not said again right away
    assert feed_lines(tracker, 'wheel', [off_line()] * 3, 110) == []


def test_the_power_off_sentences_are_prepared():
    from craft_voice.prepare import fixed_sentences
    sentences = fixed_sentences('cute', {}, {})
    assert '팔 모터 전원이 0볼트야! 전원 올려 줘.' in sentences and '바퀴 모터 전원이 0볼트야! 전원 올려 줘.' in sentences


def test_the_narrator_speaks_the_power_off_event():
    narrator, texts = make(Backend(), clock=Clock(), notice=True)
    narrator.event('power_off_arm')
    narrator.tick()
    run(narrator)
    assert texts == [('thought', '팔 모터 전원이 0볼트예요. 전원을 올려 주세요.')]


def test_the_power_off_remark_waits_behind_other_speech():
    tracker = StateTracker()
    tracker.update_event('power_off_arm', 100.0)
    assert [n.key for n in [tracker.notice(130.0)] if n] == ['power_off_arm']   # 30 s later: still said
    tracker.update_event('firmware_done', 200.0)
    assert tracker.notice(210.0) is None                                         # others stay 5 s news
