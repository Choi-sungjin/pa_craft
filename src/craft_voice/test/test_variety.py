"""Further wordings in turn, and drives said with what the robot knows (variety.py)."""

import math

from craft_voice import variety
from craft_voice.prepare import fixed_sentences
from craft_voice.robot_state import StateTracker

DIAG = {'step': 'base_goal', 'task_id': '1', 'queued': '0', 'waiting_for': ''}
BLOCKED = 'path blocked; waiting before replanning'


def leg(t: StateTracker, now: float, goal, pose, seconds: float = 10.0, waiting: str = ''):
    """One drive to a goal: (sentence at the start, sentence at the arrival, time after it)."""
    t.update_goal(*goal, now)
    t.update_planning('Run.PrepareTask', now + 0.1)
    t.update_diagnostics('planning/state_machine', 0, 'Run.PrepareTask: ', DIAG, now + 0.2)
    t.update_planning('Run.Compute', now + 0.3)
    t.update_pose(*pose, now + 0.4)
    start = t.update_planning('Run.Execute', now + 0.5)
    said = t.template(start)
    if waiting:
        t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', dict(DIAG, waiting_for=waiting), now + 2.0)
        t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', DIAG, now + 4.0)
    done = t.update_planning('Run.Complete', now + 0.5 + seconds)
    arrived = t.template(done)
    t.update_planning('Run.AwaitRequest', now + 0.6 + seconds)
    return said, arrived, now + 1.0 + seconds


def tracker(**kw) -> StateTracker:
    t = StateTracker(style='cute', variety=True, cooldown_s=0.0, step_name_wait_s=0.0, **kw)
    t.update_planning('Ready', 0.0)
    return t


def test_numbers_are_rounded_and_bounded():
    assert [variety.metres(d) for d in (0.2, 0.6, 3.4, 12.4, 12.6, None, math.nan, -1)] == [0, 1, 3, 12, None, None, None, None]
    assert [variety.seconds(s) for s in (1, 7, 8, 58, 63, None)] == [5, 5, 10, 60, None, None]
    assert [variety.ordinal(n) for n in (1, 3, 20, 21, 0)] == ['첫 번째', '세 번째', '스무 번째', '', '']
    quarter = math.pi / 2
    assert [variety.direction(b) for b in (0, 0.7, quarter, math.pi, -quarter, -0.7)] == \
        ['앞쪽', '앞쪽', '왼쪽', '뒤쪽', '오른쪽', '앞쪽']


def test_off_by_default_nothing_changes():
    t = StateTracker(style='cute', cooldown_s=0.0, step_name_wait_s=0.0)
    t.update_planning('Ready', 0.0)
    now = 1.0
    for _ in range(3):
        said, arrived, now = leg(t, now, (3.0, 0.0), (0.0, 0.0, 0.0))
        assert (said, arrived) == ('목표 지점 이동 시작할게!', '목표 지점 이동 끝났어!')


def test_a_tour_is_not_the_same_two_sentences():
    t = tracker()
    now, starts, arrivals = 1.0, [], []
    for i in range(8):
        said, arrived, now = leg(t, now, (3.0 + i, 0.0), (0.0, 0.0, 0.0), seconds=8.0 + i)
        starts.append(said)
        arrivals.append(arrived)
    assert len(set(starts)) >= 6 and len(set(arrivals)) >= 6
    assert all(a != b for a, b in zip(starts, starts[1:])) and all(a != b for a, b in zip(arrivals, arrivals[1:]))
    assert starts[0] == '다음 지점까지 3미터, 출발!' and arrivals[0] == '첫 번째 지점 도착!'
    known = set(fixed_sentences('cute', {}, {}, variety=True))
    assert set(starts) | set(arrivals) <= known          # all of them can be synthesized beforehand


def test_the_facts_are_the_real_ones():
    t = tracker()
    said, arrived, now = leg(t, 1.0, (0.0, 4.2), (0.0, 0.0, 0.0), seconds=23.0)     # goal on the left, 4.2 m
    assert said == '다음 지점까지 4미터, 출발!' and arrived == '첫 번째 지점 도착!'
    leg(t, now, (9.0, 9.0), (0.0, 0.0, 0.0))                                         # plain wording in between
    said, arrived, now = leg(t, now + 20, (0.0, -2.0), (0.0, 0.0, 0.0), seconds=23.0)
    assert said == '2미터 앞으로 가 볼게!' and arrived == '세 번째 지점 찍었어!'
    leg(t, now, (9.0, 9.0), (0.0, 0.0, 0.0))
    said, arrived, now = leg(t, now + 20, (0.0, -2.0), (0.0, 0.0, 0.0), seconds=23.0)
    assert said == '오른쪽 2미터 지점으로 갈게!' and arrived == '도착! 25초쯤 걸렸어.'


def test_a_blocked_leg_is_said_as_blocked():
    t = tracker()
    said, arrived, _ = leg(t, 1.0, (3.0, 0.0), (0.0, 0.0, 0.0), waiting=BLOCKED)
    assert arrived == '길이 막혔었지만 첫 번째 지점 도착!'


def test_without_a_fresh_goal_or_pose_the_plain_words_are_used():
    t = tracker()
    said, _, now = leg(t, 1.0, (math.nan, 0.0), (0.0, 0.0, 0.0))           # no goal received
    assert '미터' not in said and '목표 지점 이동' in said
    t.update_goal(3.0, 0.0, now)
    t.update_pose(0.0, 0.0, 0.0, now)
    said, _, now = leg(t, now, (math.nan, 0.0), (math.nan, 0.0, 0.0))      # the old goal and an old pose
    said, _, now = leg(t, now, (math.nan, 0.0), (math.nan, 0.0, 0.0))
    assert '미터' not in said
    said, _, _ = leg(t, now, (40.0, 0.0), (0.0, 0.0, 0.0))                 # farther than MAX_M
    assert '미터' not in said


def test_the_same_event_gets_the_same_sentence_again():
    t = tracker()
    t.update_goal(3.0, 0.0, 1.0)
    t.update_planning('Run.PrepareTask', 1.1)
    t.update_diagnostics('planning/state_machine', 0, 'Run.PrepareTask: ', DIAG, 1.2)
    t.update_pose(0.0, 0.0, 0.0, 1.3)
    event = t.update_planning('Run.Execute', 1.5)
    assert t.template(event) == t.template(event) == '다음 지점까지 3미터, 출발!'


def test_what_are_you_doing_says_how_far_is_left():
    t = tracker()
    t.update_goal(5.0, 0.0, 1.0)
    t.update_planning('Run.PrepareTask', 1.1)
    t.update_diagnostics('planning/state_machine', 0, 'Run.PrepareTask: ', DIAG, 1.2)
    t.update_planning('Run.Execute', 1.5)
    t.update_pose(2.2, 0.0, 0.0, 6.0)
    t.update_planning('Run.Execute', 6.0)
    t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', DIAG, 6.0)
    assert t.doing(6.1) == '지금 목표 지점 이동 중이야! 3미터쯤 남았어.'
    t.update_pose(4.8, 0.0, 0.0, 9.0)
    t.update_planning('Run.Execute', 9.0)
    t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', DIAG, 9.0)
    assert t.doing(9.1) == '지금 목표 지점 이동 중이야! 거의 다 왔어!'
    assert t.doing(11.0) == '지금 목표 지점 이동 중이야!'                   # the pose is old: no distance
    known = fixed_sentences('cute', {}, {}, variety=True)
    assert '지금 목표 지점 이동 중이야! 3미터쯤 남았어.' in known and '지금 목표 지점 이동 중이야! 거의 다 왔어!' in known


def test_situations_and_the_ack_take_turns_too():
    t = tracker()
    said = [t.phrase('situation.blocked') for _ in range(6)]
    assert said[0] == '길이 막혀서 잠깐 기다리는 중이야.' and len(set(said[:5])) == 5 and said[5] == said[0]
    assert '아? 예? 여기 뭐 있는데요.' in said and '아, 깜짝이야!' in said          # lines of the StarCraft II SCV
    assert len({t.phrase('ack') for _ in range(4)}) == 4
    assert t.phrase('doing.Ready') == t.phrase('doing.Ready')             # answers stay as they are
    custom = StateTracker(style='cute', variety=True, lines={'situation.blocked': '막혔다!'})
    assert [custom.phrase('situation.blocked') for _ in range(2)] == ['막혔다!', '막혔다!']   # a phrases file wins


def test_a_path_found_again_is_cheered_when_the_drive_goes_on():
    replanning = 'path still blocked; replanning'

    def run(announce, end_label='Run.Execute'):
        t = tracker(announce_situations=announce, situation_cooldown_s=0.0)
        t.update_planning('Run.Execute', 1.0)
        t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', DIAG, 1.1)
        t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', dict(DIAG, waiting_for=BLOCKED), 2.0)
        searching = t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                                         dict(DIAG, waiting_for=replanning), 5.0)
        if end_label != 'Run.Execute':
            t.update_planning(end_label, 6.9)
        return t, searching, t.update_diagnostics('planning/state_machine', 0, end_label + ': ', DIAG, 7.0)

    t, searching, found = run(('replan', 'blocked', 'rerouted'))
    assert searching.situation == 'replan' and found.situation == 'rerouted' and t.template(found) == '나이사!'
    assert run(('replan', 'blocked'))[2] is None                              # only when asked for
    assert run(('replan', 'blocked', 'rerouted'), 'Run.Complete')[2] is None  # the step ended instead: no drive
    t = tracker(announce_situations=('replan', 'blocked', 'rerouted'), situation_cooldown_s=0.0)
    t.update_planning('Run.Execute', 1.0)
    t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', dict(DIAG, waiting_for=BLOCKED), 2.0)
    assert t.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', DIAG, 3.0) is None   # no search
