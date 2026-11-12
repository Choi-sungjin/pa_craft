"""Signals besides the state machine: control's task, the command, the barcode, the gripper, obstacles, the box."""

import math

from craft_voice.robot_state import StateTracker
from craft_voice.signals import FAILED, REJECTED, RUNNING, SUCCEEDED, Signals, command_ko
from test_narrator import Backend, make

GRASP = '/control/gripper_grasped'


def running(tracker, now=100.0):
    tracker.update_planning('Run.Execute', now)
    return tracker


def keys(tracker, now):
    if tracker.label:
        tracker.update_planning(tracker.label, now)   # the state machine keeps publishing
    found = []
    while True:
        noticed = tracker.notice(now)
        if noticed is None:
            return found
        found.append(noticed.key)


def test_a_failed_task_is_remembered_once_with_its_reason():
    tracker = running(StateTracker())
    tracker.update_task_result(7, RUNNING, [0, 0, 1, 0, 0], '', 100.0)
    assert '- 제어의 작업 결과(/control/task_result): task 7 실행 중, 아직 움직이는 축: 바퀴' in tracker.context_text(100.5)
    for now in (101.0, 101.1, 102.0):                                    # published every cycle
        tracker.update_task_result(7, FAILED, [0, 0, 3, 0, 0], 'serial command write failed', now)
    assert [text for _, text in tracker.journal].count('제어가 task 7을 실패로 끝냄: serial command write failed') == 1
    assert 'task 7 실패, 이유: serial command write failed' in tracker.context_text(102.5)
    late = tracker.context_text(200.0)                                   # control went quiet: the failure stays known
    assert '/control/task_result' not in late and '제어가 실패로 끝낸 작업: task 7, 이유: serial command write failed' in late
    tracker.update_task_result(8, REJECTED, [0] * 5, 'no matching navigation task', 201.0)
    assert '제어가 task 8을 거부하고 끝냄: no matching navigation task' in [text for _, text in tracker.journal]
    tracker.update_task_result(9, SUCCEEDED, [0, 0, 2, 0, 0], '', 202.0)
    assert 'task 9 완료' in tracker.context_text(202.5)


def test_the_command_is_told_by_its_kind():
    assert command_ko(0, 0) == '주행' and command_ko(1, 0) == '주행 경로 갱신' and command_ko(2, 0) == '정지'
    assert command_ko(0, 2, close=True) == '그리퍼 닫기' and command_ko(0, 2) == '그리퍼 열기'
    assert command_ko(0, 3, clock_slot=4) == '회전판(clock) 4번 칸으로 돌리기' and command_ko(0, 4, 3.0) == '3.0초 대기'
    tracker = running(StateTracker())
    tracker.update_task_command(5, 0, 0, 0.0, 0, False, 100.0)            # a drive: the state machine says it already
    tracker.update_task_command(5, 1, 0, 0.0, 0, False, 101.0)
    assert not [text for _, text in tracker.journal if text.startswith('제어에 내린 명령')]
    tracker.update_task_command(6, 0, 4, 3.0, 0, False, 102.0)
    assert [text for _, text in tracker.journal][-1] == '제어에 내린 명령: 3.0초 대기'
    assert '마지막으로 내린 명령(/planning/task_command): 3.0초 대기 (task 6)' in tracker.context_text(103.0)


def test_a_grasp_is_said_when_it_happens():
    tracker = running(StateTracker(style='cute'))
    tracker.update_context(GRASP, 'true', 100.0)                          # the first value: not a change
    assert keys(tracker, 100.1) == []
    tracker.update_context(GRASP, 'false', 101.0)
    tracker.update_context(GRASP, 'true', 110.0)
    tracker.update_context(GRASP, 'true', 110.1)
    assert keys(tracker, 110.2) == ['grasped']
    tracker.update_context(GRASP, 'false', 111.0)
    tracker.update_context(GRASP, 'true', 112.0)                          # chatter right after: not again
    assert keys(tracker, 112.1) == []
    waiting = StateTracker()
    waiting.update_planning('Ready', 100.0)                               # not running: nothing to cheer
    waiting.update_context(GRASP, 'false', 100.0)
    waiting.update_context(GRASP, 'true', 101.0)
    assert keys(waiting, 101.1) == []


def test_the_barcode_is_said_with_its_order():
    tracker = StateTracker(style='cute')
    tracker.update_planning('Ready', 100.0)
    tracker.update_barcode('P2-02', 100.0, quiet=True)                    # read before the voice started
    assert keys(tracker, 100.1) == [] and '읽은 바코드: P2-02 (내림차순)' in tracker.context_text(100.2)
    tracker.update_barcode('', 101.0)                                     # reset
    assert '읽은 바코드' not in tracker.context_text(101.1)
    tracker.update_barcode('P2-03', 102.0)
    tracker.update_barcode('P2-03', 102.5)                                # latched again
    noticed = tracker.notice(102.6)
    assert noticed.key == 'barcode_ascending' and tracker.phrase('notice.barcode_ascending') == '좋아, 바코드 읽었어! 오름차순이야.'
    assert keys(tracker, 102.7) == []
    tracker.update_barcode('X9', 110.0)
    assert keys(tracker, 110.1) == ['barcode_unknown'] and '읽은 바코드: X9 (등록되지 않은 코드)' in tracker.context_text(110.2)
    tracker.update_barcode('P2-01', 120.0)
    assert keys(tracker, 130.0) == []                                     # too late to be news


def test_the_nearest_obstacle_is_placed_from_the_robot():
    signals = Signals()
    signals.obstacles([(2.0, 0.0), (0.0, 0.8), (9.0, 9.0)], 100.0)          # seen from the robot
    distance, bearing = signals.nearest_obstacle(100.1)
    assert math.isclose(distance, 0.8) and math.isclose(bearing, math.pi / 2)
    assert signals.obstacle_words(100.1) == ('왼쪽', 1) and signals.nearest_obstacle(110.0) is None
    signals.obstacles([(9.0, 9.0)], 120.0)
    assert signals.nearest_obstacle(120.1) is None and signals.lines(120.1) == ['- 라이다 장애물: 3 m 안에 없음']
    tracker = running(StateTracker(style='cute'))
    tracker.update_obstacles([(1.0, 3.0)], 100.0)                         # map frame, no pose yet: not used
    assert tracker.obstacle_sentence(100.1) == ''
    tracker.update_pose(1.0, 1.0, math.pi / 2, 100.0)                     # facing +y: the point is 2 m ahead
    tracker.update_obstacles([(1.0, 3.0), (3.5, 1.0)], 100.2)
    assert tracker.obstacle_sentence(100.3) == '앞쪽 2미터쯤에 뭐가 있어.'
    assert '- 가장 가까운 라이다 장애물: 앞쪽 2.0 m (장애물 점 2개)' in tracker.context_text(100.3)
    tracker.update_obstacles([(1.0, 1.3)], 100.6)
    assert tracker.obstacle_sentence(100.7) == '바로 앞쪽에 뭐가 있어.'
    assert StateTracker().phrase('obstacle', dir='뒤쪽', d=3) == '뒤쪽 3미터쯤에 장애물이 있어요.'


def test_a_blocked_robot_says_where_the_obstacle_is_when_asked():
    tracker = StateTracker(style='cute', step_name_wait_s=0.0)
    tracker.update_planning('Run.Execute', 100.0)
    tracker.update_diagnostics('planning/state_machine', 0, 'Run.Execute', {
        'step': 'base_goal', 'waiting_for': 'path blocked; waiting before replanning'}, 100.5)
    assert tracker.doing(101.0) == '길이 막혀서 잠깐 기다리는 중이야.'
    tracker.update_obstacles([(0.0, -1.2)], 101.0, in_map=False)
    assert tracker.doing(101.5) == '길이 막혀서 잠깐 기다리는 중이야. 오른쪽 1미터쯤에 뭐가 있어.'


def test_a_box_seen_after_none_is_said_once():
    tracker = running(StateTracker(style='cute'))
    tracker.update_box(False, 'lidar', 'no face', math.nan, math.nan, math.nan, 100.0)
    assert keys(tracker, 100.1) == [] and '- 상자 관측(lidar): 실패, 이유: no face' in tracker.context_text(100.2)
    tracker.update_box(True, 'lidar', '', 0.42, -0.03, math.radians(2), 101.0)
    tracker.update_box(True, 'lidar', '', 0.41, -0.03, math.radians(2), 101.2)
    assert keys(tracker, 101.3) == ['box_found']
    assert '- 상자 관측(lidar): 앞 간격 0.41 m, 좌우 어긋남 -0.03 m, 기울기 +2도' in tracker.context_text(101.4)
    tracker.update_box(False, 'lidar', 'lost', 0.0, 0.0, 0.0, 105.0)
    tracker.update_box(True, 'lidar', '', 0.40, 0.0, 0.0, 106.0)          # found again right away: not again
    assert keys(tracker, 106.1) == []
    tracker.update_box(True, 'uppercam', '', 0.30, 0.0, 0.0, 200.0)       # none for a long while: news again
    assert keys(tracker, 200.1) == ['box_found']


def test_remarks_are_spoken_in_fixed_words_even_with_gpt():
    backend = Backend(reply='GPT가 지어낸 말')
    narrator, texts = make(backend=backend, notice=True)
    narrator.tracker.style = 'cute'
    narrator.planning_state('Run.Execute')
    narrator._jobs.clear()
    narrator._utterances.clear()
    narrator.context(GRASP, 'false')
    narrator.clock.now += 1.0
    narrator.context(GRASP, 'true')
    narrator.tick()
    assert not narrator._jobs and narrator.speak_once() is True and texts[-1] == ('thought', '잡았다!')
    narrator.clock.now += 1.0
    narrator.barcode('P2-04')
    narrator.tick()
    assert not narrator._jobs and narrator.speak_once() is True and texts[-1] == ('thought', '좋아, 바코드 읽었어! 내림차순이야.')
    narrator.task_result(3, FAILED, [0, 0, 3, 0, 0], 'invalid path update')
    narrator.task_command(4, 0, 4, 2.0, 0, False)
    narrator.box(True, 'lidar', '', 0.5, 0.0, 0.0)
    narrator.obstacles([(0.5, 0.0)], in_map=False)
    facts = narrator.tracker.context_text(narrator.clock.now)
    for part in ('task 3 실패, 이유: invalid path update', '2.0초 대기 (task 4)', '상자 관측(lidar): 앞 간격 0.50 m',
                 '가장 가까운 라이다 장애물: 앞쪽 0.5 m'):
        assert part in facts, part


def fault(tracker, reason, now):
    tracker.update_planning('Fault', now)
    if reason:
        tracker.update_diagnostics('planning/state_machine', 2, 'Fault: ' + reason, {}, now + 0.1)
    return tracker.poll(now + (0.2 if reason else 2.0))


def test_the_fault_sentence_says_why_control_ended_the_task():
    from craft_voice.robot_state import REASON_KO, fault_situation
    said = {reason: fault_situation('control task: ' + reason, 'cute') for reason in (
        'serial command write failed', 'serial feedback or board fault', 'odometry timeout',
        'camera observation timeout', 'task timeout', 'replayed camera update', 'invalid path update',
        'control command queue full', 'control is held or unavailable', 'arm disabled (arm_enabled:=false)',
        'invalid task target/calibration')}
    assert said['serial feedback or board fault'] == '모터 보드에 문제가 생겨서 멈췄어.'
    assert said['camera observation timeout'] == '카메라 정보가 끊겨서 멈췄어.'       # not the box approach's 'camera'
    assert said['task timeout'] == '동작이 제시간에 안 끝나서 멈췄어.'
    assert '지금은 멈춰 있어. 확인이 필요해.' not in said.values() and len(set(said.values())) == len(said)
    for reason in ('no matching navigation task', 'unknown stop task', ''):       # no words of their own
        assert fault_situation('control task: ' + reason, 'cute') == '제어가 동작을 못 끝내서 멈췄어.'
    assert fault_situation('Run.Execute: stale control task result', 'cute') == '제어 쪽 응답이 끊겨서 멈췄어.'
    needles = [needle for needle, _, _ in REASON_KO]
    assert needles.index('camera observation timeout') < needles.index('camera')


def test_a_fault_that_names_no_cause_takes_the_one_control_gave():
    tracker = running(StateTracker(style='cute'))
    tracker.update_task_result(7, FAILED, [0, 0, 3, 0, 0], 'odometry timeout', 109.5)
    event = fault(tracker, 'box L1: stage task failed', 110.0)
    assert tracker.template(event) == '위치 정보가 끊겨서 멈췄어.'
    assert tracker.current_template(111.0) == '위치 정보가 끊겨서 멈췄어.'              # "왜 멈췄어?" says the same
    old = running(StateTracker(style='cute'))                                    # a failure long before: not the cause
    old.update_task_result(7, FAILED, [0, 0, 3, 0, 0], 'odometry timeout', 100.0)
    assert old.template(fault(old, 'box L1: stage task failed', 110.0)) == '가던 길에서 문제가 생겨서 멈췄어.'
    own = running(StateTracker(style='cute'))                                    # planning names the cause: kept
    own.update_task_result(7, FAILED, [0, 0, 3, 0, 0], 'odometry timeout', 109.5)
    assert own.template(fault(own, 'Run.Execute: localization lost', 110.0)) == '내 위치를 놓쳐서 멈췄어.'
    odd = running(StateTracker(style='cute'))                                    # a reason without words: unchanged
    odd.update_task_result(7, REJECTED, [0] * 5, 'something new', 109.5)
    assert odd.template(fault(odd, '', 110.0)) in ('지금은 멈춰 있어.', '지금은 멈춰 있어. 확인이 필요해.')


def light(tracker, now, blinks, red, yellow, green, complete=False, valid=False, discarded=0, quiet=False):
    tracker.update_traffic_light(red, yellow, green, blinks, complete, valid, discarded, now, quiet)


def test_the_traffic_light_is_said_without_the_state_machine():
    tracker = StateTracker(style='cute')                                  # no planning state at all
    light(tracker, 100.0, 1, 1, 0, 0)
    noticed = tracker.notice(100.1)
    assert noticed.key == 'traffic_start' and noticed.facts == '신호등 깜빡인다! 몇 번인지 세어 볼게.'
    for n in range(2, 12):                                                # blinks in between: nothing to say
        light(tracker, 100.0 + n, n, 1, 0, n - 1)
        assert tracker.notice(100.1 + n) is None
    assert '신호등 깜빡임(/perception/vision/uppercam/traffic_light): 빨강 1번' in tracker.context_text(111.5)
    light(tracker, 113.0, 12, 4, 3, 5, complete=True, valid=True)
    noticed = tracker.notice(113.1)
    assert noticed.key == 'traffic_done' and noticed.facts == '신호등 다 셌어! 빨강 4번, 노랑 3번, 초록 5번이야.'
    light(tracker, 114.0, 12, 4, 3, 5, complete=True, valid=True)       # the same result again (latched)
    assert tracker.notice(114.1) is None
    assert '12번 다 셈' in tracker.context_text(114.2)


def test_a_wrong_count_is_said_once_and_a_known_result_is_quiet():
    tracker = StateTracker(style='polite')
    light(tracker, 100.0, 4, 2, 1, 1)
    light(tracker, 101.0, 1, 1, 0, 0, discarded=1)                        # recounting from the start
    assert tracker.notice(101.1).key == 'traffic_recount'
    assert tracker.notice(101.2) is None
    other = StateTracker(style='cute')
    light(other, 100.0, 12, 4, 4, 4, complete=True, valid=True, quiet=True)  # finished before the voice started
    assert other.notice(100.1) is None and '빨강 4번, 노랑 4번, 초록 4번' in other.context_text(100.2)


def test_the_traffic_light_result_is_spoken_with_its_counts():
    narrator, texts = make(backend=Backend(reply='GPT가 지어낸 말'), notice=True)
    narrator.tracker.style = 'cute'
    narrator.traffic_light(2, 6, 4, 12, True, True, 0)
    narrator.tick()
    assert not narrator._jobs and narrator.speak_once() is True
    assert texts[-1] == ('thought', '신호등 다 셌어! 빨강 2번, 노랑 6번, 초록 4번이야.')


def test_every_traffic_light_result_is_prepared():
    from craft_voice.prepare import fixed_sentences
    sentences = fixed_sentences('cute', {}, {})
    counts = [s for s in sentences if s.startswith('신호등 다 셌어!')]
    assert '신호등 다 셌어! 빨강 2번, 노랑 6번, 초록 4번이야.' in counts and len(counts) == 25
    assert '신호등 깜빡인다! 몇 번인지 세어 볼게.' in sentences
