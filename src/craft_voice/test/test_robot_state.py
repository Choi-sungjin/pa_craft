from craft_voice.robot_state import StateTracker, parse_list, parse_mapping, reason_ko


def diag(tracker, message, now, **values):
    tracker.update_diagnostics('planning/state_machine', 0, message, values, now)


def test_changes_only_and_announce_filter():
    t = StateTracker()
    assert t.update_planning('Idle', 0.0) is None            # not announced
    event = t.update_planning('Ready', 1.0)
    assert event.label == 'Ready' and event.previous == 'Idle'
    assert t.update_planning('Ready', 1.05) is None           # 20 Hz repeats are not events
    assert t.update_planning('Run.Compute', 2.0) is None


def test_cooldown_per_label():
    t = StateTracker(cooldown_s=10.0)
    assert t.update_planning('Run.Execute', 0.0)
    t.update_planning('Run.Complete', 1.0)
    t.update_planning('Run.AwaitRequest', 1.05)
    assert t.update_planning('Run.Execute', 2.0) is None      # within cooldown
    t.update_planning('Run.Compute', 5.0)
    assert t.update_planning('Run.Execute', 11.0)


def test_one_tick_complete_is_announced():
    t = StateTracker()
    t.update_planning('Run.Execute', 0.0)
    event = t.update_planning('Run.Complete', 3.0)            # published for a single 50 ms loop
    assert event and event.label == 'Run.Complete'
    assert t.update_planning('Run.AwaitRequest', 3.05) is None


def test_fault_waits_for_reason():
    t = StateTracker(fault_reason_wait_s=1.2)
    t.update_planning('Run.Execute', 0.0)
    assert t.update_planning('Fault', 5.0) is None
    assert t.poll(5.5) is None
    diag(t, 'Fault: Run.Execute: execution timeout', 5.6)
    event = t.poll(5.6)
    assert event.fault and event.reason == 'Run.Execute: execution timeout'
    assert t.template(event) == '문제가 생겨서 멈췄어요. 원인: 실행 시간 초과.'
    assert t.poll(6.0) is None                                # announced once


def test_fault_without_reason_after_wait_and_reset_cancels():
    t = StateTracker(fault_reason_wait_s=1.2)
    t.update_planning('Fault', 0.0)
    assert t.poll(1.0) is None
    event = t.poll(1.3)
    assert event.reason == '' and t.template(event) == '문제가 생겨서 멈췄어요.'
    t.update_planning('Idle', 2.0)
    t.update_planning('Fault', 3.0)
    t.update_planning('Idle', 3.2)                            # RESET before the reason came
    assert t.poll(10.0) is None


def test_old_fault_reason_is_not_reused():
    t = StateTracker(fault_reason_wait_s=1.0)
    t.update_planning('Fault', 0.0)
    diag(t, 'Fault: Run.Compute: compute timeout', 0.5)
    assert t.poll(0.5).reason.endswith('compute timeout')
    t.update_planning('Idle', 2.0)
    t.update_planning('Fault', 10.0)
    assert t.poll(11.1).reason == ''


def test_reason_arriving_just_before_state():
    t = StateTracker()
    diag(t, 'Fault: Run.Execute: stale odom/lidar input', 4.99)
    event = t.update_planning('Fault', 5.0)
    assert event and reason_ko(event.reason) == '오도메트리나 라이다 입력 끊김'


def test_step_names_in_templates():
    t = StateTracker(step_names=parse_mapping('approach:접근, grasp_pose : 잡기 자세'))
    diag(t, 'Run.Execute: ', 0.0, step='base_goal', task_id='3', queued='0')
    assert t.template(t.update_planning('Run.Execute', 0.1)) == '목표 지점 이동 작업을 시작할게요.'
    diag(t, 'Run.Execute: ', 20.0, step='approach')
    t.update_planning('Run.Complete', 20.1)
    assert t.current_template(20.2) == '접근 작업을 마쳤어요.'


def test_step_from_previous_step_is_not_used():
    """Diagnostics are 1 Hz: the next step's Run.Execute can come before any diagnostics naming it."""
    t = StateTracker(step_names=parse_mapping('approach:접근'), cooldown_s=0.0)
    t.update_planning('Run.Execute', 5.0)
    diag(t, 'Run.Execute: ', 9.98, step='approach')
    assert t.template(t.update_planning('Run.Complete', 10.0)) == '접근 작업을 마쳤어요.'
    for at, label in ((10.05, 'Run.AwaitRequest'), (10.10, 'Run.PrepareTask'), (10.15, 'Run.Compute')):
        t.update_planning(label, at)
    event = t.update_planning('Run.Execute', 10.4)
    assert event.step == '' and t.template(event) == '작업을 시작할게요.'
    assert '현재 단계' not in t.context_text(10.5)
    diag(t, 'Run.Execute: ', 10.98, step='grasp')
    assert t.step_name() == 'grasp' and '현재 단계: grasp' in t.context_text(11.0)


def test_new_step_detected_without_prepare_task_sample():
    t = StateTracker(cooldown_s=0.0)
    diag(t, 'Run.Execute: ', 9.98, step='approach')
    t.update_planning('Run.Complete', 10.0)
    t.update_planning('Run.AwaitRequest', 10.05)
    assert t.update_planning('Run.Compute', 10.2) is None     # PrepareTask sample lost
    assert t.update_planning('Run.Execute', 10.4).step == ''


def test_context_text():
    t = StateTracker()
    assert '데이터 없음' in t.context_text(0.0)
    t.update_planning('Run.Execute', 0.0)
    t.update_planning('Fault', 2.0)
    diag(t, 'Fault: Run.Execute: execution timeout', 2.5, step='base_goal', task_id='4', queued='1')
    t.update_control(True, False, True, True, '', 2.5)
    t.update_context('/perception/localization/state', 'dead_reckoning', 2.5)
    text = t.context_text(4.0)
    assert '상태머신: Fault (고장으로 정지), 2초째' in text
    assert '고장 원인: Run.Execute: execution timeout (실행 시간 초과)' in text
    assert 'base_goal (목표 지점 이동), task 4' in text
    assert '최근 상태 변화: Run.Execute -> Fault' in text
    assert '제어(control) 상태 플래그: ready, held, stopped' in text
    assert '/perception/localization/state: dead_reckoning' in text
    assert '데이터 없음' in t.context_text(60.0)                # planning stopped publishing


def test_parsers():
    assert parse_list(' a, ,b ,') == ['a', 'b']
    assert parse_mapping('x:1, bad, :2, y: ') == {'x': '1'}
    assert reason_ko('weird thing') == 'weird thing'
