import pytest
from craft_voice.robot_state import StateTracker
from craft_voice.narrator import Narrator, NarratorConfig
from craft_voice.stops import StopMonitor
from test_narrator import Clock, Backend, Player


def feed(t, at, v=0, w=0, waiting='', state='Run.Execute', control=None):
    t.update_planning(state, at)
    t.update_diagnostics('planning/state_machine',0,state+': ',{'step':'base_goal','waiting_for':waiting},at)
    if control:t.update_control(True,False,True,True,'',at)
    t.update_motion(v,w,at)
    return t.notice(at)


def moving(t):
    for i in range(5):feed(t, i*.1,.1)


@pytest.mark.parametrize('waiting,key', [('', 'driving'),('path blocked; waiting before replanning','blocked'),('path still blocked; replanning','replan'),('waiting for target observation','target'),('waiting for fresh odom','sensors')])
def test_stop_after_movement_once_and_reason(waiting,key):
    t=StateTracker(style='cute');moving(t)
    for i in range(5,20):assert feed(t,i*.1,waiting=waiting) is None
    n=feed(t,2,waiting=waiting)
    assert n.key=='wheel_stop_'+key
    for i in range(21,40):assert feed(t,i*.1,waiting=waiting) is None


def test_initial_idle_brief_pause_turn_reverse_and_stale():
    t=StateTracker()
    for i in range(30):assert feed(t,i*.1) is None
    for v,w in [(-.1,0),(0,.2)]:
        t=StateTracker()
        for i in range(5):assert feed(t,i*.1,v,w) is None
        for i in range(5,10):assert feed(t,i*.1) is None
        assert feed(t,1,v,w) is None
        assert t.wheel_stop_label(1)==''
    t=StateTracker();moving(t)
    assert feed(t,.5) is None
    t.update_planning('Run.Execute',3)
    assert t.notice(3) is None
    assert t.wheel_stop_label(3)==''
    assert feed(t,3) is None  # input gap does not count as a stationary interval


def test_nan_noise_and_new_stop_epoch():
    t=StateTracker();moving(t)
    for i in range(5,21):n=feed(t,i*.1)
    label=t.wheel_stop_label(2)
    feed(t,2.1,.1)  # one noise spike does not rearm an entire movement
    for i in range(22,42):assert feed(t,i*.1) is None
    assert label!=t.wheel_stop_label(4.1)
    for i in range(42,47):feed(t,i*.1,.1)
    for i in range(47,63):n=feed(t,i*.1)
    assert n.key=='wheel_stop_driving' and n.episode==2
    feed(t,6.3,float('nan'));assert t.wheel_stop_label(6.3)==''


def test_fresh_control_hold_and_stale_reason_says_the_state():
    t=StateTracker();moving(t)
    for i in range(5,21):n=feed(t,i*.1,control=True)
    assert n.key=='wheel_stop_hold'
    t.control.at=-10;t.diag_at=-10
    assert t.wheel_stop_label(2).startswith('wheel_stop_executing:')   # the step is not known any more


def test_reason_never_uses_gpt_and_queued_notice_dropped_on_resume():
    t=StateTracker(style='cute');moving(t)
    for i in range(5,21):n=feed(t,i*.1)
    clock=Clock();clock.now=2
    backend=Backend();player=Player()
    r=Narrator(NarratorConfig(state_mode='gpt', notice=True, narrate_period_s=0),t,backend=backend,player=player,clock=clock,start=False)
    try:
        with r._lock:r._on_notice(n,2)
        t.update_motion(.1,0,2.1);clock.now=2.1
        r.speak_once()
        assert not backend.calls and not backend.spoken and not player.played
    finally:r.close()


def test_stop_notice_plays_fixed_reason_without_gpt():
    t=StateTracker(style='cute');moving(t)
    for i in range(5,21):n=feed(t,i*.1,waiting='path blocked; waiting before replanning')
    clock=Clock();clock.now=2;backend=Backend();player=Player()
    r=Narrator(NarratorConfig(state_mode='gpt', notice=True, narrate_period_s=0),t,backend=backend,player=player,clock=clock,start=False)
    try:
        with r._lock:r._on_notice(n,2)
        assert r.speak_once()
        assert not backend.calls
        assert backend.spoken==['길이 막혀서 잠깐 멈춰 있어.']
        assert player.played
    finally:r.close()


def test_odometry_header_stale_and_future_are_ignored():
    from types import SimpleNamespace as NS
    from craft_voice.voice_node import VoiceNode
    seen=[]
    node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000)),
            narrator=NS(motion=lambda *args:seen.append(args)))
    for sec in [0,8,11,10]:
        msg=NS(header=NS(stamp=NS(sec=sec,nanosec=0)),twist=NS(twist=NS(linear=NS(x=.1),angular=NS(z=0))))
        VoiceNode._on_odometry(node,msg)
    assert seen==[(.1,0)]


def test_slow_middle_band_then_full_stop_still_announces():
    t=StateTracker();moving(t)
    for i in range(5,10):assert feed(t,i*.1) is None
    assert feed(t,1,.004) is None
    for i in range(11,26):assert feed(t,i*.1) is None
    assert feed(t,2.7).key=='wheel_stop_driving'


def test_wheel_stop_without_planning_says_there_is_no_state_signal():
    t=StateTracker()
    for i in range(5):t.update_motion(.1,0,i*.1)
    for i in range(5,21):t.update_motion(0,0,i*.1)
    n=t.notice(2)
    assert n and n.key=='wheel_stop_no_signal'
    assert t.notice(2.1) is None


def test_reason_changed_while_queued_is_replaced():
    t=StateTracker(style='cute');moving(t)
    for i in range(5,21):n=feed(t,i*.1)
    clock=Clock();clock.now=2;backend=Backend();player=Player()
    r=Narrator(NarratorConfig(state_mode='gpt', notice=True, narrate_period_s=0),t,backend=backend,player=player,clock=clock,start=False)
    try:
        with r._lock:r._on_notice(n,2)
        t.update_control(True,False,True,True,'',2.1);clock.now=2.1
        r.speak_once()  # discard the old reason
        assert not backend.spoken
        r.tick();r.speak_once()
        assert backend.spoken==['일시정지 신호가 들어와서 기다리고 있어.']
        assert not backend.calls
        r.tick();assert not r._utterances
    finally:r.close()


@pytest.mark.parametrize('state,key,said', [
    ('Run.Execute', 'driving', '목표 지점으로 가는 중인데 지금은 멈춰 있어.'),
    ('Run.PrepareTask', 'preparing', '다음 작업을 준비하느라 멈춰 있어.'),
    ('Run.Complete', 'finished', '이번 단계를 끝내고 멈춰 있어.')])
def test_without_a_reason_the_state_is_said_not_a_guess(state, key, said):
    t=StateTracker(style='cute');moving(t)
    for i in range(5,20):feed(t,i*.1,state=state)
    n=feed(t,2,state=state)
    assert n.key=='wheel_stop_'+key and t.phrase('notice.'+n.key)==said
    assert '확인 중' not in said and '?' not in said


def drive(tracker, start, seconds, v, step=0.05):
    t = start
    while t < start + seconds:
        tracker.update_motion(v, 0.0, t)
        t += step
    return t


def test_a_departure_without_planning_says_a_start_line():
    from craft_voice.robot_state import StateTracker
    tracker = StateTracker(style='cute', variety=True)
    t = drive(tracker, 100.0, 4.0, 0.0)                    # standing still
    t = drive(tracker, t, 0.5, 0.1)                        # the wheels start
    noticed = tracker.notice(t)
    assert noticed.key == 'depart' and noticed.facts in ('출발!', '갑니다!', '시키면 해야죠.', '예, 알았다고요.', '라져!')
    assert tracker.notice(t + 0.1) is None
    t = drive(tracker, t, 1.0, 0.0)                        # a short stop: not a new departure
    t = drive(tracker, t, 0.5, 0.1)
    assert all(n is None or n.key != 'depart' for n in [tracker.notice(t)])


def test_no_departure_line_while_the_state_machine_announces():
    from craft_voice.robot_state import StateTracker
    tracker = StateTracker(style='cute', variety=True)
    t = 100.0
    while t < 104.5:
        tracker.update_planning('Run.Execute', t)
        tracker.update_motion(0.0 if t < 104.0 else 0.1, 0.0, t)
        t += 0.05
    noticed = tracker.notice(t)
    assert noticed is None or noticed.key != 'depart'
