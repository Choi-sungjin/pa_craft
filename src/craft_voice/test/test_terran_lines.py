"""스타1 리마스터 한국어판 원본 대사 네 개: 퍽 축(벡터 계산 완료), 회전판에서 퍽 꺼내기(드랍쉽),
플래닝 하트비트 끊김(메딕), 펌웨어 업로드 끝(어드바이저)."""

import wave

from craft_voice.robot_state import StateTracker
from craft_voice.signals import CLOCK, EXECUTE, GRIPPER, MANIPULATOR, UPDATE_PATH, NAVIGATE
from test_narrator import make, run
from test_signals import keys

AXIS, NO_AXIS = [(True, 0.42)], [(None, None)]


def command(tracker, kind, now, close=False, slot=0, operation=EXECUTE):
    tracker.update_task_command(1, operation, kind, 0.0, slot, close, now)


def test_a_puck_axis_found_anew_is_said_once():
    tracker = StateTracker()                                   # no planning: perception alone says it
    tracker.update_pucks(NO_AXIS, 100.0)
    assert keys(tracker, 100.1) == []
    tracker.update_pucks(AXIS, 101.0)
    assert keys(tracker, 101.1) == ['puck_axis'] and tracker.phrase('notice.puck_axis') == '벡터 계산 완료.'
    for t in (101.2, 102.0, 103.0):                            # the same puck, every frame: once
        tracker.update_pucks(AXIS, t)
    assert keys(tracker, 103.1) == []
    tracker.update_pucks(AXIS, 106.0)                          # lost for 3 s only: the same look
    tracker.update_pucks(AXIS, 120.0)                          # found again after 14 s, but within a minute
    assert keys(tracker, 120.1) == []
    tracker.update_pucks(AXIS, 200.0)
    assert keys(tracker, 200.1) == ['puck_axis']


def test_taking_a_stored_puck_out_of_the_clock_is_said_at_the_grasp():
    tracker = StateTracker()
    # dice: clock to the guide with an empty gripper, arm into the slot, close on the stored puck
    command(tracker, CLOCK, 100.0, slot=12)
    command(tracker, MANIPULATOR, 101.0)
    assert keys(tracker, 101.1) == []
    command(tracker, GRIPPER, 102.0, close=True)
    assert keys(tracker, 102.1) == ['clock_unload']
    assert tracker.phrase('notice.clock_unload') == '머리 위 선반에서 수화물을 꺼낼 때는 조심해 주십시오.'
    command(tracker, MANIPULATOR, 103.0)
    command(tracker, GRIPPER, 110.0)                           # released on the box
    command(tracker, CLOCK, 112.0, slot=1)                     # the next stored puck
    command(tracker, GRIPPER, 115.0, close=True)
    assert keys(tracker, 115.1) == ['clock_unload']
    command(tracker, GRIPPER, 116.0, close=True)               # a second close: not again
    assert keys(tracker, 116.1) == []


def test_loading_the_clock_is_not_taking_out():
    tracker = StateTracker()
    command(tracker, GRIPPER, 100.0, close=True)               # the puck from the floor
    command(tracker, CLOCK, 105.0, slot=12)                    # turned while holding it
    command(tracker, GRIPPER, 108.0)                           # dropped into the slot
    command(tracker, NAVIGATE, 110.0)
    command(tracker, NAVIGATE, 111.0, operation=UPDATE_PATH)
    command(tracker, GRIPPER, 130.0, close=True)               # the next puck from the floor
    assert keys(tracker, 130.1) == []


def test_a_clock_turn_long_ago_does_not_count():
    tracker = StateTracker()
    command(tracker, CLOCK, 100.0, slot=12)
    command(tracker, GRIPPER, 400.0, close=True)
    assert keys(tracker, 400.1) == []


def test_a_stopped_heartbeat_is_said_once_per_outage():
    tracker = StateTracker()
    tracker.update_planning('Run.Execute', 100.0)
    for i in range(20):
        tracker.update_heartbeat(100.0 + i * 0.05)
    assert keys(tracker, 101.0) == []                           # 0.05 s since the last one
    tracker.update_planning('Run.Execute', 101.5)
    assert tracker.notice(102.0).key == 'heartbeat_lost'        # 1.05 s without it
    assert tracker.phrase('notice.heartbeat_lost') == '심장박동이 멎었습니다.'
    assert [tracker.notice(t) for t in (103.0, 110.0, 120.0)] == [None, None, None]   # no "상태 신호가 끊겼어" too
    tracker.update_planning('Run.Execute', 130.0)               # planning is back
    tracker.update_heartbeat(130.0)
    assert keys(tracker, 130.1) == ['state_back']
    tracker.update_heartbeat(130.5)
    assert keys(tracker, 132.0) == ['heartbeat_lost']           # a new outage


def test_no_heartbeat_ever_is_not_a_lost_one():
    tracker = StateTracker()
    tracker.update_planning('Idle', 100.0)
    assert keys(tracker, 110.0) == []


def test_the_firmware_upload_event_is_said():
    tracker = StateTracker()
    tracker.update_event('firmware_done', 100.0)
    assert keys(tracker, 100.1) == ['firmware_done'] and tracker.phrase('notice.firmware_done') == '업그레이드 완료.'
    tracker.update_event('firmware_done', 103.0)                # resetall: the second board right after
    tracker.update_event('something_else', 104.0)
    assert keys(tracker, 104.1) == []


def test_the_recording_is_played_instead_of_the_sentence(tmp_path):
    clips = {}
    for key in ('puck_axis', 'clock_unload', 'heartbeat_lost', 'firmware_done'):
        clips[key] = str(tmp_path / f'{key}.wav')
        with wave.open(clips[key], 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(bytes([len(clips)]) * 4800)
    narrator, texts = make(reactions=clips, notice=True)
    narrator.event('firmware_done')
    narrator.tick()
    run(narrator)
    assert narrator.player.played == [open(clips['firmware_done'], 'rb').read()]
    assert texts == [('thought', '업그레이드 완료.')]
    narrator.pucks([(True, 0.4)])
    narrator.tick()
    run(narrator)
    assert narrator.player.played[-1] == open(clips['puck_axis'], 'rb').read()


BLOCKED = 'path blocked; waiting before replanning'
REPLAN = 'path still blocked; replanning'


def drive(tracker, start, end, pattern, step=0.5):
    """planning keeps publishing Run.Execute; diagnostics follow pattern(t) -> waiting_for. Returns the notices."""
    found, t = [], start
    while t < end:
        tracker.update_planning('Run.Execute', t)
        tracker.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                                   {'step': 'base_goal', 'task_id': '7', 'queued': '0', 'waiting_for': pattern(t)}, t)
        while (noticed := tracker.notice(t)) is not None:
            found.append((round(t, 1), noticed.key, noticed.facts))
        t += step
    return found


def back_and_forth(t):  # blocked 2 s, replanning 1 s, a new path that blocks again after 1 s
    return [BLOCKED, BLOCKED, BLOCKED, BLOCKED, REPLAN, REPLAN, '', ''][int(t * 2) % 8]


def test_trapped_for_30_seconds_says_one_of_the_scv_lines_once():
    tracker = StateTracker()
    found = drive(tracker, 100.0, 160.0, back_and_forth)
    trapped = [f for f in found if f[1] == 'trapped']
    assert len(trapped) == 1 and 130.0 <= trapped[0][0] < 131.0
    assert trapped[0][2] in ('난 폐쇄 공포증이 있다니까 여기서 나가야 된다구.', '여기 쥐똥만한 공간에 갇혀서 아주 꼼짝도 못하겠군.')
    found = drive(tracker, 160.0, 166.0, lambda t: '')             # free for 6 s: the stretch is over
    found += drive(tracker, 166.0, 200.0, back_and_forth)           # trapped again
    assert [f[1] for f in found].count('trapped') == 1


def test_blocked_without_replanning_or_shorter_is_not_trapped():
    tracker = StateTracker()
    assert 'trapped' not in [f[1] for f in drive(tracker, 100.0, 140.0, lambda t: BLOCKED)]
    tracker = StateTracker()
    found = drive(tracker, 100.0, 120.0, back_and_forth) + drive(tracker, 120.0, 125.0, lambda t: '') + \
        drive(tracker, 125.0, 145.0, back_and_forth)                # 20 s, free 5 s, 20 s
    assert 'trapped' not in [f[1] for f in found]


def test_the_recording_of_the_chosen_line_is_played(tmp_path, monkeypatch):
    clips = []
    for i in range(2):
        clips.append(str(tmp_path / f'scv{i}.wav'))
        with wave.open(clips[-1], 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(bytes([i + 1]) * 4800)
    narrator, texts = make(reactions={'trapped': '|'.join(clips)}, notice=True)
    monkeypatch.setattr('craft_voice.robot_state.random.randrange', lambda n: 1)
    t = 0.0
    while t < 35.0:
        narrator.clock.now = t
        narrator.planning_state('Run.Execute')
        narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                             {'step': 'base_goal', 'task_id': '7', 'queued': '0', 'waiting_for': back_and_forth(t)})
        narrator.tick()
        run(narrator)
        t += 0.5
    assert ('thought', '여기 쥐똥만한 공간에 갇혀서 아주 꼼짝도 못하겠군.') in texts
    assert open(clips[1], 'rb').read() in narrator.player.played and open(clips[0], 'rb').read() not in narrator.player.played


def test_a_reaction_may_list_several_recordings(tmp_path):
    from craft_voice.wiring import load_reactions
    good = tmp_path / 'a.wav'
    with wave.open(str(good), 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(b'\x00\x01' * 100)
    class Log:
        def warning(self, text): pass
    assert load_reactions(f'trapped={good}|{tmp_path}/missing.wav, x={tmp_path}/none.wav', Log()) == {'trapped': str(good)}


def depart(tracker, box, now):
    """mission_master sends the box, planning computes and starts driving. Returns (event, notices)."""
    if box:
        tracker.update_approach(box, now)
    tracker.update_planning('Run.Compute', now + 0.5)
    event = tracker.update_planning('Run.Execute', now + 2.0)
    notices = []
    while (noticed := tracker.notice(now + 2.1)) is not None:
        notices.append(noticed)
    tracker.update_planning('Run.Complete', now + 20.0)
    return event, notices


def test_each_tank_line_once_for_the_five_trips():
    from craft_voice.robot_state import TANK_LINES
    tracker = StateTracker(step_name_wait_s=0.0)
    tracker.update_planning('Ready', 90.0)
    said = []
    assert len(TANK_LINES) == 5 and '기꺼이.' not in TANK_LINES
    for i, box in enumerate(['S2', 'S3', 'S4', 'S5', 'S6']):
        event, notices = depart(tracker, box, 100.0 + i * 100)
        assert event is None and [n.key for n in notices] == ['approach_line']   # instead of "출발" line
        said.append(notices[0].facts)
        assert dict(notices[0].values)['choice'] == TANK_LINES.index(notices[0].facts)
    assert sorted(said) == sorted(TANK_LINES)                                    # each once
    event, notices = depart(tracker, 'S2', 800.0)                                # a next round starts over
    assert [n.key for n in notices] == ['approach_line']


def test_other_boxes_and_retries_keep_the_usual_departure():
    tracker = StateTracker(step_name_wait_s=0.0)
    tracker.update_planning('Ready', 90.0)
    for i, box in enumerate(['S1', 'L1', 'L2', 'L3']):
        event, notices = depart(tracker, box, 20.0 + i * 20)
        assert event is not None and event.label == 'Run.Execute' and notices == []
    event, notices = depart(tracker, 'S3', 200.0)
    assert event is None and [n.key for n in notices] == ['approach_line']
    event, notices = depart(tracker, 'S3', 230.0)                                # the same box again: a retry
    assert event is not None and notices == []
    event, notices = depart(tracker, '', 300.0)                                  # a drive without a box
    assert event is not None and notices == []


def test_the_tank_recording_matches_the_line(tmp_path):
    clips = []
    for i in range(5):
        clips.append(str(tmp_path / f'tank{i}.wav'))
        with wave.open(clips[-1], 'wb') as w:
            w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(bytes([i + 1]) * 4800)
    from craft_voice.robot_state import TANK_LINES
    narrator, texts = make(reactions={'approach_line': '|'.join(clips)}, notice=True)
    narrator.planning_state('Ready')
    for i, box in enumerate(['S2', 'S3', 'S4']):
        narrator.clock.now = 100.0 + i * 50
        narrator.approach_box(box)
        narrator.planning_state('Run.Compute')
        narrator.clock.now += 2.0
        narrator.planning_state('Run.Execute')
        narrator.tick()
        run(narrator)
        narrator.planning_state('Run.Complete')
        run(narrator)
    lines = [text for kind, text in texts if kind == 'thought']
    assert len(lines) == 3 and len(set(lines)) == 3
    for line in lines:
        assert open(clips[TANK_LINES.index(line)], 'rb').read() in narrator.player.played


# /planning/state as recorded on 2026-11-15 01:09 (mission_1115_0109): the execute phase names what it does.
RECORDED_BOX_TRIP = ['Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute', 'Run.Execute.Box.Stage',
                     'Run.Execute.Box.CameraDock', 'Run.Execute.Box.FinalStop', 'Run.Execute.Box.CameraVerify',
                     'Run.Complete']
RECORDED_LEAVE = ['Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute', 'Run.Execute.Leave', 'Run.Complete']


def recorded_trip(tracker, box, now):
    """approach_box, then the recorded labels 0.1 s apart, then leave_box. Returns the notice keys said."""
    tracker.update_approach(box, now)
    said = []
    for i, label in enumerate(RECORDED_BOX_TRIP):
        tracker.update_planning(label, now + 0.01 + i * 0.1)
        while (noticed := tracker.notice(now + 0.02 + i * 0.1)) is not None:
            said.append(noticed)
    for i, label in enumerate(RECORDED_LEAVE):
        tracker.update_planning(label, now + 10 + i * 0.1)
        while (noticed := tracker.notice(now + 10.01 + i * 0.1)) is not None:
            said.append(noticed)
    return said


def test_the_recorded_mission_says_a_tank_line_on_each_trip_s2_to_s6():
    from craft_voice.robot_state import TANK_LINES
    tracker = StateTracker(step_name_wait_s=0.0)
    tracker.update_planning('Ready', 90.0)
    lines = {}
    for i, box in enumerate(['S1', 'S2', 'S3', 'S4', 'S5', 'S6', 'L1', 'L2', 'L3']):
        said = [n for n in recorded_trip(tracker, box, 100.0 + i * 40) if n.key == 'approach_line']
        lines[box] = [n.facts for n in said]
    assert all(len(lines[b]) == 1 for b in ('S2', 'S3', 'S4', 'S5', 'S6')), lines
    assert sorted(sum((lines[b] for b in ('S2', 'S3', 'S4', 'S5', 'S6')), [])) == sorted(TANK_LINES)
    assert lines['S1'] == lines['L1'] == lines['L2'] == lines['L3'] == []


def test_execute_detail_labels_map_to_the_phases_the_voice_knows():
    from craft_voice.robot_state import planning_phase
    assert planning_phase('Run.Execute.Box.CameraDock') == 'Run.Execute'
    assert planning_phase('Run.Execute.Leave') == 'Run.Execute'
    assert planning_phase('Run.Execute.Box.Stage.Replan') == 'Run.Execute.Replan'
    assert planning_phase('Run.Execute.Box.CameraDock.DockBlocked') == 'Run.Execute.Blocked'
    assert planning_phase('Run.Execute.NoRoute') == 'Run.Execute.Replan'
    assert planning_phase('Run.Execute.Blocked') == 'Run.Execute.Blocked'
    assert planning_phase('Run.Compute') == 'Run.Compute'


def test_box_phases_are_one_drive_and_the_detail_reaches_the_facts():
    tracker = StateTracker(step_name_wait_s=0.0)
    tracker.update_planning('Ready', 90.0)
    tracker.update_planning('Run.Compute', 100.0)
    generation = tracker.generation
    assert tracker.update_planning('Run.Execute.Box.Stage', 100.1) is None   # the mission flow line says it
    assert tracker.generation == generation + 1 and tracker.step_started_at == 100.0   # began at Compute
    generation = tracker.generation
    for t, label in ((110, 'Run.Execute.Box.CameraDock'), (118, 'Run.Execute.Box.FinalStop')):
        assert tracker.update_planning(label, t) is None
    assert tracker.generation == generation and tracker.label == 'Run.Execute'
    assert 'Run.Execute.Box.FinalStop' in tracker.context_text(118.5)


def test_a_plain_drive_keeps_its_start_line():
    tracker = StateTracker(step_name_wait_s=0.0)
    tracker.update_planning('Ready', 90.0)
    tracker.update_planning('Run.Compute', 100.0)
    event = tracker.update_planning('Run.Execute', 100.1)
    assert event is not None and event.label == 'Run.Execute'
