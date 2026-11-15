"""미션 통합(11-16): mission_master -> puck_picker -> execute.py --planning. 팔 한 동작이 계획 단계 하나라서
주행 안내가 팔 동작마다 나오면 안 되고, 집기 로그의 그리퍼캠 줄과 집기 결과는 말해야 한다."""
import wave

from test_narrator import Backend, Clock, make, run


def clip(tmp_path, name):
    path = tmp_path / name
    with wave.open(str(path), 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(b'\0\0' * 240)
    return str(path)


def narrator_with_clips(tmp_path):
    clock = Clock()
    reactions = {'step_done': clip(tmp_path, 'jjasseu.wav'), 'run_done': clip(tmp_path, 'complete.wav'),
                 'gripcam_measured': clip(tmp_path, 'vector.wav')}
    n, texts = make(Backend(), clock=clock, notice=True, reactions=reactions, state_mode='template',
                    narrate_period_s=0.0)   # as on the robot (voice_robot.yaml)
    n.tracker.style = 'cute'
    return n, texts, clock


def step(n, clock, step_id='pick_step'):
    """One arm move through planning: request -> prepare -> compute -> execute -> complete, queue empty."""
    for label in ('Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute', 'Run.Execute'):
        n.planning_state(label); clock.now += 0.1
    n.diagnostics('planning/state_machine', 0, 'Run.Execute: ', {'step': step_id, 'queued': '0', 'task_id': '7'})
    clock.now += 1.5
    n.planning_state('Run.Complete')
    n.diagnostics('planning/state_machine', 0, 'Run.Complete: ', {'step': step_id, 'queued': '0', 'task_id': '7'})
    for _ in range(25):
        n.tick(); run(n); clock.now += 0.1


def test_arm_moves_in_a_mission_pick_say_no_drive_lines_and_play_no_step_recordings(tmp_path):
    n, texts, clock = narrator_with_clips(tmp_path)
    n.planning_state('Ready'); n.tick(); run(n)
    n.mission_state('Collect: approach_box S2'); n.mission_state('Collect: pick_puck S2 #1 -> slot 3')
    for _ in range(15):
        n.tick(); run(n); clock.now += 0.1
    texts.clear(); played = len(n.player.played)
    for _ in range(6):
        step(n, clock)
    said = [t for _, t in texts]
    assert not [t for t in said if '경로' in t or '끝났어' in t or '시작할게' in t], said
    assert len(n.player.played) == played                       # no "짜스" / "미션 완료" per arm move


def test_without_the_mission_a_known_pick_step_is_quiet_too(tmp_path):
    n, texts, clock = narrator_with_clips(tmp_path)
    n.planning_state('Ready'); n.tick(); run(n); texts.clear()
    for _ in range(3):
        for label in ('Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute'):
            n.planning_state(label); clock.now += 0.1
        n.diagnostics('planning/state_machine', 0, 'Run.Compute: ', {'step': 'pick_step', 'queued': '0'})
        clock.now += 0.2
        n.planning_state('Run.Execute'); clock.now += 1.0
        n.planning_state('Run.Complete')
        for _ in range(25):
            n.tick(); run(n); clock.now += 0.1
    assert not [t for _, t in texts if '경로' in t or '끝났어' in t], texts


def test_a_drive_still_gets_its_lines(tmp_path):
    n, texts, clock = narrator_with_clips(tmp_path)
    n.planning_state('Ready'); n.tick(); run(n); texts.clear()
    step(n, clock, 'base_goal')
    assert texts, 'a drive must still be announced'


def test_pick_log_gripper_cam_line_and_the_results(tmp_path):
    n, texts, clock = narrator_with_clips(tmp_path)
    n.pick_request('1794760998719 S2 3 find 2', str(tmp_path))
    log = tmp_path / 'pick_1794760998719.log'
    log.write_text('stand pick: wait pose\n   YAW +12.3 deg CENTRE +1.0 -2.0 221.0 mm (depth: raw +13.4, '
                   'footprint 27 x 28 mm, top 61 mm, 10/10 frames)\n')
    for _ in range(5):
        n.tick(); run(n); clock.now += 0.2
    assert ('thought', '벡터 계산 완료.') in texts
    assert '윗면 높이 61 mm, 각도 +12.3도' in n.tracker.memory_text(clock.now)
    n.pick_result('1794760998719 OK exit=0, cancelled=False, log=x')
    for _ in range(5):
        n.tick(); run(n); clock.now += 0.2
    assert texts[-1] == ('thought', '회전판 3번 칸에 실었어!')
    n.pick_request('1794761000000 S2 4 find 1', str(tmp_path))
    n.pick_result('1794761000000 FAILED exit=1, cancelled=False, log=x')
    for _ in range(5):
        n.tick(); run(n); clock.now += 0.2
    assert texts[-1] == ('thought', '앗, 퍽 집기 실패했어.')
    count = len(texts)
    n.pick_request('1794761000001 S3 5 find 2', str(tmp_path))
    n.pick_result('1794761000001 FAILED exit=-2, cancelled=True, log=x')   # STOP by the operator: silent
    n.pick_result('999 OK exit=0')                                         # someone else's token: ignored
    for _ in range(5):
        n.tick(); run(n); clock.now += 0.2
    assert len(texts) == count
