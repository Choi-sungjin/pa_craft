from craft_voice.mission import DICE_SCENARIO, MISSION_CONTEXT, STEP_NAMES
from craft_voice.narrator import Job, Narrator, NarratorConfig
from craft_voice.robot_state import StateTracker, name_step, parse_mapping
from craft_voice.sim import SimStateMachine, parse_scenario


def test_step_name_patterns():
    names = parse_mapping(STEP_NAMES)
    assert name_step('load_S3', names) == '시작 박스 3 퍽 싣기'
    assert name_step('place_L2', names) == 'L2 주사위 눈 놓기'
    assert name_step('verify_L3', names) == 'L3 다시 확인'
    assert name_step('observe_signal', names) == '신호등 관측'
    assert name_step('load_S', names) == '' and name_step('unknown_step', names) == ''


def test_dice_mission_is_narrated_in_order():
    tracker = StateTracker(step_names=parse_mapping(STEP_NAMES), cooldown_s=0.0)
    sim = SimStateMachine(parse_scenario(DICE_SCENARIO), loop=False)
    said, t = [], 0.0
    while t < sim.total:
        current = sim.at(t)
        event = tracker.update_planning(current.label, t) or tracker.poll(t)   # state 20 Hz, poll like tick()
        if round(t * 20) % 20 == 0:                           # diagnostics at 1 Hz, like planning_master
            tracker.update_diagnostics(*sim.diagnostics(current), t)
        if event and event.label == 'Run.Execute':
            said.append(tracker.template(event))
        t += 0.05
    # The 1 Hz step name is waited for, so every step is announced by name, in mission order.
    assert said == ['바코드 읽기, 시작할게요.', '신호등 관측, 시작할게요.', '횟수 계산과 정렬, 시작할게요.',
                    '시작 박스 1 퍽 싣기, 시작할게요.', '시작 박스 2 퍽 싣기, 시작할게요.', '장소별 퍽 배정, 시작할게요.',
                    'L1 주사위 눈 놓기, 시작할게요.', 'L2 주사위 눈 놓기, 시작할게요.', 'L1 다시 확인, 시작할게요.',
                    '출발 구역으로 복귀, 시작할게요.']
    tracker.update_planning('Run.PrepareTask', t + 1.0)
    tracker.update_diagnostics('planning/state_machine', 0, 'Run.PrepareTask: ', {'step': 'place_L3'}, t + 1.1)
    tracker.update_planning('Run.Execute', t + 1.2)
    assert tracker.doing(t + 1.3) == 'L3 주사위 눈 놓기 중이에요.'


def test_mission_context_reaches_gpt_as_background_only():
    narrator = Narrator(NarratorConfig(mission_context=MISSION_CONTEXT), StateTracker(), start=False)
    instructions, prompt = narrator._prompt(Job('scene', 0.0), has_image=False)
    assert '[미션] 주사위 기억 미션' in instructions and '배경 지식일 뿐' in instructions
    assert '[미션]' not in prompt                                # facts stay in the prompt, mission is not a fact
    plain, _ = Narrator(NarratorConfig(), StateTracker(), start=False)._prompt(Job('scene', 0.0), False)
    assert '[미션]' not in plain


def test_step_start_waits_for_its_name_then_gives_up():
    tracker = StateTracker(step_names=parse_mapping(STEP_NAMES), step_name_wait_s=1.2)
    tracker.update_planning('Ready', 0.0)
    assert tracker.update_planning('Run.PrepareTask', 1.0) is None
    assert tracker.update_planning('Run.Execute', 1.3) is None           # name not in yet: wait
    assert tracker.poll(1.8) is None
    tracker.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', {'step': 'observe_signal'}, 1.9)
    event = tracker.poll(1.95)
    assert event and tracker.template(event) == '신호등 관측, 시작할게요.'
    assert tracker.poll(2.0) is None                                     # said once
    tracker.update_planning('Run.Complete', 3.0)
    tracker.update_planning('Run.AwaitRequest', 3.05)
    tracker.update_planning('Run.PrepareTask', 3.1)
    tracker.cooldown_s = 0.0
    assert tracker.update_planning('Run.Execute', 3.3) is None
    assert tracker.poll(4.0) is None and tracker.template(tracker.poll(4.6)) == '작업을 시작할게요.'  # gave up
    tracker.update_planning('Run.Complete', 5.0)
    tracker.update_planning('Run.PrepareTask', 5.1)
    tracker.update_planning('Run.Execute', 5.2)
    tracker.update_planning('Run.Complete', 5.4)                          # moved on before the name came
    assert tracker.poll(7.0) is None
