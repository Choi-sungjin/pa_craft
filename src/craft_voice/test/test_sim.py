import pytest

from craft_voice.robot_state import StateTracker
from craft_voice.sim import DEFAULT_SCENARIO, SimStateMachine, SimStep, parse_item, parse_scenario


def test_parse_items():
    assert parse_item('Ready:5') == SimStep('Ready', '', '', 5.0)
    assert parse_item(' Run.Execute@approach:8 ') == SimStep('Run.Execute', 'approach', '', 8.0)
    assert parse_item('Fault@approach!Run.Execute: execution timeout:12') == \
        SimStep('Fault', 'approach', 'Run.Execute: execution timeout', 12.0)
    assert parse_item('Fault!localization lost', with_duration=False) == SimStep('Fault', '', 'localization lost')
    for bad in ('Ready', 'Ready:0', 'Running:3', 'Ready:x'):
        with pytest.raises(ValueError):
            parse_item(bad)
    with pytest.raises(ValueError):
        parse_scenario(' , ')


def test_default_scenario_parses_and_ends_in_fault():
    steps = parse_scenario(DEFAULT_SCENARIO)
    assert steps[0].label == 'Idle' and steps[-1].label == 'Fault'
    assert steps[-1].reason == 'Run.Execute: execution timeout'


def test_timeline_loop_once_and_override():
    sim = SimStateMachine(parse_scenario('Ready:1, Run.Execute@a:2'), loop=True)
    assert [sim.at(t).label for t in (0.0, 0.99, 1.0, 2.9, 3.0, 4.5)] == \
        ['Ready', 'Ready', 'Run.Execute', 'Run.Execute', 'Ready', 'Run.Execute']
    once = SimStateMachine(parse_scenario('Ready:1, Run.Execute@a:2'), loop=False)
    assert once.at(100.0).label == 'Run.Execute'
    sim.set_override(SimStep('Fault', '', 'x'))
    assert sim.at(0.0).label == 'Fault'
    sim.set_override(None)
    assert sim.at(0.0).label == 'Ready'


def test_diagnostics_match_planning_master_and_feed_the_tracker():
    sim = SimStateMachine(parse_scenario('Run.PrepareTask@approach:1, Run.Execute@approach:1, '
                                         'Run.PrepareTask@grasp:1, Fault@grasp!Run.Execute: compute timeout:1'))
    tracker = StateTracker(fault_reason_wait_s=1.0)
    events = []
    for tenth in range(40):
        current = sim.at(tenth / 10)
        if tenth % 5 == 0:
            tracker.update_diagnostics(*sim.diagnostics(current), tenth / 10)
        event = tracker.update_planning(current.label, tenth / 10) or tracker.poll(tenth / 10)
        if event:
            events.append(tracker.template(event))
    name, level, message, values = sim.diagnostics(sim.at(3.5))
    assert (name, level, message) == ('planning/state_machine', 2, 'Fault: Run.Execute: compute timeout')
    assert values['task_id'] == '2' and values['step'] == 'grasp'
    assert events[-1] == '문제가 생겨서 멈췄어요. 원인: 계산 시간 초과.'
