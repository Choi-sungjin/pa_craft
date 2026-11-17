"""Recorded planner execution substates must not create new task-start speech."""
import threading

import pytest

from craft_voice.robot_state import StateTracker
from test_narrator import Backend, Player, make, run


@pytest.mark.parametrize('substates', [
    ('Run.Execute.Blocked',),
    ('Run.Execute.Replan',),
    ('Run.Execute.Blocked', 'Run.Execute.Replan'),
])
def test_execution_retries_preserve_task_identity_and_do_not_announce_departure(substates):
    t = StateTracker(step_name_wait_s=0)
    t.update_planning('Ready', 100)
    t.update_planning('Run.PrepareTask', 101)
    t.update_planning('Run.Compute', 102)
    assert t.update_planning('Run.Execute', 103)
    t.step_counts['replan'] = 2
    t._noticed.add('many_replans')
    start = t.step_started_at
    now = 120
    for _ in range(4):
        for state in substates:
            t.update_planning(state, now)
            now += 1
        assert t.update_planning('Run.Execute', now) is None
        assert t.step_started_at == start
        assert t.step_counts['replan'] == 2 and 'many_replans' in t._noticed
        now += 20  # much longer than cooldown: this must not be a timer-only suppression


def test_actual_next_task_still_announces_and_resets_task_memory():
    t = StateTracker(step_name_wait_s=0)
    t.update_planning('Ready', 100)
    t.update_planning('Run.PrepareTask', 101)
    t.update_planning('Run.Execute', 102)
    t.update_planning('Run.Execute.Blocked', 120)
    t.step_counts['replan'] = 3
    t._noticed.add('many_replans')
    t.update_planning('Run.Complete', 121)
    t.update_planning('Run.PrepareTask', 122)
    assert t.step_started_at == 122 and not t.step_counts and 'many_replans' not in t._noticed
    assert t.update_planning('Run.Execute', 123).label == 'Run.Execute'


def test_retry_still_has_its_existing_situation_reactions():
    t = StateTracker(step_name_wait_s=0)
    t.update_planning('Ready', 100)
    t.update_planning('Run.PrepareTask', 101)
    t.update_planning('Run.Execute', 102)
    t.update_planning('Run.Execute.Blocked', 120)
    blocked = t.update_diagnostics('planning/state_machine', 0, 'Run.Execute.Blocked',
        {'step': 'base_goal', 'waiting_for': 'path blocked; waiting before replanning'}, 120)
    t.update_planning('Run.Execute.Replan', 122)
    replan = t.update_diagnostics('planning/state_machine', 0, 'Run.Execute.Replan',
        {'step': 'base_goal', 'waiting_for': 'path still blocked; replanning'}, 122)
    assert blocked.situation == 'blocked' and replan.situation == 'replan'
    assert t.step_counts['blocked'] == t.step_counts['replan'] == 1


def test_narrator_does_not_queue_repeated_start_sentences():
    n, texts = make(backend=Backend(), state_mode='template', narrate_period_s=0)
    n.planning_state('Ready'); run(n); texts.clear()
    n.planning_state('Run.PrepareTask')
    n.planning_state('Run.Execute'); run(n)
    first = list(texts)
    assert len(first) == 1
    for _ in range(5):
        n.clock.now += 20
        n.planning_state('Run.Execute.Blocked')
        n.clock.now += 2
        n.planning_state('Run.Execute.Replan')
        n.clock.now += .2
        n.planning_state('Run.Execute'); run(n)
    assert texts == first
    n.clock.now += 20
    n.planning_state('Run.Complete'); run(n)
    n.planning_state('Run.PrepareTask')
    n.planning_state('Run.Execute'); run(n)
    assert texts[-1] == first[0]


@pytest.mark.parametrize('waiting', [
    'path blocked; waiting before replanning',
    'path still blocked; replanning',
])
def test_fault_diagnosis_clears_waiting_without_discarding_the_fault(waiting):
    t = StateTracker(fault_reason_wait_s=1.2)
    t.update_planning('Run.Execute', 100)
    t.update_diagnostics('planning/state_machine', 0, 'Run.Execute',
        {'step': 'base_goal', 'waiting_for': waiting}, 101)
    assert t.situation in ('blocked', 'replan')
    assert t.update_planning('Fault', 102) is None
    generation = t.generation
    t.update_diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: execution timeout',
        {'step': 'base_goal', 'waiting_for': waiting}, 102.05)
    assert not t.situation and not t.situation_text
    assert t.generation == generation
    assert all(text != '길이 다시 열림' for _, text in t.journal)
    event = t.poll(102.1)
    assert event and event.fault and event.reason == 'Run.Execute: execution timeout'
    assert t.template(event) == '목표까지 너무 오래 걸려서 멈춰 있어요.'
    assert t.poll(102.2) is None
    t.update_diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: execution timeout',
        {'step': 'base_goal', 'waiting_for': ''}, 103)
    assert t.poll(103) is None


def test_fault_after_blocked_is_delivered_to_narrator_once():
    n, texts = make(backend=Backend(), state_mode='template', narrate_period_s=0)
    n.planning_state('Run.Execute'); run(n)
    n.diagnostics('planning/state_machine', 0, 'Run.Execute',
        {'step': 'base_goal', 'waiting_for': 'path blocked; waiting before replanning'})
    run(n); texts.clear()
    n.clock.now += 2
    n.planning_state('Fault')
    n.clock.now += .05
    n.diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: execution timeout',
        {'step': 'base_goal', 'waiting_for': ''})
    n.tick(); run(n)
    assert len(texts) == 1 and texts[0][1] == '목표까지 너무 오래 걸려서 멈춰 있어요.'
    n.clock.now += 2
    n.tick(); run(n)
    assert len(texts) == 1


def queue_timeout_fault(n):
    n.planning_state('Fault')
    n.clock.now += .05
    n.diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: execution timeout', {})
    n.tick()
    assert any(u.kind == 'fault' for u in n._utterances)


@pytest.mark.parametrize('recovered_state', ['Idle', 'Ready', 'Run.PrepareTask'])
def test_queued_fault_is_not_spoken_after_recovery(recovered_state):
    n, texts = make(backend=Backend(), state_mode='template', narrate_period_s=0)
    queue_timeout_fault(n)
    n.planning_state(recovered_state)
    run(n)
    assert all(kind != 'fault' for kind, _ in texts)


def test_previous_fault_is_not_spoken_during_a_different_fault():
    n, texts = make(backend=Backend(), state_mode='template', narrate_period_s=0)
    queue_timeout_fault(n)
    n.planning_state('Ready')
    n.clock.now += 2
    n.planning_state('Fault')
    run(n)  # the new fault is waiting for its reason; the old queued reason must not play
    assert not texts
    n.clock.now += .05
    n.diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: localization lost', {})
    n.tick(); run(n)
    assert texts == [('fault', '제 위치를 놓쳐서 멈춰 있어요.')]


def test_recovery_during_fault_synthesis_prevents_obsolete_playback():
    backend = Backend(urgent_delay=.2)
    n, texts = make(backend=backend, state_mode='template', narrate_period_s=0)
    queue_timeout_fault(n)
    worker = threading.Thread(target=n.speak_once)
    worker.start()
    try:
        assert backend.tts_started.wait(2)
        n.planning_state('Ready')
        worker.join(2)
        assert not worker.is_alive() and not texts and not n.player.played
    finally:
        n.close(); worker.join(2)


def test_recovery_at_fault_device_start_prevents_obsolete_playback():
    n, _ = make(backend=Backend(), state_mode='template', narrate_period_s=0)
    queue_timeout_fault(n)
    n.on_text = lambda *_: n.planning_state('Ready')
    n.speak_once()
    assert not n.player.played and n.stats['spoken'] == 0


def test_recovery_does_not_cut_fault_audio_already_playing():
    player = Player(block=True)
    n, texts = make(backend=Backend(), player=player, state_mode='template', narrate_period_s=0)
    queue_timeout_fault(n)
    worker = threading.Thread(target=n.speak_once)
    worker.start()
    try:
        assert player.started.wait(2)
        n.planning_state('Ready')
        assert worker.is_alive() and player.stops == 0 and not n._current.cancelled
        player.finish_current(); worker.join(2)
        assert not worker.is_alive() and n.stats['spoken'] == 1
        player.block = False
        run(n)
        assert [kind for kind, _ in texts] == ['fault', 'state'] and player.stops == 0
    finally:
        n.close(); worker.join(2)
