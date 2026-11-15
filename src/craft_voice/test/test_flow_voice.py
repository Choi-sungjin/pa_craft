"""Recorded mission replay plus missing/stale/changed evidence. Never publishes ROS commands."""
import json
import math
from pathlib import Path

import pytest

from craft_voice.flow import BoxEvidence, MissionProgress, mission_words
from craft_voice.signals import Signals
from craft_voice.narrator import Job, Utterance
from test_narrator import Backend, make, run

VALID_REASON = 'lidar face fit, both face ends seen (width 235.0 mm)'

QUALITY = dict(request_id=7, face_centre_observed=True, extrapolated=False, age_s=0,
               gap_uncertainty_m=.01, face_centre_uncertainty_m=.01)


def observations():
    return json.loads((Path(__file__).parent / 'fixtures/mission_1114_voice.json').read_text())['events']


def replay():
    signals = Signals()
    notices = []
    for row in observations():
        topic, d, now = row['topic'], row['data'], row['t']
        key = ''
        if topic.endswith('/traffic_light'):
            seq = d['sequence']
            key = signals.traffic_light(d['red'], d['yellow'], d['green'], len(seq),
                d['complete'], d['valid'], d['discarded_cycles'], now, sequence=seq)
        elif topic.endswith('/box_observation_request'):
            signals.box_request(d['request_id'], d['source'], now, d['box_id'], box_side=d.get('box_side', math.nan))
        elif topic.endswith('/box/observation'):
            stamp = d['header']['stamp']
            quality = {k: d[k] for k in QUALITY if k != 'age_s'}
            quality['age_s'] = now - (stamp['sec'] + stamp['nanosec'] / 1e9)
            key = signals.box_observation(d['valid'], d['source'], d['reason'], d['gap_m'],
                                         d['lateral_m'], d['yaw_rad'], now, **quality)
            if key:
                assert d['valid'] and d['face_centre_observed'] and not d['extrapolated']
        if key:
            notices.append((row['bag'], row['elapsed'], key))
    return signals, notices


def test_recorded_recounts_do_not_restart_narration_and_partial_faces_are_not_found():
    signals, notices = replay()
    first = [k for bag, _, k in notices if bag == 'mission_1114_0222']
    assert first == ['traffic_recount', 'traffic_done']  # bag starts at blink 2, not a new start
    second = [(t, k) for bag, t, k in notices if bag == 'mission_1114_0225']
    assert [k for _, k in second if k.startswith('traffic_')] == [
        'traffic_start', 'traffic_start', 'traffic_recount', 'traffic_done']
    # The two starts surround an actual mission reset, not discarded count / retry.
    found = [t for t, k in second if k == 'box_found']
    assert found and min(found) > 138.295  # raw valid began at 120.895, both ends first seen at 138.295
    assert len(found) == 1  # remeasuring the same box does not repeat the announcement
    assert signals.box is None  # request 'off' clears old evidence


@pytest.mark.parametrize('bad', [
    {'face_centre_observed': False}, {'request_id': 999}, {'extrapolated': True},
    {'age_s': 2}, {'age_s': -0.01}, {'age_s': math.nan}, {'gap_uncertainty_m': math.nan},
    {'face_centre_uncertainty_m': -1},
])
def test_bad_box_evidence_never_announces(bad):
    b = BoxEvidence(); b.request(7, 'lidar', 90, box_side=.205)
    quality = dict(QUALITY, **bad)
    for t in (100, 100.1, 100.2, 101, 102):
        assert b.observe(True, 'lidar', (.4, 0, 0), t, reason=VALID_REASON, **quality) == (False, False)


def test_box_requires_active_request_new_scans_and_finite_geometry():
    b = BoxEvidence()
    for t in (100, 101, 102):
        assert b.observe(True, 'lidar', (.4, 0, 0), t, reason=VALID_REASON, **QUALITY) == (False, False)
    b.request(7, 'lidar', 100, box_side=.205)
    for _ in range(20):
        assert b.observe(True, 'lidar', (.4, 0, 0), 100, reason=VALID_REASON, **QUALITY) == (False, False)
    assert b.observe(True, 'lidar', (.4, math.nan, 0), 100.1, reason=VALID_REASON, **QUALITY) == (False, False)
    for t in (101, 101.1):
        assert b.observe(True, 'lidar', (.4, 0, 0), t, reason=VALID_REASON, **QUALITY) == (False, False)
    assert b.observe(True, 'lidar', (.4, 0, 0), 101.2, reason=VALID_REASON, **QUALITY) == (True, True)
    assert b.observe(True, 'lidar', (.4, 0, 0), 101.3, reason=VALID_REASON, **QUALITY) == (True, False)
    b.request(8, 'off', 101.4)
    assert b.observe(True, 'lidar', (.4, 0, 0), 101.5, reason=VALID_REASON, **QUALITY) == (False, False)


def test_invalid_observation_cancels_queued_box_notice():
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.planning_state('Run.Execute'); n._jobs.clear(); n._utterances.clear()
    n.box_request(7, 'lidar', box_side=.205)
    for t in (100.1, 100.2, 100.3):
        n.clock.now = t; n.box(True, 'lidar', VALID_REASON, .4, 0, 0, **QUALITY)
    n.tick()
    assert any(u.label.startswith('box_notice:') for u in n._utterances)
    n.box(False, 'lidar', 'lost', 0, 0, 0, **QUALITY)
    run(n)
    assert not texts


def test_delayed_box_data_keeps_acquisition_age_before_audio():
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.planning_state('Run.Execute'); n._jobs.clear(); n._utterances.clear()
    n.box_request(7, 'lidar', box_side=.205)
    for t in (101, 101.1, 101.2):
        n.clock.now = t
        n.box(True, 'lidar', VALID_REASON, .4, 0, 0, **dict(QUALITY, age_s=.9))
    n.tick()
    assert n._utterances
    n.clock.now = 102.1
    run(n)
    assert not texts


@pytest.mark.parametrize('command', ['stop', 'mute'])
def test_stop_clears_not_yet_polled_perception_notices(command):
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.barcode('P2-01')
    n.traffic_light(1, 0, 0, 1, False, False, 0, sequence=[0])
    n.command(command); n.command('unmute'); n.tick(); run(n)
    assert not texts


def test_mute_cancels_current_perception_utterance():
    n, _ = make()
    n._current = Utterance('thought', '바코드 결과', n.clock(), label='barcode_notice:barcode_ascending')
    n.command('mute')
    assert n._current.cancelled and n.player.stops == 1


def test_barcode_result_changes_and_reset_cancel_old_speech():
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.tracker.style = 'cute'
    n.barcode('P2-01'); n.tick()
    n.barcode('P2-02'); n.tick(); run(n)
    assert texts == [('thought', '좋아, 바코드 읽었어! 내림차순이야.')]
    n.barcode('P2-03'); n.tick(); n.barcode(''); run(n)
    assert len(texts) == 1 and not n.backend.calls


def test_start_speech_survives_next_blink_but_not_recount():
    n, _ = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.traffic_light(1, 0, 0, 1, False, False, 0, sequence=[0]); n.tick()
    n.traffic_light(1, 1, 0, 2, False, False, 0, sequence=[0, 1])
    assert [u.label for u in n._utterances] == ['traffic_notice:traffic_start']
    n.traffic_light(0, 0, 0, 0, False, False, 1, sequence=[])
    assert not n._utterances


def test_mission_is_read_only_fact_and_initial_latched_state_is_quiet():
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.mission_state('Idle'); n.tick(); run(n)
    n.mission_state('Collect: approach_box S1')
    n.clock.now += 1.1; n.tick(); run(n)
    assert texts == [('event', 'S1 쪽으로 접근하는 단계야.')]
    assert not n.backend.calls
    for t in (102, 103, 180):
        n.clock.now = t; n.mission_state('Collect: approach_box S1'); n.tick(); run(n)
    assert len(texts) == 1
    _, prompt = n._prompt(Job('ask', n.clock(), text='뭘 하는 중이야?'), False)
    assert '미션이 마지막으로 보고한 단계' in prompt and 'approach_box S1' in prompt
    p = MissionProgress(); p.update('Done', 10)
    assert not p.poll(20)


@pytest.mark.parametrize('command', ['stop', 'mute'])
def test_stop_mute_and_new_state_cancel_pending_progress(command):
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.mission_state('Idle'); n.mission_state('ReadBarcode')
    n.command(command); n.clock.now += 2; n.tick(); run(n)
    assert not texts
    n.command('unmute'); n.mission_state('Collect: approach_box S1')
    n.clock.now += 2; n.tick()
    n.mission_state('Idle'); run(n)
    assert not texts


def test_recorded_mission_states_have_no_invented_success():
    labels = [r['data']['data'] for r in observations() if r['topic'] == '/mission/state']
    assert 'ReadBarcode: view_barcode' in labels and 'Collect: approach_box S1' in labels
    words = [mission_words(label) for label in labels]
    assert any('바코드' in word for word in words)
    assert all('찾았' not in word and '집었' not in word and '놓았' not in word for word in words)
    assert mission_words('Unknown: invented action') == ''
    assert mission_words('Collect: approach_box S99') == ''
    assert mission_words('Return: leave_box L3') == 'L3에서 빠져나오는 단계야.'
