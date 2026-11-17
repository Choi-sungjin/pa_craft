"""Voice confidence only: reported size mismatch must not become a found-box claim."""
import math

import pytest

from craft_voice.flow import BoxEvidence
from test_narrator import Backend, make, run


QUALITY = dict(request_id=7, face_centre_observed=True, extrapolated=False,
               age_s=0, gap_uncertainty_m=.01, face_centre_uncertainty_m=0)


def reason(width=235):
    return f'lidar face fit, both face ends seen (width {width} mm)'


def observed(evidence, at, width=235, **quality):
    return evidence.observe(True, 'lidar', (.4, 0, 0), at,
                            reason=reason(width), **dict(QUALITY, **quality))


def requested(side=.205):
    b = BoxEvidence()
    b.request(7, 'lidar', 90, 'S5', box_side=side)
    return b


def test_recorded_s5_gross_widths_wait_for_three_recovered_scans():
    b = requested()
    # Original 0458 S5 widths; the same request later reported a plausible face.
    for at, width in ((100, 1029.9), (100.1, 1057.3), (100.2, 1016.5)):
        assert observed(b, at, width) == (False, False)
    assert not b.announced
    assert observed(b, 102.7, 245.6) == (False, False)
    assert observed(b, 102.8, 238.8) == (False, False)
    assert observed(b, 102.9, 235.6) == (True, True)
    assert observed(b, 103, 236.0) == (True, False)


@pytest.mark.parametrize('side', [None, math.nan, math.inf, -math.inf, 0, -.205, 'bad'])
def test_missing_or_invalid_requested_size_cannot_confirm(side):
    b = requested(side)
    for at in (100, 100.1, 100.2):
        assert observed(b, at) == (False, False)
    assert not b.announced


@pytest.mark.parametrize('text', [
    '', 'lidar face fit', 'width 235 mm', reason(0), reason(-1),
    reason('nan'), reason('inf'), reason('1e999'), reason('235,0'),
    'prefix ' + reason(), reason() + ' suffix',
    reason().replace(' mm)', ' m)'),
])
def test_only_complete_known_positive_width_report_can_confirm(text):
    b = requested()
    for at in (100, 100.1, 100.2):
        assert b.observe(True, 'lidar', (.4, 0, 0), at,
                         reason=text, **QUALITY) == (False, False)


def test_two_times_size_boundary_and_rejection_breaks_scan_run():
    b = requested()
    assert observed(b, 100, 410) == (False, False)
    assert observed(b, 100.1, 410) == (False, False)
    assert observed(b, 100.2, 410.0001) == (False, False)
    assert observed(b, 100.3, 410) == (False, False)
    assert observed(b, 100.4, 410) == (False, False)
    assert observed(b, 100.5, 410) == (True, True)


def test_same_request_repeats_preserve_evidence_but_size_conflict_invalidates_it():
    b = requested()
    assert observed(b, 100) == (False, False)
    assert not b.request(7, 'lidar', 100.05, 'S5', box_side=.205)
    assert observed(b, 100.1) == (False, False)
    assert b.request(7, 'lidar', 100.15, 'S5', box_side=.4)
    for at in (100.2, 100.3, 100.4):
        assert observed(b, at) == (False, False)
    b.request(7, 'lidar', 100.45, 'S5', box_side=.205)
    assert observed(b, 100.5) == (False, False)
    b.request(8, 'lidar', 100.6, 'S5', box_side=.205)
    assert observed(b, 100.7, request_id=8) == (False, False)
    assert observed(b, 100.8, request_id=8) == (False, False)
    assert observed(b, 100.9, request_id=8) == (True, True)


def test_rejected_width_does_not_rearm_an_already_announced_target():
    b = requested()
    for at in (100, 100.1, 100.2):
        result = observed(b, at)
    assert result == (True, True)
    assert observed(b, 100.3, 1100) == (False, False)
    b.request(8, 'lidar', 100.4, 'S5', box_side=.205)
    for at in (100.5, 100.6, 100.7):
        result = observed(b, at, request_id=8)
    assert result == (True, False)
    b.request(9, 'off', 100.8)
    b.request(10, 'lidar', 100.9, 'S5', box_side=.205)
    for at in (101, 101.1, 101.2):
        result = observed(b, at, request_id=10)
    assert result == (True, True)


def test_same_id_off_rearms_target_but_does_not_restore_conflicting_id():
    b = requested()
    for at in (100, 100.1, 100.2):
        result = observed(b, at)
    assert result == (True, True)
    b.request(7, 'off', 100.3)
    assert b.source == 'off' and not b.announced
    b.request(7, 'lidar', 100.4, 'S5', box_side=.205)
    for at in (100.5, 100.6, 100.7):
        assert observed(b, at) == (False, False)
    b.request(8, 'lidar', 100.8, 'S5', box_side=.205)
    for at in (100.9, 101, 101.1):
        result = observed(b, at, request_id=8)
    assert result == (True, True)


def test_gross_width_cancels_pending_discovery_and_removes_confirmed_fact():
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.planning_state('Run.Execute'); n._jobs.clear(); n._utterances.clear()
    n.box_request(7, 'lidar', 'S5', box_side=.205)
    for at in (100.1, 100.2, 100.3):
        n.clock.now = at
        n.box(True, 'lidar', reason(), .4, 0, 0, **QUALITY)
    n.tick()
    assert any(u.label.startswith('box_notice:') for u in n._utterances)
    n.clock.now = 100.4
    n.box(True, 'lidar', reason(1029.9), .4, 0, 0, **QUALITY)
    assert not n.tracker.signals.box['valid']
    assert '상자 확인 미완료' in n.tracker.context_text(100.4)
    run(n)
    assert texts == []
