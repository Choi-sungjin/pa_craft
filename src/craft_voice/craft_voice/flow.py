"""Read-only mission progress and conservative box evidence for voice. No ROS or API calls."""
from __future__ import annotations

import math
import re


def _positive_finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError):
        return math.nan
    return number if math.isfinite(number) and number > 0 else math.nan


_BOX_FACE_WIDTH = re.compile(
    r'lidar face fit, both face ends seen \(width '
    r'([+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?) mm\)')


def mission_words(label: str) -> str:
    phase, _, action = label.partition(': ')
    if phase == 'WatchSignal':
        return '신호등 점멸 결과를 기다리는 중이야.'
    if phase == 'ReadBarcode':
        return ('바코드를 볼 위치로 접근하는 단계야.' if action == 'view_barcode'
                else '바코드 판독 결과를 기다리는 중이야.' if not action else '')
    match = re.fullmatch(r'(approach_box|leave_box|observe_pucks|verify_location) ([SL][1-6])', action)
    if phase in ('Collect', 'Deliver', 'Verify', 'Return') and match:
        verb, box = match.groups()
        return {
            'approach_box': f'{box} 쪽으로 접근하는 단계야.',
            'leave_box': f'{box}에서 빠져나오는 단계야.',
            'observe_pucks': f'{box}의 퍽을 확인하는 단계야.',
            'verify_location': f'{box}에 놓인 퍽을 검사하는 단계야.',
        }[verb]
    if phase == 'Collect' and re.fullmatch(r'pick_puck S[1-6] #\d+ -> slot \d+', action):
        return '퍽을 집는 단계야. 성공 여부는 아직 확인 중이야.'
    if phase == 'Deliver' and action.startswith('place_puck '):
        return '퍽을 놓는 단계야. 성공 여부는 아직 확인 중이야.'
    if phase == 'Allocate' and not action:
        return '퍽을 옮길 순서를 계산하는 단계야.'
    if phase == 'Return':
        if action == 'go_home':
            return '시작 위치로 돌아가는 단계야.'
        if re.fullmatch(r'hold_still [\d.]+ s', action):
            return '마지막 정지 확인을 기다리는 중이야.'
    if label == 'Done':
        return '미션이 완료 상태로 바뀌었어.'
    if phase == 'Fault':
        return '미션 진행에 문제가 생겼어. 원인을 확인해야 해.'
    return ''


class MissionProgress:
    """State is change-only and latched: first sample is history, never a new event."""
    def __init__(self):
        self.label = None
        self.words = ''
        self.at = -math.inf
        self.version = 0
        self.pending = False

    def update(self, label, now):
        label = label.strip()
        if label == self.label:
            return False
        first = self.label is None
        words = mission_words(label)
        changed = words != self.words
        self.label, self.words, self.at = label, words, now
        if changed:
            self.version += 1
            self.pending = bool(words) and not first
        return changed

    def poll(self, now):
        if self.pending and now - self.at >= 1.0:
            self.pending = False
            return self.words
        return ''

    def facts(self, now):
        if self.label is None or now - self.at > 1800:
            return ''
        return (f'- 미션이 마지막으로 보고한 단계({now - self.at:.0f}초 전): {self.label}. '
                f'{self.words} 현재 완료·물체 발견을 별도로 추측하지 말 것.')


class BoxEvidence:
    """A fitted plane alone is not a detected box. Match an active request and several fresh face scans."""
    def __init__(self):
        self.request_id = None
        self.source = ''
        self.box_id = ''
        self.box_side = math.nan
        self.size_conflict = False
        self.request_at = -math.inf
        self.announced = False
        self.count = 0
        self.since = self.last = -math.inf

    def request(self, request_id, source, now, box_id='', box_side=math.nan):
        box_side = _positive_finite(box_side)
        same_id = self.request_id is not None and request_id == self.request_id
        same_side = (box_side == self.box_side
                     or math.isnan(box_side) and math.isnan(self.box_side))
        if same_id and not same_side:
            # A request ID must describe one size; only a new ID can restore trust.
            self.size_conflict = True
            self.count = 0
            self.since = self.last = -math.inf
            if source != 'off':
                return True
        if (request_id, source) == (self.request_id, self.source):
            return False
        same_target = bool(box_id) and box_id == self.box_id and source == self.source == 'lidar'
        self.request_id, self.source, self.request_at, self.box_id = request_id, source, now, box_id
        self.box_side = box_side
        if not same_id:
            self.size_conflict = False
        self.count = 0
        self.since = self.last = -math.inf
        if not same_target:
            self.announced = False
        return True

    def observe(self, valid, source, geometry, now, *, request_id=None, face_centre_observed=False,
                extrapolated=True, age_s=math.inf, gap_uncertainty_m=math.inf,
                face_centre_uncertainty_m=math.inf, reason=''):
        observed_at = now - age_s
        match = _BOX_FACE_WIDTH.fullmatch(reason) if isinstance(reason, str) else None
        width_mm = _positive_finite(match.group(1)) if match else math.nan
        # Twice the requested side is a voice policy against large size mismatches,
        # not a validated physical tolerance or proof that this object is the box.
        width_matches = (not self.size_conflict and math.isfinite(self.box_side)
                         and math.isfinite(width_mm) and width_mm <= self.box_side * 2000)
        accepted = (valid and self.request_id is not None and request_id == self.request_id
                    and width_matches
                    and source == self.source == 'lidar' and face_centre_observed and not extrapolated
                    and math.isfinite(age_s) and 0 <= age_s <= 1.0
                    and observed_at >= self.request_at
                    and all(math.isfinite(v) for v in geometry)
                    and all(math.isfinite(v) and v >= 0 for v in
                            (gap_uncertainty_m, face_centre_uncertainty_m)))
        if not accepted:
            self.count = 0
            return False, False
        if observed_at <= self.last:  # repeats/out-of-order scans are not independent evidence
            return self.count >= 3 and self.last - self.since >= 0.15, False
        if not self.count or observed_at - self.last > 0.5:
            self.count, self.since = 0, observed_at
        self.count += 1
        self.last = observed_at
        confirmed = self.count >= 3 and observed_at - self.since >= 0.15
        notice = confirmed and not self.announced
        if notice:
            self.announced = True
        return confirmed, notice
