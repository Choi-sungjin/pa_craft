"""Exact supplied square_far20 postprocessing, diagnostic-only until geometry is restored.

Raw input must be produced by the missing selected constrained_square detector:
(near_x_base - 0.19067)*1000 - 19. Do not feed the other detector's raw values.
This module does not establish the identity or correctness of an upstream detector.
"""
import json,math
from pathlib import Path
from uppercam_gap_calibration import apply
from uppercam_trigger_filter import LatestGate

class SelectedPostprocessor:
    def __init__(self, model_path):
        self.model=json.loads(Path(model_path).read_text())
        if self.model.get('method')!='square_far20' or self.model.get('input_reference')!='nominal_LDS03_front_face_dx_mm__origin_minus_19':
            raise ValueError('not the provided selected model')
        # Latest valid frame behind a jump gate (2026-11-10, was CausalMedian3: +1-2 frames of lag, cleared by
        # every invalid frame). See uppercam_trigger_filter.py.
        self.filter=LatestGate(jump_mm=8.0,max_rate_mm_s=120.0,max_gap_s=.3)

    def reset(self):self.filter.reset()

    def update(self,raw_gap_mm,frame_index,timestamp_s,valid=True):
        result=dict(valid=False,state='invalid',diagnostic_gap_mm=None,extrapolated=False,
                    geometry_available=False,method='square_far20',stamp_s=timestamp_s,frame_index=frame_index)
        if not valid or raw_gap_mm is None:
            self.filter.miss(timestamp_s);return result
        try:
            ts=float(timestamp_s);raw=float(raw_gap_mm)
            if not math.isfinite(ts) or not math.isfinite(raw):raise ValueError('nonfinite input')
            value,extrapolated=apply(self.model,raw)
            filtered=self.filter.update(value,frame_index,ts,valid=True,extrapolated=extrapolated)
        except (ValueError,TypeError,KeyError,IndexError):
            self.filter.miss(timestamp_s);return result
        result.update(state=filtered['state'],current_extrapolated=extrapolated,
                      extrapolated=filtered.get('extrapolated',extrapolated))
        if filtered['valid']:
            result.update(state='geometry_unverified',diagnostic_gap_mm=filtered['value'],
                          source_frame_index=filtered['source_frame_index'],sample_age_s=filtered['sample_age_s'])
        return result
