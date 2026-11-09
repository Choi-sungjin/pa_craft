"""Exact supplied square_far20 postprocessing, diagnostic-only until geometry is restored.

Raw input must be produced by the missing selected constrained_square detector:
(near_x_base - 0.19067)*1000 - 19. Do not feed the other detector's raw values.
This module does not establish the identity or correctness of an upstream detector.
"""
import json,math
from pathlib import Path
from uppercam_gap_calibration import apply
from uppercam_causal_median import CausalMedian3

class SelectedPostprocessor:
    def __init__(self, model_path):
        self.model=json.loads(Path(model_path).read_text())
        if self.model.get('method')!='square_far20' or self.model.get('input_reference')!='nominal_LDS03_front_face_dx_mm__origin_minus_19':
            raise ValueError('not the provided selected model')
        self.filter=CausalMedian3(max_gap_s=.15)

    def reset(self):self.filter.reset()

    def update(self,raw_gap_mm,frame_index,timestamp_s,valid=True):
        result=dict(valid=False,state='invalid',diagnostic_gap_mm=None,extrapolated=False,
                    geometry_available=False,method='square_far20',stamp_s=timestamp_s,frame_index=frame_index)
        if not valid or raw_gap_mm is None:
            self.reset();return result
        try:
            ts=float(timestamp_s);raw=float(raw_gap_mm)
            if not math.isfinite(ts) or not math.isfinite(raw):raise ValueError('nonfinite input')
            value,extrapolated=apply(self.model,raw)
            filtered=self.filter.update(value,frame_index,ts,valid=True,extrapolated=extrapolated)
        except (ValueError,TypeError,KeyError,IndexError):
            self.reset();return result
        result.update(state=filtered['state'],current_extrapolated=extrapolated,
                      extrapolated=filtered.get('extrapolated',extrapolated))
        if filtered['valid']:
            result.update(state='geometry_unverified',diagnostic_gap_mm=filtered['value'],
                          source_frame_index=filtered['source_frame_index'],sample_age_s=filtered['sample_age_s'])
        return result
