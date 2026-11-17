"""Base rectangle candidate; not the unavailable constrained_square method.

uppercam_box_range.py is an unchanged copy of boxgap_1108_1236/code/box_range.py.
The URDF and physical front reference must be verified against the live robot.
This adapter never loads the selected square_far20 calibration.
"""
import json
import hashlib
import math
from pathlib import Path
import numpy as np
import uppercam_box_range as br
from uppercam_causal_median import CausalMedian3

METHOD = 'base_rectangle_complete_square_far_v1'
INPUT_REFERENCE = '(near_x_base-0.19067)*1000-19'

class UppercamEstimator:
    method = METHOD
    def __init__(self, urdf_path, calibration_path=None):
        R, t, floor, laser = br.urdf_frames(str(urdf_path))
        if abs(laser[0] - 0.19067) > 1e-6:
            raise ValueError('URDF front laser differs from calibration reference')
        self.R = br._rpy(0, 0, np.radians(1.5)) @ R
        self.t = t.copy(); self.t[1] = 0.0322; self.t[2] = floor + 0.6229
        self.up = self.R.T @ np.array([0., 0., 1.])
        self.filter = CausalMedian3(max_gap_s=0.15)
        self.calibration = None
        self.validated_for_motion = False
        if calibration_path:
            model = json.loads(Path(calibration_path).read_text())
            if (model.get('method') != self.method or model.get('input_reference') != INPUT_REFERENCE or
                    model.get('urdf_sha256') != hashlib.sha256(Path(urdf_path).read_bytes()).hexdigest()):
                raise ValueError('calibration model is incompatible with this detector')
            x = np.asarray(model['input_knots_mm'], float); y = np.asarray(model['gap_knots_mm'], float)
            if len(x) < 2 or len(x) != len(y) or not (np.isfinite(x).all() and np.isfinite(y).all()) or not (np.diff(x)>0).all() or not (np.diff(y)>0).all():
                raise ValueError('invalid calibration knots')
            self.calibration = (x,y)
            self.validated_for_motion = model.get('validated_for_motion') is True

    def reset(self):
        self.filter.reset()

    def geometry(self, depth_m, K):
        depth_m = np.asarray(depth_m, dtype=float); K = np.asarray(K, dtype=float)
        if depth_m.ndim != 2 or K.shape != (3,3) or not np.isfinite(K).all() or K[0,0] <= 0 or K[1,1] <= 0:
            raise ValueError('invalid image or intrinsics')
        top, cen, normal, area, rms = br.box_top(depth_m, K, self.up, .6229, np.random.default_rng(0), .5, None)
        corners, _ = br.rectangle(top, K, cen, normal)
        seen = br.far_corner_visibility(depth_m, K, cen, normal, corners, self.up)
        square, flags = br.complete_square_far(corners, self.up, seen)
        points = square @ self.R.T + self.t
        near = points[np.argsort(points[:,0])[:2]].mean(axis=0)
        tangent = points[1,:2]-points[0,:2]
        if tangent[1] < 0: tangent = -tangent
        yaw = math.atan2(-tangent[0], tangent[1])
        values = [float((near[0]-.19067)*1000.-19.), float(near[1]), float(yaw)]
        if not np.isfinite(values).all(): raise ValueError('nonfinite geometry')
        return dict(raw_gap_mm=values[0], lateral_m=values[1], box_yaw_rad=values[2], flags=flags,
                    plane_rmse_mm=float(rms*1000), top_area_m2=float(area))

    def estimate(self, depth_m, K, frame_index, timestamp_s, frame_id):
        result = dict(valid=False, state='invalid', gap_mm=None, box_yaw_rad=None, lateral_m=None,
                      stamp_s=timestamp_s, frame_id=frame_id, geometry_frame_id='base_link', frame_index=frame_index, method=self.method,
                      extrapolated=False, current_extrapolated=False, raw_gap_mm=None)
        if not isinstance(frame_id, str) or not frame_id or not isinstance(timestamp_s, (float, int)) or not math.isfinite(float(timestamp_s)):
            self.reset(); result['state']='invalid_frame_or_time'; return result
        try:
            geo = self.geometry(depth_m,K); result.update(geo)
            if self.calibration is None:
                self.reset(); result['state']='calibration_required'; return result
            x,y = self.calibration; raw = geo['raw_gap_mm']
            i = int(np.clip(np.searchsorted(x,raw,side='right')-1, 0,len(x)-2))
            calibrated = float(y[i]+(raw-x[i])*(y[i+1]-y[i])/(x[i+1]-x[i]))
            extrap = bool(raw<x[0] or raw>x[-1])
            filtered = self.filter.update(calibrated,frame_index,timestamp_s,extrapolated=extrap)
            result.update(valid=filtered['valid'],state=filtered['state'],gap_mm=filtered['value'],
                          current_extrapolated=extrap,
                          extrapolated=any(item['extrapolated'] for item in self.filter.history))
            if filtered['valid'] and not self.validated_for_motion:
                result['valid'] = False
                result['state'] = 'calibration_unverified'
            if filtered['valid']:
                result['selected_extrapolated'] = filtered['extrapolated']
                result['sample_age_s'] = filtered['sample_age_s']
                result['source_frame_index'] = filtered['source_frame_index']
            return result
        except (RuntimeError, ValueError, ArithmeticError, np.linalg.LinAlgError, IndexError) as e:
            self.reset(); result['error'] = str(e); return result
