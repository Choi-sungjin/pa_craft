"""Monotone piecewise-linear scalar gap calibration. No temporal state.

Fit input medians at known physical LDS03-front-face gaps. At runtime ONLY the
algorithm input is used; placement labels are never supplied to apply(). End
segments extrapolate linearly, explicitly flagged rather than clipping values
and artificially suppressing jitter. No polygon/point-cloud geometry is moved.
"""
import numpy as np


def fit_model(inputs_mm, gaps_mm, method, input_reference):
    x=np.asarray(inputs_mm,dtype=float);y=np.asarray(gaps_mm,dtype=float)
    if x.ndim!=1 or y.shape!=x.shape or len(x)<2:
        raise ValueError('At least two input/gap pairs required')
    if not np.isfinite(x).all() or not np.isfinite(y).all():
        raise ValueError('Calibration pairs must be finite')
    if not np.all(np.diff(x)>0) or not np.all(np.diff(y)>0):
        raise ValueError('Input medians and physical gaps must both increase strictly')
    return dict(method=method,input_reference=input_reference,
                output_reference='physical_LDS03_front_face_to_box_forward_gap_mm',
                input_knots_mm=x.tolist(),gap_knots_mm=y.tolist(),
                interpolation='piecewise_linear',extrapolation='linear_end_segments',
                temporal_filter=False)


def apply(model, input_mm):
    x=np.asarray(model['input_knots_mm'],float);y=np.asarray(model['gap_knots_mm'],float)
    value=np.asarray(input_mm,dtype=float)
    if not np.isfinite(value).all():raise ValueError('Input must be finite')
    # Interval search handles interpolation and endpoint extrapolation uniformly.
    i=np.clip(np.searchsorted(x,value,side='right')-1,0,len(x)-2)
    result=y[i]+(value-x[i])*(y[i+1]-y[i])/(x[i+1]-x[i])
    extrapolated=(value<x[0])|(value>x[-1])
    if value.ndim==0:return float(result),bool(extrapolated)
    return result,extrapolated
