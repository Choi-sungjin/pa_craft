"""Robust geometry with cached random hypotheses and sparse plane checks."""
from uppercam_robust_estimator import RobustUppercamEstimator
from uppercam_fast5_box_range import box_top
class Fast5RobustUppercamEstimator(RobustUppercamEstimator):
    top_detector = staticmethod(box_top)
