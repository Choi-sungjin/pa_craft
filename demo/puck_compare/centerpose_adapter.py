"""CenterPose -> the same per-frame puck rows puck_axis_offline writes (camera optical frame, metres).

Fill CenterPoseAdapter.__init__/_infer once the repo is cloned; compare.py only calls predict().
CenterPose (NVlabs) returns, per object, 2D/3D projected cuboid keypoints in Objectron order
(0 = centroid, 1..8 = corners) plus relative scale; with a known puck size it gives a metric pose.
The cylinder axis is the cuboid's object y axis (Objectron: y up, for bottle/cup categories) - check that
against the repo's category convention before trusting the numbers.
"""
import time

import numpy as np

# Objectron corner signs (x, y, z) for keypoints 1..8.
CORNERS = np.array([[-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
                    [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]])


def axis_from_cuboid(points_3d):
    """(9, 3) centroid + 8 corners in camera frame -> (center, unit y axis)."""
    p = np.asarray(points_3d, float)
    corners = p[1:9]
    top, bottom = corners[CORNERS[:, 1] > 0].mean(0), corners[CORNERS[:, 1] < 0].mean(0)
    axis = top - bottom
    return p[0], axis / np.linalg.norm(axis)


class CenterPoseAdapter:
    def __init__(self, repo=None, weights=None, category='bottle', puck_size=(0.03, 0.06, 0.03)):
        self.repo, self.weights, self.category = repo, weights, category
        self.puck_size = puck_size  # x, y (axis), z [m]; plan.yaml puck_size, axis along y
        self.model = None  # TODO(centerpose): build the detector from repo + weights

    def _infer(self, bgr, K):
        """-> list of dict(points_3d=(9,3) camera frame [m], score=float). TODO(centerpose)."""
        raise NotImplementedError('CenterPose not wired yet: fill CenterPoseAdapter._infer')

    def predict(self, bgr, K):
        t = time.perf_counter()
        objects = self._infer(bgr, K)
        ms = (time.perf_counter() - t) * 1e3
        rows = []
        for i, o in enumerate(objects):
            center, axis = axis_from_cuboid(o['points_3d'])
            if axis[0] < 0:
                axis = -axis  # 180 deg symmetric, same sign rule as PuckAxis lying
            rows.append(dict(id=i, score=o['score'], axis_valid=1, cx=center[0], cy=center[1], cz=center[2],
                             ax=axis[0], ay=axis[1], az=axis[2], detect_ms=ms))
        return rows, ms
