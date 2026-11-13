"""CenterPose -> the same per-frame puck rows puck_axis_offline writes (camera optical frame, metres).

Uses the perception-v2 package's trained puck CenterPose (DLA34, crop around a box, 9-point cuboid -> PnP
with the known 30x30x60 size), i.e. its scripts/centerpose_pucks.py load_model / infer, unchanged.
The crop needs a puck box: the YOLO puck model gives it (ultralytics .pt).

Needs torch + ultralytics: run compare.py with ~/yolov8-ubuntu24-practice/.venv-yolo/bin/python.
Package results (synthetic test, 2026-11-08 summary): lying-puck axis 2.3 deg median, centre 22.9 mm
(depth method 1.7 deg / 1.7 mm) -> the comparison on real sessions is a TODO (docs/todo/puck_perception.md).
"""
import sys
import time
from pathlib import Path

import numpy as np

PACKAGE = Path('~/Downloads/perception-v2-puck-box-signal-barcode-20261108.tar/'
               'perception-v2-puck-box-signal-barcode-20261108').expanduser()
WEIGHTS = PACKAGE / 'models/centerpose-pucks-v7tag/best.pt'
YOLO_WEIGHTS = Path('~/dataset/export_pucks_v8n_v2/pucks_v8n_v2_ep24.pt').expanduser()


class CenterPoseAdapter:
    def __init__(self, repo=None, weights=None, yolo=None, min_box_score=0.4, device=None):
        repo = Path(repo or PACKAGE).expanduser()
        for p in (repo, repo / 'scripts'):
            if str(p) not in sys.path:
                sys.path.insert(0, str(p))
        import torch
        import centerpose_pucks as cp  # the package module: its load_model / infer
        from ultralytics import YOLO
        self.cp = cp
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = cp.load_model(str(Path(weights or WEIGHTS).expanduser()), self.device)
        self.yolo = YOLO(str(Path(yolo or YOLO_WEIGHTS).expanduser()))
        self.min_box_score = min_box_score

    def _infer(self, bgr, K):
        """-> list of dict(id, score, center (3,), axis (3,)) for valid CenterPose results."""
        boxes = self.yolo.predict(bgr, imgsz=640, conf=self.min_box_score, verbose=False)[0].boxes
        out = []
        for (x1, y1, x2, y2), c, s in zip(boxes.xyxy.tolist(), boxes.cls.int().tolist(), boxes.conf.tolist()):
            r = self.cp.infer(self.model, bgr, (x1, y1, x2, y2), np.asarray(K, np.float64), device=self.device)
            if r.get('valid'):
                out.append(dict(id=int(c), score=float(s) * float(r['confidence']),
                                center=np.array(r['center_camera_m']), axis=np.array(r['axis_camera'])))
        return out

    def predict(self, bgr, K):
        t = time.perf_counter()
        objects = self._infer(bgr, K)
        ms = (time.perf_counter() - t) * 1e3
        rows = []
        for o in objects:
            axis = o['axis'] / np.linalg.norm(o['axis'])
            if axis[0] < 0:
                axis = -axis  # 180 deg symmetric, same sign rule as PuckAxis lying
            c = o['center']
            rows.append(dict(id=o['id'], score=o['score'], axis_valid=1, cx=c[0], cy=c[1], cz=c[2],
                             ax=axis[0], ay=axis[1], az=axis[2], detect_ms=ms))
        return rows, ms
