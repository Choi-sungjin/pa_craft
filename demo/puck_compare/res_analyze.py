#!/usr/bin/env python3
"""TODO A1 / B1 / B6 / B7 (docs/todo/puck_perception.md): res_record.py sessions -> per-resolution table.

For each session (one scene at one resolution):
  * YOLO on the frame downscaled to 640 wide (as production: the model input is 640 anyway), both puck
    models (pucks_v8n_v2 and pucks_v8b) -> boxes scaled back to full resolution
  * AprilTag 36h11 only inside each puck box (grown 1.4x), on the FULL-resolution frame, with the system
    OpenCV the C++ build uses (4.6; OpenCV 5 finds more tags, so it would flatter the result)
  * the red-prism rule of PuckShapeRule (>= 2 tags, or 1 face-on tag on a long box -> prism), mirrored here

Run with the venv that has ultralytics; the tag pass re-runs itself under /usr/bin/python3:
  ~/craft/custom_yolo26/.venv/bin/python res_analyze.py ~/bags/puck_res/four_a_*
"""
import json
import subprocess
import sys
from pathlib import Path

MODELS = {
    'v8n_v2': Path('~/dataset/export_pucks_v8n_v2/pucks_v8n_v2_ep24.pt').expanduser(),
    'v8b': Path('~/Downloads/perception-v2-puck-box-signal-barcode-20261108.tar/perception-v2-puck-box-signal-barcode-'
                '20261108/runs/detect/pucks_v8b/weights/best.pt').expanduser(),
}
NAMES = ['red_cylinder', 'red_prism', 'green_cylinder', 'green_prism']
GROW, MIN_SCORE = 1.4, 0.4
SIDE_RATIO, SKEW_DEG, ASPECT = 0.85, 10.0, 1.6  # perception.yaml shape_rule


def yolo_pass(sessions):
    import cv2
    from ultralytics import YOLO
    models = {k: YOLO(str(p)) for k, p in MODELS.items() if p.exists()}
    for s in sessions:
        out = {}
        for name, m in models.items():
            frames = {}
            for png in sorted((s / 'frames').glob('*.png')):
                img = cv2.imread(str(png))
                scale = 640 / img.shape[1]
                small = cv2.resize(img, (640, round(img.shape[0] * scale)), interpolation=cv2.INTER_AREA)
                r = m.predict(small, imgsz=640, conf=0.25, verbose=False)[0]
                frames[png.name] = [[*(v / scale for v in b), NAMES[c] if c < 4 else str(c), sc]
                                    for b, c, sc in zip(r.boxes.xyxy.tolist(), r.boxes.cls.int().tolist(),
                                                        r.boxes.conf.tolist())]
            out[name] = frames
        (s / 'res_boxes.json').write_text(json.dumps(out))
        print(f'yolo done: {s.name} ({", ".join(models)})', flush=True)


def tag_pass(sessions):
    import time
    import cv2
    import numpy as np
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    par = cv2.aruco.DetectorParameters_create()
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG

    def face_on(c):
        sides = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        if max(sides) <= 0 or min(sides) / max(sides) < SIDE_RATIO:
            return False
        for i in range(4):
            a, b = c[i - 1] - c[i], c[(i + 1) % 4] - c[i]
            ang = np.degrees(np.arccos(np.clip(a @ b / (np.linalg.norm(a) * np.linalg.norm(b)), -1, 1)))
            if abs(ang - 90) > SKEW_DEG:
                return False
        return True

    print(f'\n{"session":<24}{"model":<8}{"frames":>7}{"boxes":>7}{"tag>=1":>8}{"tag>=2":>8}{"side px":>8}'
          f'{"ms/fr":>7}   red: yolo cyl -> rule prism / yolo prism -> confirmed')
    for s in sessions:
        data = json.loads((s / 'res_boxes.json').read_text())
        meta = json.loads((s / 'meta.json').read_text())
        for model, frames in data.items():
            n_box = n1 = n2 = 0
            sides, ms = [], []
            red = dict(cyl=0, cyl_to_prism=0, prism=0, prism_confirmed=0)
            per_class = {k: [] for k in NAMES}
            for name, boxes in frames.items():
                gray = cv2.cvtColor(cv2.imread(str(s / 'frames' / name)), cv2.COLOR_BGR2GRAY)
                H, W = gray.shape
                t0 = time.perf_counter()
                tags = []
                for x1, y1, x2, y2, cls, sc in boxes:
                    if sc < MIN_SCORE:
                        continue
                    cx, cy, w, h = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) * GROW, (y2 - y1) * GROW
                    a, b = int(max(0, cx - w / 2)), int(max(0, cy - h / 2))
                    e, f = int(min(W, cx + w / 2 + 1)), int(min(H, cy + h / 2 + 1))
                    corners, ids, _ = cv2.aruco.detectMarkers(gray[b:f, a:e], dic, parameters=par)
                    for i, c in zip([] if ids is None else ids.ravel(), corners or []):
                        c = c.reshape(4, 2) + [a, b]
                        if not any(i == j and np.linalg.norm(c.mean(0) - q.mean(0)) < 3 for j, q in tags):
                            tags.append((int(i), c))
                ms.append((time.perf_counter() - t0) * 1e3)
                for x1, y1, x2, y2, cls, sc in boxes:
                    if sc < MIN_SCORE:
                        continue
                    per_class.setdefault(cls, []).append(sc)
                    inside = [c for i, c in tags if i < 12 and x1 <= c[:, 0].mean() <= x2 and y1 <= c[:, 1].mean() <= y2]
                    n_box += 1
                    n1 += len(inside) >= 1
                    n2 += len(inside) >= 2
                    sides += [float(np.mean([np.linalg.norm(c[(k + 1) % 4] - c[k]) for k in range(4)])) for c in inside]
                    long_box = max(x2 - x1, y2 - y1) / max(1e-6, min(x2 - x1, y2 - y1)) >= ASPECT
                    prism_by_tags = len(inside) >= 2 or (len(inside) == 1 and long_box and face_on(inside[0]))
                    if cls == 'red_cylinder':
                        red['cyl'] += 1
                        red['cyl_to_prism'] += prism_by_tags
                    elif cls == 'red_prism':
                        red['prism'] += 1
                        red['prism_confirmed'] += prism_by_tags
            f = max(n_box, 1)
            print(f'{s.name:<24}{model:<8}{len(frames):>7}{n_box:>7}{n1 / f:>8.0%}{n2 / f:>8.0%}'
                  f'{(np.median(sides) if sides else 0):>8.1f}{(np.mean(ms) if ms else 0):>7.2f}'
                  f'   {red["cyl"]} -> {red["cyl_to_prism"]} / {red["prism"]} -> {red["prism_confirmed"]}')
            print(' ' * 32 + 'per class (n, mean score): ' + ', '.join(
                f'{k} {len(v)} {np.mean(v):.2f}' for k, v in per_class.items() if v)
                + f'   camera {meta.get("camera_fps", "?")} fps')


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] in ('--tags-only',) else None
    sessions = [Path(p).expanduser() for p in sys.argv[1:] if not p.startswith('--')]
    if not sessions:
        sys.exit(__doc__)
    if mode == '--tags-only':
        tag_pass(sessions)
        return
    yolo_pass(sessions)
    subprocess.run(['/usr/bin/python3', __file__, '--tags-only', *map(str, sessions)], check=True)


if __name__ == '__main__':
    main()
