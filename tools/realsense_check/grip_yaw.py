#!/usr/bin/env python3
"""Gripper-cam (그립캠) yaw of a standing prism at the pick wait pose (execute.py --stand --cell N, tool_roll raw 90).
2026-11-15.

DEPTH (the reading used): the prism's own shape, not its tag (a tag can be stuck on turned, user 2026-11-15).
  1. depth -> points (depth intrinsics), moved into the colour camera frame (depth->colour extrinsic), so the angle
     frame is the same as the tag method's and one calibration serves both.
  2. surface plane: the biggest RANSAC plane (the box / floor the prism stands on); its normal = up.
  3. the prism: points DEPTH_H_MM above the plane, blobs in plane coordinates (x = the camera x axis projected,
     y = up x x), the prism-sized one nearest the camera; its top height and top-face pixels.
  4. raw yaw: the top-face pixel rays cut with the top plane, rect_fit (smallest percentile-extent rectangle).
     --capture averages LIVE_FRAMES frames.
  Known limit: the D435 fattens the depth to the left of a near edge (overlay 11-15: the top mask bulges past the
  top's left side), 1-2 deg per frame scatter; a right-boundary-only line fit was tried and was worse.
Needs depth on the prism: the top is ~22 cm from the camera, and the D435 sees nothing nearer than 26 cm at
1280x720 depth (the first bags: no depth reading possible), 17 cm at 848x480, 13 cm at 640x360 (grip_yaw_bag.py
records 640x360 since).
TAG (cross-check only): the top tag36h11's corner rays cut with the surface plane, its four edges folded to 90 deg.

yaw = SIGN * (raw - OFFSET) folded into [-45, 45) (grip_yaw.json; --calibrate fits it on bags with known yaws; from
the tag on the first bags: offset +1.08, sign -1: yaw + = turned clockwise seen from above). raw is measured from the
camera x axis in the surface plane, counter-clockwise seen from above. Square prism: yaw 0 = yaw 90.
Run with the venv python3 (pyrealsense2 for --capture; its OpenCV also reads the tag corners better).

Bags 2026-11-15 13:51-13:54 (depth 640x360; cells 2, 5, 8 = far / middle / near centre; 0 / 22.5 / 45 / 67.5 / 90
set by hand): depth found the prism in every frame, top 60-62 mm. Calibrated on depth (offset -2.4, sign -1; the tag
reads ~2.7 deg the other way: tag stuck on turned or depth bias, not separable). Each cell held out: depth error rms
1.7-2.9 deg (8-frame means), tag 1.6-2.7; the worst yaws are worst for both (cell 5 set 45: -4.4 both) = hand
placement. Earlier bags (depth 1280x720) have no depth on the prism (tag only).

  python3 grip_yaw.py DIR [DIR ...]                     every bag frame: per-yaw mean / spread / error, depth and tag
  python3 grip_yaw.py --calibrate [--by tag] DIR ...    fit OFFSET and SIGN (default on the depth reading)
  python3 grip_yaw.py --capture                         one frame now (close the live window first); prints YAW
  python3 grip_yaw.py --video OUT.mp4 DIR [DIR ...]     every bag frame with both readings drawn, one mp4
  --png OUT_DIR: one drawn frame per yaw (not opened)
"""
import json
import os
import sqlite3
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from read_bag import cdr_compressed_image  # noqa: E402

CAL = os.path.join(HERE, 'grip_yaw.json')
GRIP_SERIAL = '261622073363'
COL = '/device_0/sensor_1/Color_0/image/data/compressed'
DEP = '/device_0/sensor_0/Depth_0/image/data/compressedDepth'
DEPTH_H_MM = (48.0, 72.0)    # prism top-face points (60 mm prism; 2026-11-15 20:19: 12-75 let a lower object beside it join)
DEPTH_SIDE_MM = (20.0, 40.0)  # a top face's rectangle sides
GRID_MM = 2.0
_DET = None


# ---------------------------------------------------------------- angles
def fold(a, period=90.0):
    """a [deg] into [-period/2, period/2)."""
    return (np.asarray(a) + period / 2) % period - period / 2


def mean_fold(a, period=90.0):
    """Circular mean of angles [deg] with that period."""
    k = 360.0 / period
    r = np.radians(np.asarray(a) * k)
    return float(np.degrees(np.arctan2(np.sin(r).mean(), np.cos(r).mean())) / k)


def calibration():
    return json.load(open(CAL)) if os.path.exists(CAL) else dict(offset_deg=0.0, sign=1)


def yaw(raw, cal=None):
    cal = cal or calibration()
    return float(fold(cal['sign'] * (raw - cal['offset_deg'])))


# ---------------------------------------------------------------- geometry
def K_of(ci):
    return np.array([[ci['fx'], 0, ci['ppx']], [0, ci['fy'], ci['ppy']], [0, 0, 1.0]])


def to_colour_points(d, Kd, R, t, zmax=1.0):
    """Depth image [m] -> (N,3) points in the colour camera frame [m] and their depth pixels (u, v)."""
    v, u = np.nonzero((d > 0.05) & (d < zmax))
    z = d[v, u]
    P = np.c_[(u - Kd[0, 2]) / Kd[0, 0] * z, (v - Kd[1, 2]) / Kd[1, 1] * z, z]
    return P @ R.T + t, u, v


def surface_plane(P, iters=400, inlier=0.003, seed=0, max_pts=15000):
    """Biggest RANSAC plane of P, refit by least squares: (n toward the camera, p0, inliers, rms [m]) or None."""
    if len(P) < 200:
        return None
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(len(P), max_pts), replace=False)]
    best = None
    for _ in range(iters):
        a, b, c = S[rng.choice(len(S), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        cnt = int((np.abs((S - a) @ n) < inlier).sum())
        if best is None or cnt > best[0]:
            best = (cnt, n, a)
    keep = np.abs((S - best[2]) @ best[1]) < inlier
    for _ in range(2):
        p0 = S[keep].mean(0)
        n = np.linalg.svd(S[keep] - p0)[2][2]
        keep = np.abs((S - p0) @ n) < inlier
    n = n if n[2] < 0 else -n
    return n, p0, int(keep.sum()), float(np.sqrt(np.mean(((S[keep] - p0) @ n) ** 2)))


def plane_axes(n):
    """In-plane axes: x = the camera x axis projected, y = n x x (counter-clockwise seen from above)."""
    ax = np.array([1.0, 0, 0]) - n[0] * n
    ax /= np.linalg.norm(ax)
    return ax, np.cross(n, ax)


# ---------------------------------------------------------------- depth reading
TOP_BAND_MM = 6.0      # top-face pixels: within this of the top height
FIT_PCT = 3.0          # rectangle extents: this percentile from each end (flying pixels at the edges)


def rect_fit(Q, raw=False):
    """Robust rectangle on 2D points [mm]: the angle (0..90, 0.25 deg steps, then 0.05 around the best) whose
    FIT_PCT..100-FIT_PCT extents give the smallest area. (angle_deg, (w, l), centre (x, y))."""
    def area(t):
        r = np.radians(t)
        u = Q @ np.array([np.cos(r), np.sin(r)]); v = Q @ np.array([-np.sin(r), np.cos(r)])
        lu, hu = np.percentile(u, [FIT_PCT, 100 - FIT_PCT]); lv, hv = np.percentile(v, [FIT_PCT, 100 - FIT_PCT])
        return (hu - lu) * (hv - lv), (hu - lu, hv - lv), ((lu + hu) / 2, (lv + hv) / 2)
    ts = np.arange(0, 90, 0.25)
    t0 = ts[int(np.argmin([area(t)[0] for t in ts]))]
    ts = np.arange(t0 - 0.5, t0 + 0.5, 0.05)
    t = ts[int(np.argmin([area(t)[0] for t in ts]))]
    _, wl, (cu, cv) = area(t)
    r = np.radians(t)
    c = (cu * np.cos(r) - cv * np.sin(r), cu * np.sin(r) + cv * np.cos(r))
    # raw: the unfolded angle, wl[0] lies along it (a lying puck needs to know which side is the long one)
    return (float(t) if raw else float(fold(t))), wl, c


def measure_depth(P, plane):
    """The prism's top face from depth: dict(raw_deg, sides_mm, height_mm, pts, centre_mm, corners3d) or None.
    1. points DEPTH_H_MM over the surface -> blobs on a 2 mm grid in plane coordinates; the prism-sized blob nearest
       the camera. 2. its top height (95th percentile) and the top-face points (within TOP_BAND_MM of it).
    3. their camera rays cut with the plane at the top height: the depth only says which pixels are the top, not
       where they lie (2026-11-15 bags: depth noise along the slanted rays smeared the footprint, +2.5 deg bias and
       1-2 deg per frame with the points as measured). 4. rect_fit on them."""
    n, p0 = plane[0], plane[1]
    ax, ay = plane_axes(n)
    h = (P - p0) @ n * 1000
    sel = (h > DEPTH_H_MM[0]) & (h < DEPTH_H_MM[1])
    if sel.sum() < 30:
        return None
    Ps, hs = P[sel], h[sel]
    Q = np.c_[(Ps - p0) @ ax, (Ps - p0) @ ay] * 1000  # plane coordinates [mm]
    lo = Q.min(0)
    ij = ((Q - lo) / GRID_MM).astype(int)
    img = np.zeros((ij[:, 1].max() + 1, ij[:, 0].max() + 1), np.uint8)
    img[ij[:, 1], ij[:, 0]] = 1
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    nlab, lab = cv2.connectedComponents(img)
    best = None
    for k in range(1, nlab):
        m = lab[ij[:, 1], ij[:, 0]] == k
        if m.sum() < 30:
            continue
        w, l = sorted(cv2.minAreaRect(Q[m].astype(np.float32))[1])
        if not (DEPTH_SIDE_MM[0] <= w and l <= DEPTH_SIDE_MM[1]):
            continue
        # the most prism-like blob (20:19: a long object nearer the camera was taken by "nearest"): top ~60, square
        cost = abs(float(np.percentile(hs[m], 95)) - 60.0) / 10.0 + (1.0 - w / l) * 5.0
        if best is None or cost < best[0]:
            best = (cost, m)
    if best is None:
        return None
    m = best[1]
    top = float(np.percentile(hs[m], 95))
    tm = m & (np.abs(hs - top) < TOP_BAND_MM)
    if tm.sum() < 30:
        return None
    rays = Ps[tm] / Ps[tm][:, 2:3]
    pt = p0 + n * top / 1000  # a point of the top plane
    X = rays * ((pt @ n) / (rays @ n))[:, None]
    Qt = np.c_[(X - pt) @ ax, (X - pt) @ ay] * 1000
    raw, (w, l), (cx, cy) = rect_fit(Qt)
    r = np.radians(raw)
    e1, e2 = np.array([np.cos(r), np.sin(r)]) * w / 2, np.array([-np.sin(r), np.cos(r)]) * l / 2
    corners = [np.array([cx, cy]) + s1 * e1 + s2 * e2 for s1, s2 in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    return dict(raw_deg=raw, sides_mm=sorted([float(w), float(l)]), height_mm=top, pts=int(tm.sum()),
                centre_mm=((pt + (cx * ax + cy * ay) / 1000) * 1000).tolist(),
                corners3d=[(pt + (c[0] * ax + c[1] * ay) / 1000).tolist() for c in corners])


# ---------------------------------------------------------------- tag reading (cross-check)
def tags(bgr):
    """[(corners 4x2 px, tag id)] over three image scales, duplicates (same centre within 5 px) dropped."""
    global _DET
    if _DET is None:
        p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        _DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)
    g1 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    out = []
    for sc in (1.0, 0.5, 2.0):
        g = g1 if sc == 1.0 else cv2.resize(g1, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA if sc < 1 else cv2.INTER_CUBIC)
        cs, ids, _ = _DET.detectMarkers(g)
        for c, i in zip(cs, ids.ravel() if ids is not None else []):
            c = c.reshape(4, 2) / sc
            if all(np.linalg.norm(c.mean(0) - o[0].mean(0)) > 5 for o in out):
                out.append((c, int(i)))
    return out


def measure_tag(bgr, K, plane):
    """The top tag: dict(raw_deg, spread_deg, square, id, corners) or None. Top tag = the most square quad on the plane."""
    n = plane[0]
    ax, ay = plane_axes(n)
    best = None
    for c, i in tags(bgr):
        rays = np.c_[(c[:, 0] - K[0, 2]) / K[0, 0], (c[:, 1] - K[1, 2]) / K[1, 1], np.ones(4)]
        X = rays / (rays @ n)[:, None]  # on a plane parallel to the surface (directions only)
        e = np.roll(X, -1, 0) - X
        side = np.linalg.norm(e, axis=1)
        sq = side.min() / side.max()
        if best is not None and sq < best['square']:
            continue
        ang = np.degrees(np.arctan2(e @ ay, e @ ax))
        m = mean_fold(ang)
        best = dict(raw_deg=m, spread_deg=float(np.max(np.abs(fold(ang - m)))), square=float(sq), id=i, corners=c.tolist())
    return best


# ---------------------------------------------------------------- one frame
class Cam:
    """Intrinsics / extrinsic of one recording or capture."""
    def __init__(self, cam):
        self.K = K_of(cam['color_intrinsics'])
        self.Kd = K_of(cam['depth_intrinsics'])
        self.R = np.array(cam['depth_to_color']['rotation']).reshape(3, 3).T  # librealsense: column-major
        self.t = np.array(cam['depth_to_color']['translation'])
        self.scale = cam['depth_scale']


def median_depth(D):
    """Per-pixel median of raw depth frames over the valid (non-zero) ones: one steady image for the surface fit
    (a single frame's RANSAC once took a wrong plane, 2026-11-15)."""
    D = np.asarray(D, np.float32)
    D[D == 0] = np.nan
    import warnings
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)  # pixels with no depth in any frame
        m = np.nanmedian(D, axis=0)
    return np.nan_to_num(m, nan=0).astype(np.uint16)


def plane_of(D, cam):
    P, _, _ = to_colour_points(median_depth(D) * cam.scale, cam.Kd, cam.R, cam.t)
    return surface_plane(P)


def measure(bgr, d16, cam, plane=None):
    """Both readings of one frame (d16 raw depth units). plane: reuse (the camera stands still in a bag)."""
    P, _, _ = to_colour_points(d16 * cam.scale, cam.Kd, cam.R, cam.t)
    plane = plane or surface_plane(P)
    if plane is None:
        return dict(depth=None, tag=None, plane=None)
    return dict(depth=measure_depth(P, plane), tag=measure_tag(bgr, cam.K, plane) if bgr is not None else None,
                plane=plane, tilt_deg=float(np.degrees(np.arccos(-plane[0][2]))))


def project(cam, X):
    X = np.asarray(X)
    return np.c_[X[:, 0] / X[:, 2] * cam.K[0, 0] + cam.K[0, 2], X[:, 1] / X[:, 2] * cam.K[1, 1] + cam.K[1, 2]]


def draw(bgr, o, cam, text):
    v = bgr.copy()
    if o['depth']:
        c = project(cam, o['depth']['corners3d'])
        cv2.polylines(v, [c.astype(np.int32)], True, (255, 255, 0), 2)  # footprint (on the surface)
    if o['tag']:
        c = np.array(o['tag']['corners'])
        cv2.polylines(v, [c.astype(np.int32)], True, (0, 255, 0), 2)
    cv2.putText(v, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    return v


def line(o, cal):
    d, t = o['depth'], o['tag']
    s = (f"YAW {yaw(d['raw_deg'], cal):+.1f} deg (depth: raw {d['raw_deg']:+.1f}, footprint {d['sides_mm'][0]:.0f} x "
         f"{d['sides_mm'][1]:.0f} mm, top {d['height_mm']:.0f} mm, {d.get('frames', str(d['pts']) + ' pts')})" if d else 'YAW NONE (depth: no prism)')
    s += f" | tag {yaw(t['raw_deg'], cal):+.1f} deg" if t else ' | tag none'
    return s


# ---------------------------------------------------------------- bags
def bag_frames(path, step=1):
    """(bgr, raw depth u16) per colour frame of a grip_yaw_bag.py .db3 (nearest depth within 20 ms)."""
    db = sqlite3.connect(path)
    tid = {n: i for i, n in db.execute('select id, name from topics')}
    q = lambda n: db.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid[n],)).fetchall()
    col, dep = q(COL), q(DEP)
    dts = np.array([t for t, _ in dep])
    for k, (tc, bc) in enumerate(col):
        if k % step:
            continue
        j = int(np.argmin(np.abs(dts - tc)))
        if abs(dts[j] - tc) > 20e6:
            continue
        img = cv2.imdecode(np.frombuffer(cdr_compressed_image(bc)[1], np.uint8), cv2.IMREAD_COLOR)
        d16 = cv2.imdecode(np.frombuffer(cdr_compressed_image(dep[j][1])[1][12:], np.uint8), cv2.IMREAD_UNCHANGED)
        yield img, d16


def read_dir(dr, step=1, frames=False):
    """cell, {yaw_deg: [measure() per frame]} (frames: [(bgr, reading)])."""
    meta = json.load(open(os.path.join(dr, 'meta.json')))
    cam = Cam(meta['camera'])
    out = {}
    for s in meta['shots']:
        fr = list(bag_frames(os.path.join(dr, s['bag']), step))
        plane = plane_of([d for _, d in fr[:10]], cam) if fr else None
        rs = []
        for img, d16 in fr:
            o = measure(img, d16, cam, plane)
            rs.append((img, o) if frames else o)
        out[s['yaw_deg']] = rs
    return meta.get('cell'), out, cam


def fit(data):
    """OFFSET, SIGN, rms from {(cell, yaw): [raw]}: raw = offset + sign * yaw."""
    best = None
    for sign in (1, -1):
        res = [r - sign * y for (cell, y), raws in data.items() for r in raws]
        off = mean_fold(res)
        rms = float(np.sqrt(np.mean(fold(np.array(res) - off) ** 2)))
        if best is None or rms < best[2]:
            best = (off, sign, rms)
    return best


def report(dirs, cal, png=None):
    errs = {'depth': [], 'tag': []}
    for dr in dirs:
        cell, res, cam = read_dir(dr, frames=bool(png))
        print(f'\n{dr}  (cell {cell})')
        print('   yaw  frames | depth: found  mean    err    sd  footprint mm  top | tag: found  mean    err    sd')
        for y, rs in res.items():
            os_ = [o[1] for o in rs] if png else rs
            row = f'  {y:5.1f}  {len(os_):5d} |'
            for k in ('depth', 'tag'):
                ok = [o[k] for o in os_ if o[k]]
                if not ok:
                    row += f'        0   ---' + (' ' * 30 if k == 'depth' else ''); continue
                ys = np.array([yaw(o['raw_deg'], cal) for o in ok])
                m = mean_fold(ys)
                err = float(fold(m - y))
                errs[k].append(err)
                row += f'  {len(ok):5d}  {m:+6.1f}  {err:+5.1f}  {fold(ys - m).std():4.2f}'
                if k == 'depth':
                    sd = np.mean([o['sides_mm'] for o in ok], 0)
                    row += f'  {sd[0]:4.0f} x {sd[1]:3.0f}  {np.mean([o["height_mm"] for o in ok]):4.0f} |'
            print(row)
            if png and rs:
                os.makedirs(png, exist_ok=True)
                img, o = rs[0]
                cv2.imwrite(os.path.join(png, f'cell{cell}_yaw{int(round(y * 10)):04d}.png'),
                            draw(img, o, cam, f'cell {cell} set {y:g}  ' + line(o, cal).split(' (')[0]))
    for k, e in errs.items():
        if e:
            e = np.array(e)
            print(f'{k}: {len(e)} yaws, error rms {np.sqrt(np.mean(e ** 2)):.2f} max {np.max(np.abs(e)):.2f} deg')


def video(dirs, out, cal):
    vw = None
    for dr in dirs:
        cell, res, cam = read_dir(dr, frames=True)
        for y, rs in res.items():
            for img, o in rs:
                v = draw(img, o, cam, f'cell {cell} set {y:g}  ' + line(o, cal).split(' (')[0] +
                         (f"  tag {yaw(o['tag']['raw_deg'], cal):+.1f}" if o['tag'] else ''))
                v = cv2.resize(v, (v.shape[1] // 2, v.shape[0] // 2))
                if vw is None:
                    vw = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*'mp4v'), 15, (v.shape[1], v.shape[0]))
                vw.write(v)
        print(dr, 'done', flush=True)
    vw.release()
    print(out)


# ---------------------------------------------------------------- live
LIVE_FRAMES = 15  # --capture averages this many frames (bags: 1-2 deg per frame, the error is not per-frame noise only)


def capture(dw=640, dh=360, warm=30, frames=LIVE_FRAMES):
    """Colour 1280x720 + LIVE_FRAMES depth dw x dh frames from the gripper cam; saved under out/gripyaw_HHMMSS_*
    (_color.png = the last colour frame, _depth.npy = (frames, h, w) raw depth)."""
    import pyrealsense2 as rs
    cfg = rs.config(); cfg.enable_device(GRIP_SERIAL)
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, 30)
    pipe = rs.pipeline(); prof = pipe.start(cfg)
    try:
        for _ in range(warm):  # auto exposure settles
            fs = pipe.wait_for_frames(2000)
        cs = prof.get_stream(rs.stream.color).as_video_stream_profile()
        ds = prof.get_stream(rs.stream.depth).as_video_stream_profile()
        ex = ds.get_extrinsics_to(cs)
        intr = lambda p: dict(width=p.width, height=p.height, fx=p.fx, fy=p.fy, ppx=p.ppx, ppy=p.ppy)
        cam = dict(color_intrinsics=intr(cs.get_intrinsics()), depth_intrinsics=intr(ds.get_intrinsics()),
                   depth_to_color=dict(rotation=list(ex.rotation), translation=list(ex.translation)),
                   depth_scale=prof.get_device().first_depth_sensor().get_depth_scale())
        ds_ = []
        for _ in range(frames):
            fs = pipe.wait_for_frames(2000)
            ds_.append(np.asanyarray(fs.get_depth_frame().get_data()).copy())
        bgr = np.asanyarray(fs.get_color_frame().get_data()).copy()
        d16 = np.stack(ds_)
    finally:
        pipe.stop()
    p = os.path.join(HERE, 'out', time.strftime('gripyaw_%H%M%S'))
    os.makedirs(os.path.dirname(p), exist_ok=True)
    cv2.imwrite(p + '_color.png', bgr); np.save(p + '_depth.npy', d16)
    json.dump(cam, open(p + '_cam.json', 'w'))
    return p


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    for opt in ('--png', '--video', '--by'):
        if opt in sys.argv:
            args.remove(sys.argv[sys.argv.index(opt) + 1])
    png = sys.argv[sys.argv.index('--png') + 1] if '--png' in sys.argv else None
    if '--video' in sys.argv:
        video(args, sys.argv[sys.argv.index('--video') + 1], calibration()); return
    if '--calibrate' in sys.argv:
        by = sys.argv[sys.argv.index('--by') + 1] if '--by' in sys.argv else 'depth'
        data = {}
        for dr in args:
            cell, res, _ = read_dir(dr)
            for y, rs in res.items():
                data[(cell, y)] = [o[by]['raw_deg'] for o in rs if o[by]]
        if not any(data.values()):
            print(f'no {by} readings in these bags'); return
        off, sign, rms = fit(data)
        cal = dict(offset_deg=round(off, 3), sign=sign, rms_deg=round(rms, 3), by=by, dirs=args,
                   stamp=time.strftime('%Y-%m-%d %H:%M'),
                   note='yaw = sign * (raw - offset) folded to [-45, 45); raw from the camera x axis in the surface plane, CCW from above')
        json.dump(cal, open(CAL, 'w'), indent=1)
        print(f'{CAL}: offset {off:+.2f} deg, sign {sign:+d}, rms {rms:.2f} deg ({by})')
        report(args, cal, png); return
    if '--capture' in sys.argv:
        args.append(capture())
    dirs = [a for a in args if os.path.isdir(a)]
    if dirs:
        report(dirs, calibration(), png)
    cal = calibration()
    for p in [a for a in args if not os.path.isdir(a)]:  # a capture prefix
        cam = Cam(json.load(open(p + '_cam.json')))
        bgr = cv2.imread(p + '_color.png')
        D = np.load(p + '_depth.npy')
        D = D[None] if D.ndim == 2 else D
        o = measure(bgr, D[-1], cam, plane_of(D, cam))
        raws = [o['depth']['raw_deg']] if o['depth'] else []
        for d16 in D[:-1]:
            r = measure(None, d16, cam, o['plane'])['depth'] if o['plane'] else None
            if r:
                raws.append(r['raw_deg'])
        if raws and not o['depth']:  # the last frame missed: draw / report from another
            o['depth'] = next(r for r in (measure(None, d16, cam, o['plane'])['depth'] for d16 in D[:-1]) if r)
        if o['depth'] and len(raws) > 1:  # the reading = the circular mean over the frames
            o['depth']['raw_deg'] = mean_fold(raws)
            o['depth']['frames'] = f"{len(raws)}/{len(D)} frames, sd {np.std(fold(np.array(raws) - mean_fold(raws))):.1f} deg"
        print(f'{p}: {line(o, cal)}')
        cv2.imwrite(p + '_yaw.png', draw(bgr, o, cam, line(o, cal).split(' (')[0]))


if __name__ == '__main__':
    main()
