#!/usr/bin/python3
"""Batched hypothesis evaluation variant; original preserved in uppercam_box_range.py.

Box top by height-aware RANSAC (no floor/table plane needed) and the distance to the box from its known 205 mm square.

  /usr/bin/python3 box_range.py --capture [--serial 261322075477]      # grab a frame with build/rs_snap first
  /usr/bin/python3 box_range.py out/<prefix> [out/<prefix> ...]       # or analyse saved rs_snap captures
  options: --cam-height 0.627 --cam-pitch 43.6 (deg, optical axis below horizontal) --lidar-ahead 0.2495 --png

Why: on a carpet the floor gives no clean plane, so the floor is never fitted. The camera's mounting is the prior:
"up" in camera coordinates is u = (0, -cos(pitch), -sin(pitch)) (optical frame: x right, y down, z forward), and a
point's height over the floor is z = cam_height + u.P. A box top lies at z = 70 mm with its normal along u.

1. depth -> 3D points P = ((u-cx)Z/fx, (v-cy)Z/fy, Z), lower half of the image only (the box is handled within
   40 cm of the LiDAR face, always below the middle of the view), and nothing beyond 40 cm + 205 mm ahead
2. candidates: prior height 70 +- 35 mm (carpet, robot parts and the room drop out)
3. RANSAC on the candidates: planes within 12 deg of u and 70 +- 40 mm high; score = inliers (6 mm) in the
   largest connected blob, kept only if that blob's real area is 0.5-1.6 x 205^2
4. that blob = box top; plane refit (SVD on the inner pixels, outliers trimmed)
5. outline pixels cast onto the plane -> minimum-area rectangle (orientation from the hull, sides from the 1-99 %
   spread) -> 4 outer corners
6. distance, two ways, from the same corners:
   a) depth:      camera -> rectangle centre on the depth plane
   The near edge can be hidden (robot front, image border), the far edge never is: a seen side shorter than
   0.95 x 205 mm is cut, so the square is completed by laying 205 mm off the far edge toward the camera.
   a) square (depth):      centre of the completed square on the depth plane
   b) square (205 scale):  the corner rays do not depend on how far the plane is; the plane distance that makes
      the far edge 205 mm is D * 205 / far_edge (no depth scale)
   c) square (PnP):        cv2.solvePnP(IPPE_SQUARE) of a 205 mm square on the completed square's image corners
   ahead / right / top height use the mounting prior (up vector, camera height).
"""
import argparse
import copy
import os
import subprocess
import sys

import cv2
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull

def _fill_holes(mask):
    padded = np.pad(mask.astype(np.uint8), 1)
    cv2.floodFill(padded, None, (0, 0), 2, flags=4)
    return padded[1:-1, 1:-1] != 2


def _label(mask):
    count, labels, stats, _ = cv2.connectedComponentsWithStatsWithAlgorithm(
        mask.astype(np.uint8), 4, cv2.CV_32S, cv2.CCL_SAUF)
    return labels, count - 1


_HYPOTHESIS_CACHE = {}
_CROSS = np.array([[0,1,0],[1,1,1],[0,1,0]], dtype=np.uint8)
SIDE = 0.205
BOX_H = 0.070
HERE = os.path.dirname(os.path.abspath(__file__))
URDF_DEFAULT_CLI = os.path.normpath(os.path.join(HERE, '..', '..', 'src', 'craft_description', 'urdf', 'v307_full_robot.urdf'))


def load(prefix):
    w, h, fx, fy, cx, cy, sc, ser = open(prefix + '_meta.txt').read().split()
    w, h = int(w), int(h)
    d = np.fromfile(prefix + '_depth.u16', dtype='<u2').reshape(h, w) * float(sc)
    return d, np.array([[float(fx), 0, float(cx)], [0, float(fy), float(cy)], [0, 0, 1]]), ser


RANSAC = dict(band=0.030, inlier=0.005, tilt_deg=8.0, height_tol=0.030, area=(0.4, 1.4), iters=500, refine=True,
              edge_drop=0.010)   # edge where the depth drops 10 mm below the top: series mean -0.9 mm, width 204 mm
RANSAC_LOOSE = dict(band=0.035, inlier=0.006, tilt_deg=12.0, height_tol=0.04, area=(0.5, 1.6), iters=250, refine=None)


def box_top(d, K, up, cam_h, rng, roi_from=0.5, max_ahead=None, prm=None):
    """roi_from: only image rows below this fraction of the height are used (the box is only ever handled within
    40 cm, where it sits in the lower half of the view). max_ahead [m]: candidates no farther ahead (along the
    floor) than this. prm: RANSAC gates (RANSAC strict by default, RANSAC_LOOSE = the first version)."""
    prm = prm or RANSAC
    H, w = d.shape
    r0 = int(H * roi_from)
    full_shape = d.shape
    d = d[r0:]
    K = K.copy(); K[1, 2] -= r0                                      # same rays, rows shifted
    h = d.shape[0]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    P = np.dstack([(uu - cx) * d / fx, (vv - cy) * d / fy, d])
    valid = d > 0
    z = np.where(valid, cam_h + P @ up, np.nan)                      # prior height over the floor
    cand = valid & (np.abs(z - BOX_H) < prm['band'])
    if max_ahead is not None:
        fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd)
        cand &= (P @ fwd) < max_ahead
    area_px = np.where(valid, (d / fx) * (d / fy), 0.0)
    idx = np.argwhere(cand)
    if len(idx) < 300:
        raise RuntimeError('no points near the box-top height (check --cam-height / --cam-pitch)')
    pts = P[idx[:, 0], idx[:, 1]]
    # Hypotheses are scored on a sample of the candidates (fast); only the best few get the full-image blob check.
    samp = pts[rng.choice(len(pts), min(4000, len(pts)), replace=False)]
    # Keep the identical RNG call sequence and stable candidate order. Score
    # candidate planes in batches to avoid a Python loop over every hypothesis.
    cache_key = (len(samp), prm['iters'], repr(rng.bit_generator.state))
    cached = _HYPOTHESIS_CACHE.get(cache_key)
    if cached is None:
        indices = np.array([rng.choice(len(samp), 3, replace=False) for _ in range(prm['iters'])])
        if len(_HYPOTHESIS_CACHE) >= 128:
            _HYPOTHESIS_CACHE.pop(next(iter(_HYPOTHESIS_CACHE)))
        _HYPOTHESIS_CACHE[cache_key] = (indices, copy.deepcopy(rng.bit_generator.state))
    else:
        indices, state_after = cached
        rng.bit_generator.state = copy.deepcopy(state_after)
    a, b, c = (samp[indices[:, i]] for i in range(3))
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1)
    ok = lengths >= 1e-9
    normals[ok] /= lengths[ok, None]
    flipped = (normals @ up) < 0
    normals[flipped] *= -1
    ok &= np.degrees(np.arccos(np.clip(normals @ up, -1, 1))) <= prm['tilt_deg']
    ok &= np.abs(cam_h + a @ up - BOX_H) <= prm['height_tol']
    selected = np.flatnonzero(ok)
    hyps = []
    # Bound the temporary matrix's size on Jetson.
    for start in range(0, len(selected), 64):
        ix = selected[start:start + 64]
        ns = normals[ix]
        residuals = samp @ ns.T - np.einsum('ij,ij->i', a[ix], ns)
        scores = np.sum(np.abs(residuals) < prm['inlier'], axis=0)
        hyps.extend((int(score), a[i], normals[i]) for score, i in zip(scores, ix))
    hyps.sort(key=lambda t: -t[0])
    best = None
    for _, a, n in hyps[:6]:
        on = np.zeros_like(cand)
        on[idx[:, 0], idx[:, 1]] = np.abs((pts - a) @ n) < prm['inlier']
        lab, k = _label(cv2.morphologyEx(on.astype(np.uint8), cv2.MORPH_OPEN, _CROSS, iterations=2, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool))
        if not k:
            continue
        sizes = np.bincount(lab.ravel(), minlength=k + 1)[1:]
        blob = lab == (int(np.argmax(sizes)) + 1)
        cos = max(abs(n @ (P[blob].mean(0) / np.linalg.norm(P[blob].mean(0)))), 0.2)
        area = area_px[blob].sum() / cos
        if prm['area'][0] * SIDE ** 2 < area < prm['area'][1] * SIDE ** 2 and (best is None or blob.sum() > best[0]):
            best = (blob.sum(), blob, area)
    if best is None:
        raise RuntimeError('no level blob of box-top size at box-top height')
    top = _fill_holes(cv2.morphologyEx(best[1].astype(np.uint8), cv2.MORPH_CLOSE, _CROSS, iterations=3, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool))
    if r0:                                                           # back to full-image rows
        full = np.zeros(full_shape, bool); full[r0:] = top
        P = np.vstack([np.zeros((r0, w, 3)), P]); valid = np.vstack([np.zeros((r0, w), bool), valid])
        best = (best[0], np.vstack([np.zeros((r0, w), bool), best[1]]), best[2]); top = full
    inner = cv2.erode(top.astype(np.uint8), _CROSS, iterations=8, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool) & valid
    X = P[inner]; cen = X.mean(0); n = np.linalg.svd(X - cen, full_matrices=False)[2][2]
    for _ in range(3):
        r = np.abs((X - cen) @ n); X = X[r < 3 * r.std() + 1e-4]; cen = X.mean(0); n = np.linalg.svd(X - cen, full_matrices=False)[2][2]
    if n @ (-cen) < 0:
        n = -n
    if prm.get('refine'):
        # Edges at half height: a pixel belongs to the top while it is above the middle between the top plane and the
        # floor (-35 mm) and not above the top (+20 mm: robot parts over the box). Edge pixels whose depth smears
        # between top and floor are split at the half-height crossing instead of being cut away, so the outline
        # (far edge first) is not pulled in. Only around the RANSAC blob (15 px), connected to it.
        hrel = np.where(valid, (P - cen) @ n, np.nan)
        on = valid & (hrel > -prm.get('edge_drop', BOX_H / 2)) & (hrel < 0.02) & cv2.dilate(top.astype(np.uint8), _CROSS, iterations=15, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        lab, k = _label(on)
        ids = np.unique(lab[top & (lab > 0)])
        if len(ids):
            sizes = np.bincount(lab.ravel())[ids]
            top = _fill_holes(cv2.morphologyEx((lab == ids[int(np.argmax(sizes))]).astype(np.uint8), cv2.MORPH_CLOSE, _CROSS, iterations=2, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool))
    return top, cen, n, best[2], np.abs((X - cen) @ n).std()


def _window(rows, cols, margin, shape, row_min=0):
    """Rows/cols slices of the bounding box of (rows, cols) grown by margin, clipped to [row_min, H) x [0, W)."""
    return (slice(max(int(rows.min()) - margin, row_min), min(int(rows.max()) + margin + 1, shape[0])),
            slice(max(int(cols.min()) - margin, 0), min(int(cols.max()) + margin + 1, shape[1])))


def box_top_cropped(d, K, up, cam_h, rng, roi_from=0.5, max_ahead=None, prm=None):
    """box_top with the morphology, labelling and refit done on windows around the box instead of whole images.

    Same output as box_top, bit for bit (uppercam_profile.py --compare): every window keeps a margin wider than the
    reach of its morphology, so a window border is either the image border box_top sees too or a ring of zeros
    the operation never reaches. The blob steps run on the lower-half ROI as box_top does (its upper border is a
    real border there); the refit and the refine run in full-image rows as box_top does after padding (rows above
    the ROI are zero / invalid, not a border). (2026-11-10: box_top was 70 of the 89 ms per frame on the Jetson.)
    """
    prm = prm or RANSAC
    H, w = d.shape
    r0 = int(H * roi_from)
    d = d[r0:]
    K = K.copy(); K[1, 2] -= r0
    h = d.shape[0]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    vv, uu = np.mgrid[0:h, 0:w]
    P = np.dstack([(uu - cx) * d / fx, (vv - cy) * d / fy, d])
    valid = d > 0
    z = np.where(valid, cam_h + P @ up, np.nan)
    cand = valid & (np.abs(z - BOX_H) < prm['band'])
    if max_ahead is not None:
        fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd)
        cand &= (P @ fwd) < max_ahead
    area_px = np.where(valid, (d / fx) * (d / fy), 0.0)
    idx = np.argwhere(cand)
    if len(idx) < 300:
        raise RuntimeError('no points near the box-top height (check --cam-height / --cam-pitch)')
    pts = P[idx[:, 0], idx[:, 1]]
    samp = pts[rng.choice(len(pts), min(4000, len(pts)), replace=False)]
    cache_key = (len(samp), prm['iters'], repr(rng.bit_generator.state))
    cached = _HYPOTHESIS_CACHE.get(cache_key)
    if cached is None:
        indices = np.array([rng.choice(len(samp), 3, replace=False) for _ in range(prm['iters'])])
        if len(_HYPOTHESIS_CACHE) >= 128:
            _HYPOTHESIS_CACHE.pop(next(iter(_HYPOTHESIS_CACHE)))
        _HYPOTHESIS_CACHE[cache_key] = (indices, copy.deepcopy(rng.bit_generator.state))
    else:
        indices, state_after = cached
        rng.bit_generator.state = copy.deepcopy(state_after)
    a, b, c = (samp[indices[:, i]] for i in range(3))
    normals = np.cross(b - a, c - a)
    lengths = np.linalg.norm(normals, axis=1)
    ok = lengths >= 1e-9
    normals[ok] /= lengths[ok, None]
    flipped = (normals @ up) < 0
    normals[flipped] *= -1
    ok &= np.degrees(np.arccos(np.clip(normals @ up, -1, 1))) <= prm['tilt_deg']
    ok &= np.abs(cam_h + a @ up - BOX_H) <= prm['height_tol']
    selected = np.flatnonzero(ok)
    hyps = []
    for start in range(0, len(selected), 64):
        ix = selected[start:start + 64]
        ns = normals[ix]
        residuals = samp @ ns.T - np.einsum('ij,ij->i', a[ix], ns)
        scores = np.sum(np.abs(residuals) < prm['inlier'], axis=0)
        hyps.extend((int(score), a[i], normals[i]) for score, i in zip(scores, ix))
    hyps.sort(key=lambda t: -t[0])
    best = None
    for _, a, n in hyps[:6]:
        inl = np.abs((pts - a) @ n) < prm['inlier']
        if not inl.any():
            continue
        rs, cs = _window(idx[inl, 0], idx[inl, 1], 4, (h, w))        # open (2 x cross) stays inside the inliers
        on = np.zeros((rs.stop - rs.start, cs.stop - cs.start), bool)
        on[idx[inl, 0] - rs.start, idx[inl, 1] - cs.start] = True
        lab, k = _label(cv2.morphologyEx(on.astype(np.uint8), cv2.MORPH_OPEN, _CROSS, iterations=2, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool))
        if not k:
            continue
        sizes = np.bincount(lab.ravel(), minlength=k + 1)[1:]
        blob = lab == (int(np.argmax(sizes)) + 1)
        Pb = P[rs, cs][blob]
        cos = max(abs(n @ (Pb.mean(0) / np.linalg.norm(Pb.mean(0)))), 0.2)
        area = area_px[rs, cs][blob].sum() / cos
        if prm['area'][0] * SIDE ** 2 < area < prm['area'][1] * SIDE ** 2 and (best is None or blob.sum() > best[0]):
            best = (blob.sum(), blob, area, rs, cs)
    if best is None:
        raise RuntimeError('no level blob of box-top size at box-top height')
    br_, bc_ = np.nonzero(best[1])
    rs, cs = _window(br_ + best[3].start, bc_ + best[4].start, 8, (h, w))  # close (3 x cross) + hole fill
    blob = np.zeros((rs.stop - rs.start, cs.stop - cs.start), np.uint8)
    blob[br_ + best[3].start - rs.start, bc_ + best[4].start - cs.start] = 1
    top_w = _fill_holes(cv2.morphologyEx(blob, cv2.MORPH_CLOSE, _CROSS, iterations=3, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool))
    tr, tc = np.nonzero(top_w); tr = tr + rs.start + r0; tc = tc + cs.start  # full-image rows from here on

    def full_window(margin):
        """Window in full-image rows; rows above the ROI come back zero / invalid, as in box_top's padding."""
        rows, cols = _window(tr, tc, margin, (H, w))
        lo = max(rows.start, r0)
        Pw = np.zeros((rows.stop - rows.start, cols.stop - cols.start, 3)); Pw[lo - rows.start:] = P[lo - r0:rows.stop - r0, cols]
        vw = np.zeros(Pw.shape[:2], bool); vw[lo - rows.start:] = valid[lo - r0:rows.stop - r0, cols]
        tw = np.zeros(Pw.shape[:2], bool); tw[tr - rows.start, tc - cols.start] = True
        return rows, cols, Pw, vw, tw

    rows, cols, Pw, vw, tw = full_window(10)                         # erode 8 x cross
    inner = cv2.erode(tw.astype(np.uint8), _CROSS, iterations=8, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool) & vw
    X = Pw[inner]; cen = X.mean(0); n = np.linalg.svd(X - cen, full_matrices=False)[2][2]
    for _ in range(3):
        r = np.abs((X - cen) @ n); X = X[r < 3 * r.std() + 1e-4]; cen = X.mean(0); n = np.linalg.svd(X - cen, full_matrices=False)[2][2]
    if n @ (-cen) < 0:
        n = -n
    if prm.get('refine'):
        rows, cols, Pw, vw, tw = full_window(20)                     # dilate 15, then close 2 x cross + hole fill
        hrel = np.where(vw, (Pw - cen) @ n, np.nan)
        on = vw & (hrel > -prm.get('edge_drop', BOX_H / 2)) & (hrel < 0.02) & cv2.dilate(tw.astype(np.uint8), _CROSS, iterations=15, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
        lab, k = _label(on)
        ids = np.unique(lab[tw & (lab > 0)])
        if len(ids):
            sizes = np.bincount(lab.ravel())[ids]
            tw = _fill_holes(cv2.morphologyEx((lab == ids[int(np.argmax(sizes))]).astype(np.uint8), cv2.MORPH_CLOSE, _CROSS, iterations=2, borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool))
    top = np.zeros((H, w), bool); top[rows, cols] = tw
    return top, cen, n, best[2], np.abs((X - cen) @ n).std()


def rectangle(top, K, cen, n):
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    edge = top & ~ndimage.binary_erosion(top)
    er, ec = np.nonzero(edge)
    ray = np.column_stack([(ec - cx) / fx, (er - cy) / fy, np.ones(len(er))])
    E3 = ray * ((cen @ n) / (ray @ n))[:, None]                      # outline cast on the plane (no edge depth)
    e1 = np.cross(n, [1.0, 0, 0]); e1 /= np.linalg.norm(e1); e2 = np.cross(n, e1)
    E2 = np.column_stack([(E3 - cen) @ e1, (E3 - cen) @ e2])
    hull = E2[ConvexHull(E2).vertices]
    best = None
    for i in range(len(hull)):
        t = hull[(i + 1) % len(hull)] - hull[i]; t /= np.linalg.norm(t); q = np.array([-t[1], t[0]])
        area = np.ptp(hull @ t) * np.ptp(hull @ q)
        if best is None or area < best[0]:
            best = (area, t, q)
    _, t, q = best
    a0, a1 = np.percentile(E2 @ t, [1, 99]); b0, b1 = np.percentile(E2 @ q, [1, 99])
    r2 = np.array([t * a0 + q * b0, t * a1 + q * b0, t * a1 + q * b1, t * a0 + q * b1])
    C3 = cen + r2[:, :1] * e1 + r2[:, 1:] * e2
    px = np.column_stack([fx * C3[:, 0] / C3[:, 2] + cx, fy * C3[:, 1] / C3[:, 2] + cy])
    return C3, px


def complete_square(C3, up, cam_pos=np.zeros(3)):
    """The 205 mm square from the seen rectangle. The far edge and the sides reaching it are always in view; the
    near edge can be hidden (robot front, image border). A side shorter than 0.95 x 205 mm is cut: keep the far
    edge and lay 205 mm off it toward the camera. Returns the 4 square corners (far-left, far-right, near-right,
    near-left as seen) and which dimension was cut."""
    fwd = np.array([0, 0, 1.0]) - (np.array([0, 0, 1.0]) @ up) * up; fwd /= np.linalg.norm(fwd)
    ahead = C3 @ fwd
    far = np.argsort(ahead)[-2:]                                      # the two corners farthest ahead
    a, b = C3[far[0]], C3[far[1]]
    along = b - a; width = np.linalg.norm(along); along /= width
    toward = np.cross(up, along)
    if toward @ fwd > 0:
        toward = -toward                                              # from the far edge toward the camera
    others = np.setdiff1d(np.arange(4), far)
    depth_seen = np.mean([(C3[i] - (a + b) / 2) @ toward for i in others])
    mid = (a + b) / 2
    # width: the far edge is complete; if it reads short (cut by the image side) keep its centre, set 205
    half = along * SIDE / 2
    fl, fr = mid - half, mid + half
    cut = []
    if width < 0.95 * SIDE:
        cut.append(f'width seen {width * 1000:.0f}')
    if depth_seen < 0.95 * SIDE:
        cut.append(f'depth seen {depth_seen * 1000:.0f}')
    nr, nl = fr + toward * SIDE, fl + toward * SIDE
    return np.array([fl, fr, nr, nl]), cut, width, depth_seen


def geometry(C3, px, K, up, cam_h, n_top=None):
    """n_top (optional, off by default): use the measured box-top normal as 'up'. Tried 2026-11-07: its ~1 deg
    frame-to-frame noise moves 'ahead' by ~10 mm at 0.68 m, worse than the fixed mounting prior."""
    up_prior = up
    if n_top is not None:
        up = n_top if n_top @ up_prior > 0 else -n_top
    sides = np.array([np.linalg.norm(C3[(i + 1) % 4] - C3[i]) for i in range(4)])
    S, cut, width, depth_seen = complete_square(C3, up)
    fwd = np.array([0, 0, 1.0]) - (np.array([0, 0, 1.0]) @ up) * up; fwd /= np.linalg.norm(fwd)
    right = -np.cross(up, fwd)                                        # optical +x is the robot's right
    # a) depth plane, square completed from the far edge (205 mm assumption on the cut side only)
    c_sq = S.mean(0)
    # b) 205 scale: the far edge (always complete) must be 205 mm -> move the plane, same rays
    c_scale = c_sq * (SIDE / width)
    # c) PnP of the completed square's image corners, no depth
    Kf = K.astype(np.float64)
    img = np.column_stack([Kf[0, 0] * S[:, 0] / S[:, 2] + Kf[0, 2], Kf[1, 1] * S[:, 1] / S[:, 2] + Kf[1, 2]])
    obj = np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * SIDE / 2
    ok, rvec, tvec = cv2.solvePnP(obj, img, Kf, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    c_pnp = tvec.ravel() if ok else np.full(3, np.nan)
    out = {'seen centre': C3.mean(0), 'square (depth)': c_sq, 'square (205 scale)': c_scale, 'square (PnP)': c_pnp}
    res = {k: dict(dist=np.linalg.norm(c), ahead=c @ fwd, right=c @ right, height=cam_h + c @ up_prior) for k, c in out.items()}
    return sides, cut, width, depth_seen, res, S


MAX_GAP = 0.40     # boxes are only handled with the near edge within 40 cm of the LiDAR face
LIDAR_FACE_TO_CENTRE = 0.019   # LiDAR scan centre (rotating head) behind its front face, URDF meshes (y 33.3 -> 52.4 mm)


LIDAR_FACE_Z = 0.030          # [m] LiDAR front face centre height (URDF body z 19-40 mm, mounted low)


def near_edge_from_lidar_face(S, C3, up, cam_h, lidar_ahead, lidar_right=0.0):
    """Frame: origin on the floor under the camera, x ahead, y left, z up [m].
    (x0, y0, z0) = LiDAR front face centre; (x1, y1, z1) = centre of the box top's near edge (of the 205 mm square:
    the true near edge, also when the camera saw it cut); the answer is the XY length between them."""
    fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd); right = -np.cross(up, fwd)
    xyz = lambda Q: np.column_stack([Q @ fwd, -(Q @ right), cam_h + Q @ up])
    p0 = np.array([lidar_ahead, -lidar_right, LIDAR_FACE_Z])
    sq = xyz(S); near_sq = sq[np.argsort(sq[:, 0])[:2]].mean(0)       # the two corners nearest the robot
    seen = xyz(C3); near_seen = seen[np.argsort(seen[:, 0])[:2]].mean(0)
    return dict(p0=p0, p1=near_sq, p1_seen=near_seen, xy=np.hypot(*(near_sq[:2] - p0[:2])),
                xy_seen=np.hypot(*(near_seen[:2] - p0[:2])))


def xy_from_lidar(S, up, lidar_ahead, lidar_right=0.0):
    """Top-down (along Z) view on the floor plane: LiDAR scan centre -> box. Returns dict with the box centre
    (x ahead, y left of the LiDAR centre), the centre distance and the distance to the nearest point of the square."""
    fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd)
    right = -np.cross(up, fwd)
    lid = np.array([lidar_ahead - LIDAR_FACE_TO_CENTRE, lidar_right])            # (ahead, right) from the camera
    Q = np.column_stack([S @ fwd, S @ right]) - lid                              # square corners, LiDAR frame
    c = Q.mean(0)
    def seg(p, a, b):
        t = np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1); return np.linalg.norm(p - (a + t * (b - a)))
    inside = all(np.cross(Q[(i + 1) % 4] - Q[i], -Q[i]) * np.sign(np.cross(Q[1] - Q[0], Q[2] - Q[1])) >= 0 for i in range(4))
    near = 0.0 if inside else min(seg(np.zeros(2), Q[i], Q[(i + 1) % 4]) for i in range(4))
    return dict(x=c[0], y=-c[1], centre=np.hypot(*c), nearest=near)


def analyse(prefix, cam_h, pitch_deg, png, lidar_ahead=None, lidar_right=0.0):
    d, K, ser = load(prefix)
    t = np.radians(pitch_deg)
    up = np.array([0.0, -np.cos(t), -np.sin(t)])
    top, cen, n, area, rms = box_top(d, K, up, cam_h, np.random.default_rng(0), 0.5,
                                     None if lidar_ahead is None else lidar_ahead + MAX_GAP + SIDE)
    C3, px = rectangle(top, K, cen, n)
    sides, cut, width, depth_seen, g, S = geometry(C3, px, K, up, cam_h)
    tilt = np.degrees(np.arccos(np.clip(n @ up, -1, 1)))
    print(f'{prefix} (D435 {ser})')
    print(f'  box top: {top.sum()} px, area {area * 1e4:.0f} cm^2 (205^2 = 420), plane rms {rms * 1000:.1f} mm, '
          f'{tilt:.1f} deg off the prior up')
    print(f'  seen rectangle: far edge {width * 1000:.0f} mm, depth {depth_seen * 1000:.0f} mm'
          + (f'  -> cut ({", ".join(cut)}): square completed from the far edge' if cut else '  -> complete'))
    print(f'  {"":20}{"camera->centre":>15}{"ahead":>9}{"right":>9}{"top height":>12}   (m; prior {cam_h:.3f} m, {pitch_deg:.0f} deg)')
    for k, v in g.items():
        print(f'  {k:20}{v["dist"]:15.3f}{v["ahead"]:9.3f}{v["right"]:9.3f}{v["height"]:12.3f}')
    if lidar_ahead is not None:
        gap = g['square (depth)']['ahead'] - SIDE / 2 - lidar_ahead
        ne = near_edge_from_lidar_face(S, C3, up, cam_h, lidar_ahead, lidar_right)
        f3 = lambda q: f'({q[0] * 1000:.0f}, {q[1] * 1000:+.0f}, {q[2] * 1000:.0f})'
        print(f'  LiDAR face (x0,y0,z0) {f3(ne["p0"])} mm, box near-edge centre (x1,y1,z1) {f3(ne["p1"])} mm'
              + (f' [seen, cut: {f3(ne["p1_seen"])}]' if cut else ''))
        print(f'  => |(x1,y1) - (x0,y0)| = {ne["xy"] * 1000:.0f} mm')
        g['gap'] = gap; g['near_edge'] = ne
    if png:
        import matplotlib; matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from PIL import Image
        rgb = np.asarray(Image.open(prefix + '_color.ppm').convert('RGB')) / 255.0
        ov = rgb * 0.5; ov[top] = np.array([1, .55, 0]) * .8 + rgb[top] * .2
        plt.figure(figsize=(8, 4.6), dpi=110); plt.imshow(ov)
        q = np.vstack([px, px[:1]]); plt.plot(q[:, 0], q[:, 1], 'r-', lw=2); plt.axis('off')
        plt.title(f'box top (height-aware RANSAC) + 205 mm square: {g["square (depth)"]["dist"]:.3f} m to the centre')
        plt.tight_layout(); plt.savefig(prefix + '_range.png'); plt.close()
        print(f'  figure: {prefix}_range.png')
    return g


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('prefix', nargs='*')
    ap.add_argument('--capture', action='store_true')
    ap.add_argument('--serial', default='261322075477')
    # Floor fit 2026-11-07 (floor_pose.py, two captures): 626.5 / 626.8 mm, pitch 43.58 / 43.62 deg, roll 0.0; tape 625.
    ap.add_argument('--cam-height', type=float, default=0.627, help='camera height over the floor [m]')
    ap.add_argument('--cam-pitch', type=float, default=43.6, help='optical axis below horizontal [deg]')
    ap.add_argument('--png', action='store_true', help='also save <prefix>_range.png (not opened)')
    ap.add_argument('--urdf', nargs='?', const=URDF_DEFAULT_CLI, default=None,
                    help='use the robot URDF (V307 by default) for the camera pose and front_laser; results in base_link')
    ap.add_argument('--lens-y', type=float, default=None, help='URDF mode: override the camera optical y in base_link [m]')
    ap.add_argument('--cam-yaw', type=float, default=CAM_YAW_DEG, help='URDF mode: camera yaw vs the chassis [deg], + = left')
    ap.add_argument('--urdf-height', type=float, default=CAM_HEIGHT, help='URDF mode: camera height over the floor [m]')
    # LiDAR face ahead of the camera along the floor. 2026-11-07: 0.2495 m with the old 44 deg prior; with the floor
    # pose (0.627 m, 43.6 deg) the same series_010mm shot (tape 10 mm, box on the desk) gives 0.2525 m. Provisional:
    # re-take one tape-10 mm shot on the carpet, and re-calibrate whenever the camera mount moves.
    ap.add_argument('--lidar-ahead', type=float, default=0.2525, help='LiDAR face ahead of the camera along the floor [m]')
# D435 body centre (1/4-20) = LiDAR centre laterally (user). The colour lens is 32.2 mm left of it: datasheet
    # 337029-005 table 4-15, 1/4-20 centreline to left imager 17.5 mm; factory extrinsic colour -> left imager
    # +14.74 mm (261322075477). So the LiDAR centre is 0.0322 m right of the colour optical axis.
    ap.add_argument('--lidar-right', type=float, default=0.0322,
                    help='LiDAR centre right of the colour camera axis on the floor [m]')
    a = ap.parse_args()
    prefixes = list(a.prefix)
    if a.capture:
        import time
        p = os.path.join(HERE, 'out', time.strftime('range_%H%M%S'))
        subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), p, '640', '360', a.serial], check=True)
        prefixes.append(p)
    if not prefixes:
        ap.error('give capture prefixes or --capture')
    for p in prefixes:
        try:
            if a.urdf:
                analyse_urdf(p, a.urdf, a.lens_y, yaw_deg=a.cam_yaw, cam_height=a.urdf_height); continue
            analyse(p, a.cam_height, a.cam_pitch, a.png, a.lidar_ahead, a.lidar_right)
        except RuntimeError as e:
            print(f'{p}: {e}')




# ---------------------------------------------------------------------------------------------------------------
# URDF mode (V307, src/craft_description): camera pose and LiDAR from the robot description, results in base_link
# (REP-103: x ahead, y left, z up; origin at the four-wheel axle centre; floor = base_footprint z).
# ---------------------------------------------------------------------------------------------------------------
URDF_DEFAULT = URDF_DEFAULT_CLI


def _rpy(r, p, y):
    cr, sr, cp, sp, cy_, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return (np.array([[cy_, -sy, 0], [sy, cy_, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def urdf_frames(path, camera='camera_top_color_optical_frame'):
    """base_link <- camera optical frame (R, t), floor height (base_footprint z) and front_laser (x, y, z).
    All are fixed joints with parent base_link in V307."""
    import xml.etree.ElementTree as ET
    root = ET.parse(path).getroot()
    out = {}
    for j in root.findall('joint'):
        c = j.find('child').get('link'); o = j.find('origin')
        if j.find('parent').get('link') == 'base_link' and o is not None:
            out[c] = (np.array([float(v) for v in o.get('xyz', '0 0 0').split()]),
                      _rpy(*[float(v) for v in o.get('rpy', '0 0 0').split()]))
    t, R = out[camera]
    return R, t, out['base_footprint'][0][2], out['front_laser'][0]


CAM_YAW_DEG = 1.5   # camera yaw vs the chassis, + = turned left (CCW): chassis -7.9 deg -> camera -6.4 deg (user, 2026-11-07)
X_BIAS = 0.0        # [m] added to x1-x0 (7 mm tried 2026-11-08, removed: edges fixed instead)
CAM_HEIGHT = 0.6229  # colour lens over the floor [m], user's triangulation (sides 86 / 64.4 / 62.5 cm) 2026-11-07;
                     # V307 says 0.6271, the depth floor fit 0.6265-0.6268 (the profile may be bent or pitched)


def analyse_urdf(prefix, urdf=URDF_DEFAULT, lens_y=None, png=False, yaw_deg=CAM_YAW_DEG, cam_height=CAM_HEIGHT):
    """Box top (height-aware RANSAC with the URDF camera pose), 205 mm square, then in base_link:
    (x0, y0, z0) = front_laser, (x1, y1, z1) = centre of the box top's near edge, answer = |(x1,y1) - (x0,y0)|.
    lens_y: override the camera optical frame's y in base_link (URDF value otherwise)."""
    R, t, z_floor, laser = urdf_frames(urdf)
    t = t.copy()
    if lens_y is not None:
        t[1] = lens_y
    if cam_height is not None:                                       # measured height replaces the URDF one
        t[2] = z_floor + cam_height
    if yaw_deg:                                                      # turn the camera about base z, at the lens
        c_, s_ = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        R = np.array([[c_, -s_, 0], [s_, c_, 0], [0, 0, 1]]) @ R
    d, K, ser = load(prefix)
    up = R.T @ np.array([0, 0, 1.0])                                  # base z in camera coordinates
    cam_h = t[2] - z_floor
    top, cen, n, area, rms = box_top(d, K, up, cam_h, np.random.default_rng(0), 0.5, None)
    C3, px = rectangle(top, K, cen, n)
    _, cut, width, depth_seen = complete_square(C3, up)
    seen = far_corner_visibility(d, K, cen, n, C3, up)
    S, flags = complete_square_far(C3, up, seen)
    to_base = lambda Q: Q @ R.T + t
    Sb, Cb = to_base(S), to_base(C3)
    near = Sb[np.argsort(Sb[:, 0])[:2]].mean(0)
    near_seen = Cb[np.argsort(Cb[:, 0])[:2]].mean(0)
    near = near.copy(); near[0] += X_BIAS                            # persistent short reading
    xy = np.hypot(*(near[:2] - laser[:2]))
    f3 = lambda q: f'({q[0] * 1000:.0f}, {q[1] * 1000:+.0f}, {q[2] * 1000:.0f})'
    pitch = np.degrees(np.arcsin(-up[2]))
    print(f'{prefix} (D435 {ser}) URDF {os.path.basename(urdf)}: camera {f3(t)} mm, height {cam_h * 1000:.0f} mm, '
          f'pitch {pitch:.1f} deg, yaw {yaw_deg:+.1f} deg')
    print(f'  box top {area * 1e4:.0f} cm^2, seen {width * 1000:.0f} x {depth_seen * 1000:.0f} mm; far corners seen '
          f'{" / ".join("yes" if v else "NO" for v in seen.values())}' + (f'  [{"; ".join(flags)}]' if flags else ''))
    print(f'  (x0,y0,z0) front_laser {f3(laser)} mm   (x1,y1,z1) box near-edge centre {f3(near)} mm'
          + (f'  [seen {f3(near_seen)}]' if cut else ''))
    print(f'  => |(x1,y1) - (x0,y0)| = {xy * 1000:.0f} mm   (x1-x0 {(near[0] - laser[0]) * 1000:+.0f}, '
          f'y1-y0 {(near[1] - laser[1]) * 1000:+.0f})')
    return dict(near=near, laser=laser, xy=xy, S=Sb)




def side_visibility(top, d, K, cen, n, px):
    """For each side of the seen rectangle (image corners px, in order), the fraction of probe points just outside it
    that see the floor (a real edge) rather than the image border, no depth or something higher (hidden edge)."""
    h, w = d.shape; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    mid = px.mean(0); out = []
    for i in range(4):
        a, b = px[i], px[(i + 1) % 4]
        t = b - a; L = np.linalg.norm(t); t /= max(L, 1e-9); nrm = np.array([-t[1], t[0]])
        if nrm @ ((a + b) / 2 - mid) < 0:
            nrm = -nrm
        good = tot = 0
        for s in np.linspace(0.15, 0.85, 15):
            q = a + (b - a) * s + nrm * 6                              # 6 px outside the side
            u, v = int(round(q[0])), int(round(q[1])); tot += 1
            if not (0 <= u < w and 0 <= v < h):
                continue                                                # image border: hidden
            if d[v, u] <= 0:
                good += 1; continue                                     # no depth: stereo shadow past a real edge
            z = d[v, u]; P = np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])
            if (P - cen) @ n < -0.035:                                  # floor beside the box: a real edge
                good += 1                                               # (box height or higher: an occluder)
        out.append(good / tot)
    return out


def complete_square_vis(C3, up, vis, thr=0.5):
    """205 mm square from the seen rectangle using only the sides that are real edges.
    Lateral centre: both side edges seen -> their middle; one -> 102.5 mm in from it; none -> far edge middle
    (flagged). Depth: far edge seen -> near = far - 205; else near edge seen -> far = near + 205; else far edge."""
    fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd)
    ahead = C3 @ fwd; order = np.argsort(ahead)
    far_i, near_i = set(order[2:]), set(order[:2])
    side_of = lambda i, j: (i, j)
    sides = {}
    for i in range(4):
        j = (i + 1) % 4; key = 'far' if {i, j} == far_i else 'near' if {i, j} == near_i else 'side'
        sides.setdefault(key, []).append((i, j, vis[i]))
    fa, fb = [C3[k] for k in sorted(far_i)]
    along = fb - fa; along /= np.linalg.norm(along)
    toward = np.cross(up, along); toward = toward if toward @ fwd < 0 else -toward
    # lateral positions of the two side edges along 'along'
    lat = []
    for i, j, v in sides.get('side', []):
        lat.append(((C3[i] + C3[j]) / 2 @ along, v))
    lat.sort()
    flags = []
    if len(lat) == 2 and lat[0][1] >= thr and lat[1][1] >= thr:
        c_lat = (lat[0][0] + lat[1][0]) / 2
    elif len(lat) == 2 and lat[0][1] >= thr:
        c_lat = lat[0][0] + SIDE / 2; flags.append('one side (low) seen')
    elif len(lat) == 2 and lat[1][1] >= thr:
        c_lat = lat[1][0] - SIDE / 2; flags.append('one side (high) seen')
    else:
        c_lat = (fa + fb) / 2 @ along; flags.append('no side edge seen: lateral from the far edge (uncertain)')
    far_v = sides['far'][0][2] if 'far' in sides else 0; near_v = sides['near'][0][2] if 'near' in sides else 0
    far_line = (fa + fb) / 2 @ toward
    if far_v >= thr or near_v < thr:
        if far_v < thr:
            flags.append('far edge hidden too')
        near_line = far_line + SIDE
    else:
        na, nb = [C3[k] for k in sorted(near_i)]
        near_line = (na + nb) / 2 @ toward; far_line = near_line - SIDE; flags.append('far edge hidden: from the near edge')
    base = (fa + fb) / 2; base = base - (base @ along) * along - (base @ toward) * toward   # the plane offset part
    P = lambda la, to: base + along * la + toward * to
    S = np.array([P(c_lat - SIDE / 2, far_line), P(c_lat + SIDE / 2, far_line),
                  P(c_lat + SIDE / 2, near_line), P(c_lat - SIDE / 2, near_line)])
    return S, flags




def far_corner_visibility(d, K, cen, n, C3, up):
    """The two far-edge corners: hidden when at the image border (8 px) or when, just past the corner, there is
    something at box height or above (an occluder); otherwise seen (floor or no depth = stereo shadow)."""
    h, w = d.shape; fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd)
    far = np.argsort(C3 @ fwd)[-2:]
    proj = lambda P: np.array([fx * P[0] / P[2] + cx, fy * P[1] / P[2] + cy])
    mid = proj(C3.mean(0)); out = {}
    for k in far:
        q = proj(C3[k]); dirn = (q - mid) / max(np.linalg.norm(q - mid), 1e-9)
        if not (8 <= q[0] < w - 8 and 8 <= q[1] < h - 8):
            out[k] = False; continue
        hidden = 0
        for s in (6, 9, 12):
            u, v = np.int32(np.round(q + dirn * s))
            if not (0 <= u < w and 0 <= v < h):
                hidden += 1; continue
            z = d[v, u]
            if z > 0 and (np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) - cen) @ n > -0.035:
                hidden += 1
        out[k] = hidden < 2
    return out


def complete_square_far(C3, up, seen):
    """205 mm square from the far edge (depth: near = far - 205 always). Lateral: both far corners seen -> far-edge
    middle; one seen -> 102.5 mm in from that corner along the far edge; none -> far-edge middle (flagged)."""
    fwd = np.array([0, 0, 1.0]) - up[2] * up; fwd /= np.linalg.norm(fwd)
    ka, kb = np.argsort(C3 @ fwd)[-2:]
    a, b = C3[ka], C3[kb]; along = (b - a) / np.linalg.norm(b - a)
    toward = np.cross(up, along); toward = toward if toward @ fwd < 0 else -toward
    sa, sb = seen.get(ka, True), seen.get(kb, True); flags = []
    if sa and sb:
        m = (a + b) / 2
    elif sa:
        m = a + along * SIDE / 2; flags.append('one far corner hidden: centre from the other')
    elif sb:
        m = b - along * SIDE / 2; flags.append('one far corner hidden: centre from the other')
    else:
        m = (a + b) / 2; flags.append('both far corners hidden: lateral uncertain')
    fl, fr = m - along * SIDE / 2, m + along * SIDE / 2
    return np.array([fl, fr, fr + toward * SIDE, fl + toward * SIDE]), flags


if __name__ == '__main__':
    sys.exit(main())
