"""Fit a known square's near face to measured map-frame LiDAR points.

No fabricated points: None means that evidence was insufficient or ambiguous.
Coordinates and distances are metres; yaw is radians. Inputs must be synchronized.
"""
import math
import numpy as np


def fit_box_face(points_xy_map, expected_box, robot_pose, side=.205, front_face_x=.2097,
                 laser_pose=None, laser_front_offset=.019):
    points = np.asarray(points_xy_map, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
        return None
    bx, by, yaw = expected_box
    rx, ry, heading = robot_pose
    if not all(math.isfinite(x) for x in (bx, by, yaw, rx, ry, heading, side, front_face_x)) or side <= 0:
        return None
    center = np.array([bx, by]); robot = np.array([rx, ry])
    candidates = np.array([[math.cos(yaw + k * math.pi / 2), math.sin(yaw + k * math.pi / 2)] for k in range(4)])
    normal = candidates[np.argmax(candidates @ (robot - center))]
    tangent = np.array([-normal[1], normal[0]])
    expected_face = center + normal * side / 2
    coords = np.column_stack(((points - expected_face) @ normal, (points - expected_face) @ tangent))
    roi = (np.abs(coords[:, 0]) <= .10) & (np.abs(coords[:, 1]) <= side / 2 + .05)
    points = points[roi]; coords = coords[roi]
    if len(points) < 6:
        return None
    # Candidate lines are x = a*y+b in the expected face's local coordinates.
    hypotheses = []
    slope_max = math.tan(math.radians(20))
    # Generate every original pair in the same order. Fit each distinct
    # inlier mask once; duplicate masks have the same fit and ambiguity result.
    left, right = np.triu_indices(len(coords), 1)
    dy = coords[right, 1] - coords[left, 1]
    usable = np.abs(dy) >= .04
    left, right, dy = left[usable], right[usable], dy[usable]
    slopes = (coords[right, 0] - coords[left, 0]) / dy
    usable = np.abs(slopes) <= slope_max
    left, slopes = left[usable], slopes[usable]
    offsets = coords[left, 0] - slopes * coords[left, 1]
    seen_masks = set()
    for first in range(0, len(slopes), 256):
        aa, bb = slopes[first:first + 256], offsets[first:first + 256]
        residuals = np.abs(coords[None, :, 0] - aa[:, None] * coords[None, :, 1] - bb[:, None]) / np.hypot(1, aa[:, None])
        masks = residuals <= .006
        unique_masks = []
        for mask in masks[masks.sum(axis=1) >= 6]:
            key = np.packbits(mask).tobytes()
            if key in seen_masks:
                continue
            seen_masks.add(key)
            if np.ptp(coords[mask, 1]) >= .08:
                unique_masks.append(mask)
        if not unique_masks:
            continue
        # Centered least squares for each inlier set, in original hypothesis order.
        weights = np.asarray(unique_masks, dtype=float)
        count = weights.sum(axis=1)
        x, y = coords[:, 1], coords[:, 0]
        mx, my = (weights @ x) / count, (weights @ y) / count
        dx = x[None, :] - mx[:, None]
        dy = y[None, :] - my[:, None]
        denominator = (weights * dx * dx).sum(axis=1)
        good = denominator > np.finfo(float).tiny
        a = np.zeros(len(count)); b = np.zeros(len(count))
        a[good] = (weights[good] * dx[good] * dy[good]).sum(axis=1) / denominator[good]
        b[good] = my[good] - a[good] * mx[good]
        residuals = np.abs(y[None, :] - a[:, None] * x[None, :] - b[:, None]) / np.hypot(1, a[:, None])
        refit_masks = residuals <= .006
        counts = refit_masks.sum(axis=1)
        spans = np.where(refit_masks, x, -np.inf).max(axis=1) - np.where(refit_masks, x, np.inf).min(axis=1)
        rmses = np.sqrt(np.sum(np.where(refit_masks, residuals ** 2, 0.), axis=1) / np.maximum(counts, 1))
        for i, original_mask in enumerate(unique_masks):
            if not good[i]:
                continue
            ai, bi, rmse, mask = a[i], b[i], rmses[i], refit_masks[i]
            # Recompute numerically delicate decisions with the former solver.
            if (np.any(np.abs(residuals[i] - .006) < 1e-12) or
                abs(abs(ai) - slope_max) < 1e-12 or abs(abs(bi) - .10) < 1e-12 or
                abs(rmse - .004) < 1e-12):
                ai, bi = np.linalg.lstsq(np.column_stack((x[original_mask], np.ones(int(count[i])))), y[original_mask], rcond=None)[0]
                residual = np.abs(y - ai*x - bi) / math.hypot(1, ai)
                mask = residual <= .006
                n = int(mask.sum())
                span = np.ptp(x[mask]) if n else 0.
                rmse = float(np.sqrt(np.mean(residual[mask] ** 2))) if n else math.inf
            else:
                n, span = int(counts[i]), spans[i]
            if n < 6 or span < .08 or abs(ai) > slope_max or abs(bi) > .10 or rmse > .004:
                continue
            hypotheses.append((n, float(rmse), float(ai), float(bi), mask, original_mask))
    if not hypotheses:
        return None
    # Equal-support candidates can have practically identical RMSE. Preserve
    # the original solver's ordering for those candidates, including ties.
    def recompute(i):
        _, _, _, _, _, original_mask = hypotheses[i]
        x, y = coords[:, 1], coords[:, 0]
        ai, bi = np.linalg.lstsq(np.column_stack((x[original_mask], np.ones(int(original_mask.sum())))), y[original_mask], rcond=None)[0]
        residual = np.abs(y - ai*x - bi) / math.hypot(1, ai)
        mask = residual <= .006
        n = int(mask.sum())
        rmse = float(np.sqrt(np.mean(residual[mask] ** 2))) if n else math.inf
        hypotheses[i] = (n, rmse, float(ai), float(bi), mask, original_mask)
    order = sorted(range(len(hypotheses)), key=lambda i: (-hypotheses[i][0], hypotheses[i][1]))
    recalculate = set()
    for i, j in zip(order, order[1:]):
        if hypotheses[i][0] == hypotheses[j][0] and abs(hypotheses[i][1] - hypotheses[j][1]) < 1e-12:
            recalculate.update((i, j))
    for i in sorted(recalculate):
        recompute(i)
    best_i = min(range(len(hypotheses)), key=lambda i: (-hypotheses[i][0], hypotheses[i][1]))
    best_candidate = hypotheses[best_i]
    boundary = set()
    for i, h in enumerate(hypotheses):
        if i == best_i or h[0] < max(6, math.ceil(best_candidate[0] * .8)):
            continue
        angle = abs(math.atan(h[2]) - math.atan(best_candidate[2]))
        offset = abs(h[3] - best_candidate[3])
        if abs(angle - math.radians(10)) < 1e-12 or abs(offset - .012) < 1e-12:
            boundary.update((best_i, i))
    for i in sorted(boundary - recalculate):
        recompute(i)
    hypotheses.sort(key=lambda h: (-h[0], h[1]))
    best = hypotheses[0]
    # A sparse accidental line in a cluttered ROI is not sufficient support.
    if best[0] < math.ceil(len(coords) * .5):
        return None
    # Distinct nearby parallel planes with comparable support are not distinguishable as the target.
    for h in hypotheses[1:]:
        if h[0] < max(6, math.ceil(best[0] * .8)):
            break
        if abs(math.atan(h[2]) - math.atan(best[2])) < math.radians(10) and abs(h[3] - best[3]) > .012:
            return None
    count, rmse, a, b, mask, _ = best
    fit_normal = (normal - a * tangent) / math.hypot(1, a)
    fit_tangent = np.array([-fit_normal[1], fit_normal[0]])
    # The input prior defines the lateral centre; a face line alone cannot identify it.
    fit_face = expected_face + b * normal
    fitted_center = fit_face - fit_normal * side / 2
    robot_forward = np.array([math.cos(heading), math.sin(heading)])
    robot_left = np.array([-robot_forward[1], robot_forward[0]])
    denom = float(fit_normal @ robot_forward)
    if abs(denom) < .3:
        return None
    forward_plane = float(fit_normal @ (fit_face - robot) / denom)
    gap = forward_plane - front_face_x
    result = {'box_center_map': fitted_center.tolist(), 'box_yaw': math.atan2(fit_normal[1], fit_normal[0]),
              'gap_m': gap, 'lateral_m': float((fit_face - robot) @ robot_left),
              'yaw_relative': math.atan2(math.sin(math.atan2(-fit_normal[1], -fit_normal[0]) - heading), math.cos(math.atan2(-fit_normal[1], -fit_normal[0]) - heading)),
              'inliers': count, 'span_m': float(np.ptp(coords[mask, 1])), 'rmse_m': rmse,
              'uncertainty_m': rmse, 'face_normal_map': fit_normal.tolist()}
    if laser_pose is not None:
        lx, ly, lyaw = laser_pose
        if all(math.isfinite(x) for x in (lx, ly, lyaw, laser_front_offset)):
            laser_forward = np.array([math.cos(lyaw), math.sin(lyaw)])
            ld = float(fit_normal @ laser_forward)
            if abs(ld) >= .3:
                result['laser_gap_m'] = float(fit_normal @ (fit_face - np.array([lx, ly])) / ld - laser_front_offset)
    return result
