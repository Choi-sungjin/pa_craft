"""Geometry and timestamp contract shared by the sensor bridge and its offline tests."""
import math

def camera_box_in_map(gap_m, lateral_m, yaw_relative, robot_xy_yaw, front_face_x, side):
    vals=(gap_m,lateral_m,yaw_relative,*robot_xy_yaw,front_face_x,side)
    if not all(math.isfinite(float(x)) for x in vals) or gap_m<0 or side<=0:
        raise ValueError('invalid camera geometry')
    rx,ry,heading=robot_xy_yaw
    # The calibrated measurement is to the near-edge midpoint along base x.
    # Offset to the centre along the box normal; subtract no additional 19 mm.
    x=front_face_x+gap_m+side*.5*math.cos(yaw_relative)
    y=lateral_m+side*.5*math.sin(yaw_relative)
    c,s=math.cos(heading),math.sin(heading)
    return (rx+c*x-s*y, ry+s*x+c*y, heading+yaw_relative)

def acquisition_valid(stamp_s, now_s, request_s, timeout=.15):
    return all(math.isfinite(float(v)) for v in (stamp_s,now_s,request_s)) and stamp_s>=request_s and -.01<=now_s-stamp_s<=timeout

def planar_pose(transform):
    t,q=transform.translation,transform.rotation
    return (t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))

SQUARE_METHOD = 'square_far20_spec_reimplementation_v1'
SQUARE_MANIFEST_SHA256 = 'f518844c6156b26b0406682bf50fb11fc0d4adbad4b74eeedab30ff48646bd12'
SQUARE_FILES = ('uppercam_square_far20_estimator.py', 'uppercam_geometry_pool.py', 'uppercam_fast5_box_range.py',
                'uppercam_box_range.py', 'uppercam_estimator.py', 'uppercam_selected_adapter.py',
                'uppercam_gap_calibration.py', 'uppercam_causal_median.py', 'uppercam_trigger_filter.py',
                'uppercam_selected_calibration.json', '@urdf')

def load_validated_square_manifest(manifest_path, module_dir, calibration_path, urdf_path):
    """Validate the exact artifacts associated with the one-bag evaluation."""
    import hashlib
    import json
    from pathlib import Path
    manifest_bytes = Path(manifest_path).read_bytes()
    if hashlib.sha256(manifest_bytes).hexdigest() != SQUARE_MANIFEST_SHA256:
        raise ValueError('square validation manifest mismatch')
    manifest = json.loads(manifest_bytes)
    if (manifest.get('schema') != 'square_far20_validation_v1' or
            manifest.get('method') != SQUARE_METHOD or
            manifest.get('provenance') != 'independent_spec_reimplementation'):
        raise ValueError('unsupported square validation manifest')
    files = manifest.get('files', {})
    if set(files) != set(SQUARE_FILES):
        raise ValueError('incomplete square validation manifest')
    limits = manifest.get('operating_limits', {})
    if (limits.get('min_gap_mm') != 10.0 or limits.get('max_gap_mm') != 80.0 or
            limits.get('extrapolated_valid') is not False or limits.get('near_uncertainty_m') != .005 or
            limits.get('far_uncertainty_m') != .010 or limits.get('uncertainty_switch_mm') != 30.0):
        raise ValueError('unsupported square operating limits')
    for name in SQUARE_FILES:
        path = (Path(urdf_path) if name == '@urdf' else
                Path(calibration_path) if name == 'uppercam_selected_calibration.json' else Path(module_dir) / name)
        try:
            actual = hashlib.sha256(path.read_bytes()).hexdigest()
        except OSError as exc:
            raise ValueError(f'cannot read validated artifact {name}') from exc
        if actual != files[name]:
            raise ValueError(f'validated artifact mismatch: {name}')
    return manifest

def square_observation_policy(result, enabled):
    """Accept experiment observations; calibration coverage stays diagnostic.

    The 0..200 mm operating envelope is distinct from the recorded 10..80 mm
    calibration interval. Outside that interval the uncertainty field is a
    temporary control budget, not a measured accuracy guarantee. The planner
    compares stopped LiDAR/camera measurements before allowing movement.
    """
    if not enabled:
        return False, 'camera diagnostic only', 1.0
    if result.get('method') != SQUARE_METHOD or result.get('provenance') != 'spec_reimplementation':
        return False, 'camera estimator identity mismatch', 1.0
    if result.get('state') != 'geometry_unverified':
        return False, str(result.get('state', 'invalid camera estimate')), 1.0
    try:
        gap=float(result['gap_mm']); lateral=float(result['lateral_m']); angle=float(result['box_yaw_rad'])
        source=float(result['source_stamp_s'])
        if not all(math.isfinite(v) for v in (gap,lateral,angle,source)):
            raise ValueError('nonfinite camera measurement')
    except (KeyError, TypeError, ValueError):
        return False, 'incomplete or nonfinite camera measurement', 1.0
    if not 0.0 <= gap <= 200.0:
        return False, 'camera gap outside 0-200 mm experiment envelope', 1.0
    experimental = bool(result.get('extrapolated', False)) or not 10.0 <= gap <= 80.0
    reason = ('square_far20 experiment outside recorded calibration coverage' if experimental
              else 'square_far20 one-bag validated')
    return True, reason, .005 if gap <= 30.0 else .010


# ---- Box depth at the validated grid while the colour stream runs at a higher resolution -----------------
# The box estimator works on depth aligned to colour, and its morphology counts pixels, so its input must
# stay 640x360 (validated) when the colour stream is raised for puck tags (2026-11-12). Two ways to get it:
#   align_nearest: rs.align to the high-res colour, then nearest-neighbour down to the box grid.
#   reproject:     skip the high-res align; lift each raw depth pixel to 3D, move it into the colour frame
#                  and splat it straight onto the box grid (nearest surface wins), as rs.align does.
# Both return metres (0 = no depth) and the box-grid intrinsics. Experiment: docs/todo/puck_perception.md.

def scale_intrinsics(K, sx, sy):
    """K of the same camera after resizing the image by (sx, sy); pixel centres at +0.5 (OpenCV/librealsense)."""
    import numpy as np
    K = np.asarray(K, float).copy()
    K[0, 0] *= sx; K[1, 1] *= sy
    K[0, 2] = (K[0, 2] + .5) * sx - .5; K[1, 2] = (K[1, 2] + .5) * sy - .5
    return K


def box_grid(colour_size, box_size):
    """(sx, sy) from the colour stream to the box grid; same aspect, not larger, else ValueError."""
    (cw, ch), (bw, bh) = colour_size, box_size
    if not (0 < bw <= cw and 0 < bh <= ch) or abs(bw * ch - bh * cw) > max(cw, ch):
        raise ValueError(f'box size {bw}x{bh} must be <= colour {cw}x{ch} with the same aspect')
    return bw / cw, bh / ch


def depth_align_nearest(aligned_m, K_colour, box_size):
    """Depth already aligned to the high-res colour -> box grid by nearest neighbour (pixel centres)."""
    import cv2
    import numpy as np
    h, w = aligned_m.shape
    sx, sy = box_grid((w, h), box_size)
    flag = getattr(cv2, 'INTER_NEAREST_EXACT', cv2.INTER_NEAREST)
    return cv2.resize(np.asarray(aligned_m, np.float32), tuple(box_size), interpolation=flag), scale_intrinsics(K_colour, sx, sy)


def depth_reproject(raw_m, K_depth, R_depth_to_colour, t_depth_to_colour, K_box, box_size):
    """Raw depth image (metres, depth camera) -> depth on the box grid of the colour camera.
    Each depth pixel is splatted over the box pixels its footprint covers (~2x2 at 424x240 -> 640x360),
    the nearest surface winning, so no holes open up between pixels. Lens distortion is ignored (D435 depth
    has none; the colour model's coefficients are ~0)."""
    import numpy as np
    z = np.asarray(raw_m, np.float64)
    bw, bh = box_size
    out = np.full((bh, bw), np.inf)
    v, u = np.nonzero(np.isfinite(z) & (z > 0))
    if not len(u):
        return np.zeros((bh, bw), np.float32)
    zd = z[v, u]
    Kd = np.asarray(K_depth, float); Kb = np.asarray(K_box, float)
    R = np.asarray(R_depth_to_colour, float); t = np.asarray(t_depth_to_colour, float).reshape(3)
    corners = []
    for du in (-.5, .5):  # project the two opposite corners of each depth pixel's footprint
        X = np.stack(((u + du - Kd[0, 2]) / Kd[0, 0] * zd, (v + du - Kd[1, 2]) / Kd[1, 1] * zd, zd))
        P = R @ X + t[:, None]
        corners.append((Kb[0, 0] * P[0] / P[2] + Kb[0, 2], Kb[1, 1] * P[1] / P[2] + Kb[1, 2], P[2]))
    (x0, y0, z0), (x1, y1, z1) = corners
    zc = (z0 + z1) / 2
    lo_x = np.ceil(np.minimum(x0, x1) - .5).astype(int); hi_x = np.floor(np.maximum(x0, x1) - .5).astype(int)
    lo_y = np.ceil(np.minimum(y0, y1) - .5).astype(int); hi_y = np.floor(np.maximum(y0, y1) - .5).astype(int)
    hi_x = np.maximum(hi_x, lo_x); hi_y = np.maximum(hi_y, lo_y)  # sub-pixel footprint: still one pixel
    span_x = int((hi_x - lo_x).max()) + 1 if len(lo_x) else 1; span_y = int((hi_y - lo_y).max()) + 1 if len(lo_y) else 1
    for dy in range(min(span_y, 8)):
        for dx in range(min(span_x, 8)):
            px, py = lo_x + dx, lo_y + dy
            ok = (px <= hi_x) & (py <= hi_y) & (px >= 0) & (py >= 0) & (px < bw) & (py < bh) & (zc > 0)
            np.minimum.at(out, (py[ok], px[ok]), zc[ok])
    out[~np.isfinite(out)] = 0
    return out.astype(np.float32)
