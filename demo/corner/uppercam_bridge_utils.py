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
