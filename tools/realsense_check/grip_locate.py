#!/usr/bin/env python3
"""Gripper-cam (wrist D435 261622073363) puck check: where the puck is relative to the open jaws, from one aligned
colour + depth frame, before the arm goes down to it.

  grip_locate.py [prefix ...]          saved rs_snap captures
  grip_locate.py --capture             grab one frame first (close the live gripper window, it holds the camera)
  grip_locate.py --set-ref <prefix>    store that frame's puck as the "lined up" reference (grip_ref.json)
  grip_locate.py --capture --set-ref   the same from a fresh frame: arm at the approach pose (bigbro stops there when
                                       the check fails), puck put exactly between the jaws by hand

One reference per shape (cylinder / prism: their boxes sit differently in the image); a shape without its own at this height uses the
other's; with none, nothing is compared (execute.py stores the frame of the first held grasp). The reference only holds at the height it was taken at (the camera does not look along the tool axis, so a
height change moves the puck in the image): REF_DZ_MM further / nearer than the reference is reported as "REF HEIGHT"
and execute.py stops on it. Re-take the reference whenever cam_pick.py GRIP_MM or plan.py approach_above changes
(2026-11-14: grip 45 -> 20 mm raised the approach 25-31 mm and every puck read 16 mm off).

Output per puck: pixel centre, depth, the centre in the camera optical frame (x right, y down, z ahead) [mm] and,
with a reference, the offset from it [mm]. Turning that offset into an arm correction needs the camera axes in the
arm frame: one LM jog of known size seen in two frames fixes the lateral axis (not done yet).
"""
import json
import os
import subprocess
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import box_range as br  # noqa: E402
import puck_locate as pl  # noqa: E402

GRIP_SERIAL = '261622073363'
REF = os.path.join(HERE, 'grip_ref.json')
REF_DZ_MM = 12.0


def shape(cls):
    return cls.split('_')[-1]


def load_refs():
    """{shape: reference}; the 2026-11-13 file held one bare reference."""
    if not os.path.exists(REF):
        return {}
    r = json.load(open(REF))
    return {shape(r['cls']): r} if 'cls' in r else r


TAG_ARM_YAW = None    # the last read top tag's turn in the arm frame [deg, -45..45], with grip_axes.json
TAG_IMAGE_DEG = None  # the last read top tag's edge direction in the image, folded into -45..+45 deg
_DET = None


def tag_point(bgr, d, K, box, prism=False):
    """Centre of the tag36h11 on the puck's top [mm, camera frame], None without a tag inside the puck's box: the
    top's centre itself, where the box centre also holds the puck's side and moves with the viewing direction."""
    global _DET
    if _DET is None:
        p = cv2.aruco.DetectorParameters(); p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        _DET = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), p)
    x, y, w, h = box
    # Three sizes of the frame: no single one reads every tag (640x360: only enlarged twice; 1280x720, 2026-11-14: the
    # top tag only at half size, the side tag only at full size).
    g1 = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    best = None
    for sc in (0.5, 1.0, 2.0):
        g = g1 if sc == 1.0 else cv2.resize(g1, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA if sc < 1 else cv2.INTER_CUBIC)
        for c in _DET.detectMarkers(g)[0]:
            u, v = c.reshape(4, 2).mean(0) / sc
            if not (x <= u <= x + w and y <= v <= y + h):
                continue
            patch = d[max(int(v) - 4, 0):int(v) + 5, max(int(u) - 4, 0):int(u) + 5]
            if not np.any(patch > 0.05):
                continue
            z = float(np.median(patch[patch > 0.05]))
            if best is None or z < best[2] - 0.003:  # a prism shows side tags too: the top one is the nearest
                e = (c.reshape(4, 2)[1] - c.reshape(4, 2)[0])
                best = (u, v, z, float(np.degrees(np.arctan2(e[1], e[0]))), c.reshape(4, 2) / sc)
    global TAG_IMAGE_DEG, TAG_ARM_YAW
    TAG_IMAGE_DEG = None if best is None else (best[3] + 45) % 90 - 45
    TAG_ARM_YAW = None
    axes = os.path.join(HERE, 'grip_axes.json')
    if best is not None and os.path.exists(axes):
        # The top's turn about the vertical in the arm frame (+ = as the upper cam's yaw: from ahead toward +LM). The
        # camera looks down at a slant, so the picture angle is not the turn (2026-11-14: -11.5 deg in the picture
        # for a prism the jaws met square at the upper cam's +3.5). The tag corners go to camera mm and through
        # --gripcal's inverse into (sideways, ahead) arm mm; the four edges, folded to +-45 deg, are averaged on the
        # 4x circle (the axes are not orthonormal, so single edges differ by up to 10 deg). Checked on seven frames:
        # within 4 deg of the upper cam.
        Mi = np.linalg.inv(np.array(json.load(open(axes))['cam_per_arm']))
        P = np.array([np.linalg.inv(K) @ np.array([cu, cv, 1.0]) * best[2] * 1000 for cu, cv in best[4]])
        ang = []
        for k in range(4):
            a = Mi @ (P[(k + 1) % 4, :2] - P[k, :2])
            ang.append(np.arctan2(a[0], a[1]))
        m4 = np.arctan2(np.mean(np.sin(4 * np.array(ang))), np.mean(np.cos(4 * np.array(ang)))) / 4
        TAG_ARM_YAW = float(np.degrees(m4))
    if best is None and not prism:
        # The tag is not decoded (2026-11-14: a cylinder's tag under glossy tape, its corner over the rim; a prism
        # frame now and then): the white paper of the top tag is still there. Cylinders only: a prism's side tags are white
        # paper at nearly the same depth (2026-11-14 12:56: a side tag read as the top, the arm moved 16 mm wrong and the
        # jaws knocked the prism over). Its patch (low saturation, bright, at
        # the puck's depth; the nearest one, side tags are further) gives the centre: within 1.5 mm of the decoded
        # centre on seven prism frames. No turn angle this way.
        x0, y0 = max(int(x), 0), max(int(y), 0)
        roi, dz = bgr[y0:int(y + h), x0:int(x + w)], d[y0:int(y + h), x0:int(x + w)]
        ok = dz > 0.05
        if roi.size and np.any(ok):
            depth = float(np.median(dz[int(0.3 * dz.shape[0]):int(0.7 * dz.shape[0]) or 1, int(0.3 * dz.shape[1]):int(0.7 * dz.shape[1]) or 1][
                dz[int(0.3 * dz.shape[0]):int(0.7 * dz.shape[0]) or 1, int(0.3 * dz.shape[1]):int(0.7 * dz.shape[1]) or 1] > 0.05]))
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            m = ((hsv[..., 1] < 70) & (hsv[..., 2] > 130) & (np.abs(dz - depth) < 0.025)).astype(np.uint8)
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
            n, lab, st, _ = cv2.connectedComponentsWithStats(m)
            for i in range(1, n):
                if st[i, 4] < 0.01 * w * h:
                    continue
                ys, xs = np.where(lab == i)
                z = float(np.median(dz[ys, xs]))
                mo = cv2.moments(cv2.convexHull(np.c_[xs, ys].astype(np.int32)))
                if mo['m00'] > 0 and (best is None or z < best[2] - 0.003):
                    best = (mo['m10'] / mo['m00'] + x0, mo['m01'] / mo['m00'] + y0, z, None)
    return None if best is None else (np.linalg.inv(K) @ np.array([best[0], best[1], 1.0]) * best[2] * 1000).tolist()


ZERO = os.path.join(HERE, 'grip_zero.json')   # the top tag's centre of a puck the jaws had just closed on and let go


def zero_point(o):
    """The point of a puck that is compared with the jaws' centre: a cylinder's box centre (its tag is stuck anywhere on
    the top, 2026-11-14: one hangs over the rim; every cylinder's box looks the same), a prism's top-tag centre (its
    box changes with its turn)."""
    return ('box', o['cam_mm']) if shape(o['cls']) == 'cylinder' else ('tag', o['tag_mm'])


def zero_offset(o):
    """(dx, dy, dz) [mm, camera frame] of the puck from where one of its shape sat after the jaws had centred it
    (grip_zero.json per shape, execute.py --gripzero: closing the jaws pushes the puck to their middle, then they open
    and the camera looks from the approach pose). None: not measured for this shape, no point, or another height."""
    z = (json.load(open(ZERO)) if os.path.exists(ZERO) else {}).get(shape(o['cls']))
    kind, p = zero_point(o)
    if z is None or p is None:
        return None
    d = np.array(p) - np.array(z['mm'])
    return d  # any height: execute.py corrects the height first (the x / y only hold at the reference height)


def zero_known(o):
    return os.path.exists(ZERO) and shape(o['cls']) in json.load(open(ZERO))


def locate(prefix):
    d, K, ser = br.load(prefix)
    bgr = cv2.imread(prefix + '_color.ppm')
    out = []
    for (x, y, w, h), c, p in pl.detect(bgr):
        patch = d[max(int(y + 0.3 * h), 0):int(y + 0.7 * h), max(int(x + 0.3 * w), 0):int(x + 0.7 * w)]
        if not np.any(patch > 0.05):
            continue
        z = float(np.median(patch[patch > 0.05]))
        u, v = x + w / 2, y + h / 2
        X = np.linalg.inv(K) @ np.array([u, v, 1.0]) * z
        out.append({'cls': pl.CLASSES[c], 'score': p, 'px': [u, v], 'box': [x, y, w, h], 'depth': z, 'cam_mm': (X * 1000).tolist(),
                    'tag_mm': tag_point(bgr, d, K, (x, y, w, h), 'prism' in pl.CLASSES[c]), 'tag_deg': TAG_IMAGE_DEG, 'yaw_deg': TAG_ARM_YAW})
    return sorted(out, key=lambda o: -o['score']), bgr


HELD = os.path.join(HERE, 'grip_held.json')   # closed-jaw reference: use the prism entry for every shape
HELD_X_PX = 10.0   # [px, 1280x720] sideways along the jaws (13:01 sideways: +14, 13:12 off: -21)
HELD_Y_PX = 10.0   # [px] its lower edge, i.e. how deep it sits in the jaws: shallower (+) than this stops (13:21 shallow: +16)
HELD_DEEP_PX = 25.0  # [px] deeper (-) is a firmer hold: allowed up to this (14:54 green cylinder -11, held fine; user)


def held_check(prefix):
    """Use the prism reference and the same grasp criteria for every puck shape (user 2026-11-16)."""
    found, _ = locate(prefix)
    if not found:
        print(f'{prefix}: HELD BAD no puck between the jaws'); return
    o = found[0]
    x, y, w, h = o['box']
    # The shape from the plan (--shape, the upper cam's class), not from this frame: with the jaws closed over it a
    # cylinder reads as a prism now and then (2026-11-14 14:58: green_prism 0.88, checked against the prism).
    sh = sys.argv[sys.argv.index('--shape') + 1] if '--shape' in sys.argv else shape(o['cls'])
    ref = None
    if os.path.exists(HELD):
        with open(HELD) as held_file:
            ref = json.load(held_file).get('prism')
    cx, bottom = x + w / 2, y + h
    if ref is None:
        print(f"{prefix}: HELD UNKNOWN {o['cls']} centre x {cx:.0f} bottom y {bottom:.0f} px (no prism reference)"); return
    dx, dy = cx - ref['cx'], bottom - ref['bottom']
    # One criterion: the prism's reference, lateral check, edge limits and confidence for all shapes.
    side_bad = abs(dx) > HELD_X_PX
    bad = side_bad or dy > HELD_Y_PX or dy < -HELD_DEEP_PX or o['score'] < 0.85
    print(f"{prefix}: HELD {'BAD' if bad else 'OK'} {sh} (prism criterion; frame reads {o['cls']} {o['score']:.2f}) sideways {dx:+.0f} px, lower edge {dy:+.0f} px"
          f" (limits sideways {HELD_X_PX:g}, edge +{HELD_Y_PX:g} shallow / -{HELD_DEEP_PX:g} deep; score >= 0.85)")


def main():
    args = [a for i, a in enumerate(sys.argv[1:], 1) if not a.startswith('--') and sys.argv[i - 1] != '--shape']
    if '--held' in sys.argv:
        for p_ in args:
            held_check(p_)
        return
    if '--capture' in sys.argv:
        p = os.path.join(HERE, 'out', time.strftime('grip_%H%M%S'))
        subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), p, '1280', '720', GRIP_SERIAL], check=True)  # 1280x720 (was 640x360): the top tag is read
        args.append(p)
    refs = load_refs()
    for prefix in args:
        found, bgr = locate(prefix)
        if not found:
            print(f'{prefix}: no puck in the gripper cam'); continue
        if '--under-jaws' in sys.argv and len(found) > 1:
            # Several pucks in view (2026-11-14: the neighbour scored 0.97, the one under the jaws 0.95, and the check
            # read the neighbour): keep the one nearest to where a lined-up puck sits in the image (the references;
            # without any, 25 / -15 mm, where four approaches on 11-14 had it).
            spot = np.mean([r['cam_mm'][:2] for r in refs.values()], axis=0) if refs else np.array([25.0, -15.0])
            found = sorted(found, key=lambda o: np.hypot(*(np.array(o['cam_mm'][:2]) - spot)))[:1]
        for o in found:
            cx, cy, cz = o['cam_mm']
            line = (f"{prefix}: {o['cls']} {o['score']:.2f} px ({o['px'][0]:.0f},{o['px'][1]:.0f}) depth {o['depth'] * 1000:.0f} mm"
                    f" -> camera x {cx:+.1f} y {cy:+.1f} z {cz:.1f} mm")
            # The shape's own reference; one taken at another height does not count, then the other shape's serves
            # (2026-11-14: cylinder 27 / -17 / 198 mm, prism 26 / -15 / 199 mm: the shapes read alike).
            ok = lambda r: r is not None and abs(o['cam_mm'][2] - r['cam_mm'][2]) <= REF_DZ_MM
            ref = refs.get(shape(o['cls']))
            if not ok(ref):
                ref = next((r for r in refs.values() if ok(r)), ref)
            if o['tag_mm']:
                line += '  | tag x {:+.1f} y {:+.1f} z {:.1f}'.format(*o['tag_mm']) + (
                    (f" turn {o['tag_deg']:+.1f} deg" if o['tag_deg'] is not None else ' (white patch, no angle)') + (
                    f" arm yaw {o['yaw_deg']:+.1f} deg" if o.get('yaw_deg') is not None else ''))
            off = zero_offset(o)
            if off is not None:  # the jaws' own centre: the learned references are not used any more
                line += f"  | vs jaw centre: dx {off[0]:+.1f} dy {off[1]:+.1f} dz {off[2]:+.1f} mm"
            elif zero_known(o):
                # With the jaws' centre known nothing else is compared: the box-centre references are another kind of
                # point (2026-11-14: one frame without the tag fell back to them, read +17 mm and the arm went wrong).
                line += '  | top tag not read (or another height): not compared'
            elif ref and '--set-ref' not in sys.argv:
                d = np.array(o['cam_mm']) - np.array(ref['cam_mm'])
                if abs(d[2]) > REF_DZ_MM:
                    line += f"  | REF HEIGHT: {d[2]:+.0f} mm from the {shape(ref['cls'])} reference's distance, x / y not comparable"
                else:
                    line += f"  | vs lined-up ref ({shape(ref['cls'])}): dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm"
            print(line)
            x, y, w, h = map(int, o['box'])
            cv2.rectangle(bgr, (x, y), (x + w, y + h), (0, 255, 0), 1)
        if ref:
            rx, ry = map(int, ref['px'])
            cv2.drawMarker(bgr, (rx, ry), (0, 0, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.imwrite(prefix + '_grip.png', bgr)
        if '--set-zero' in sys.argv:
            o = found[0]
            kind, m = zero_point(o)
            if m is None:
                print('zero NOT stored: the top tag was not read'); sys.exit(1)
            # The flat jaws centre a prism front-back only: sideways its target is moved by side_mm of arm travel (+ =
            # away from the LM motor; kept from the file, 2026-11-14 user: 8), through --gripcal's axes. A cylinder sits
            # in the jaws' groove and is centred both ways (2026-11-14: closing moved one 5 mm sideways): no shift.
            zs = json.load(open(ZERO)) if os.path.exists(ZERO) else {}
            sh = shape(o['cls'])
            side = 0.0 if sh == 'cylinder' else float(zs.get(sh, {}).get('side_mm', 8.0))
            axes = os.path.join(HERE, 'grip_axes.json')
            # A cylinder keeps its back_mm (2026-11-14 user: 5, the arm that far toward the robot; 2026-11-16: kept
            # through a re-zero): the second axis of --gripcal is ahead, so - back_mm there.
            back = float(zs.get(sh, {}).get('back_mm', 0.0)) if sh == 'cylinder' else 0.0
            shift = (np.array(json.load(open(axes))['cam_per_arm']) @ np.array([side, -back])) if os.path.exists(axes) else np.zeros(2)
            zs[sh] = {'point': kind, 'mm': [m[0] + float(shift[0]), m[1] + float(shift[1]), m[2]], 'measured_mm': list(m),
                      'side_mm': side, 'cls': o['cls'], 'stamp': time.strftime('%Y-%m-%d %H:%M')}
            if back:
                zs[sh]['back_mm'] = back
            json.dump(zs, open(ZERO, 'w'), indent=1)
            print('zero stored for {} ({} centre): x {:+.1f} y {:+.1f} z {:.1f} mm'.format(sh, kind, *zs[sh]['mm']))
        if '--set-ref' in sys.argv:
            refs[shape(found[0]['cls'])] = found[0]
            json.dump(refs, open(REF, 'w'), indent=1)
            print(f"reference stored for {shape(found[0]['cls'])}: {REF} (now: {', '.join(refs)})")


if __name__ == '__main__':
    main()
