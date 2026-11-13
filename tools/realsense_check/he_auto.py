#!/usr/bin/env python3
"""Automatic hand-eye samples: the gripper hovers, pointing down, over empty spots next to a standing cylinder, and
handeye.py captures both cameras at each spot (the cylinder's top tag is the shared point).

  source ~/craft/install/setup.bash; /usr/bin/python3 he_auto.py [--dry-run]

Needs control up, the arm at the nominal pose, a standing cylinder within reach and the live camera windows closed.
Spots: the nearest cylinder (upper cam) + offsets of 70 mm sideways / ahead, at two hover heights (jaw tips 50 and
100 mm over the cylinder top, so they never reach it). Each spot is planned by plan.py; unreachable ones are skipped.
Moves: via first; into the first spot LM + arm with the shoulder held, then the shoulder (as bigbro); between spots
one smooth timed move; at the end via, then the nominal pose (slow).
"""
import math
import os
import subprocess
import sys
import threading
import time

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PP = os.path.normpath(os.path.join(HERE, '..', '..', 'demo', 'puck_pick'))
sys.path.insert(0, HERE); sys.path.insert(0, PP)
DRY = '--dry-run' in sys.argv
sys.argv = [sys.argv[0]]                     # execute.py reads sys.argv at import
import cam_pick  # noqa: E402
import execute as ex  # noqa: E402

OUT = '/tmp/claude-1000/-home-craft-craft/af6c0472-7c9c-4305-9240-a5d754be5799/scratchpad'
OFFSETS = [(0.0, 0.07), (0.0, -0.07), (0.07, 0.0), (-0.06, 0.05), (0.06, -0.05), (0.0, 0.07)]
LIFTS = [0.05, 0.07, 0.04, 0.06, 0.07, 0.05]      # jaw tips over the cylinder top [m] (100 mm did not solve)
MOVE_S = 5.0


def spots():
    pre = os.path.join(HERE, 'out', time.strftime('heauto_%H%M%S'))
    subprocess.run([os.path.join(HERE, 'build', 'rs_snap'), pre, '1280', '720', cam_pick.pl.UPPER_SERIAL], check=True)
    pucks, laser = cam_pick.puck_centres(pre)
    cyl = sorted([p for p in pucks if 'cylinder' in p[1]], key=lambda p: p[2][0])
    if not cyl:
        sys.exit('no standing cylinder in the upper cam')
    sc, name, cen, top = cyl[0]
    print(f'target: {name} {sc:.2f} at x {cen[0] * 1000:.0f} y {cen[1] * 1000:+.0f} mm, top {top * 1000:.0f} mm')
    face_x = laser[0] + cam_pick.br.LIDAR_FACE_TO_CENTRE
    out = []
    # Spots between the robot and the cylinder (the arm reaches LiDAR face + 90..170 mm, the LM -63..+35 mm): the
    # gripper cam looks down and ahead at the top tag. Grid nearest the cylinder first, alternating hover heights.
    cand = []
    for dx in (-0.07, -0.09, -0.11, -0.08, -0.12):
        for dy in (0.0, -0.04, 0.04, -0.02, 0.02):
            x, y = cen[0] + dx, cen[1] + dy
            ahead, xoff = x - face_x, cam_pick.LATERAL_SIGN * (y - laser[1])
            if 0.09 <= ahead <= 0.17 and -0.063 <= xoff <= 0.035:
                cand.append((dx, dy))
    print(f'  {len(cand)} reachable spots 70-130 mm in front of it')
    for k, ((dx, dy), lift) in enumerate(zip(cand[:6], LIFTS)):
        x, y = cen[0] + dx, cen[1] + dy
        ahead, xoff = x - face_x, cam_pick.LATERAL_SIGN * (y - laser[1])
        # plan.py puts the jaw tips grip_mm under the "puck top": a virtual puck whose top is lift over the real one
        shelf = top + lift - cam_pick.PUCK_H
        f = os.path.join(OUT, f'he_spot_{len(out)}.yaml')
        r = subprocess.run([sys.executable, os.path.join(PP, 'plan.py'), '--puck-ahead', f'{ahead:.4f}', '--puck-x', f'{xoff:.4f}',
                            '--shelf-height', f'{shelf:.4f}', '--grip-mm', '45', '--out', f], capture_output=True, text=True)
        wp = {w['name']: w['joints'] for w in yaml.safe_load(open(f))['waypoints']} if os.path.exists(f) else {}
        ok = r.returncode == 0 and 'grasp' in wp and yaml.safe_load(open(f))['frames']
        print(f'  spot {len(out) + 1}: x {x * 1000:.0f} y {y * 1000:+.0f} mm, tips {lift * 1000:.0f} mm over the top -> ' + ('ok' if ok else 'unreachable, skipped'))
        if ok:
            g = wp['grasp']
            out.append([g['slide_joint'] + ex.LM_BACKLASH] + [g[n] for n in ex.ARM[1:]])
    return out


def main():
    poses = spots()
    if DRY or not poses:
        print(f'{len(poses)} spots planned' + (' (dry run: nothing moved)' if DRY else '')); return
    import rclpy
    rclpy.init(); node = ex.Runner(); threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t = time.time()
    while (node.status is None or node.joints is None) and time.time() - t < 5:
        time.sleep(0.1)
    time.sleep(2.0)                                   # let the command publisher match control
    if node.status is None or not node.status.ready:
        sys.exit('control not ready')

    def go(q, seconds, label):
        st, why = node.wait(node.send(ex.TaskCommand.MANIPULATOR, q, seconds))
        print(f'  {label}: {st} {why}')
        if st != 'SUCCEEDED':
            node.stop(); sys.exit('stopped')

    via = ex.via_pose()[:5]
    go(via, 6.0, 'to the via pose')
    first = poses[0]
    go([first[0], via[1]] + first[2:], MOVE_S, 'LM and arm over spot 1, shoulder held')
    for k, q in enumerate(poses):
        go(q, MOVE_S, f'spot {k + 1}')
        time.sleep(1.0)
        r = subprocess.run(['/usr/bin/python3', os.path.join(HERE, 'handeye.py'), 'capture', 'auto'], capture_output=True, text=True)
        print('   ' + ' | '.join(l for l in r.stdout.splitlines() if 'tags' in l or 'no tag' in l))
    last = poses[-1]
    go([last[0], via[1]] + last[2:], MOVE_S, 'shoulder up (the rest held)')
    go(via, MOVE_S, 'to the via pose')
    nom = ex.nominal_pose()
    go(nom, 12.0, 'to the nominal pose (slow)')
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
