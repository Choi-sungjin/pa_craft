#!/usr/bin/env python3
"""Shadow run of the R1 residual standing-puck agent (~/rl_packs/standing_puck_robot_pack) on our arm model.
Moves nothing: the robot is simulated (joints reach their target each 30 Hz step, a contact force appears when the
closing jaws are at the puck), so this checks the plumbing, not the policy.

  ~/craft/venv-craft/bin/python3 rl_shadow.py [plan_cam.yaml]     (venv-craft: it has torch)

Frames: the pack works in V311 (x ahead, y left, z over the floor, LM y = lm - 0.0435), the same layout as the V307
base_link the upper camera reports in. Our control uses V75 joint values (plan.py, control.yaml offsets). Mapping
V75 -> V311, fitted by FK over 300 random poses (TCP RMS 5.7 mm; rl_fit in the 2026-11-13 session):
  lm = -slide + LM_OFF, shoulder/elbow/wrist + offsets, roll as is.
The LM origin is not fixed by FK (a prismatic joint): LM_OFF is anchored on the 2026-11-13 bigbro grasp (camera
puck y -41 mm, the gripper cam showed the puck between the jaws). The sign follows LATERAL_SIGN +1 (two grasps).
The same grasp sits 15 mm short of the camera puck in x: CAM_TO_ARM_X corrects the camera position.
"""
import math
import sys
from pathlib import Path

import numpy as np
import torch
import yaml

HERE = Path(__file__).resolve().parent
PACK = Path.home() / 'rl_packs' / 'standing_puck_robot_pack'
sys.path.insert(0, str(HERE)); sys.path.insert(0, str(PACK / 'src'))
import plan  # noqa: E402
import robot_kinematics as rk  # noqa: E402
from standing_puck_agent import StandingPuckAgent  # noqa: E402

MODEL = PACK / 'models' / 'r1_resid_policy_model_400_golden.pt'   # TorchScript (README: recommended)
# V75 -> V311 joint offsets [rad] and LM (fit + grasp anchor)
SH_OFF, EL_OFF, WR_OFF = None, None, None
FIT = Path('/tmp/claude-1000/-home-craft-craft/af6c0472-7c9c-4305-9240-a5d754be5799/scratchpad/v75_to_v311_A.npy')
CAM_TO_ARM_X = -0.015      # camera puck x -> where the arm actually grasped [m] (319 vs 304 mm)
CAM_PUCK = (0.319, -0.041)  # 2026-11-13 bigbro puck, camera (V307 base_link) [m]; the anchor
PUCK_ARG = None             # --puck x y: run the agent on this camera position instead
PUCK_CENTRE_Z = 0.110       # over the floor: 70 mm shelf + 30 mm half puck + 10 mm (plan.py hw_z_correction)


def load_map():
    m = np.load(FIT)
    o = m[5:9]                               # slide, shoulder, elbow, wrist offsets (sign -1 on the slide)
    return {'lm': o[0], 'sh': o[1], 'el': o[2], 'wr': o[3]}


def to311(q, M):
    return [-q[0] + M['lm'], q[1] + M['sh'], q[2] + M['el'], q[3] + M['wr'], q[4]]


_LIM = None


def to75(q, M):
    """V311 -> V75; revolute joints wrapped by 2 pi into our URDF limits (the fit offsets are modulo 2 pi)."""
    global _LIM
    if _LIM is None:
        _LIM = [plan.Chain(plan.URDF).limits(n) for n in plan.Chain.NAMES]
    out = [-(q[0] - M['lm']), q[1] - M['sh'], q[2] - M['el'], q[3] - M['wr'], q[4]]
    for i in (1, 2, 3, 4):
        lo, hi = _LIM[i]
        for k in (0, -1, 1, -2, 2):
            v = out[i] + 2 * math.pi * k
            if lo - 1e-6 <= v <= hi + 1e-6:
                out[i] = v; break
    return out


def main():
    plan_file = 'plan_cam.yaml'
    M = load_map()
    wp = {w['name']: w['joints'] for w in yaml.safe_load(open(HERE / plan_file))['waypoints']}
    g75 = [wp['grasp'][n] for n in plan.Chain.NAMES]
    # LM anchor: today's grasp must land on the camera puck y
    y_now = float(rk.fk_tcp(torch.tensor([to311(g75, M)], dtype=torch.float32))[0, 1])
    M['lm'] += CAM_PUCK[1] - y_now
    tg = rk.fk_tcp(torch.tensor([to311(g75, M)], dtype=torch.float32))[0].numpy()
    print(f'our grasp (plan) in the pack frame: x {tg[0] * 1000:.0f} y {tg[1] * 1000:+.0f} z {tg[2] * 1000:.0f} mm')

    cx, cy = (float(sys.argv[sys.argv.index('--puck') + 1]), float(sys.argv[sys.argv.index('--puck') + 2])) if '--puck' in sys.argv else CAM_PUCK
    puck = [cx + CAM_TO_ARM_X, cy, PUCK_CENTRE_Z]
    agent = StandingPuckAgent(str(MODEL), backend='torchscript')
    ready = [M['lm'] - (-(0.0435 + 0.0) - 0) * 0, *rk.READY_Q3_STAND, 0.0]   # placeholder, replaced below
    ready[0] = 0.0435 + puck[1]              # LM over the puck line, the pack's ready arm pose
    q = torch.tensor(ready, dtype=torch.float32)
    agent.reset(q)
    jaw, log = rk.JAW_OPEN, []
    lo = [plan.Chain(plan.URDF).limits(n) for n in plan.Chain.NAMES]
    for k in range(900):
        tcp = rk.fk_tcp(q.unsqueeze(0))[0]
        gp = torch.tensor([puck[0], puck[1], puck[2] + 0.015])
        at_puck = float(torch.norm(tcp - gp)) < 0.008
        force = 5.0 if (jaw < 0.0 and at_puck) or (log and log[-1]['held']) else 0.0
        out = agent.step({'q_actual': q.tolist(), 'jaw_pos': jaw, 'contact_force': force, 'puck_pos': puck,
                          'puck_yaw': 0.0, 'is_cylinder': False,
                          # The policy always saw an obstacle puck in training (run_demo.py: puck + (60, 115, 0) mm);
                          # without one the residual sits at its limit. A far one stands in until the camera gives it.
                          'other_puck_pos': [puck[0] + 0.060, puck[1] + 0.115, puck[2]]})
        q = torch.tensor(out['q_target'], dtype=torch.float32)
        target_jaw = out['jaw_target'] if 'jaw_target' in out else (rk.JAW_CLOSE if out['gripper_action'] > 0 else rk.JAW_OPEN)
        jaw += max(-0.0015, min(0.0015, target_jaw - jaw))
        if force > 0 and jaw < 0.0:
            jaw = max(jaw, -0.0)                 # the puck stops the jaws
        q75 = to75(q.tolist(), M)
        bad = [n for n, v, (a, b) in zip(plan.Chain.NAMES, q75, lo) if not a - 1e-6 <= v <= b + 1e-6]
        log.append({'k': k, 'phase': out['phase_name'], 'tcp': rk.fk_tcp(q.unsqueeze(0))[0].numpy(), 'q75': q75,
                    'res': out['raw_residual'], 'held': force > 0 and jaw <= 0.0, 'bad': bad, 'ik': out.get('ik_ok', True)})
        if out['phase_name'] in ('HOLD', 'GIVEUP'):
            break
    phases = []
    for e in log:
        if not phases or phases[-1][0] != e['phase']:
            phases.append([e['phase'], e['k'], e['k']])
        phases[-1][2] = e['k']
    print('phases (30 Hz steps):', ', '.join(f'{p} {a}-{b}' for p, a, b in phases))
    close = next((e for e in log if e['phase'] == 'CLOSE'), log[-1])
    print(f'TCP at CLOSE: x {close["tcp"][0] * 1000:.0f} y {close["tcp"][1] * 1000:+.0f} z {close["tcp"][2] * 1000:.0f} mm '
          f'(grasp point {puck[0] * 1000:.0f}, {puck[1] * 1000:+.0f}, {(puck[2] + 0.015) * 1000:.0f})')
    print('that pose in our joints (V75): ' + ', '.join(f'{v:.4f}' for v in close['q75']) + '  plan grasp: ' + ', '.join(f'{v:.4f}' for v in g75))
    res = np.array([[float(x) for x in e['res']] for e in log])
    print(f'residual |raw| mean {np.abs(res).mean():.2f}, max {np.abs(res).max():.2f} (x beta 0.3 = +-0.3 of a step)')
    nbad = sum(1 for e in log if e['bad']); nik = sum(1 for e in log if not e['ik'])
    print(f'steps outside our joint limits: {nbad}' + (f' (first {log[[e["bad"] != [] for e in log].index(True)]["bad"]})' if nbad else '') + f'; IK failures: {nik}; end phase {log[-1]["phase"]} after {len(log)} steps ({len(log) / 30:.1f} s)')
    np.save(HERE / 'rl_shadow_traj.npy', np.array([e['q75'] for e in log]))
    print(f'trajectory (V75 joints, 30 Hz): {HERE / "rl_shadow_traj.npy"}')


if __name__ == '__main__':
    main()
