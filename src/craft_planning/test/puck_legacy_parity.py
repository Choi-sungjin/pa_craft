#!/usr/bin/env python3
"""Compare scenario generators without starting ROS, cameras, or robot control.

Usage: source install/setup.bash && python3 src/craft_planning/test/puck_legacy_parity.py /path/to/puck_legacy_cli
"""
import json
import os
import shutil
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml

ROOT = Path(__file__).resolve().parents[3]
CLI = sys.argv[1] if len(sys.argv) > 1 else 'puck_legacy_cli'
CASES = [('plan', []), ('unload', []), ('bigbro', []), ('bigbro', ['--plan', 'plan_cam.yaml', '--slot', '5']),
         ('gripzero', []), ('gripzero', ['--plan', 'plan_cam.yaml'])]
CASES += [('dice', [str(n)]) for n in range(1, 7)]
CASES += [('dice', ['cal', 'motor']), ('dice', ['6', '--dice-ex']), ('dice', ['6', '--dice-0329'])]
CASES += [('math', []), ('stand', []), ('find', []), ('find', ['--no-record'])]
CASES += [('stand', ['--cell', str(c)]) for c in (2, 5, 8)]
CASES += [('standcal', ['--cell', str(c)]) for c in (2, 5, 8)]
CASES += [('lie', ['--cell', str(c), *flags]) for c in (2, 5) for flags in ([], ['--pick'], ['--lie-ref'])]
CASES += [('dice_line', [row]) for row in ('near', 'middle', 'far')]


def python_result(mode, args):
    # Reimport in a separate interpreter: execute.py reads flags at import time.
    script = '''import sys,json
sys.path.insert(0, sys.argv.pop(1))
mode=sys.argv.pop(1)
import execute as e
if "PUCK_PICK_DIR" in __import__("os").environ: e.HERE=__import__("pathlib").Path(__import__("os").environ["PUCK_PICK_DIR"])
if mode == 'dice_line': result=e.dice_line(e.dice_lm('centre')[0],sys.argv[1])
elif mode == 'math':
 q=e.stand_grasp(5); app=e.approach_at(112.5,q[0],q[4]); result={'nominal':e.nominal_pose(),'via':e.via_pose(),'carry':e.carry_via_path(),'below':e.below(e.load_pose(),.005),'above':e.above(e.load_pose(),.005),'jaw':e.jaw_fix(q),'shift':e.shift_ahead(q,2.),'line':e.line_up(q,3.,1),'descent':e.descent(q,app),'wait':[e.wait_at(a,q[0]) for a in (52.5,97.,157.)]}
else:
 functions={'plan':e.steps_from_plan,'lying':e.lying_steps_from_plan,'dice':e.dice_steps,'unload':e.unload_steps,'bigbro':e.bigbro_steps,'gripzero':e.gripzero_steps,'gripcal':e.gripcal_steps,'stand':e.stand_steps,'find':e.find_steps,'standcal':e.standcal_steps,'lie':e.lie_steps}
 result=functions[mode](None,e.nominal_pose())
 result=[(n,{'standfind':100,'standcal':101,'pause':102,'lieyaw':103,'standyaw':104,'gripcheck':105,'gripzero':106,'gripcal':107}.get(k,k),a) for n,k,a in result]
print(json.dumps(result))'''
    return json.loads(subprocess.check_output([sys.executable, '-c', script, str(ROOT / 'demo/puck_pick'), mode, *args], text=True))


def equal(a, b, where=''):
    if isinstance(a, dict) and isinstance(b, dict):
        assert a.keys() == b.keys(), (where, a.keys(), b.keys())
        for key in a: equal(a[key], b[key], f'{where}/{key}')
    elif isinstance(a, list) and isinstance(b, list):
        assert len(a) == len(b), (where, len(a), len(b))
        for i, (x, y) in enumerate(zip(a, b)):
            equal(x, y, f'{where}/{i}')
    elif isinstance(a, (int, float)) and not isinstance(a, bool) and isinstance(b, (int, float)) and not isinstance(b, bool):
        assert math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-9), (where, a, b)
    else:
        assert a == b, (where, a, b)


for mode, args in CASES:
    py = python_result(mode, args)
    cpp = json.loads(subprocess.check_output([CLI, mode, *args], text=True))
    equal(py, cpp, mode + ' ' + ' '.join(args))
    print('PASS', mode, *args)

# Exercise actual replan output and all frames using only temporary files.
with tempfile.TemporaryDirectory(prefix='puck-replan-parity-') as directory:
    temp = Path(directory)
    for side, ahead, x, distance, height, grip, roll, approach in [
        (0., 0., -.01, .13, .07, 52., 0., .04),
        (4., -4., -.01, .13, .07, 34., 0., .04),
        (-3., 3., -.01, .12, .07, 38., .15, .04),
    ]:
        source = temp / 'source.yaml'
        source.write_text(yaml.safe_dump({'assume': {'puck_ahead': distance, 'puck_x_offset': x, 'shelf_height': height,
            'grasp_coverage': grip / 60., 'puck_height': .06, 'tool_roll': roll, 'approach_above': approach}}))
        cpp = temp / 'cpp.yaml'; py = temp / 'py.yaml'
        subprocess.run([CLI, 'replan', str(source), str(cpp), str(side), str(ahead)], check=True, stdout=subprocess.PIPE, text=True)
        subprocess.run([sys.executable, str(ROOT / 'demo/puck_pick/plan.py'), '--puck-ahead', f'{distance + ahead / 1000:.4f}',
            '--puck-x', f'{x + side / 1000:.4f}', '--shelf-height', f'{height:.4f}', '--grip-mm', f'{grip:.1f}',
            '--tool-roll', f'{roll:.4f}', '--approach-above', f'{approach:.3f}', '--out', str(py)],
            check=True, stdout=subprocess.PIPE, text=True)
        equal(yaml.safe_load(py.read_text()), yaml.safe_load(cpp.read_text()), 'replan')
        print('PASS replan', side, ahead, grip, roll)

# gripcal regenerates plans: use a temporary workspace so no user plan is overwritten.
with tempfile.TemporaryDirectory(prefix='puck-gripcal-parity-') as directory:
    temp = Path(directory); demo = temp / 'demo/puck_pick'; demo.mkdir(parents=True)
    (temp / 'src').symlink_to(ROOT / 'src', target_is_directory=True)
    for name in ('plan.py', 'plan.yaml'):
        shutil.copyfile(ROOT / 'demo/puck_pick' / name, demo / name)
    prior = os.environ.get('PUCK_PICK_DIR'); os.environ['PUCK_PICK_DIR'] = str(demo)
    prior_home = os.environ.get('HOME'); os.environ['HOME'] = str(temp / 'home')
    try:
        py = python_result('gripcal', [])
        cpp = json.loads(subprocess.check_output([CLI, 'gripcal'], text=True))
        equal(py, cpp, 'gripcal')
        print('PASS gripcal (temporary workspace)')
    finally:
        if prior is None: os.environ.pop('PUCK_PICK_DIR', None)
        else: os.environ['PUCK_PICK_DIR'] = prior
        if prior_home is None: os.environ.pop('HOME', None)
        else: os.environ['HOME'] = prior_home
