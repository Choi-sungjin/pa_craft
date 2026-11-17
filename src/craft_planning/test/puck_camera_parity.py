#!/usr/bin/env python3
"""Offline parity of C++ camera step handlers against execute.py with fake detector output."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
CLI = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'build/craft_planning/puck_camera_cli'
DEMO = ROOT / 'demo/puck_pick'
sys.path.insert(0, str(DEMO))
sys.argv = ['execute.py', '--find', '--cell', '5', '--pick', '--dry-run']
spec = importlib.util.spec_from_file_location('execute_camera_parity', DEMO / 'execute.py')
py = importlib.util.module_from_spec(spec)
spec.loader.exec_module(py)

UPPER = 'PUCK red_prism 0.95 ahead 132.6 lateral -12.0 top 60 posture standing height 60\n'
STANDING = 'sample: YAW 12.0 deg CENTRE 10.0 20.0 60.0 mm\n'
LYING = 'sample: LYING YAW 12.0 deg\n'
BAD = 'sample: no puck\n'

def cpp(mode, output, env):
    p = subprocess.run([str(CLI), mode], input=output, text=True, capture_output=True, env=env, check=True)
    return json.loads(next(line[7:] for line in p.stdout.splitlines() if line.startswith('RESULT ')))

def run():
    with tempfile.TemporaryDirectory(prefix='puck-camera-parity-') as tmp:
        fakehome = Path(tmp)
        real_cal = ROOT / 'tools/realsense_check/stand_cal.json'
        caldir = fakehome / 'craft/tools/realsense_check'
        caldir.mkdir(parents=True)
        shutil.copy(real_cal, caldir / 'stand_cal.json')
        env = dict(os.environ, HOME=tmp, PUCK_PICK_DIR=str(DEMO))
        py.STAND_CAL = caldir / 'stand_cal.json'
        py.STAND_YAW_SAVE = caldir
        py.RUN_DIR = None
        q = py.wait_at(112.5, py.dice_lm('centre')[0])
        ref = json.loads((caldir / 'stand_cal.json').read_text())['5']['grip_ref']
        grip = f"sample: YAW {ref['yaw_deg']:.4f} deg CENTRE {ref['centre_mm'][0]:.4f} {ref['centre_mm'][1]:.4f} 60.0 mm\n"
        stored_prefix = ROOT / 'demo/puck_pick/runs/find_1115_194832/grip_wait_194848'
        stored = subprocess.run([str(ROOT/'tools/realsense_check/build/grip_yaw'), '--stack', str(stored_prefix)], capture_output=True, text=True, check=True).stdout
        assert ': YAW ' in stored, stored
        lie_prefix = ROOT / 'tools/realsense_check/out/lieyaw_231533'
        stored_lie = subprocess.run([str(ROOT/'tools/realsense_check/build/grip_yaw'), '--stack', str(lie_prefix)], capture_output=True, text=True, check=True).stdout
        assert 'LYING YAW ' in stored_lie, stored_lie
        checks = 0
        for mode, output in [('lie_yaw', stored_lie), ('grip', stored), ('stand_yaw', stored), ('grip', grip), ('grip', BAD), ('stand_yaw', STANDING), ('stand_yaw', BAD), ('lie_yaw', LYING), ('lie_yaw', BAD), ('upper', UPPER), ('photo', BAD), ('standcal_grip', STANDING), ('standcal_grip', BAD), ('grip3', BAD)]:
            py.camera_run = lambda *a, **k: SimpleNamespace(stdout=output, stderr='', returncode=0)
            py.POLICY_TARGET = None
            py.POLICY_CANDIDATES = []
            py.STANDCAL_SHOTS.clear()
            py.FIND.clear()
            if mode == 'stand_yaw':
                steps = [('wait', py.TaskCommand.MANIPULATOR, [q]), ('camera', py.STANDYAW, None), ('stand pick: descend', py.TaskCommand.MANIPULATOR, [q])]
                call = lambda: py.stand_yaw_step(steps, 1)
            elif mode == 'lie_yaw':
                steps = [('wait', py.TaskCommand.MANIPULATOR, [q]), ('camera', py.LIEYAW, None), ('lie pick: descend', py.TaskCommand.MANIPULATOR, [q])]
                call = lambda: py.lie_yaw_step(steps, 1, {'cell': 5, 'roll0': py.lie_grasp(5)[4]})
            elif mode == 'upper':
                steps = [('upper', py.STANDFIND, {'what': 'upper'}), ('standfind 2: wait', py.TaskCommand.MANIPULATOR, [q])]
                call = lambda: py.find_step(steps, 0, steps[0][2])
            elif mode == 'grip':
                py.FIND.update(cls='red_prism', ahead=112.5, lat=0., lm=q[0])
                steps = [('grip', py.STANDFIND, {'what':'grip'}), ('standfind 4: approach', py.TaskCommand.MANIPULATOR, [q]), ('stand pick: descend', py.TaskCommand.MANIPULATOR, [q]), ('stand pick: up to 75 mm', py.TaskCommand.MANIPULATOR, [q])]
                call = lambda: py.find_step(steps, 0, steps[0][2])
            elif mode == 'photo':
                steps = [('photo', py.STANDFIND, {'what': 'photo', 'tag': 'grasp_open'})]
                call = lambda: py.find_step(steps, 0, steps[0][2])
            elif mode == 'standcal_grip':
                steps = []
                call = lambda: py.standcal_step({'what':'grip', 'tag':'ref', 'dlm':0., 'dahead':0.})
            else:
                steps = [('final', py.STANDFIND, {'what':'grip3'})]
                py.FIND.update(ahead2=112.5, lm2=q[0], roll=q[4])
                call = lambda: py.find_step(steps, 0, steps[0][2])
            with contextlib.redirect_stdout(io.StringIO()):
                ok, _ = call()
            c = cpp(mode, output, env)
            assert c['ok'] == ok, (mode, output, c, ok)
            if mode in ('stand_yaw', 'lie_yaw') and ok:
                assert len(c['steps']) == len(steps), (mode,c['steps'],steps)
                for a,b in zip(c['steps'][-1]['arg'][-1],steps[-1][2][-1]):
                    assert abs(a-b)<1e-8, (mode,a,b)
            if mode == 'grip' and ok:
                for a_step, b_step in zip(c['steps'][1:], steps[1:]):
                    for a_pose, b_pose in zip(a_step['arg'], b_step[2]):
                        for a, b in zip(a_pose, b_pose):
                            assert abs(a-b) < 1e-7, (mode,a,b)
            if mode == 'standcal_grip' and ok:
                assert c['shots']['ref'] == py.STANDCAL_SHOTS['ref']
            checks += 1
        py.STANDCAL_SHOTS.clear()
        for tag, centre, dlm, dahead in [('ref',[10.,20.,60.],0.,0.),('lm+10',[20.,20.,60.],10.,0.),('lm-10',[0.,20.,60.],-10.,0.),('ahead+10',[10.,30.,60.],0.,10.),('ahead-10',[10.,10.,60.],0.,-10.)]:
            py.STANDCAL_SHOTS[tag] = dict(centre=centre,yaw=2.,dlm=dlm,dahead=dahead)
        with contextlib.redirect_stdout(io.StringIO()):
            ok, _ = py.standcal_step(dict(what='solve', cell=5, row='middle', ahead=112.5, lm=q[0]))
        original_result = json.loads((caldir/'stand_cal.json').read_text())['5']
        shutil.copy(real_cal,caldir/'stand_cal.json')
        c = cpp('standcal_solve','',env)
        port_result = json.loads((caldir/'stand_cal.json').read_text())['5']
        assert ok and c['ok']
        for key in ('row','ahead_mm','lm','grasp','grip_ref','cam_per_arm','fit_rms_mm','shots','grasp_ref','upper'):
            assert port_result[key] == original_result[key], (key,port_result[key],original_result[key])
        checks += 1
        for command,timeout,expect in [(['/usr/bin/printf','fake detector'], 2, 'ok'), (['/bin/sh','-c','exit 7'], 2, 'exit'), (['/bin/sleep','2'], .05, 'timeout'), (['/tmp/nonexistent-puck-camera-command'], 2, 'missing')]:
            result=cpp('process', json.dumps({'argv':command,'timeout':timeout}),env)
            if expect == 'ok': assert result['returncode']==0 and result['out']=='fake detector', result
            elif expect == 'exit': assert result['returncode']==7, result
            elif expect == 'timeout': assert 'timed out' in result['error'], result
            else: assert 'spawn failed' in result['error'], result
            checks += 1
        print(f'camera handler parity: {checks} cases passed')
if __name__ == '__main__':
    run()
