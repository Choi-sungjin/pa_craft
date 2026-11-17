#!/usr/bin/env python3
"""Offline parity for the C++ executor helpers. Pass the built math CLI path as argv[1]."""
import json
import math
import os
import pathlib
import random
import shutil
import subprocess
import sys
import tempfile
import unittest
import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'demo/puck_pick'))
import plan
import teach
import pick_policy
import puck_observations

CLI = str(pathlib.Path(sys.argv.pop(1)).resolve())
CALLS = 0
def run(env=None, **kwargs):
    global CALLS
    CALLS += 1
    return json.loads(subprocess.run([CLI], input=json.dumps(kwargs), text=True, check=True, capture_output=True, env=env).stdout)

def obs(msg):
    return {'frame': msg.header.frame_id, 'items': [dict(label=o.detection.label, confidence=o.detection.confidence, standing=o.standing, axis_valid=o.axis_valid, center=[o.center.x,o.center.y,o.center.z], axis=[o.axis.x,o.axis.y,o.axis.z], height_m=o.height_m, puck_id=o.puck_id) for o in msg.observations]}

def close(a, b):
    if isinstance(a, dict):
        assert a.keys() == b.keys(), (a, b)
        for key in a: close(a[key],b[key])
    elif isinstance(a, list):
        assert len(a) == len(b), (a,b)
        for x,y in zip(a,b): close(x,y)
    elif isinstance(a, (float,int)) and not isinstance(a,bool):
        assert math.isclose(a,b,rel_tol=1e-7,abs_tol=1e-8), (a,b)
    else: assert a == b, (a,b)

class Parity(unittest.TestCase):
    def test_teach(self):
        limits = teach.motor_limits()
        rng = random.Random(1611)
        cases = [[0,0,0,0,0,0],[180,210,320,345,270,45],[-50,150,240,300,-80,7]]
        cases += [[math.degrees(pair[edge]) for pair in limits] for edge in (0,1)]
        cases += [[math.degrees(rng.uniform(lo,hi)) for lo,hi in limits] for _ in range(24)]
        for raw in cases:
            got = run(mode='teach',raw=raw)
            close(got['target'],teach.raw_to_target(raw))
            close(got['raw'],raw)
            close(got['limits'],[list(x) for x in teach.motor_limits()])
    def test_plan(self):
        ch=plan.Chain(plan.URDF)
        rng=random.Random(1612)
        cases=[({}, [.128,-.15,.12],0), (dict(zip(ch.NAMES,[.01,1.3,3.0,-2.0,.4])), [.12,-.25,.13],20), ({}, [.2,-3.,.1],90)]
        for _ in range(32):
            q={n:rng.uniform(*ch.limits(n)) for n in ch.NAMES}
            cases.append((q,[rng.uniform(.08,.18),rng.uniform(-.38,.08),rng.uniform(.04,.28)],rng.uniform(-30,100)))
        for edge in (0,1):
            cases.append(({n:ch.limits(n)[edge] for n in ch.NAMES},[.128,-.2,.12],30*edge))
        for q,tcp,tilt in cases:
            got=run(mode='plan',q=q,tcp=tcp,tilt=tilt)
            close(got['fk'],ch.fk(q).tolist())
            close(got['limits'],{n:list(ch.limits(n)) for n in ch.NAMES})
            close(got['ik'],plan.ik_top_down(ch,np.array(tcp),tilt))
    def test_policy(self):
        cases=['', 'PUCK red_prism 0.7 ahead 100 lateral 10 posture lying height 30\nPUCK green_cylinder 0.4 ahead 180 lateral -20 posture lying height 60', 'PUCK red_prism 0.9 ahead 12 lateral 0 posture unknown', 'PUCK green_prism 0.9 ahead 99 lateral 3 posture standing\nPUCK red_prism 0.9 ahead 99 lateral 3 posture standing', 'PUCK green_cylinder 0.3 ahead 2 lateral 1\nPUCK red_prism 0.9 ahead 300 lateral 0']
        for text in cases:
            for expected in (None,1,2):
                args=dict(mode='policy',text=text,ahead_bias=2.,lateral_bias=-1.)
                if expected is not None: args['expected']=expected
                got=run(**args); found=pick_policy.candidates(text,2.,-1.)
                close(got['candidates'],[p.record() for p in found])
                try: selected=pick_policy.select(found,expected).record()
                except ValueError as e: self.assertEqual(got['error'],str(e))
                else: close(got['selected'],selected)

    def test_policy_boundaries_and_random(self):
        rng=random.Random(1613)
        cases=[]
        for ahead in (-.001,0.,205.,205.001):
            for lateral in (-102.501,-102.5,102.5,102.501):
                for score in (.3999,.4):
                    cases.append(f'PUCK red_prism {score} ahead {ahead} lateral {lateral} posture standing height 60')
        for _ in range(32):
            rows=[]
            for _ in range(rng.randrange(4)):
                cls=rng.choice(('red_prism','green_prism','red_cylinder','green_cylinder','unknown'))
                rows.append(f'PUCK {cls} {rng.uniform(.3,1):.4f} ahead {rng.uniform(-5,210):.4f} lateral {rng.uniform(-110,110):.4f} posture {rng.choice(("standing","lying","unknown"))} height 60')
            cases.append('\r\n'.join(rows))
        cases += ['PUCK red_prism 0.9 ahead 1 lateral 0 posture standing\n'*3,
                  '  PUCK red_cylinder 0.9 ahead 100 lateral 0',
                  'PUCK red_cylinder 0.9 ahead 100 lateral 0\vPUCK green_prism 0.8 ahead 90 lateral 1 posture lying']
        for separator in ('\n','\r','\r\n','\v','\f','\x1c','\x1d','\x1e','\x85','\u2028','\u2029'):
            cases.append('PUCK red_cylinder 0.9 ahead 100 lateral 0'+separator+'PUCK green_prism 0.8 ahead 90 lateral 1 posture lying')
        for text in cases:
            kwargs=dict(ahead_bias=0.,lateral_bias=0.,face_x=257.67,laser_y=.02,min_score=.4)
            got=run(mode='policy',text=text,**kwargs)
            found=pick_policy.candidates(text,**kwargs)
            close(got['candidates'],[p.record() for p in found])
            try: selected=pick_policy.select(found).record()
            except ValueError as e: self.assertEqual(got['error'],str(e))
            else: close(got['selected'],selected)
    def test_observations(self):
        pose=str(ROOT/'tools/realsense_check/upper_cam_pose.txt')
        for text in ('', 'PUCK red_prism 0.8 ahead 33 lateral -20 top 60 posture lying\nPUCK green_cylinder 0.4 ahead 60 lateral 10 top 80 posture unknown', 'PUCK green_prism 0.9 ahead 10 lateral 1 top 20'):
            close(run(mode='upper',text=text,pose=pose,ahead_bias=2.,lateral_bias=-1.),obs(puck_observations.upper_observations(text,pose,2.,-1.)))
        for text in ('', 'OK: YAW -12.5 deg CENTRE 1 -2 250 mm 8/10 frames VIZ_AXIS 1 0 -0.2', 'OK: LYING YAW 30 deg CENTRE 0 3 123 mm 0/0 frames VIZ_SHAPE cylinder'):
            close(run(mode='grip',text=text),obs(puck_observations.grip_observations(text)))

    def test_runtime_paths_and_control(self):
        with tempfile.TemporaryDirectory(prefix='puck_parity_') as d:
            root=pathlib.Path(d)
            here=root/'demo/puck_pick'; here.mkdir(parents=True)
            control=root/'src/craft_control/config/control.yaml'; control.parent.mkdir(parents=True)
            urdf=root/'src/craft_planning/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf'
            urdf.parent.mkdir(parents=True); shutil.copy2(plan.URDF,urdf)
            data=yaml.safe_load(plan.CONTROL_YAML.read_text())
            c=data['control_master']['ros__parameters']; c['lm_motor_offset']+=.2; c['arm_joint_offsets'][0]-=.17
            control.write_text(yaml.safe_dump(data))
            env=dict(os.environ,CRAFT_REPO=str(root)); env.pop('PUCK_PICK_DIR',None)
            expected=dict(here=str(here),ws=str(root),control=str(control),urdf=str(urdf))
            close(run(env=env,mode='paths'),expected)
            env['PUCK_PICK_DIR']=str(here); env['CRAFT_REPO']='/unused'
            close(run(env=env,mode='paths'),expected)
            old=teach.CONTROL_YAML
            try:
                teach.CONTROL_YAML=control
                close(run(env=env,mode='teach',raw=[100,200,300,280,180,10])['target'],teach.raw_to_target([100,200,300,280,180,10]))
            finally: teach.CONTROL_YAML=old

    def test_mod(self):
        for a in (-720.,-360.,-180.,-1.,0.,1.,180.,359.,360.,720.):
            for b in (-360.,180.,360.):
                close(run(mode='mod',a=a,b=b)['value'],a%b)

if __name__ == '__main__':
    result=unittest.main(exit=False)
    print(f'{CALLS} Python/C++ parity inputs checked (offline only)')
    raise SystemExit(not result.result.wasSuccessful())
