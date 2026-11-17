#!/usr/bin/env python3
"""Offline fold audit: actual camera handlers with saved observations, FK gate and firmware timing.

No node/device is created. JSON stdout includes every candidate intermediate and per-joint
arrival. With a CLI argument, also compare each prepared move against production C++.
Times assume exact arrival at each previous target; feedback/settling/communication add time.
"""
import builtins
import contextlib
import io
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import yaml

ROOT = Path(__file__).resolve().parents[3]
CLI = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else None
sys.path.insert(0, str(ROOT / 'demo/puck_pick'))
sys.argv = ['execute.py', '--find', '--dry-run', '--no-record', '--auto']
import execute as e  # noqa: E402
import fold_motion as fold  # noqa: E402

params = yaml.safe_load((ROOT / 'src/craft_control/config/control.yaml').read_text())['control_master']['ros__parameters']
assert all(params[k] == 0 for k in ('arm_max_speed_rad_s', 'arm_max_accel_rad_s2', 'lm_max_speed_m_s', 'lm_max_accel_m_s2'))
# CraftMotorFirmware::moveMs/writeTargets; firmware config.h remains unchanged.
caps = [v*.229*2*math.pi/60 for v in (180, 50, 50, 50, 50, 30)]
scales = [1., 1., 1., 1., 1., .7]


def round_tick(x):
    return math.floor(x+.5) if x >= 0 else math.ceil(x-.5)


def motor(q):
    result = [params['lm_motor_offset']+params['lm_motor_direction']*q[0]/params['lm_meters_per_rad']]
    result += [params['arm_joint_offsets'][i]+params['arm_joint_directions'][i]*q[i+1] for i in range(4)]
    if len(q) == 6: result.append(q[5])
    return result


def profile(start, target, seconds):
    a, b = motor(start), motor(target)
    delta = [abs(y-x) for x,y in zip(a,b)]
    changed = [round_tick(x*4096/(2*math.pi)) != round_tick(y*4096/(2*math.pi)) for x,y in zip(a,b)]
    requested_ms = round_tick(seconds*1000)
    needed = [1000*d/(v*.75*s) for d,v,s,c in zip(delta,caps,scales,changed) if c]
    common = min(32767, max(100, math.ceil(max([requested_ms, *needed])))) if any(changed) else 0
    arrival = [max(100, math.floor(common*s))/1000 if c else 0. for c,s in zip(changed,scales)]
    return common/1000, arrival[:5]


def smoothed(steps, previous):
    """Mirror existing smooth_steps without mutating the taught or camera data."""
    e.SLOW_SECONDS.clear(); prev = list(previous); result = []
    home = e.nominal_pose()
    for name,kind,path in steps:
        if kind != e.TaskCommand.MANIPULATOR:
            result.append((name,kind,path)); continue
        into_nominal = ('nominal pose' in name and 'via pose' not in name) or all(abs(a-b)<1e-6 for a,b in zip(path[-1][:5],home[:5]))
        if not (into_nominal or 'via pose' in name):
            prev = path[-1]; result.append((name,kind,path)); continue
        out,times = [],[]
        for j,q in enumerate(path):
            deg = max(abs(q[i]-prev[i]) for i in range(1,5))*180/math.pi
            if into_nominal and j == len(path)-1 and deg > 2*e.APPROACH_DEG:
                f = e.APPROACH_DEG/deg
                out.append([q[i]-(q[i]-prev[i])*f if i<len(prev) else q[i] for i in range(len(q))])
                times.append(max(e.GOTO_MIN_S,(deg-e.APPROACH_DEG)/e.GOTO_DEG_S))
                out.append(q); times.append(max(e.APPROACH_MIN_S,e.APPROACH_DEG/e.APPROACH_DEG_S))
            else:
                speed = e.STAND_VIA_DEG_S if name.startswith(('stand ','standfind ')) else e.VIA_DEG_S
                out.append(q); times.append(max(e.GOTO_MIN_S,deg/(e.GOTO_DEG_S if into_nominal else speed)))
            prev = q
        e.SLOW_SECONDS[name] = times; result.append((name,kind,out))
    return result


def find_case(record):
    data = json.loads((ROOT/'demo/puck_pick/runs'/record/'log.json').read_text())
    steps = smoothed(e.stage(e.find_steps(),e.nominal_pose()),e.nominal_pose())
    e.FIND.clear(); e.POLICY_TARGET = None; e.POLICY_CANDIDATES=[]; e.RUN_DIR=None
    picked,grip = data['upper']['picked'],data['grip']
    readings = [f"PUCK {picked['cls']} {picked['score']} ahead {picked['ahead_mm']+e.UPPER_AHEAD_BIAS_MM} lateral {picked['lateral_mm']+e.UPPER_LATERAL_BIAS_MM} posture standing height 60\n",
                f"saved: YAW {grip['yaw_read']} deg CENTRE {grip['centre'][0]} {grip['centre'][1]} 60 mm\n"]
    for what,text in zip(('upper','grip'),readings):
        e.camera_run = lambda *a, output=text, **kw: SimpleNamespace(stdout=output,stderr='',returncode=0)
        i=next(i for i,s in enumerate(steps) if s[1]==e.STANDFIND and s[2].get('what')==what)
        with contextlib.redirect_stdout(io.StringIO()): ok,reason=e.find_step(steps,i,steps[i][2])
        assert ok,(record,what,reason)
    return steps


def lie_case(cell):
    e.POLICY_TARGET=None; sys.argv=['execute.py','--lie','--cell',str(cell),'--pick','--dry-run','--auto']
    steps=smoothed(e.stage(e.lie_steps(),e.nominal_pose()),e.nominal_pose())
    prefix=ROOT/'tools/realsense_check/out/lieyaw_231533'
    output=subprocess.run([str(ROOT/'tools/realsense_check/build/grip_yaw'),'--stack',str(prefix)],check=True,capture_output=True,text=True).stdout
    # The stored stack supplies yaw; its FLOOR belongs to its own cell and is deliberately omitted here.
    import re
    yaw=re.search(r'LYING YAW ([+-]?[\d.]+) deg',output).group(1)
    e.camera_run=lambda *a,**kw: SimpleNamespace(stdout=f'saved: LYING YAW {yaw} deg\n',stderr='',returncode=0)
    i=next(i for i,s in enumerate(steps) if s[1]==e.LIEYAW)
    with contextlib.redirect_stdout(io.StringIO()): ok,reason=e.lie_yaw_step(steps,i,steps[i][2])
    assert ok,(cell,reason)
    return steps


def recovery_case(current):
    saved_run,saved_input,saved_sleep=e.run_steps,builtins.input,e.time.sleep
    captured=[]
    e.run_steps=lambda node,steps: captured.extend(steps) or True
    builtins.input=lambda *_:''; e.time.sleep=lambda _:None
    node=SimpleNamespace(status=SimpleNamespace(fault=False),joints=SimpleNamespace(name=e.ARM,position=current),stop=lambda:None)
    try:
        with contextlib.redirect_stdout(io.StringIO()): e.go_home(node,e.nominal_pose(),False)
    finally:
        e.run_steps,builtins.input,e.time.sleep=saved_run,saved_input,saved_sleep
    return captured


proc=subprocess.Popen([str(CLI)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True) if CLI else None
rows=[]; scenarios=[]
for record in ('find_1115_190950','find_1115_194832','find_1115_202437'):
    scenarios.append((record,find_case(record),e.nominal_pose(),dict(e.SLOW_SECONDS)))
for cell in (2,5):
    scenarios.append((f'lie cell {cell}',lie_case(cell),e.nominal_pose(),dict(e.SLOW_SECONDS)))
for cell in (2,5,8):
    sys.argv=['execute.py','--stand','--cell',str(cell),'--pick','--dry-run']
    e.POLICY_TARGET=None
    steps=smoothed(e.stage(e.stand_steps(),e.nominal_pose()),e.nominal_pose())
    scenarios.append((f'stand cell {cell}',steps,e.nominal_pose(),dict(e.SLOW_SECONDS)))
q=e.stand_grasp(5)
scenarios.append(('go_home from cell5 grasp',recovery_case(q),q,{}))
for scenario,steps,start,times in scenarios:
    e.SLOW_SECONDS.clear();e.SLOW_SECONDS.update(times)
    prev=list(start); held=False; box_known=True
    for name,kind,path in steps:
        if kind==e.PAUSE:
            if name.startswith('lie regrasp 1:'): box_known=False
            elif name.startswith('lie regrasp 3:'): box_known=True
        if kind==e.TaskCommand.GRIPPER: held=path is True
        if kind!=e.TaskCommand.MANIPULATOR: continue
        for k,target in enumerate(path):
            secs,tol=e.step_seconds(name,k),e.step_tolerance(name)
            if 'the vertical line' in name and k+1<len(path): tol=math.radians(e.VIA_TOLERANCE_DEG)
            if 'pick: descend' in name and len(path)>1:
                secs=e.STAND_DESCEND_PIECE_S if name.startswith('stand pick: descend') else e.DESCEND_PIECE_S
                if k+1<len(path): tol=math.radians(e.DESCEND_PASS_DEG)
            if fold.candidate(name):
                mid=list(target);mid[1]=prev[1]+fold.SHOULDER_EARLY_FRACTION*(target[1]-prev[1])
                before,before_arrival=profile(prev,target,secs)
                moves,reason=fold.prepare_motion(name,prev[:5],target,secs,tol,held,box_known)
                if proc:
                    args=dict(name=name,start=prev[:5],target=target,seconds=secs,tolerance=tol,held=held,box_known=box_known)
                    proc.stdin.write(json.dumps(args)+'\n');proc.stdin.flush();cpp=json.loads(proc.stdout.readline())
                    assert 'error' not in cpp,(scenario,name,cpp)
                    assert len(moves)==len(cpp['moves']),(scenario,name,reason,cpp)
                    for a,b in zip(moves,cpp['moves']):
                        assert len(a[0])==len(b['target']) and all(abs(x-y)<1e-10 for x,y in zip(a[0],b['target']))
                        assert a[1]==b['seconds'] and abs(a[2]-b['tolerance'])<1e-12 and a[3]==b['intermediate']
                after=0.; arrivals=[0.]*5; stage_start=prev
                segment_times=[]
                for q,seconds,tolerance,intermediate in moves:
                    duration,arrival=profile(stage_start,q,seconds)
                    segment_times.append(duration)
                    for j,t in enumerate(arrival):
                        if t: arrivals[j]=after+t
                    after+=duration;stage_start=q
                rows.append(dict(scenario=scenario,name=name,waypoint=k+1,decision='split' if len(moves)==2 else 'unchanged',reason=reason,
                                 start=prev[:5],intermediate=mid,target=target,host_seconds=secs,tolerance_rad=tol,
                                 before_total_s=before,after_total_s=after,before_arrival_s=before_arrival,after_arrival_s=arrivals,segment_times_s=segment_times))
            prev=list(target)
if proc:
    proc.stdin.close();assert proc.wait()==0
print(json.dumps(dict(joints=['LM','shoulder','elbow','wrist_pitch','tool_roll'],
                     timing_assumption='exact previous target; firmware cap/ramp/timeScale and 4096-tick changed-goal detection; excludes settling/feedback/transport',
                     clearance_assumption='sampled visual component OBB/FK; model-only, not continuous collision certification; shelf model 70mm, robot and all 12 carousel slot angles',
                     rows=rows),indent=2,ensure_ascii=False))
