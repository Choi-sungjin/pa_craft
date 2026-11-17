#!/usr/bin/env python3
"""Wait for a complete simulated tour and save evidence from the actual planner/plant run."""
import argparse
import json
from pathlib import Path
import re
import time
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--timeout',type=float,default=180.)
    args=p.parse_args()
    here=Path(__file__).resolve().parent
    world=yaml.safe_load((here/'world.yaml').read_text())
    count=len(world['goals']);end=time.monotonic()+args.timeout
    while time.monotonic()<end:
        try:
            report=here/'runs/report.json'
            started_file=here/'runs/run_started.json'
            started=json.loads(started_file.read_text())['started'] if started_file.exists() else 0
            if report.stat().st_mtime < started:
                time.sleep(.2);continue
            d=json.loads(report.read_text())
        except (FileNotFoundError,json.JSONDecodeError):
            time.sleep(.2);continue
        if d['collisions'] or d['planner_state']=='Fault':
            raise SystemExit('FAIL: simulated collision or planner fault; inspect runs/*.log')
        if d['reached']>=count:
            log=(here/'runs/planning.log').read_text()
            timings=[float(v) for v in re.findall(r'path plan ([\d.]+) ms',log)]
            assert len(timings)>d['path_executions'],'No evidence of a global replan beyond initial navigation tasks'
            assert d['path_updates']>0,'No planner path updates received'
            assert all(n>0 for n in d['ray_counts'].values()),'Missing LiDAR returns'
            assert d['max_tick_seconds']<.5,'Sensor processing exceeded planner watchdog'
            result=dict(result='PASS',goals_reached=d['reached'],collisions=d['collisions'],
                        global_plans=len(timings),initial_navigation_tasks=d['path_executions'],
                        replans_beyond_initial_task_plans=len(timings)-d['path_executions'],
                        path_updates=d['path_updates'],planning_ms=timings,
                        max_sensor_tick_ms=round(1000*d['max_tick_seconds'],2),
                        approx_min_body_clearance_m=d['min_body_clearance_approx_m'],
                        scope='actual C++ planner/controllers; ideal wheel plant and localization; URDF LiDAR; no cameras')
            (here/'runs/verification.json').write_text(json.dumps(result,indent=2)+'\n')
            (here/'runs/validated_run.json').write_text(json.dumps(d,indent=2)+'\n')
            print(json.dumps(result,indent=2));return
        time.sleep(.2)
    raise SystemExit('FAIL: tour verification timed out; inspect runs/*.log')


if __name__=='__main__':
    main()
