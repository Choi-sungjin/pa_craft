#!/usr/bin/env python3
"""Offline Python/C++ parity and shoulder-last invariants."""
import json
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'demo/puck_pick'))
import fold_motion as fold

q = [-0.03972205786964742, 1.6642803331929867, 0.5741673686148951,
     1.1349305716494746, 2.1305753757442503]
t = q.copy(); t[1] += .02; t[2] += .02; t.append(.01)
cases = [dict(name='standfind 2:', start=q, target=t),
         dict(name='standfind 2:', start=q, target=[q[0], q[1]+.02, *q[2:]]),
         dict(name='standfind 2:', start=q, target=[q[0], q[1], q[2]+.02, *q[3:]]),
         dict(name='standfind 2:', start=None, target=t),
         dict(name='standfind 2:', start=q, target=t, box_known=False),
         dict(name='standfind 2:', start=q, target=t, held=True),
         dict(name='unrelated', start=None, target=t),
         dict(name='standfind 2:', start=[0.]*5, target=[.001,.01,.01,0.,0.]),
         dict(name='standfind 2:', start=q, target=[.2,*t[1:]]),
         dict(name='standfind 2:', start=q, target=[-.043,*t[1:]])]
proc = subprocess.Popen([sys.argv[1]], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
for case in cases:
    proc.stdin.write(json.dumps(case)+'\n'); proc.stdin.flush()
    cpp = json.loads(proc.stdout.readline())
    try:
        moves, reason = fold.prepare_motion(case['name'],case['start'],case['target'],2.,.01,
                                            case.get('held',False),case.get('box_known',True))
        py = dict(moves=[dict(target=m[0],seconds=m[1],tolerance=m[2],intermediate=m[3]) for m in moves], reason=reason)
    except ValueError as exc:
        py = dict(error=str(exc))
    assert ('error' in py)==('error' in cpp), (case, py, cpp)
    if 'error' not in py:
        assert len(py['moves'])==len(cpp['moves']), (case,py,cpp)
        for a,b in zip(py['moves'],cpp['moves']):
            assert a['intermediate']==b['intermediate'] and a['seconds']==b['seconds'] and a['tolerance']==b['tolerance']
            assert all(math.isclose(x,y,abs_tol=1e-10) for x,y in zip(a['target'],b['target']))
        assert bool(py['reason']) == bool(cpp['reason']), (case,py,cpp)
    if case is cases[0]:
        assert len(py['moves'])==2 and py['moves'][0]['target'][1]==q[1]
        assert py['moves'][0]['target'][5]==t[5] and py['moves'][1]['target']==t
proc.stdin.close(); assert proc.wait()==0
old_fraction = fold.SHOULDER_EARLY_FRACTION
for fraction in (float('nan'), -.01, 1.01):
    fold.SHOULDER_EARLY_FRACTION = fraction
    try: fold.prepare_motion('standfind 2:', q, t, 2., .01)
    except ValueError: pass
    else: raise AssertionError('invalid fraction accepted')
fold.SHOULDER_EARLY_FRACTION = old_fraction
print(f'{len(cases)} Python/C++ cases passed')
