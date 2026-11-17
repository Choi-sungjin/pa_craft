"""Validate actual 12-puck inventory against R/Y/G counts. Standard library only."""
import itertools, json, sys

def solve(pucks, counts):
    if len(pucks)!=12 or len({p['id'] for p in pucks})!=12:
        raise ValueError('Exactly 12 distinct puck IDs required')
    if any(p['color'] not in ('R','G') or p['shape'] not in ('원기둥','직육면체') for p in pucks):
        raise ValueError('Invalid puck color/shape')
    if set(counts)!=set('RYG') or any(type(v)!=int or not 1<=v<=6 for v in counts.values()) or sum(counts.values())!=12:
        raise ValueError('Each count 1..6; total 12')
    for ys in itertools.combinations(pucks,counts['Y']):
        if len({p['shape'] for p in ys})!=1: continue
        used={p['id'] for p in ys}
        a={'Y':sorted(used),'R':[],'G':[]}
        for p in pucks:
            if p['id'] not in used:a[p['color']].append(p['id'])
        if len(a['R'])==counts['R'] and len(a['G'])==counts['G']:
            return a
    return None

if __name__=='__main__':
    if len(sys.argv)!=2: raise SystemExit('python validate_problem.py problem.json  (keys: pucks, counts)')
    with open(sys.argv[1],encoding='utf8') as f:d=json.load(f)
    result=solve(d['pucks'],d['counts'])
    print(json.dumps({'solvable':result is not None,'allocation':result},ensure_ascii=False,indent=2))
    raise SystemExit(0 if result else 2)
