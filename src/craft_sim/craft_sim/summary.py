#!/usr/bin/env python3
"""runs/ 의 시나리오별 최신 결과를 표로 요약 (터미널 + runs/summary.html)."""
import html
import json
import os
import sys
from pathlib import Path



def main():
    runs = Path(sys.argv[1] if len(sys.argv) > 1 else os.environ.get('CRAFT_SIM_OUT', str(Path.home() / 'craft_sim_runs')))
    latest = {}
    for d in sorted(runs.glob('*/result.json')):
        r = json.loads(d.read_text())
        latest[r['name']] = (d.parent, r)
    rows = []
    for name, (d, r) in sorted(latest.items()):
        for a in r.get('metrics', {}).get('actions', []) or [{}]:
            fb = a.get('final_box', {})
            rows.append([name, '통과' if r.get('pass') else '실패',
                         ('-' if 'truth_ok' not in fb else ('허용' if fb['truth_ok'] else '초과')), a.get('result', '-'), a.get('elapsed_s', '-'),
                         a.get('xte_rmse_mm', '-'), a.get('xte_max_mm', '-'),
                         fb.get('gap_mm', a.get('final_goal_dist_mm', '-')), fb.get('lateral_mm', '-'), fb.get('yaw_err_deg', '-'),
                         a.get('collisions', '-'), str(d / 'report.html')])
    head = ['시나리오', '판정', '참값', '결과', '시간[s]', '횡오차RMSE[mm]', '횡오차최대[mm]', '최종간격/남은거리[mm]', '최종횡[mm]', '최종방향[°]', '충돌', '보고서']
    w = [max(len(str(x)) for x in col) for col in zip(head, *rows)] if rows else []
    for row in [head] + rows:
        print('  '.join(str(x).ljust(n) for x, n in zip(row[:-1], w[:-1])))
    doc = ['<!doctype html><meta charset="utf-8"><title>시뮬레이션 요약</title><style>body{font:14px system-ui,sans-serif;margin:16px}'
           'table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:4px 8px}</style><h1>시뮬레이션 요약</h1><table><tr>']
    doc += [f'<th>{html.escape(h)}</th>' for h in head] + ['</tr>']
    for row in rows:
        doc.append('<tr>' + ''.join(f'<td>{html.escape(str(x))}</td>' for x in row[:-1]) +
                   f'<td><a href="{html.escape(Path(row[-1]).relative_to(runs).as_posix())}">열기</a></td></tr>')
    doc.append('</table>')
    (runs / 'summary.html').write_text(''.join(doc))


if __name__ == '__main__':
    main()
