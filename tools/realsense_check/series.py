#!/usr/bin/python3
"""Box-gap series: capture at gap N x 10 mm (box near edge ahead of the LiDAR face), analyse, append to the CSV.

  /usr/bin/python3 series.py N [--reuse]   # N = 1..10 -> 10..100 mm; out/series_<mm>mm_*, out/series.csv
"""
import csv, os, subprocess, sys, time
reuse = '--reuse' in sys.argv          # re-analyse the saved capture, no new shot
sys.argv, N = sys.argv[:1], int(sys.argv[1])
import boxsize, occlusion  # noqa: E402

mm = 10 * N
prefix = f'out/series_{mm:03d}mm'
if not reuse:
    subprocess.run(['./build/rs_snap', prefix], check=True)
occ = occlusion.main(prefix)
size = boxsize.main(prefix)
row = dict(n=N, gap_mm=mm, time=time.strftime('%H:%M:%S'), visible_pct=round(occ['visible'], 1),
           hidden_pct=round(occ['hidden'], 1), other_pct=round(occ['other'], 1),
           far_mm=round(size['far'], 1), right_mm=round(size['right'], 1), near_mm=round(size['near'], 1),
           left_mm=round(size['left'], 1), width_mm=round(size['width'], 1), depth_mm=round(size['depth'], 1),
           hidden_strip_mm=round(max(0.0, 205 - size['depth']), 1), bottom_margin_px=size['bottom_margin'],
           plane_rms_mm=round(size['plane_rms'], 2), box_dist_m=round(size['box_dist'], 3))
if reuse:
    import os.path
    row['time'] = time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(prefix + '_meta.txt')))
path = 'out/series.csv'
rows = []
if os.path.exists(path):
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if int(r['n']) != N]   # a retake replaces that gap
rows.append({k: str(v) for k, v in row.items()})
rows.sort(key=lambda r: int(r['n']))
with open(path, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(row)); w.writeheader(); w.writerows(rows)
print(f"[{N}] gap {mm} mm: depth {row['depth_mm']} mm (hidden strip {row['hidden_strip_mm']}), width {row['width_mm']}, "
      f"visible {row['visible_pct']}% hidden {row['hidden_pct']}%, bottom margin {row['bottom_margin_px']} px")
