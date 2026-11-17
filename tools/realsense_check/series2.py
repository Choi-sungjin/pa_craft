#!/usr/bin/python3
"""Gap series in base_link (V307 pose, colour lens y +32.2 mm, yaw +1.5 deg): one capture per tape gap.

  /usr/bin/python3 series2.py N      # tape gap N x 10 mm (LiDAR face -> box near edge); out/s2_<mm>mm_*, out/series2.csv
"""
import csv, os, subprocess, sys, time, io, contextlib
import box_range as br

reuse = '--reuse' in sys.argv                     # re-analyse a saved capture (no new shot)
N = int(sys.argv[1]); mm = 10 * N
prefix = f'out/s2_{mm:03d}mm'
if not reuse:
    subprocess.run(['./build/rs_snap', prefix], check=True, stdout=subprocess.DEVNULL)
buf = io.StringIO()
with contextlib.redirect_stdout(buf):
    r = br.analyse_urdf(prefix, br.URDF_DEFAULT, lens_y=0.0322, yaw_deg=br.CAM_YAW_DEG)
Sb = r['S']; far_x = Sb[Sb[:, 0].argsort()[-2:], 0].mean() * 1000   # far (back) edge, always seen
dx, dy = (r['near'][:2] - r['laser'][:2]) * 1000                       # near edge = far edge - 205 mm
row = dict(n=N, tape_mm=mm, time=time.strftime('%H:%M:%S', time.localtime(os.path.getmtime(prefix + '_meta.txt')))
           if reuse else time.strftime('%H:%M:%S'), x1_x0_mm=round(dx, 1), y1_y0_mm=round(dy, 1),
           xy_mm=round(r['xy'] * 1000, 1), x_err_mm=round(dx - mm, 1), far_edge_x_mm=round(far_x, 1),
           near_x_mm=round(far_x - 205, 1))
path = 'out/series2.csv'
rows = []
if os.path.exists(path):
    rows = [{k: x.get(k, '') for k in row} for x in csv.DictReader(open(path)) if int(x['n']) != N]
rows.append({k: str(v) for k, v in row.items()}); rows.sort(key=lambda x: int(x['n']))
with open(path, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(row)); w.writeheader(); w.writerows(rows)
print(f"[{N}] tape {mm} mm: far edge x {far_x:.1f} -> x1 = far - 205 = {far_x - 205:.1f}; x1-x0 {dx:+.1f} "
      f"(err {dx - mm:+.1f}), y1-y0 {dy:+.1f}, XY {r['xy'] * 1000:.1f} mm")
