import sys, math, numpy as np, rosbag2_py, matplotlib
matplotlib.use('Agg'); import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
MOUNT = {'/lidar/front/scan': (0.155, -1, 1, 0.012), '/lidar/rear/scan': (-0.1295, 1, -1, 0.011)}
acc = {k: [] for k in MOUNT}; n = 0
while r.has_next():
    topic, data, t = r.read_next()
    if topic in acc:
        n += 1
        if n % 3: continue
        m = deserialize_message(data, get_message(types[topic]))
        rg = np.array(m.ranges); ang = m.angle_min + np.arange(len(rg)) * m.angle_increment
        ok = np.isfinite(rg) & (rg > m.range_min) & (rg < m.range_max)
        acc[topic].append(np.c_[rg[ok], ang[ok]])
fig, axes = plt.subplots(1, 2, figsize=(15, 6))
for ax, (topic, ch) in zip(axes, acc.items()):
    x0, sx, sy, h = MOUNT[topic]
    R = np.vstack(ch); rg, th = R[:, 0], R[:, 1]
    px, py = rg * np.cos(th), rg * np.sin(th)
    bx, by = x0 + sx * px, sy * py
    floor = (np.hypot(bx - 0.01275, by) >= 0.30) & (px > -0.10) & (rg < 1.8)
    # descent angle model: h / r = c + A cos(th) + B sin(th)  (small angles)
    X = np.c_[np.ones(floor.sum()), np.cos(th[floor]), np.sin(th[floor])]; yv = h / rg[floor]
    (c, A, B), *_ = np.linalg.lstsq(X, yv, rcond=None)
    pred = X @ np.array([c, A, B]); rpred = h / pred
    err = rg[floor] - rpred
    tilt = math.degrees(math.hypot(A, B)); phi = math.degrees(math.atan2(B, A))
    out_rise = -(c + A * math.cos(math.pi) + B * math.sin(math.pi))   # outward = laser -x (th = 180 deg)
    print(f'{topic}: {floor.sum()} floor points  cone c = {math.degrees(c):+.2f} deg (all directions down), '
          f'mount tilt {tilt:.2f} deg toward laser {phi:+.0f} deg; range error rms {np.sqrt(np.mean(err**2))*100:.1f} cm')
    print(f'    straight outward: beam {"rises" if out_rise > 0 else "falls"} {abs(out_rise)*100:.1f} cm per metre '
          f'(7 cm box top under the beam beyond {(0.07 - h) / out_rise:.2f} m)' if out_rise > 0 else
          f'    straight outward: beam falls {abs(out_rise)*100:.1f} cm/m -> floor at {h / -out_rise:.2f} m')
    for side_deg in (90, -90, 135, -135):
        e = c + A * math.cos(math.radians(side_deg)) + B * math.sin(math.radians(side_deg))
        print(f'    beam at laser {side_deg:+4d} deg: ' + (f'floor at {h / e:.2f} m' if e > 0 else f'rises {-e*100:.1f} cm/m, no floor'))
    order = np.argsort(th[floor])
    ax.scatter(np.degrees(th[floor]), rg[floor], s=2, c='#eb6834', label='floor hits')
    tt = np.radians(np.linspace(-180, 180, 721)); ee = c + A * np.cos(tt) + B * np.sin(tt)
    rr = np.where(ee > 0, h / np.maximum(ee, 1e-9), np.nan)
    ax.plot(np.degrees(tt), rr, color='#2a78d6', lw=1.5, label=f'model: cone {math.degrees(c):+.2f} deg + tilt {tilt:.2f} deg')
    ax.set_ylim(0, 2.5); ax.set_xlim(-180, 180); ax.grid(alpha=.3); ax.set_xlabel('beam angle in laser frame [deg] (0 = toward robot centre, +-180 = outward)')
    ax.set_ylabel('range to floor [m]'); ax.set_title(topic); ax.legend(fontsize=9)
fig.tight_layout(); fig.savefig(sys.argv[2], dpi=80)
