#!/usr/bin/python3
"""Box top size from one rs_snap capture: the top plane from the inner (eroded) box pixels only (no side faces,
no flying edge pixels), then the 4 image corners of the box outline cast onto that plane; side lengths in mm."""
import sys
import numpy as np
from PIL import Image
from scipy import ndimage

def corners_from_mask(m):
    r, c = np.nonzero(m)
    s, t = r + c, r - c
    return np.array([[c[np.argmin(s)], r[np.argmin(s)]], [c[np.argmin(t)], r[np.argmin(t)]],      # TL, TR
                     [c[np.argmax(s)], r[np.argmax(s)]], [c[np.argmax(t)], r[np.argmax(t)]]], float)  # BR, BL

def main(P):
    w, h, fx, fy, cx, cy, sc, ser = open(P + '_meta.txt').read().split()
    w, h = int(w), int(h); fx, fy, cx, cy = map(float, (fx, fy, cx, cy))
    d = np.fromfile(P + '_depth.u16', dtype='<u2').reshape(h, w) * float(sc)
    img = Image.open(P + '_color.ppm').convert('RGB'); hsv = np.asarray(img.convert('HSV')).astype(float)
    H, S, V = hsv[..., 0] * 360 / 255, hsv[..., 1] / 255, hsv[..., 2] / 255
    card = (H > 30) & (H < 62) & (S > 0.2) & (S < 0.65) & (V > 0.3) & (V < 0.8) & ~((d > 0) & (d < 0.55))
    card = ndimage.binary_opening(card, iterations=1)
    lab, n = ndimage.label(card)
    box = lab == (np.argmax(ndimage.sum(card, lab, range(1, n + 1))) + 1)
    box = ndimage.binary_fill_holes(ndimage.binary_closing(box, iterations=3))
    inner = ndimage.binary_erosion(box, iterations=12) & (d > 0)
    r, c = np.nonzero(inner); z = d[inner]
    X = np.column_stack([(c - cx) * z / fx, (r - cy) * z / fy, z])
    cen = X.mean(0); nrm = np.linalg.svd(X - cen, full_matrices=False)[2][2]
    for _ in range(3):  # trim outliers, refit
        keep = np.abs((X - cen) @ nrm) < 3 * np.abs((X - cen) @ nrm).std() + 1e-4
        X = X[keep]; cen = X.mean(0); nrm = np.linalg.svd(X - cen, full_matrices=False)[2][2]
    K = corners_from_mask(box)
    rays = np.column_stack([(K[:, 0] - cx) / fx, (K[:, 1] - cy) / fy, np.ones(4)])
    P3 = rays * ((cen @ nrm) / (rays @ nrm))[:, None]
    side = [np.linalg.norm(P3[(i + 1) % 4] - P3[i]) * 1000 for i in range(4)]
    print(f'{P}: corners (px) TL {K[0].astype(int)} TR {K[1].astype(int)} BR {K[2].astype(int)} BL {K[3].astype(int)}, '
          f'bottom margin {h - 1 - int(K[2:, 1].max())} px')
    print(f'  top plane rms {np.abs((X - cen) @ nrm).std() * 1000:.1f} mm from {len(X)} inner pixels; '
          f'sides far {side[0]:.0f}  right {side[1]:.0f}  near {side[2]:.0f}  left {side[3]:.0f} mm (spec 205)')
    print(f'  width (far/near avg) {(side[0] + side[2]) / 2:.0f} mm, depth (left/right avg) {(side[1] + side[3]) / 2:.0f} mm'
)
    return dict(far=side[0], right=side[1], near=side[2], left=side[3], width=(side[0] + side[2]) / 2,
                depth=(side[1] + side[3]) / 2, bottom_margin=h - 1 - int(K[2:, 1].max()),
                plane_rms=np.abs((X - cen) @ nrm).std() * 1000, box_dist=np.linalg.norm(cen))

if __name__ == '__main__':
    for P in sys.argv[1:]:
        main(P)
