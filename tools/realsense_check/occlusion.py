#!/usr/bin/python3
"""How much of the box in front of the gripper the gripper hides, from one rs_snap capture.

  /usr/bin/python3 occlusion.py out/snap     # reads out/snap_{color.ppm,depth.u16,meta.txt}

  box pixels:  cardboard colour (HSV hue 30-62 deg, sat 0.2-0.65, val 0.3-0.8 under this camera's white balance),
               farther than near_m, blobs > 80 px in the lower half, within 0.10 m of the main blob's depth
  gripper:     depth < near_m, or no depth and dark (the black gripper parts return no depth), the connected
               blob(s) touching the box outline
  box outline: convex hull of the visible pieces plus the bottom-right corner completed as a parallelogram
               (top edge + left edge); still a lower bound if the whole bottom edge is hidden
Writes <prefix>_occlusion.png and prints visible / hidden / other percentages.
"""
import sys

import numpy as np
from matplotlib.path import Path
from PIL import Image
from scipy import ndimage
from scipy.spatial import ConvexHull

NEAR_M = 0.55
BOX_BAND_M = 0.10   # box pixels within this of the main box blob's depth


def main(prefix):
    w, h, fx, fy, cx, cy, sc, ser = open(prefix + '_meta.txt').read().split()
    w, h = int(w), int(h)
    d = np.fromfile(prefix + '_depth.u16', dtype='<u2').reshape(h, w) * float(sc)
    img = Image.open(prefix + '_color.ppm').convert('RGB'); img.save(prefix + '_color.png')
    rgb = np.asarray(img)
    hsv = np.asarray(img.convert('HSV')).astype(float)
    H, S, V = hsv[..., 0] * 360 / 255, hsv[..., 1] / 255, hsv[..., 2] / 255
    near = (d > 0) & (d < NEAR_M)
    grip = near | ((d == 0) & (V < 0.3))
    card = (H > 30) & (H < 62) & (S > 0.2) & (S < 0.65) & (V > 0.3) & (V < 0.8) & ~near
    card = ndimage.binary_opening(card, iterations=1)
    lab, n = ndimage.label(card)
    box = np.zeros_like(card)
    for i in range(1, n + 1):
        r, c = np.nonzero(lab == i)
        if len(r) > 80 and r.mean() > h * 0.4:
            box |= lab == i
    if box.sum() < 50:
        print('no box found (cardboard colour in the lower half)'); return 1
    # Same colour farther away (a hand or skin behind the box) is not the box: keep the depth band of the largest blob.
    lab, n = ndimage.label(box)
    big = lab == (np.argmax(ndimage.sum(box, lab, range(1, n + 1))) + 1)
    z0 = np.median(d[big & (d > 0)])
    box &= (d == 0) | (np.abs(d - z0) < BOX_BAND_M)
    lab, n = ndimage.label(box)
    for i in range(1, n + 1):
        if (lab == i).sum() < 80:
            box[lab == i] = False
    r, c = np.nonzero(box)
    # The box face is a parallelogram in the image (205 mm square, seen at a slant): from the top edge corners and
    # the visible bottom-left corner, the hidden bottom-right corner is TR + (BL - TL).
    top = r <= r.min() + 4
    tl = np.array([c[top].min(), r.min()]); tr = np.array([c[top].max(), r.min()])
    k = np.argmax(r - np.abs(c - tl[0])); bl = np.array([c[k], r[k]])
    k = np.argmax(r - np.abs(c - tr[0])); br_seen = np.array([c[k], r[k]])
    br = tr + (bl - tl) if bl[1] >= br_seen[1] else br_seen
    pts = np.vstack([np.column_stack([c, r]), br[None, :]]); poly = pts[ConvexHull(pts).vertices]
    yy, xx = np.mgrid[0:h, 0:w]
    inside = Path(poly).contains_points(np.column_stack([xx.ravel(), yy.ravel()])).reshape(h, w)
    glab, _ = ndimage.label(ndimage.binary_closing(grip, iterations=2))
    ids = np.unique(glab[ndimage.binary_dilation(inside, iterations=3)]); ids = ids[ids > 0]
    gr = np.isin(glab, ids) & grip
    A = inside.sum(); vis = (inside & box).sum(); occ = (inside & gr & ~box).sum(); oth = A - vis - occ
    zbox = np.median(d[box & (d > 0)]) if (box & (d > 0)).any() else float('nan')
    free = sum(1 for col in range(c.min(), c.max() + 1) if not (gr[:, col] & inside[:, col]).any())
    print(f'D435 {ser}: box {A} px (rows {r.min()}-{r.max()}, cols {c.min()}-{c.max()}), depth {zbox:.3f} m')
    print(f'visible {100 * vis / A:.0f}%  hidden by gripper {100 * occ / A:.0f}%  other {100 * oth / A:.0f}%  '
          f'(clear columns {free}/{c.max() - c.min() + 1})')
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    ov = rgb.astype(float) / 255
    ov[inside & box] = ov[inside & box] * .35 + np.array([1, .6, 0]) * .65
    ov[inside & gr & ~box] = ov[inside & gr & ~box] * .3 + np.array([1, 0, 0]) * .7
    ov[gr & ~inside] = ov[gr & ~inside] * .5 + np.array([0, .4, 1]) * .5
    fig, ax = plt.subplots(1, 2, figsize=(15, 4.6), dpi=110)
    ax[0].imshow(ov); ax[0].plot(*np.vstack([poly, poly[:1]]).T, 'y-', lw=1.5)
    ax[0].set_title(f'box: visible {100 * vis / A:.0f}% (orange), hidden {100 * occ / A:.0f}% (red), other {100 * oth / A:.0f}%')
    im = ax[1].imshow(np.where(d > 0, d, np.nan), cmap='turbo', vmin=0, vmax=1.5)
    ax[1].set_title('aligned depth [m]'); plt.colorbar(im, ax=ax[1])
    fig.tight_layout(); fig.savefig(prefix + '_occlusion.png')
    return dict(visible=100 * vis / A, hidden=100 * occ / A, other=100 * oth / A, box_px=int(A))


if __name__ == '__main__':
    out = main(sys.argv[1] if len(sys.argv) > 1 else 'out/snap')
    sys.exit(out if isinstance(out, int) else 0)
