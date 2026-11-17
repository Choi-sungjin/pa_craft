#!/usr/bin/env python3
"""Build the simulation's known occupancy map. Unmapped dynamic objects are excluded."""
from pathlib import Path
import argparse
import numpy as np
from PIL import Image
import yaml


def generate(path):
    path=Path(path).resolve()
    world=yaml.safe_load(path.read_text())
    a=world['arena']; width,height,res,wall,border=[float(a[k]) for k in ('width','height','resolution','wall_thickness','border')]
    if not (width>2*wall>0 and height>2*wall and res>0 and border>=0):
        raise ValueError('Invalid simulation arena dimensions')
    w,h=int(round((width+2*border)/res)),int(round((height+2*border)/res))
    x=-border+(np.arange(w)+.5)*res; y=-border+(np.arange(h)+.5)*res
    xx,yy=np.meshgrid(x,y)
    occupied=(xx<wall)|(xx>width-wall)|(yy<wall)|(yy>height-wall)
    for rect in world.get('static_rectangles',[]):
        occupied|=(abs(xx-rect['x'])<=rect['width']/2)&(abs(yy-rect['y'])<=rect['depth']/2)
    meta=path.parent/world['map_file'];image=meta.with_suffix('.pgm')
    Image.fromarray(np.flipud(np.where(occupied,0,254)).astype(np.uint8)).save(image)
    meta.write_text(yaml.safe_dump(dict(image=image.name,resolution=res,origin=[-border,-border,0.],
                                     negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')))
    return meta


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('world',nargs='?',type=Path,default=Path(__file__).with_name('world.yaml'))
    print(generate(p.parse_args().world))
