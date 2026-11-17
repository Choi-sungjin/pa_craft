#!/usr/bin/env python3
import math
import os
import select
import time
import unittest
from pathlib import Path
import numpy as np
import yaml
from hardware_sim import Board, EXTRINSICS, N, ROOT, box_segments, load_world, raycast, scan_geometry, write_ports

class HardwareTest(unittest.TestCase):
    def test_wall_origin_and_roll(self):
        poses=np.tile([0.,0.,0.],(N,1)); seg=np.array([[1.,-10.,1.,10.]])
        for side, ext in EXTRINSICS.items():
            ox,oy,dx,dy=scan_geometry(poses,ext)
            i=int(round((ext[3]%(2*math.pi))/(2*math.pi)*N))%N
            dist=raycast(seg,ox,oy,dx,dy)
            self.assertAlmostEqual(dist[i],1-ext[0],places=3)
            # A positive raw angle points clockwise after the upside-down transform.
            self.assertLess(dy[(i+1)%N],dy[i])
    def test_box(self):
        seg=box_segments([[1.,0.,45.]])
        d=raycast(seg,np.array([0.]),np.array([0.]),np.array([1.]),np.array([0.]))
        self.assertAlmostEqual(d[0],1-.205/math.sqrt(2),places=6)
    def test_pty_watchdog_and_noise(self):
        with open(ROOT/'src/craft_control/config/control.yaml') as f: cfg=yaml.safe_load(f)['control_master']['ros__parameters']
        b=Board(cfg,[0.,0.,0.]); fd=os.open(b.port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
        try:
            os.write(fd,b'CONFIG,1\n2,2,0.5\n'); time.sleep(.12)
            with b.lock: self.assertGreater(b.pose[0],.005); self.assertEqual(b.clock,.5)
            data=b''; deadline=time.monotonic()+.2
            while time.monotonic()<deadline:
                if select.select([fd],[],[],.03)[0]: data+=os.read(fd,8192)
            rows=[r.split(b',') for r in data.splitlines() if len(r.split(b','))==12]
            self.assertGreater(len(rows),2); self.assertGreater(len({r[8] for r in rows}),1)
            time.sleep(.52)
            with b.lock: p=b.pose.copy()
            time.sleep(.12)
            with b.lock: self.assertAlmostEqual(b.pose[0],p[0],places=5)
        finally: os.close(fd); b.close()
    def test_real_map_speed(self):
        world,boxes=load_world('/home/craft/maps/noboxes_legend_opt_arena.yaml',ROOT/'src/craft_planning/config/planning.yaml')
        self.assertEqual(len(boxes),9)
        poses=np.tile([.648,1.,math.pi],(N,1)); start=time.perf_counter()
        for ext in EXTRINSICS.values(): world.cast(*scan_geometry(poses,ext))
        print('pair seconds',round(time.perf_counter()-start,4))

if __name__=='__main__': unittest.main()
