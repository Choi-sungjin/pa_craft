import math
import unittest
import numpy as np
from sensors import Grid, cast, sensor_pose, merge_scans, footprint_collision, box_distance

class SensorsTest(unittest.TestCase):
    def setUp(self):
        self.grid=Grid(np.zeros((100,100),bool),.1,(-5,-5,0))
        self.obj={'enabled':False}
        self.rng=np.random.default_rng(1)
    def test_wall_and_occlusion(self):
        self.grid.occupied[:,70]=True
        a,r=cast(self.grid,(0,0,0),(0,0,0,0),self.obj,self.rng,0,0)
        self.assertAlmostEqual(r[180],2.,delta=.06)
        obj={'enabled':True,'x':1.,'y':0.,'width':.2,'depth':.2,'height':.2}
        _,r=cast(self.grid,(0,0,0),(0,0,0,0),obj,self.rng,0,0)
        self.assertAlmostEqual(r[180],.9,places=5)
    def test_rear_transform_and_height(self):
        x,y,yaw,z=sensor_pose((1,2,math.pi/2),(-.1,0,.05,math.pi))
        self.assertAlmostEqual(x,1); self.assertAlmostEqual(y,1.9); self.assertAlmostEqual(yaw,3*math.pi/2)
        obj={'enabled':True,'x':1,'y':0,'width':.2,'depth':.2,'height':.05}
        self.assertTrue(np.isinf(box_distance(0,0,np.array([1.]),np.array([0.]),obj,.084)[0]))
    def test_noise_dropout_and_no_nan(self):
        self.grid.occupied[:,70]=True
        _,r=cast(self.grid,(0,0,0),(0,0,0,0),self.obj,self.rng,.003,.3)
        self.assertFalse(np.isnan(r).any()); self.assertGreater(np.isinf(r).sum(),10)
        self.assertTrue(np.all((r[np.isfinite(r)]>=.1)&(r[np.isfinite(r)]<=8)))
    def test_below_min_range_is_missing_not_invalid_number(self):
        obj={'enabled':True,'x':.08,'y':0.,'width':.04,'depth':.10,'height':.2}
        _,r=cast(self.grid,(0,0,0),(0,0,0,0),obj,self.rng,0,0)
        self.assertTrue(np.isinf(r[180]))
        self.assertTrue(np.all(r[np.isfinite(r)]>=.1))
    def test_merged_extrinsics(self):
        m=merge_scans({'front_laser':(np.array([0.]),np.array([1.]))},{'front_laser':(.2,.2,0,0)})
        idx=int((math.atan2(.2,1.2)+math.pi)/(2*math.pi)*720)
        self.assertAlmostEqual(m[idx],math.hypot(1.2,.2))
    def test_rotated_map_and_collision(self):
        a=np.zeros((20,20),bool); a[5,10]=True
        grid=Grid(a,.1,(1,2,math.pi/2))
        self.assertTrue(grid.query(.45,3.05))
        self.assertTrue(footprint_collision(grid,(.45,3.05,0),(-.05,.05,-.05,.05),self.obj))
        self.assertFalse(footprint_collision(grid,(0,0,0),(-.05,.05,-.05,.05),self.obj))
if __name__=='__main__': unittest.main()
