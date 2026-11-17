#!/usr/bin/env python3
"""Exercise the real LiDAR bridge callback with fake clock/TF and raw scans."""
import math, unittest
from sensor_msgs.msg import LaserScan
from uppercam_test_lidar_tf_retry import Fake

class BridgeEndpointTest(unittest.TestCase):
 def scan(self, offset=.034, missing=False):
  m=LaserScan();m.header.frame_id='laser_frame';m.header.stamp.sec=100
  m.angle_min=0.;m.angle_increment=2*math.pi/400;m.range_min=.05;m.range_max=12.
  values=[]
  for i in range(400):
   a=m.angle_min+i*m.angle_increment;c=math.cos(a);r=.110/c if c>0 else math.inf
   y=r*math.sin(a) if math.isfinite(r) else math.inf
   values.append(r if abs(y-offset)<=.1025 else math.inf)
  if missing:
   positive=[i for i in range(1,200) if math.isfinite(values[i])]
   values[max(positive)+1]=0.
  m.ranges=values;return m
 def test_observed_centre_replaces_prior_without_changing_gap(self):
  node=Fake();node.buffer.available=True;node.request.expected_box.position.x=.3675
  node.request.front_face_x=.165
  node.scan(self.scan());self.assertEqual(len(node.output),1)
  result=node.output[0][1];self.assertTrue(result['valid']);self.assertTrue(result['face_centre_observed'],result)
  self.assertAlmostEqual(result['box_map'][1],.034,delta=.002)
  self.assertAlmostEqual(result['gap_m'],.1,delta=.001)
  self.assertAlmostEqual(result['lateral_m'],.034,delta=.002)
 def test_missing_endpoint_keeps_valid_gap_and_prior_centre(self):
  node=Fake();node.buffer.available=True;node.request.expected_box.position.x=.3675
  node.request.front_face_x=.165
  node.scan(self.scan(missing=True));result=node.output[0][1]
  self.assertTrue(result['valid']);self.assertFalse(result['face_centre_observed'])
  self.assertIn('centre:',result['reason']);self.assertAlmostEqual(result['box_map'][1],0.,delta=.001)
  self.assertAlmostEqual(result['gap_m'],.1,delta=.001)
if __name__=='__main__':unittest.main(verbosity=2)
