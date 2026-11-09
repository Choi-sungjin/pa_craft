#!/usr/bin/env python3
"""Real isolated ROS pub/sub/TF; inject deterministic camera results without hardware."""
import argparse,os,sys,time,unittest
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile,DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from craft_interfaces.msg import BoxObservationRequest,BoxObservation
from uppercam_sensor_bridge import SensorBridge
HERE=Path(__file__).resolve().parent
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'
class SquareRosTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  rclpy.init();cls.client=Node('square_contract_test');cls.received=[]
  cls.client.create_subscription(BoxObservation,'/perception/box/observation',lambda m:cls.received.append(m),50)
  cls.pub=cls.client.create_publisher(BoxObservationRequest,'/planning/box_observation_request',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));cls.tf=TransformBroadcaster(cls.client)
  cls.saved_worker=SensorBridge.camera_worker;SensorBridge.camera_worker=lambda self:self.quit.wait()
  args=argparse.Namespace(policy=str(HERE/'uppercam_sensor_policy.json'),estimator='square_far20',enable_validated_square=True,
    manifest=str(HERE/'uppercam_square_far20_validation_manifest.json'),calibration=str(HERE/'uppercam_selected_calibration.json'),
    urdf=str(HERE.parents[1]/'src/craft_description/urdf/v307_full_robot.urdf'),scan_topic='/unused/lidar',serial='none')
  cls.bridge=SensorBridge(args);cls.exec=SingleThreadedExecutor();cls.exec.add_node(cls.client);cls.exec.add_node(cls.bridge);cls.spin(.15)
 @classmethod
 def tearDownClass(cls):
  cls.bridge.close();cls.exec.remove_node(cls.bridge);cls.exec.remove_node(cls.client);cls.bridge.destroy_node();cls.client.destroy_node();cls.exec.shutdown();rclpy.shutdown();SensorBridge.camera_worker=cls.saved_worker
 @classmethod
 def spin(cls,duration):
  stop=time.monotonic()+duration
  while time.monotonic()<stop:cls.exec.spin_once(timeout_sec=.005)
 def request(self,ident,age=.20):
  q=BoxObservationRequest();q.header.frame_id='map';q.header.stamp=Time(nanoseconds=self.client.get_clock().now().nanoseconds-int(age*1e9)).to_msg();q.request_id=ident;q.source='uppercam';q.box_side=.205;q.front_face_x=.20967;q.expected_box.orientation.w=1.;self.pub.publish(q);self.spin(.025);self.assertEqual(self.bridge.current().request_id,ident);return self.bridge.current()
 def transform(self,stamp,x):
  m=TransformStamped();m.header.stamp=Time(nanoseconds=int(stamp*1e9)).to_msg();m.header.frame_id='map';m.child_frame_id='base_link';m.transform.translation.x=x;m.transform.rotation.w=1.;self.tf.sendTransform(m)
 def inject(self,req,source,capture,gap=20.,extrap=False):
  result=dict(state='geometry_unverified',method='square_far20_spec_reimplementation_v1',provenance='spec_reimplementation',
              gap_mm=gap,lateral_m=.02,box_yaw_rad=0.,source_stamp_s=source,extrapolated=extrap,valid=False)
  self.bridge.enqueue((req,capture,result,None))
 def wait_message(self,before):
  self.spin(.06);new=self.received[before:];self.assertTrue(new);return new[-1]
 def test_01_source_pose_repeat_stale(self):
  q=self.request(1001);now=self.client.get_clock().now().nanoseconds*1e-9;source=now-.1;capture=now-.01
  self.transform(source,.1);self.transform(capture,2.);self.spin(.025)
  before=len(self.received);self.inject(q,source,capture);m=self.wait_message(before)
  print('NORMAL',m.valid,m.reason,'x',m.box_pose.position.x,'gap',m.gap_m,flush=True)
  self.assertTrue(m.valid,m.reason);self.assertAlmostEqual(m.gap_m,.020,places=6);self.assertAlmostEqual(m.gap_uncertainty_m,.005)
  self.assertAlmostEqual(m.box_pose.position.x,.1+.20967+.020+.1025,delta=.0001)
  self.assertAlmostEqual(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,source,delta=.000002)
  before=len(self.received);self.inject(q,source,capture);self.inject(q,source-.001,capture);self.spin(.08);self.assertEqual(len(self.received),before)
  q=self.request(1002,age=.55);now=self.client.get_clock().now().nanoseconds*1e-9;source=now-.4;self.transform(source,.1);self.spin(.02)
  before=len(self.received);self.inject(q,source,now);m=self.wait_message(before);self.assertFalse(m.valid);self.assertIn('stale',m.reason)
 def test_02_range_and_extrapolation(self):
  q=self.request(1003);now=self.client.get_clock().now().nanoseconds*1e-9;self.transform(now-.04,.1);self.transform(now,.1);self.spin(.02)
  for i,(gap,extra,valid,unc) in enumerate([(30.,False,True,.005),(100.,True,True,.010),(9.99,True,True,.005),(200.01,True,False,1.),(-.01,True,False,1.)]):
   before=len(self.received);self.inject(q,now-.04+i*.005,now,gap,extra);m=self.wait_message(before)
   self.assertEqual(m.valid,valid,(gap,m.reason));self.assertAlmostEqual(m.gap_uncertainty_m,unc)
   self.assertEqual(m.extrapolated,extra)
 def test_03_missing_source_is_immediate_invalid_zero_stamp(self):
  q=self.request(1004);now=self.client.get_clock().now().nanoseconds*1e-9
  for bad in (None,float('nan')):
   before=len(self.received)
   self.bridge.enqueue((q,now,dict(state='camera_exception',source_stamp_s=bad,error='fixture'),None))
   m=self.wait_message(before)
   self.assertFalse(m.valid);self.assertEqual(m.header.stamp.sec,0);self.assertEqual(m.header.stamp.nanosec,0)
   self.assertIn('camera_exception',m.reason)
 def test_04_geometry_miss_publishes_capture_stamp_and_suppresses_repeat(self):
  q=self.request(1005);capture=self.client.get_clock().now().nanoseconds*1e-9-.02
  result=dict(valid=False,state='invalid',source_stamp_s=None,error='no level blob of box-top size at box-top height')
  before=len(self.received);self.bridge.enqueue((q,capture,result.copy(),None));m=self.wait_message(before)
  self.assertFalse(m.valid);self.assertEqual(m.reason,'uppercam_geometry_not_detected');self.assertEqual(m.gap_m,0.)
  self.assertAlmostEqual(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,capture,delta=.000002)
  before=len(self.received);self.bridge.enqueue((q,capture,result.copy(),None));self.spin(.08);self.assertEqual(len(self.received),before)
if __name__=='__main__':unittest.main(verbosity=2)
