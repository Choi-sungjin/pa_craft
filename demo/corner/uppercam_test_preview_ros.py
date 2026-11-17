#!/usr/bin/env python3
"""Isolated ROS pub/sub preview verification, no camera or robot required."""
import argparse, os, time, types, unittest
from pathlib import Path
import cv2, numpy as np, rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage
from craft_interfaces.msg import BoxObservationRequest
from uppercam_sensor_bridge import SensorBridge
HERE=Path(__file__).resolve().parent
assert os.environ.get('ROS_DOMAIN_ID')=='88' and os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')=='LOCALHOST'

class Colour:
 def __init__(self,stamp,frame):self.stamp=stamp;self.frame=frame;self.reads=0
 def get_timestamp(self):return self.stamp*1000
 def get_frame_timestamp_domain(self):return 1
 def get_data(self):self.reads+=1;return self.frame
RS=types.SimpleNamespace(timestamp_domain=types.SimpleNamespace(global_time=1,system_time=2))

class PreviewRosTest(unittest.TestCase):
 def setUp(self):
  rclpy.init();self.node=Node('preview_test');self.images=[];self.old=SensorBridge.camera_worker;SensorBridge.camera_worker=lambda s:s.quit.wait()
  args=argparse.Namespace(policy=str(HERE/'uppercam_sensor_policy.json'),estimator='square_far20',enable_validated_square=False,scan_topic='/unused/lidar',serial='none')
  self.bridge=SensorBridge(args);self.exe=SingleThreadedExecutor();self.exe.add_node(self.node);self.exe.add_node(self.bridge)
  self.req=BoxObservationRequest();self.req.header.frame_id='map';self.req.header.stamp=Time(nanoseconds=self.node.get_clock().now().nanoseconds-int(1e9)).to_msg();self.req.request_id=10;self.req.source='uppercam';self.req.box_side=.205;self.req.expected_box.orientation.w=1.;self.bridge.receive_request(self.req)
 def tearDown(self):
  self.bridge.close();self.exe.remove_node(self.bridge);self.exe.remove_node(self.node);self.bridge.destroy_node();self.node.destroy_node();self.exe.shutdown();rclpy.shutdown();SensorBridge.camera_worker=self.old
 def spin(self,t):
  end=time.monotonic()+t
  while time.monotonic()<end:self.exe.spin_once(timeout_sec=.01)
 def test_jpeg_timestamp_subscriber_rate_and_off(self):
  frame=np.zeros((36,64,3),dtype=np.uint8);frame[:,:,1]=190
  colour=Colour(self.node.get_clock().now().nanoseconds*1e-9,frame)
  self.bridge.offer_preview(self.req,colour,RS);self.assertEqual(colour.reads,0)
  self.node.create_subscription(CompressedImage,'/perception/uppercam/image_raw/compressed',lambda m:self.images.append(m),qos_profile_sensor_data);self.spin(.25)
  colour=Colour(self.node.get_clock().now().nanoseconds*1e-9,frame)
  self.bridge.offer_preview(self.req,colour,RS);self.bridge.offer_preview(self.req,colour,RS);self.spin(.2)
  self.assertEqual(colour.reads,1);self.assertEqual(len(self.images),1);m=self.images[0]
  self.assertEqual(m.header.frame_id,'camera_top_color_optical_frame');self.assertEqual(m.format,'jpeg')
  self.assertAlmostEqual(m.header.stamp.sec+m.header.stamp.nanosec*1e-9,colour.stamp,delta=.000002)
  decoded=cv2.imdecode(np.frombuffer(m.data,dtype=np.uint8),cv2.IMREAD_COLOR)
  self.assertEqual(decoded.shape,(36,64,3));self.assertAlmostEqual(float(decoded[:,:,1].mean()),190,delta=3)
  self.spin(.35);stale=Colour(self.node.get_clock().now().nanoseconds*1e-9-2.,frame);self.bridge.offer_preview(self.req,stale,RS);self.spin(.1);self.assertEqual(len(self.images),1)
  fresh=Colour(self.node.get_clock().now().nanoseconds*1e-9,frame);self.bridge.offer_preview(self.req,fresh,RS);self.spin(.2);self.assertEqual(len(self.images),2)
  off=BoxObservationRequest();off.source='off';off.request_id=11;self.bridge.receive_request(off);self.spin(.35)
  fresh=Colour(self.node.get_clock().now().nanoseconds*1e-9,frame);self.bridge.offer_preview(self.req,fresh,RS);self.spin(.2);self.assertEqual(len(self.images),2)
  print('PREVIEW_JPEG_ROS_PASS no_subscriber decode original_stamp 2hz stale off')
if __name__=='__main__':unittest.main(verbosity=2)
