#!/usr/bin/env python3
"""Real sensors -> explicit planner observation contract; never publishes motor commands.

Run under ROS environment plus pyrealsense2, numpy, scipy, cv2. The camera is
opened at startup and retained across request changes until shutdown.
Unverified calibration remains diagnostic-only; no last-good-value recycling.
"""
import argparse, json, math, queue, threading, time, sys
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from craft_interfaces.msg import BoxObservationRequest,BoxObservation
from sensor_msgs.msg import LaserScan,CompressedImage
from std_msgs.msg import String
import tf2_ros
from uppercam_fast5_robust_estimator import Fast5RobustUppercamEstimator
from uppercam_square_far20_estimator import SquareFar20Estimator
from uppercam_geometry_pool import OrderedGeometryQueue,new_geometry_pool,warm_worker
from uppercam_lidar_geometry import fit_box_face
from uppercam_bridge_utils import (camera_box_in_map,acquisition_valid,planar_pose,
                                   load_validated_square_manifest,square_observation_policy)

def sec(stamp):return stamp.sec+stamp.nanosec*1e-9

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

def quaternion_rotation(q):
 x,y,z,w=q.x,q.y,q.z,q.w
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

class SensorBridge(Node):
 def __init__(self,args):
  super().__init__('box_sensor_bridge');self.args=args;self.request=None;self.lock=threading.Lock()
  self.highest_request_id=-1;self.last_lidar_stamp_ns=None;self.last_camera_stamp_ns=None
  self.square_enabled=bool(getattr(args,'enable_validated_square',False))
  self.estimator_kind=getattr(args,'estimator','square_far20')
  self.camera_timeout=float(getattr(args,'camera_timeout',.30))
  if not math.isfinite(self.camera_timeout) or not 0<self.camera_timeout<=.5:raise ValueError('camera timeout must be >0 and <=0.5 s')
  if self.square_enabled:
   if self.estimator_kind!='square_far20':raise ValueError('validated square requires square_far20 estimator')
   load_validated_square_manifest(args.manifest,Path(sys.modules[SquareFar20Estimator.__module__].__file__).parent,args.calibration,args.urdf)
  self.policy=json.loads(Path(args.policy).read_text())
  self.lidar_min=float(self.policy['operational_raw_min_m']);self.lidar_max=float(self.policy['operational_raw_max_m'])
  if not 0<float(self.policy['spec_range_min_m'])<=self.lidar_min<self.lidar_max<=float(self.policy['spec_range_max_m']):raise ValueError('invalid LiDAR policy')
  self.results=queue.Queue(maxsize=2);self.quit=threading.Event();self.pending_lidar=deque()
  self.buffer=tf2_ros.Buffer();self.listener=tf2_ros.TransformListener(self.buffer,self)
  self.pub=self.create_publisher(BoxObservation,'/perception/box/observation',10)
  self.camera_pub=self.create_publisher(BoxObservation,'/perception/uppercam/observation',10)
  self.last_continuous_stamp_ns=None;self.last_future_stamp_ns=None
  self.camera_front_face_x=float(getattr(args,'camera_front_face_x',.165))
  self.camera_box_side=float(getattr(args,'camera_box_side',.205))
  if not math.isfinite(self.camera_front_face_x) or self.camera_front_face_x<0 or not math.isfinite(self.camera_box_side) or self.camera_box_side<=0:raise ValueError('invalid camera reference geometry')
  self.diag=self.create_publisher(String,'/perception/uppercam/diagnostics',10)
  self.preview_pub=self.create_publisher(CompressedImage,'/perception/uppercam/image_raw/compressed',qos_profile_sensor_data)
  self.preview_queue=queue.Queue(maxsize=1);self.preview_last_at=-float('inf');self.preview_published_at=-float('inf')
  self.preview_thread=threading.Thread(target=self.preview_worker,daemon=True);self.preview_thread.start()
  self.create_subscription(BoxObservationRequest,'/planning/box_observation_request',self.receive_request,
      QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(LaserScan,args.scan_topic,self.scan,qos_profile_sensor_data)
  self.create_timer(.01,self.drain)
  self.create_timer(.005,self.retry_lidar)
  self.worker=threading.Thread(target=self.camera_worker,daemon=True);self.worker.start()
 def receive_request(self,m):
  if m.source=='off':
   # Stop is unconditional, including default/empty geometry and an old ID.
   with self.lock:
    self.highest_request_id=max(self.highest_request_id,m.request_id)
    self.request=m;self.last_lidar_stamp_ns=None;self.last_camera_stamp_ns=None;self.pending_lidar.clear()
   return
  if m.source not in ('lidar','uppercam') or m.header.frame_id!='map':return
  vals=(m.box_side,m.front_face_x,m.expected_box.position.x,m.expected_box.position.y,
        m.expected_box.orientation.x,m.expected_box.orientation.y,m.expected_box.orientation.z,m.expected_box.orientation.w)
  if not all(math.isfinite(v) for v in vals) or m.box_side<=0:return
  with self.lock:
   if m.request_id<=self.highest_request_id:return
   self.highest_request_id=m.request_id;self.request=m;self.last_lidar_stamp_ns=None;self.last_camera_stamp_ns=None;self.pending_lidar.clear()
 def current(self):
  with self.lock:return self.request
 def diagnostic(self,d):
  def clean(v):
   if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
   if isinstance(v,(list,tuple)):return [clean(x) for x in v]
   if isinstance(v,(float,np.floating)) and not math.isfinite(v):return None
   return v
  self.diag.publish(String(data=json.dumps(clean(d),ensure_ascii=False,allow_nan=False,default=str)))
 def publish(self,req,stamp_s,result,stamp_msg=None):
  cur=self.current()
  if not cur or (cur.request_id,cur.source)!=(req.request_id,req.source):return
  m=BoxObservation();m.header.frame_id='map'
  m.header.stamp=(stamp_msg if stamp_msg is not None else Time(nanoseconds=max(0,int(stamp_s*1e9))).to_msg())
  m.request_id=req.request_id;m.source=req.source;m.valid=bool(result.get('valid',False))
  m.reason=str(result.get('reason',''));m.extrapolated=bool(result.get('extrapolated',False))
  m.gap_uncertainty_m=float(result.get('uncertainty_m',1.0))
  timeout=self.camera_timeout if req.source=='uppercam' else .20
  if not acquisition_valid(stamp_s,self.get_clock().now().nanoseconds*1e-9,sec(req.header.stamp),timeout):
   m.valid=False
   if not (req.source=='uppercam' and stamp_s==0):m.reason='acquisition stale, future, or predates request'
  for attr,key in [('gap_m','gap_m'),('lateral_m','lateral_m'),('yaw_rad','yaw_rad')]:
   value=float(result.get(key,0.0));setattr(m,attr,value if math.isfinite(value) else 0.)
   if not math.isfinite(value):m.valid=False;m.reason='nonfinite measurement'
  centre=result.get('box_map',(0.,0.,0.))
  m.box_pose.position.x=float(centre[0]);m.box_pose.position.y=float(centre[1])
  m.box_pose.orientation.z=math.sin(centre[2]/2);m.box_pose.orientation.w=math.cos(centre[2]/2)
  self.pub.publish(m)
 def scan(self,scan):
  req=self.current()
  if not req or req.source!='lidar':return
  stamp=sec(scan.header.stamp)
  stamp_ns=scan.header.stamp.sec*1000000000+scan.header.stamp.nanosec
  with self.lock:
   if self.request is not req:return
   if self.last_lidar_stamp_ns is not None and stamp_ns<=self.last_lidar_stamp_ns:return
  if not acquisition_valid(stamp,self.get_clock().now().nanoseconds*1e-9,sec(req.header.stamp),.20):
   # Invalid acquisition stamps do not advance the accepted-scan watermark.
   self.pending_lidar.clear()
   self.publish(req,stamp,{'valid':False,'reason':'stale scan'},stamp_msg=scan.header.stamp);return
  with self.lock:
   if self.request is not req:return
   if self.last_lidar_stamp_ns is not None and stamp_ns<=self.last_lidar_stamp_ns:return
   self.last_lidar_stamp_ns=stamp_ns
  # Do not block the executor waiting for TF: its subscription must run too.
  if len(self.pending_lidar)>=4:
   self.pending_lidar.clear()
   self.publish(req,stamp,{'valid':False,'reason':'lidar TF wait queue overflow'},stamp_msg=scan.header.stamp);return
  self.pending_lidar.append((req,scan,stamp))
  self.retry_lidar()
 def retry_lidar(self):
  while self.pending_lidar:
   req,scan,stamp=self.pending_lidar[0]
   current=self.current()
   if current is not req:
    self.pending_lidar.popleft();continue
   if not acquisition_valid(stamp,self.get_clock().now().nanoseconds*1e-9,sec(req.header.stamp),.20):
    self.pending_lidar.popleft()
    self.publish(req,stamp,{'valid':False,'reason':'lidar TF unavailable before acquisition deadline'},stamp_msg=scan.header.stamp);continue
   try:
    tf=self.buffer.lookup_transform('map',scan.header.frame_id,Time.from_msg(scan.header.stamp)).transform
    base=self.buffer.lookup_transform('map','base_link',Time.from_msg(scan.header.stamp)).transform
   except tf2_ros.TransformException:
    return
   self.pending_lidar.popleft()
   try:
    ranges=np.asarray(scan.ranges);a=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
    # Conservative operating bound is distinct from the LDS03 specification (50 mm..12 m).
    keep=np.isfinite(ranges)&(ranges>=max(scan.range_min,self.lidar_min))&(ranges<=min(scan.range_max,self.lidar_max))
    pts=np.column_stack((ranges[keep]*np.cos(a[keep]),ranges[keep]*np.sin(a[keep]),np.zeros(keep.sum())))
    pts=pts@quaternion_rotation(tf.rotation).T+np.array([tf.translation.x,tf.translation.y,tf.translation.z])
    p=req.expected_box.position;q=req.expected_box.orientation
    fit=fit_box_face(pts[:,:2],(p.x,p.y,yaw(q)),planar_pose(base),side=req.box_side,front_face_x=req.front_face_x,laser_pose=planar_pose(tf))
    if fit is None:raise ValueError('no unambiguous LiDAR box face')
    centre=fit['box_center_map'];result=dict(valid=True,reason='lidar face fit',box_map=(*centre,fit['box_yaw']),
       gap_m=fit['gap_m'],lateral_m=fit['lateral_m'],yaw_rad=fit['yaw_relative'],uncertainty_m=max(.010,fit['uncertainty_m']))
    self.publish(req,stamp,result,stamp_msg=scan.header.stamp)
   except ValueError as e:self.publish(req,stamp,{'valid':False,'reason':str(e)},stamp_msg=scan.header.stamp)
 def offer_preview(self,colour,rs):
  try:
   if not self.preview_pub.get_subscription_count():return
   now=time.monotonic()
   if now-self.preview_last_at<.5:return
   domain=colour.get_frame_timestamp_domain();stamp=colour.get_timestamp()/1000.
   if domain not in (rs.timestamp_domain.global_time,rs.timestamp_domain.system_time):return
   if not acquisition_valid(stamp,self.get_clock().now().nanoseconds*1e-9,0.,self.camera_timeout):return
   frame=np.asanyarray(colour.get_data()).copy()
   self.preview_queue.put_nowait((stamp,frame));self.preview_last_at=now
  except Exception:return
 def preview_worker(self):
  import cv2
  while not self.quit.is_set():
   try:stamp,frame=self.preview_queue.get(timeout=.1)
   except queue.Empty:continue
   try:
    if time.monotonic()-self.preview_published_at<.5:continue
    ok,encoded=cv2.imencode('.jpg',frame,[int(cv2.IMWRITE_JPEG_QUALITY),80])
    if not ok or not self.preview_pub.get_subscription_count():continue
    if not acquisition_valid(stamp,self.get_clock().now().nanoseconds*1e-9,0.,self.camera_timeout):continue
    msg=CompressedImage();msg.header.frame_id='camera_top_color_optical_frame'
    msg.header.stamp=Time(nanoseconds=int(stamp*1e9)).to_msg();msg.format='jpeg';msg.data=encoded.tobytes()
    self.preview_pub.publish(msg);self.preview_published_at=time.monotonic()
   except Exception:continue
 def camera_worker(self):
  pipe=None;global_options=[];estimator=None;pool=None;ordered=None;rs=None
  def release():
   nonlocal pipe,global_options
   if pipe is not None:
    try:pipe.stop()
    except RuntimeError:pass
    pipe=None
   for sensor,value in global_options:
    try:sensor.set_option(rs.option.global_time_enabled,value)
    except RuntimeError:pass
   global_options=[]
  def stop_pool():
   nonlocal pool,ordered
   if ordered is not None:ordered.clear();ordered=None
   if pool is not None:pool.shutdown(wait=True,cancel_futures=True);pool=None
  def collect():
   if ordered is None:return
   for context,computed in ordered.ready():
    stamp,index=context
    geo,error,processing_ms=computed
    result=estimator.estimate_from_geometry(geo,index,stamp,'camera_top_color_optical_frame',error=error)
    result['processing_ms']=processing_ms
    self.enqueue((None,stamp,result,None))
  def recover_geometry(error):
   nonlocal estimator,pool,ordered
   self.enqueue((None,time.time(),{'valid':False,'state':'geometry_exception','error':str(error)},None))
   try:stop_pool()
   except Exception:pool=None;ordered=None
   estimator=None
   self.quit.wait(1.)
  try:
   while not self.quit.is_set():
    try:
     import pyrealsense2 as rs
     try:
      if estimator is None:
       estimator=(SquareFar20Estimator(self.args.urdf,self.args.calibration) if self.estimator_kind=='square_far20'
                  else Fast5RobustUppercamEstimator(self.args.urdf,self.args.calibration))
      if self.estimator_kind=='square_far20' and pool is None:
       pool=new_geometry_pool(self.args.urdf,self.args.calibration);ordered=OrderedGeometryQueue(pool,limit=2)
       warm=[pool.submit(warm_worker) for _ in range(2)]
       while not all(f.done() for f in warm) and not self.quit.is_set():self.quit.wait(.01)
       if self.quit.is_set():break
       if not all(f.result() for f in warm):raise RuntimeError('geometry worker initialization failed')
     except Exception as e:
      recover_geometry(e);continue
     if pipe is None:
      ctx=rs.context();device=next(d for d in ctx.query_devices() if d.get_info(rs.camera_info.serial_number)==self.args.serial)
      for sensor in device.query_sensors():
       if sensor.supports(rs.option.global_time_enabled):
        global_options.append((sensor,sensor.get_option(rs.option.global_time_enabled)))
        sensor.set_option(rs.option.global_time_enabled,1.)
      pipe=rs.pipeline(ctx);cfg=rs.config();cfg.enable_device(self.args.serial)
      cfg.enable_stream(rs.stream.depth,424,240,rs.format.z16,15)
      cfg.enable_stream(rs.stream.color,640,360,rs.format.bgr8,15)
      profile=pipe.start(cfg);scale=profile.get_device().first_depth_sensor().get_depth_scale();align=rs.align(rs.stream.color)
     try:collect()
     except Exception as e:
      recover_geometry(e);continue
     frames=pipe.poll_for_frames()
     if not frames:
      self.quit.wait(.002);continue
     colour=frames.get_color_frame();depth=frames.get_depth_frame()
     if not colour or not depth:continue
     stamp=depth.get_timestamp()/1000.;domain=depth.get_frame_timestamp_domain()
     if domain not in (rs.timestamp_domain.global_time,rs.timestamp_domain.system_time):
      self.enqueue((None,time.time(),{'valid':False,'state':'unmapped_hardware_timestamp'},None));continue
     self.offer_preview(colour,rs)
     if ordered is not None and len(ordered.pending)>=ordered.limit:continue
     aligned=align.process(frames);d=aligned.get_depth_frame();intr=aligned.get_color_frame().profile.as_video_stream_profile().intrinsics
     try:
      K=np.array([[intr.fx,0,intr.ppx],[0,intr.fy,intr.ppy],[0,0,1.]])
      frame=np.asanyarray(d.get_data())*scale;index=depth.get_frame_number()
      if ordered is not None:
       ordered.submit(frame,K,(stamp,index))
      else:
       begin=time.monotonic();result=estimator.estimate(frame,K,index,stamp,'camera_top_color_optical_frame')
       result['processing_ms']=(time.monotonic()-begin)*1000;self.enqueue((None,stamp,result,None))
     except Exception as e:
      recover_geometry(e);continue
    except Exception as e:
     self.enqueue((None,time.time(),{'valid':False,'state':'camera_exception','error':str(e)},None))
     release();stop_pool()
     if estimator is not None:estimator.reset()
     self.quit.wait(1.)
  finally:
   release();stop_pool()
 def enqueue(self,item):
  try:self.results.put_nowait(item)
  except queue.Full:
   try:self.results.get_nowait()
   except queue.Empty:pass
   self.results.put_nowait(item)
 def publish_continuous(self,capture_stamp,result,now):
  source=result.get('source_stamp_s')
  usable=isinstance(source,(int,float)) and math.isfinite(source) and source>0
  no_blob=(result.get('state')=='invalid' and result.get('valid') is False and
           result.get('error')=='no level blob of box-top size at box-top height')
  stamp=source if usable else (capture_stamp if no_blob and isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) and capture_stamp>0 else 0.)
  future=stamp>now
  if stamp>0:
   stamp_ns=int(stamp*1e9)
   if future:
    if self.last_future_stamp_ns==stamp_ns:return
    self.last_future_stamp_ns=stamp_ns
   else:
    if self.last_continuous_stamp_ns is not None and stamp_ns<=self.last_continuous_stamp_ns:return
    self.last_continuous_stamp_ns=stamp_ns
  m=BoxObservation();m.header.frame_id='base_link';m.header.stamp=Time(nanoseconds=max(0,int(stamp*1e9))).to_msg()
  m.request_id=0;m.source='uppercam';m.extrapolated=bool(result.get('extrapolated',False))
  valid,reason,uncertainty=square_observation_policy(result,self.square_enabled and self.estimator_kind=='square_far20')
  if not usable:
   valid=False
   reason=('uppercam_geometry_not_detected' if no_blob else 'camera source timestamp missing or invalid: '+str(result.get('state','invalid')))
  elif future or not acquisition_valid(stamp,now,0.,self.camera_timeout):valid=False;reason='camera source timestamp stale or future'
  m.valid=valid;m.reason=reason;m.gap_uncertainty_m=float(uncertainty)
  for attr,key,scale in [('gap_m','gap_mm',.001),('lateral_m','lateral_m',1.),('yaw_rad','box_yaw_rad',1.)]:
   try:
    v=float(result.get(key,0.))*scale
    if not math.isfinite(v):raise ValueError('nonfinite')
    setattr(m,attr,v)
   except (TypeError,ValueError):m.valid=False;m.reason='nonfinite camera geometry'
  m.box_pose.orientation.w=1.
  if m.valid:
   try:
    bx,by,heading=camera_box_in_map(m.gap_m,m.lateral_m,m.yaw_rad,(0.,0.,0.),self.camera_front_face_x,self.camera_box_side)
    m.box_pose.position.x=bx;m.box_pose.position.y=by
    m.box_pose.orientation.z=math.sin(heading/2);m.box_pose.orientation.w=math.cos(heading/2)
   except (TypeError,ValueError):m.valid=False;m.reason='invalid camera pose geometry'
  self.camera_pub.publish(m)
 def drain(self):
  try:_,capture_stamp,result,_=self.results.get_nowait()
  except queue.Empty:return
  now=self.get_clock().now().nanoseconds*1e-9
  self.publish_continuous(capture_stamp,result,now)
  req=self.current()
  if not req or req.source!='uppercam':return
  # A pre-request capture or selected median frame is never a new request observation.
  if isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) and capture_stamp<=sec(req.header.stamp):return
  selected=result.get('source_stamp_s')
  if isinstance(selected,(int,float)) and math.isfinite(selected) and selected<=sec(req.header.stamp):return
  source_stamp=result.get('source_stamp_s')
  result['request_id']=req.request_id
  result['capture_age_ms']=((now-capture_stamp)*1000 if isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) else None)
  usable_source=isinstance(source_stamp,(int,float)) and math.isfinite(source_stamp) and source_stamp>0
  if usable_source:result['source_age_ms']=(now-source_stamp)*1000
  if (usable_source and source_stamp>now) or (isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) and capture_stamp>now):
   self.publish(req,0.,dict(valid=False,reason='camera timestamp future',uncertainty_m=1.0));return
  self.diagnostic(result)
  if result.get('state') in ('warmup','warming_up'):return
  # A detected capture with no box-top blob is distinct from camera/TF errors.
  # Its capture time is the only trustworthy time when geometry is absent.
  no_blob=(result.get('state')=='invalid' and result.get('valid') is False and
           result.get('error')=='no level blob of box-top size at box-top height' and
           not result.get('extrapolated',False) and not result.get('current_extrapolated',False) and
           all(result.get(k) is None for k in ('source_stamp_s','source_frame_index','gap_mm','raw_gap_mm','lateral_m','box_yaw_rad')))
  if no_blob and isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) and capture_stamp>0:
   with self.lock:
    if self.request is not req:return
    if self.last_camera_stamp_ns is not None and int(capture_stamp*1e9)<=self.last_camera_stamp_ns:return
  # Before any camera observation, a stale no-detection startup frame carries
  # no usable geometry. Drop it rather than manufacture a stamp=0 failure.
  # The planner remains held and its original acquisition deadline still runs.
  if (no_blob and isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) and
      sec(req.header.stamp)<capture_stamp<=now and now-capture_stamp>self.camera_timeout):
   with self.lock:
    if self.request is req and self.last_camera_stamp_ns is None:return
  usable_capture=(no_blob and isinstance(capture_stamp,(int,float)) and math.isfinite(capture_stamp) and
                  capture_stamp>0 and sec(req.header.stamp)<capture_stamp<=now and
                  now-capture_stamp<=self.camera_timeout)
  if no_blob and usable_capture:
   capture_ns=int(capture_stamp*1e9)
   with self.lock:
    if self.request is not req:return
    if self.last_camera_stamp_ns is not None and capture_ns<=self.last_camera_stamp_ns:return
    self.last_camera_stamp_ns=capture_ns
   self.publish(req,capture_stamp,dict(valid=False,reason='uppercam_geometry_not_detected',uncertainty_m=1.0))
   return
  # A median may legitimately choose the same source frame more than once.
  # Do not publish it as a newer measurement or reset the planner deadline.
  if usable_source:
   source_ns=int(source_stamp*1e9)
   with self.lock:
    if self.request is not req:return
    if self.last_camera_stamp_ns is not None and source_ns<=self.last_camera_stamp_ns:return
    self.last_camera_stamp_ns=source_ns
   stamp=source_stamp
  else:
   stamp=0.0
  enabled=self.square_enabled and self.estimator_kind=='square_far20'
  valid,reason,uncertainty=square_observation_policy(result,enabled)
  if not usable_source:
   valid=False;reason='camera source timestamp missing or invalid: '+str(result.get('state','invalid'))
  data=dict(valid=valid,reason=reason,extrapolated=bool(result.get('extrapolated',False)),uncertainty_m=uncertainty)
  if usable_source and result.get('gap_mm') is not None and result.get('lateral_m') is not None:
   try:
    data.update(gap_m=float(result['gap_mm'])/1000,lateral_m=float(result['lateral_m']),yaw_rad=float(result['box_yaw_rad']))
    tf=self.buffer.lookup_transform('map','base_link',Time(nanoseconds=int(stamp*1e9))).transform
    data['box_map']=camera_box_in_map(data['gap_m'],data['lateral_m'],data['yaw_rad'],planar_pose(tf),req.front_face_x,req.box_side)
   except (ValueError,TypeError,KeyError,tf2_ros.TransformException) as e:data.update(valid=False,reason=str(e))
  self.publish(req,stamp,data)
 def close(self):
  self.quit.set();self.worker.join()
  preview=getattr(self,'preview_thread',None)
  if preview is not None:preview.join()

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--urdf',required=True);ap.add_argument('--calibration',required=True)
 ap.add_argument('--estimator',choices=['square_far20','fast5'],default='square_far20')
 ap.add_argument('--enable-validated-square',action='store_true')
 ap.add_argument('--camera-timeout',type=float,default=.30)
 ap.add_argument('--camera-front-face-x',type=float,default=.165)
 ap.add_argument('--camera-box-side',type=float,default=.205)
 ap.add_argument('--manifest',default=str(Path(__file__).with_name('uppercam_square_far20_validation_manifest.json')))
 ap.add_argument('--serial',default='261322075477');ap.add_argument('--scan-topic',default='/lidar/front/scan')
 ap.add_argument('--policy',default=str(Path(__file__).with_name('uppercam_sensor_policy.json')))
 args,ros=ap.parse_known_args();rclpy.init(args=ros);node=SensorBridge(args)
 try:rclpy.spin(node)
 finally:node.close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
