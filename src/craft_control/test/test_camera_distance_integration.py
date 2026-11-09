#!/usr/bin/env python3
"""Camera distance control against PTY fake boards, isolated ROS domain only."""
import math, os, time
from pathlib import Path
os.environ.setdefault('ROS_DOMAIN_ID', '87')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
os.environ.setdefault('ROS_STATIC_PEERS', '')
import rclpy
from geometry_msgs.msg import PoseStamped
from craft_interfaces.msg import TaskCommand, TaskResult
from test_planning_watchdog_integration import Harness


def command(h, task_id, gap, revision=1, update=False, publish=True):
 c=TaskCommand(task_id=task_id,revision=revision,operation=TaskCommand.UPDATE_PATH if update else TaskCommand.EXECUTE,
               kind=TaskCommand.NAVIGATE,speed=.005,tracking_reference_x=.165,lookahead_distance=.02,
               camera_distance_control=True,camera_gap_m=gap,camera_target_gap_m=.01,
               camera_observation_stamp=h.node.get_clock().now().to_msg())
 c.path.header.frame_id='map'
 for x in (0.,.01):
  p=PoseStamped();p.pose.position.x=x;p.pose.orientation.w=1.;c.path.poses.append(p)
 if publish: h.task_pub.publish(c)
 return c

def start():
 h=Harness(Path(os.environ['CRAFT_CONTROL_BINARY']),sim_time=False)
 try:
  h.pump(lambda:h.status is not None and h.status.ready and h.status.stopped,hold=True)
  h.pump(lambda:not h.status.held,hold=False,commands=True)
  h.run_for(.12,hold=False,commands=True); return h
 except BaseException:
  h.close();raise

def main():
 rclpy.init(); h=start(); scenario=os.environ['CAMERA_CASE']
 try:
  if scenario=='speed':
   command(h,301,.08)
   h.pump(lambda:301 in h.results and h.drive.command is not None and h.drive.command[0]>.12,hold=False,commands=True)
   assert h.results[301].status==TaskResult.RUNNING, str(h.results[301])
   command(h,301,.018,2,True)
   h.pump(lambda:h.results[301].revision==2,hold=False,commands=True)
   h.pump(lambda:abs(h.drive.command[0]-.004/.033)<.002,hold=False,commands=True)
   command(h,301,.014,3,True)
   h.pump(lambda:h.results[301].revision==3,hold=False,commands=True)
   h.pump(lambda:abs(h.drive.command[0]-.002/.033)<.002,hold=False,commands=True)
   command(h,301,.0129,4,True)
   h.pump(lambda:h.results[301].revision==4 and abs(h.drive.command[0])<.001,hold=False,commands=True)
   print('CAMERA_SPEED_PASS capped=.005 proportional=.004,.002 stop=.0129 fixed_odom')
  elif scenario in ('replay','older'):
   c=command(h,302,.08);h.pump(lambda:302 in h.results,hold=False,commands=True)
   c.operation=TaskCommand.UPDATE_PATH;c.revision=2
   if scenario=='older':
    ns=c.camera_observation_stamp.sec*1000000000+c.camera_observation_stamp.nanosec-10000000
    c.camera_observation_stamp.sec=ns//1000000000;c.camera_observation_stamp.nanosec=ns%1000000000
   else:c.camera_gap_m=.07
   h.task_pub.publish(c)
   h.pump(lambda:h.results[302].status==TaskResult.FAILED,hold=False,commands=True)
   h.pump(lambda:h.board_holds(h.drive),hold=True,commands=True)
   print('CAMERA_'+scenario.upper()+'_STOP_PASS')
  elif scenario=='invalid':
   for task_id,mode in [(401,'negative'),(402,'nan'),(403,'future'),(404,'stale'),(405,'overspeed')]:
    c=command(h,task_id,.08,publish=False)
    if mode=='negative':c.camera_gap_m=-.01
    elif mode=='nan':c.camera_gap_m=math.nan
    elif mode=='future':c.camera_observation_stamp.sec+=1
    elif mode=='stale':c.camera_observation_stamp.sec-=2
    else:c.speed=.1
    h.task_pub.publish(c)
    h.pump(lambda:task_id in h.results,hold=False,commands=True)
    assert h.results[task_id].status==TaskResult.REJECTED
   print('CAMERA_INVALID_INITIAL_REJECT_PASS negative,nan,future,stale,overspeed')
  elif scenario=='pause':
   c=command(h,303,.08);h.pump(lambda:303 in h.results,hold=False,commands=True)
   c.operation=TaskCommand.UPDATE_PATH;c.revision=2;c.speed=0.0;h.task_pub.publish(c)
   h.pump(lambda:h.results[303].revision==2,hold=False,commands=True)
   c.revision=3;c.speed=.005;h.task_pub.publish(c)
   h.pump(lambda:h.results[303].revision==3,hold=False,commands=True)
   h.pump(lambda:h.results[303].status==TaskResult.FAILED,hold=False,commands=True,timeout=2)
   h.pump(lambda:h.board_holds(h.drive),hold=True,commands=True)
   print('CAMERA_PAUSE_REUSE_AND_STALE_STOP_PASS')
  elif scenario=='engage':
   # camera_only_longitudinal: a plain dock along the LiDAR line, then one update turns camera distance control on.
   c=command(h,304,.08,publish=False);c.camera_distance_control=False;c.camera_gap_m=0.;c.camera_target_gap_m=0.
   c.path.poses[-1].pose.position.x=.3;h.task_pub.publish(c)
   h.pump(lambda:304 in h.results and h.results[304].status==TaskResult.RUNNING,hold=False,commands=True)
   command(h,304,.018,2,True)
   h.pump(lambda:h.results[304].revision==2,hold=False,commands=True)
   assert h.results[304].status==TaskResult.RUNNING, str(h.results[304])
   h.pump(lambda:h.drive.command is not None and h.drive.command[0]>.001,hold=False,commands=True)  # camera_distance_kp from control.yaml
   command(h,304,.0129,3,True)
   h.pump(lambda:h.results[304].revision==3 and abs(h.drive.command[0])<.001,hold=False,commands=True)
   c=command(h,304,.012,4,True,publish=False);c.camera_distance_control=False;h.task_pub.publish(c)
   h.pump(lambda:h.results[304].status==TaskResult.FAILED,hold=False,commands=True)
   print('CAMERA_ENGAGE_PASS off->on accepted, drives on the camera gap, stop .0129, on->off fails')
  else: raise ValueError(scenario)
 finally:h.close();rclpy.shutdown()
if __name__=='__main__':main()
