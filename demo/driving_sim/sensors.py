#!/usr/bin/env python3
"""Offline geometry and ROS 2 synthetic LiDAR for the driving demo.

Localization is ground truth, not SLAM. Measurements are synthetic ray casts.
"""
import copy
import json
import math
import os
import random
import struct
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]


def resolve(path, relative):
    p = Path(os.path.expanduser(str(path)))
    if p.is_absolute():
        return p
    # World paths normally use repository-relative names; map image is YAML-relative.
    q = Path(relative) / p
    return q if q.exists() else ROOT / p


def load_world(path):
    path = Path(path).resolve()
    with path.open() as f:
        world = yaml.safe_load(f)
    world['_path'] = str(path)
    world['map_file'] = str(resolve(world['map_file'], path.parent))
    world['urdf'] = str(resolve(world.get('urdf', 'src/craft_description/urdf/v307_full_robot.urdf'), path.parent))
    return world


def extrinsics(path):
    root = ET.parse(path).getroot()
    result = {}
    for name in ('front_laser', 'rear_laser'):
        joint = next(j for j in root.findall('joint') if j.find('child').get('link') == name)
        origin = joint.find('origin')
        xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
        rpy = list(map(float, origin.get('rpy', '0 0 0').split()))
        if abs(rpy[0]) > 1e-6 or abs(rpy[1]) > 1e-6:
            raise ValueError('Only horizontal LiDAR is supported')
        result[name] = (*xyz, rpy[2])
    return result


class Grid:
    def __init__(self, occupied, resolution, origin=(0., 0., 0.)):
        self.occupied = np.asarray(occupied, dtype=bool)  # bottom row first
        self.res = float(resolution)
        self.origin = tuple(map(float, origin))
        self.h, self.w = self.occupied.shape
        # Eight-neighbour chamfer distance, accurate enough for map filtering.
        self.distance = np.where(self.occupied, 0., np.inf)
        for _ in range(int(math.ceil(.6 / self.res))):
            old = self.distance.copy()
            for dy, dx, cost in ((1,0,1),(-1,0,1),(0,1,1),(0,-1,1),(1,1,math.sqrt(2)),(1,-1,math.sqrt(2)),(-1,1,math.sqrt(2)),(-1,-1,math.sqrt(2))):
                ys = slice(max(0,dy), min(self.h,self.h+dy)); xs = slice(max(0,dx),min(self.w,self.w+dx))
                yt = slice(max(0,-dy),min(self.h,self.h-dy)); xt = slice(max(0,-dx),min(self.w,self.w-dx))
                self.distance[ys,xs] = np.minimum(self.distance[ys,xs],old[yt,xt]+cost*self.res)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            cfg = yaml.safe_load(f)
        image = np.asarray(Image.open(resolve(cfg['image'], Path(path).parent)).convert('L'), dtype=float)
        p = image / 255. if cfg.get('negate', 0) else (255. - image) / 255.
        obj = cls(np.flipud(p > float(cfg.get('occupied_thresh', .65))), cfg['resolution'], cfg.get('origin', [0, 0, 0]))
        obj.values = np.flipud(np.where(p > float(cfg.get('occupied_thresh', .65)), 100, np.where(p < float(cfg.get('free_thresh', .196)), 0, -1))).astype(np.int8)
        return obj

    def coordinates(self, x, y):
        dx, dy = np.asarray(x)-self.origin[0], np.asarray(y)-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (c*dx+s*dy)/self.res, (-s*dx+c*dy)/self.res

    def query(self, x, y, distance=False):
        gx, gy = self.coordinates(x, y)
        ix, iy = np.floor(gx).astype(int), np.floor(gy).astype(int)
        valid = (ix >= 0) & (iy >= 0) & (ix < self.w) & (iy < self.h)
        ix, iy = np.clip(ix, 0, self.w-1), np.clip(iy, 0, self.h-1)
        return np.where(valid, self.distance[iy, ix] if distance else self.occupied[iy, ix], np.inf if distance else False)


def sensor_pose(base, ext):
    x,y,yaw = base
    ex,ey,ez,eyaw = ext
    c,s = math.cos(yaw), math.sin(yaw)
    return x+c*ex-s*ey, y+s*ex+c*ey, yaw+eyaw, .034346+ez


def box_distance(ox, oy, dx, dy, obj, height):
    if not obj.get('enabled', False) or height > float(obj.get('height', .25)):
        return np.full(np.shape(dx), np.inf)
    xmin = obj['x'] - obj.get('width', .25)/2
    xmax = obj['x'] + obj.get('width', .25)/2
    ymin = obj['y'] - obj.get('depth', .25)/2
    ymax = obj['y'] + obj.get('depth', .25)/2
    with np.errstate(divide='ignore', invalid='ignore'):
        ax, bx = (xmin-ox)/dx, (xmax-ox)/dx
        ay, by = (ymin-oy)/dy, (ymax-oy)/dy
        near = np.maximum(np.minimum(ax,bx), np.minimum(ay,by))
        far = np.minimum(np.maximum(ax,bx), np.maximum(ay,by))
    return np.where((far >= np.maximum(near, 0)), np.maximum(near, 0), np.inf)


def cast(grid, base, ext, obj, rng, noise=.003, dropout=.01, range_min=.1, range_max=8.):
    ox,oy,yaw,height = sensor_pose(base,ext)
    angles = np.linspace(-math.pi/2, math.pi/2, 361)
    dx,dy = np.cos(yaw+angles),np.sin(yaw+angles)
    # Sample all beams in a vectorized grid at half-cell spacing. The first
    # occupied sample differs from the exact cell boundary by at most one step.
    step = min(grid.res/2, .025)
    ts = np.arange(range_min, range_max+step/2, step)
    hits = grid.query(ox + dx[:,None]*ts, oy + dy[:,None]*ts)
    first = np.argmax(hits,axis=1)
    dist = np.where(np.any(hits,axis=1),ts[first],np.inf)
    dist = np.minimum(dist, box_distance(ox,oy,dx,dy,obj,height))
    finite = np.isfinite(dist) & (dist >= range_min) & (dist <= range_max)
    dist[~finite] = np.inf
    if noise: dist[finite] = np.clip(dist[finite]+rng.normal(0,noise,finite.sum()), range_min, range_max)
    if dropout: dist[finite & (rng.random(361) < dropout)] = np.inf
    dist[~np.isfinite(dist)] = np.inf
    return angles, dist


def merge_scans(scans, exts):
    merged = np.full(720, np.inf)
    for name, (angles, ranges) in scans.items():
        ex,ey,_,yaw = exts[name]
        ok = np.isfinite(ranges)
        x = ex+ranges[ok]*np.cos(angles[ok]+yaw)
        y = ey+ranges[ok]*np.sin(angles[ok]+yaw)
        idx = np.floor((np.arctan2(y,x)+math.pi)/(2*math.pi)*720).astype(int) % 720
        np.minimum.at(merged, idx, np.hypot(x,y))
    return merged


def mesh_bounds(urdf):
    root = ET.parse(urdf).getroot()
    visual = root.find("link[@name='base_link']/visual")
    off = list(map(float,visual.find('origin').get('xyz').split()))
    filename = visual.find('geometry/mesh').get('filename').replace('package://craft_description/', '')
    path = Path(urdf).resolve().parent.parent / filename
    data = path.read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84+count*50: raise ValueError('Expected binary STL: '+str(path))
    triangles = np.frombuffer(data, dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')]), offset=84, count=count)
    v = triangles['vertices'].reshape(-1,3)
    return float(v[:,0].min()+off[0]), float(v[:,0].max()+off[0]), float(v[:,1].min()+off[1]), float(v[:,1].max()+off[1])


def footprint_collision(grid, base, bounds, obj):
    # Sample occupied cell centers and dynamic rectangle against transformed footprint,
    # using SAT for exact overlap of rectangles. Map cell centers are expanded by half cell.
    x,y,yaw = base; xmin,xmax,ymin,ymax = bounds
    c,s = math.cos(yaw),math.sin(yaw)
    corners = np.array([[xmin,ymin],[xmin,ymax],[xmax,ymax],[xmax,ymin]]) @ np.array([[c,s],[-s,c]]) + [x,y]
    gx,gy = grid.coordinates(corners[:,0],corners[:,1])
    ix0,ix1 = max(0,int(np.floor(gx.min()))-1), min(grid.w-1,int(np.ceil(gx.max()))+1)
    iy0,iy1 = max(0,int(np.floor(gy.min()))-1), min(grid.h-1,int(np.ceil(gy.max()))+1)
    def overlap(poly):
        for p in (corners, poly):
            edges = np.roll(p,-1,axis=0)-p
            for edge in edges:
                axis = np.array([-edge[1],edge[0]])
                a,b = corners@axis,poly@axis
                if a.max() < b.min() or b.max() < a.min(): return False
        return True
    gc,gs = math.cos(grid.origin[2]),math.sin(grid.origin[2])
    for iy in range(iy0,iy1+1):
        for ix in range(ix0,ix1+1):
            if not grid.occupied[iy,ix]: continue
            cell = np.array([[ix,iy],[ix+1,iy],[ix+1,iy+1],[ix,iy+1]])*grid.res
            cell = cell @ np.array([[gc,gs],[-gs,gc]]) + grid.origin[:2]
            if overlap(cell): return True
    if obj.get('enabled',False) and obj.get('height',.25)>0:
        ox,oy=obj['x'],obj['y']; w=obj.get('width',.25)/2; d=obj.get('depth',.25)/2
        return overlap(np.array([[ox-w,oy-d],[ox+w,oy-d],[ox+w,oy+d],[ox-w,oy+d]]))
    return False


def main():
    if '--verify-only' in sys.argv:
        import unittest
        import test_sensors
        result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromModule(test_sensors))
        return 0 if result.wasSuccessful() else 1
    import rclpy
    from rclpy.impl.implementation_singleton import rclpy_implementation
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String, Bool
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry, OccupancyGrid, Path as NavPath
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from craft_interfaces.msg import TaskCommand
    from diagnostic_msgs.msg import DiagnosticArray

    class Sensors(Node):
        def __init__(self):
            super().__init__('drive_sim_sensors')
            self.declare_parameter('world_yaml', '')
            self.declare_parameter('noise_std', .003)
            self.declare_parameter('dropout', .01)
            self.declare_parameter('static_clearance', .03)
            self.declare_parameter('persist_frames', 3)
            self.declare_parameter('report_file', '')
            path = self.get_parameter('world_yaml').value
            if not path: raise ValueError('Set world_yaml parameter')
            self.world = load_world(path); self.grid = Grid.load(self.world['map_file'])
            self.repeat = self.declare_parameter('repeat',bool(self.world.get('repeat',False))).value
            self.ext = extrinsics(self.world['urdf']); self.bounds = mesh_bounds(self.world['urdf'])
            self.obj = dict(self.world.get('dynamic', {})); self.rng = np.random.default_rng(self.world.get('seed',16))
            self.noise = float(self.world.get('lidar',{}).get('noise_std',self.get_parameter('noise_std').value))
            self.dropout = float(self.world.get('lidar',{}).get('dropout',self.get_parameter('dropout').value))
            self.lidar_hz = float(self.world.get('lidar',{}).get('hz',10.))
            self.range_min = float(self.world.get('lidar',{}).get('range_min',.1))
            self.range_max = float(self.world.get('lidar',{}).get('range_max',8.))
            if not (self.lidar_hz>0 and 0<self.range_min<self.range_max and 0<=self.dropout<=1 and self.noise>=0):
                raise ValueError('invalid LiDAR simulation parameters')
            self.truth = None; self.state = ''; self.goal_index = 0; self.reached = 0
            self.pending = False; self.complete = False; self.next_time = 0.; self.started = False
            self.goals = self.world.get('goals', []); self.robot_path = 0.; self.last_xy = None
            self.planner_path = None; self.obstacle_event = False; self.collisions = 0; self.hit_latched = False
            self.paused = False; self.previous_points = {}; self.path_updates = 0; self.path_executions = 0
            self.plan_diagnostics = {}
            self.min_clearance = float('inf'); self.max_tick_seconds = 0.; self.last_report = 0.
            self.scan_pub = {n:self.create_publisher(LaserScan, '/lidar/'+n+'/scan',10) for n in ('front','rear')}
            self.merged_pub = self.create_publisher(LaserScan,'/lidar/merged',10)
            self.points_pub = self.create_publisher(PoseArray,'/perception/lidar/obstacle',10)
            self.odom_pubs = [self.create_publisher(Odometry, p,10) for p in ('/perception/state/odom','/perception/lidar/odom')]
            self.imu_pub = self.create_publisher(Imu,'/perception/imu/data',10)
            self.map_pub = self.create_publisher(OccupancyGrid,'/drive_sim/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.markers_pub = self.create_publisher(MarkerArray,'/drive_sim/world_markers',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.state_pub = self.create_publisher(String,'/drive_sim/state',10)
            self.collision_pub = self.create_publisher(Bool,'/drive_sim/collision',10)
            self.command_pub = self.create_publisher(String,'/planning/command',10)
            self.target_pub = self.create_publisher(PoseStamped,'/planning/target_base_pose',10)
            self.goal_pub = self.create_publisher(PoseStamped,'/drive_sim/goal',10)
            self.direct_pub = self.create_publisher(NavPath,'/drive_sim/direct_goal',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.initial_pub = self.create_publisher(NavPath,'/drive_sim/initial_plan',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.path_pub = self.create_publisher(NavPath,'/drive_sim/planner_path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(Odometry,'/drive_sim/truth',self.on_truth,10)
            self.create_subscription(String,'/planning/state',self.on_state,10)
            self.create_subscription(TaskCommand,'/planning/task_command',self.on_task,10)
            self.create_subscription(DiagnosticArray,'/planning/diagnostics',self.on_diagnostics,10)
            self.create_subscription(String,'/drive_sim/command',self.on_command,10)
            self.create_subscription(PoseStamped,'/goal_pose',self.on_rviz_goal,10)
            self.publish_map(); self.create_timer(1./self.lidar_hz,self.tick)

        def on_truth(self,msg):
            xy = (msg.pose.pose.position.x,msg.pose.pose.position.y)
            if self.last_xy: self.robot_path += math.dist(xy,self.last_xy)
            self.last_xy=xy; self.truth=msg
            stamp=self.get_clock().now().to_msg()
            for pub in self.odom_pubs:
                out=copy.deepcopy(msg); out.header.stamp=stamp; pub.publish(out)
            imu=Imu(); imu.header.stamp=stamp; imu.header.frame_id='imu_link'
            imu.orientation=copy.deepcopy(msg.pose.pose.orientation)
            imu.angular_velocity.z=msg.twist.twist.angular.z+float(self.rng.normal(0,.001))
            imu.linear_acceleration.z=9.80665+float(self.rng.normal(0,.01))
            imu.orientation_covariance[0]=imu.orientation_covariance[4]=imu.orientation_covariance[8]=1e-6
            imu.angular_velocity_covariance[0]=imu.angular_velocity_covariance[4]=imu.angular_velocity_covariance[8]=1e-6
            imu.linear_acceleration_covariance[0]=imu.linear_acceleration_covariance[4]=imu.linear_acceleration_covariance[8]=1e-4
            self.imu_pub.publish(imu)

        def base(self):
            p=self.truth.pose.pose.position; q=self.truth.pose.pose.orientation
            return p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        def on_state(self,msg):
            self.state=msg.data
            if self.pending and msg.data=='Run.Complete': self.complete=True
            if self.pending and self.complete and msg.data=='Run.AwaitRequest':
                self.pending=False; self.complete=False; self.reached+=1; self.goal_index+=1; self.next_time=time.monotonic()+2

        def on_diagnostics(self,msg):
            for status in msg.status:
                if status.name == 'planning/state_machine':
                    self.plan_diagnostics = {v.key:v.value for v in status.values}

        def on_task(self,msg):
            if msg.kind == TaskCommand.NAVIGATE and msg.operation in (TaskCommand.EXECUTE,TaskCommand.UPDATE_PATH):
                self.planner_path=msg.path; self.path_pub.publish(msg.path)
                if msg.operation == TaskCommand.UPDATE_PATH: self.path_updates += 1
                else:
                    self.path_executions += 1
                    self.initial_pub.publish(msg.path)

        def on_command(self,msg):
            if msg.data=='obstacle': self.obj['enabled']=not self.obj.get('enabled',False)
            elif msg.data=='pause': self.paused=True
            elif msg.data=='play': self.paused=False
            elif msg.data=='reset':
                self.get_logger().warn('Reset is handled by the launcher/plant; restart the demo to reset the coordinator.')

        def on_rviz_goal(self,msg):
            if self.state=='Run.AwaitRequest' and not self.pending:
                if msg.header.frame_id not in ('map',''): self.get_logger().warn('Goal must be in map frame'); return
                self.send_goal(msg,False)

        def send_goal(self,goal,automated=True):
            goal.header.frame_id='map'; goal.header.stamp=self.get_clock().now().to_msg()
            self.target_pub.publish(goal); self.goal_pub.publish(goal)
            line=NavPath(); line.header=goal.header
            start=PoseStamped(); start.header=goal.header; start.pose=copy.deepcopy(self.truth.pose.pose)
            line.poses=[start,goal]; self.direct_pub.publish(line)
            self.pending=True; self.complete=False
            if not automated:
                # Completion of an interactive request must not consume an automatic goal.
                self.goal_index -= 1

        def publish_map(self):
            m=OccupancyGrid(); m.header.frame_id='map'; m.header.stamp=self.get_clock().now().to_msg()
            m.info.resolution=self.grid.res; m.info.width=self.grid.w; m.info.height=self.grid.h
            m.info.origin.position.x=self.grid.origin[0]; m.info.origin.position.y=self.grid.origin[1]
            m.info.origin.orientation.z=math.sin(self.grid.origin[2]/2); m.info.origin.orientation.w=math.cos(self.grid.origin[2]/2)
            m.data=(self.grid.values if hasattr(self.grid,'values') else self.grid.occupied.astype(int)*100).flatten().tolist(); self.map_pub.publish(m)

        def scan_msg(self,name,angles,values,stamp):
            msg=LaserScan(); msg.header.stamp=stamp; msg.header.frame_id=name
            msg.angle_min=float(angles[0]); msg.angle_max=float(angles[-1]); msg.angle_increment=float(angles[1]-angles[0]); msg.scan_time=1./self.lidar_hz
            msg.time_increment=0.; msg.range_min=self.range_min; msg.range_max=self.range_max; msg.ranges=values.tolist(); return msg

        def auto_obstacle(self,base):
            threshold=self.world.get('auto_obstacle_after_m')
            if threshold is None or self.obstacle_event or self.robot_path < float(threshold) or not self.planner_path: return
            poses=self.planner_path.poses
            if not poses: return
            xy=np.array([(p.pose.position.x,p.pose.position.y) for p in poses])
            nearest=int(np.argmin(np.linalg.norm(xy-np.array(base[:2]),axis=1)))
            for p in xy[nearest:]:
                if np.linalg.norm(p-np.array(base[:2])) < .9: continue
                clearance=float(self.grid.query(p[0],p[1],True))
                if clearance > max(self.obj.get('width',.25),self.obj.get('depth',.25))/2+.08:
                    self.obj.update(x=float(p[0]),y=float(p[1]),enabled=True); self.obstacle_event=True; break

        def tick(self):
            if not self.truth: return
            tick_started=time.monotonic(); now=self.get_clock().now().to_msg(); base=self.base(); self.auto_obstacle(base)
            if self.state=='Ready' and not self.started:
                self.command_pub.publish(String(data='START')); self.started=True
            if self.state=='Run.AwaitRequest' and not self.pending and time.monotonic()>=self.next_time and self.goal_index<len(self.goals):
                x,y,yaw=self.goals[self.goal_index]; goal=PoseStamped(); goal.pose.position.x=float(x); goal.pose.position.y=float(y)
                goal.pose.orientation.z=math.sin(yaw/2); goal.pose.orientation.w=math.cos(yaw/2); self.send_goal(goal)
            if self.goal_index>=len(self.goals) and self.repeat and not self.pending and time.monotonic()>=self.next_time:
                self.goal_index=0
                self.obj['enabled']=False; self.obstacle_event=False; self.robot_path=0.; self.previous_points={}
            scans={}; points=[]
            for side in ('front','rear'):
                name=side+'_laser'; angles,ranges=cast(self.grid,base,self.ext[name],self.obj,self.rng,self.noise,self.dropout,self.range_min,self.range_max)
                scans[name]=(angles,ranges); self.scan_pub[side].publish(self.scan_msg(name,angles,ranges,now))
                ox,oy,yaw,_=sensor_pose(base,self.ext[name]); ok=np.isfinite(ranges)
                px=ox+ranges[ok]*np.cos(yaw+angles[ok]); py=oy+ranges[ok]*np.sin(yaw+angles[ok])
                keep=self.grid.query(px,py,True)>float(self.get_parameter('static_clearance').value)
                points.extend(zip(px[keep],py[keep]))
            merged=merge_scans(scans,self.ext); self.merged_pub.publish(self.scan_msg('base_link',np.linspace(-math.pi,math.pi,720,endpoint=False),merged,now))
            # Downsample measured points to 3 cm cells, without substituting ground truth centers.
            pts=PoseArray(); pts.header.frame_id='map'; pts.header.stamp=now
            cells={}
            for x,y in points:
                key=(round(x/.03),round(y/.03))
                cells.setdefault(key,(x,y))
            current={}
            threshold=max(1,int(self.get_parameter('persist_frames').value))
            for (ix,iy),(x,y) in cells.items():
                previous=max((self.previous_points.get((ix+dx,iy+dy),0) for dx in (-1,0,1) for dy in (-1,0,1)),default=0)
                current[(ix,iy)]=min(threshold,previous+1)
                if current[(ix,iy)] < threshold: continue
                p=Pose(); p.position.x=float(x); p.position.y=float(y); p.orientation.w=1.; pts.poses.append(p)
            self.previous_points=current
            self.points_pub.publish(pts)
            hit=footprint_collision(self.grid,base,self.bounds,self.obj)
            if hit and not self.hit_latched: self.collisions+=1; self.hit_latched=True
            self.collision_pub.publish(Bool(data=self.hit_latched)); self.publish_markers(now,len(pts.poses))
            # Sampling the footprint and subtracting one map cell gives an approximate conservative clearance.
            xmin,xmax,ymin,ymax=self.bounds
            xx,yy=np.meshgrid(np.linspace(xmin,xmax,9),np.linspace(ymin,ymax,9))
            c,s=math.cos(base[2]),math.sin(base[2]); wx=base[0]+c*xx-s*yy; wy=base[1]+s*xx+c*yy
            clearance=float(np.min(self.grid.query(wx,wy,True)))-self.grid.res*math.sqrt(2)
            if self.obj.get('enabled',False):
                dx=np.maximum(np.abs(wx-self.obj['x'])-self.obj.get('width',.25)/2,0)
                dy=np.maximum(np.abs(wy-self.obj['y'])-self.obj.get('depth',.25)/2,0)
                clearance=min(clearance,float(np.min(np.hypot(dx,dy))))
            self.min_clearance=min(self.min_clearance,max(0.,clearance))
            self.max_tick_seconds=max(self.max_tick_seconds,time.monotonic()-tick_started)
            state={'planner_state':self.state,'goal_index':self.goal_index,'reached':self.reached,'ray_counts':{n:int(np.isfinite(r).sum()) for n,(_,r) in scans.items()},'unknown_hit_count':len(pts.poses),'collisions':self.collisions,'collision_latched':self.hit_latched,'extrinsics':self.ext,'localization':'ideal truth','requested_line':'/drive_sim/direct_goal','planner_path':'/drive_sim/planner_path','obstacle_event':self.obstacle_event,'paused':self.paused,'path_updates':self.path_updates,'path_executions':self.path_executions,'min_body_clearance_approx_m':self.min_clearance if math.isfinite(self.min_clearance) else None,'max_tick_seconds':self.max_tick_seconds,'planner_diagnostics':self.plan_diagnostics,'pose':list(base),'traveled_m':self.robot_path,'sensor_model':'URDF extrinsics; 180deg outward; 10Hz; 3mm noise; 1pct dropout','cameras':False}
            self.state_pub.publish(String(data=json.dumps(state)))
            report=self.get_parameter('report_file').value
            if report and time.monotonic()-self.last_report >= 1.:
                output=Path(report); output.parent.mkdir(parents=True,exist_ok=True)
                temporary=output.with_suffix(output.suffix+'.tmp'); temporary.write_text(json.dumps(state,indent=2)+'\n'); temporary.replace(output)
                self.last_report=time.monotonic()

        def publish_markers(self,stamp,unknown_hits):
            markers=MarkerArray(); m=Marker(); m.header.frame_id='map'; m.header.stamp=stamp; m.ns='dynamic'; m.id=0; m.type=Marker.CUBE
            m.action=Marker.ADD if self.obj.get('enabled',False) else Marker.DELETE
            m.pose.position.x=float(self.obj.get('x',0)); m.pose.position.y=float(self.obj.get('y',0)); m.pose.position.z=float(self.obj.get('height',.25))/2; m.pose.orientation.w=1.
            m.scale.x=float(self.obj.get('width',.25)); m.scale.y=float(self.obj.get('depth',.25)); m.scale.z=float(self.obj.get('height',.25)); m.color.r=1.; m.color.g=.45; m.color.a=1.; markers.markers.append(m)
            label=Marker(); label.header=m.header; label.ns='hud'; label.id=1; label.type=Marker.TEXT_VIEW_FACING; label.action=Marker.ADD
            label.pose.position.x=3.; label.pose.position.y=4.25; label.pose.position.z=.05; label.pose.orientation.w=1.
            label.scale.z=.13; label.color.r=label.color.g=label.color.b=label.color.a=1.
            label.text=f'{self.state}/goals_{self.reached}/path_updates_{self.path_updates}/hits_{unknown_hits}/collisions_{self.collisions}'; markers.markers.append(label)
            legend=copy.deepcopy(label); legend.id=2; legend.pose.position.y=4.48; legend.text='Known_map->Padding->Cpp_plan->Actual_track'
            timing=copy.deepcopy(label); timing.id=3; timing.pose.position.y=4.04; timing.scale.z=.09
            timing.text=f"Cpp_planning_ms:{self.plan_diagnostics.get('plan_ms','-')}/ideal_wheels_and_localization/NO_CAMERAS"
            markers.markers.append(timing); markers.markers.append(legend)
            for i, goal in enumerate(self.goals):
                marker=Marker(); marker.header=m.header; marker.ns='goals'; marker.id=i; marker.type=Marker.SPHERE; marker.action=Marker.ADD
                marker.pose.position.x=float(goal[0]); marker.pose.position.y=float(goal[1]); marker.pose.position.z=.03; marker.pose.orientation.w=1.
                marker.scale.x=marker.scale.y=.12; marker.scale.z=.05; marker.color.r=1.; marker.color.a=.9; markers.markers.append(marker)
                number=copy.deepcopy(label); number.ns='goal_labels'; number.id=i; number.pose.position.x=float(goal[0]); number.pose.position.y=float(goal[1])+.15; number.text=str(i+1); number.scale.z=.12; markers.markers.append(number)
            self.markers_pub.publish(markers)

    rclpy.init()
    node=Sensors()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException): pass
    except rclpy_implementation.RCLError:
        if rclpy.ok(): raise
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    return 0


if __name__=='__main__':
    sys.exit(main())
