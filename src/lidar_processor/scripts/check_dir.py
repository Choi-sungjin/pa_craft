# 각 LiDAR 에서 몸체 밖 가장 가까운 점을 base_link 기준 방향으로 출력
# 실행: python3 check_dir.py  (Pi 의 lidar_processor.launch.py 실행 중)
import math, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
CFG = {'front': (0.163, math.pi), 'rear': (-0.163, 0.0)}  # (x, yaw) = launch 의 TF 와 같게
class C(Node):
    def __init__(self):
        super().__init__('check_dir')
        for s in CFG:
            self.create_subscription(LaserScan, f'/lidar/{s}/scan', lambda m, s=s: self.cb(s, m), qos_profile_sensor_data)
    def cb(self, side, m):
        x0, yaw = CFG[side]; best = None
        for i, r in enumerate(m.ranges):
            if not (m.range_min < r < 3.0): continue
            a = m.angle_min + i * m.angle_increment + yaw
            x, y = x0 + r * math.cos(a), r * math.sin(a)
            if abs(x) < 0.40 and abs(y) < 0.40: continue   # 몸체 근처 제외
            d = math.hypot(x, y)
            if best is None or d < best[0]: best = (d, x, y)
        if best:
            d, x, y = best
            print(f'{side:5s} 가장 가까운 점: 거리 {d:.2f}m  방향 {math.degrees(math.atan2(y, x)):6.0f}°  (x={x:+.2f}, y={y:+.2f})')
rclpy.init(); rclpy.spin(C())
