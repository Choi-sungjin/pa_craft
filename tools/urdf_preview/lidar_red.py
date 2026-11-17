#!/usr/bin/python3
"""RViz preview of the robot URDF (v75_robot.urdf) with the two LiDARs in red.

  /usr/bin/python3 lidar_red.py         # writes out/ (meshes, URDF, RViz config) and starts RViz; Ctrl+C / close ends
The LiDARs are not separate links: they sit inside base_link_black.stl. Its connected parts are split and the
LiDAR ones (front: body 52x44x21 mm at URDF y 33-78 mm + the head above it; rear: the same at y 317-363 mm;
centres 285 mm apart, x ~129 mm = dice_demo lidar_center_x 0.128, front face y 33.3 = lidar_front_y 0.034) go
into lidar_red.stl on a red visual; the rest stays in the black base mesh. Picked by position, so a remodelled
base mesh may need the boxes below updated.
"""
import os
import struct
import subprocess
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DESC = os.path.join(REPO, 'src/craft_planning/v75_ws/src/v75_robot_description')
OUT = os.path.join(HERE, 'out')
# LiDAR parts by their bounding box in base_link [m]: x 0.10-0.16, z 0.015-0.055, near either end in y
LIDAR_BOX = dict(x=(0.100, 0.160), z=(0.015, 0.056), y_front=(0.030, 0.080), y_rear=(0.314, 0.366))


def read_stl(path):
    b = open(path, 'rb').read(); n = struct.unpack('<I', b[80:84])[0]
    a = np.frombuffer(b[84:], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]), count=n)
    return a['v'].reshape(-1, 3, 3).astype(np.float64)


def write_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'lidar_red.py'.ljust(80, b'\0')); f.write(struct.pack('<I', len(tris)))
        for t in tris:
            nrm = np.cross(t[1] - t[0], t[2] - t[0]); nn = np.linalg.norm(nrm); nrm = nrm / nn if nn else nrm
            f.write(struct.pack('<12fH', *nrm, *t.ravel(), 0))


def split_lidar(tris):
    V = tris.reshape(-1, 3)
    uniq, inv = np.unique(np.round(V / 1e-5).astype(np.int64), axis=0, return_inverse=True)
    t = inv.ravel().reshape(-1, 3)
    g = coo_matrix((np.ones(3 * len(t)), (np.r_[t[:, 0], t[:, 1], t[:, 2]], np.r_[t[:, 1], t[:, 2], t[:, 0]])),
                   shape=(len(uniq),) * 2)
    _, lab = connected_components(g, directed=False)
    part = lab[t[:, 0]]
    lidar = np.zeros(len(tris), bool)
    for c in np.unique(part):
        P = tris[part == c].reshape(-1, 3); lo, hi = P.min(0), P.max(0)
        inside = lambda r, a, b: r[0] <= a and b <= r[1]
        if inside(LIDAR_BOX['x'], lo[0], hi[0]) and inside(LIDAR_BOX['z'], lo[2], hi[2]) and (
                inside(LIDAR_BOX['y_front'], lo[1], hi[1]) or inside(LIDAR_BOX['y_rear'], lo[1], hi[1])):
            lidar[part == c] = True
    return tris[~lidar], tris[lidar]


RVIZ = """Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Cell Size: 0.05
      Plane Cell Count: 20
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Value: /robot_description
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
  Global Options:
    Fixed Frame: base_link
    Background Color: 48; 48; 48
  Tools:                       # mouse: left drag = rotate, middle / shift+left = pan, wheel = zoom
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: <Fixed Frame>
      Distance: 1.0
      Focal Point: {X: 0.14, Y: 0.19, Z: 0.15}
      Pitch: 0.55
      Yaw: 0.8
Window Geometry:
  Width: 1400
  Height: 900
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    rest, red = split_lidar(read_stl(os.path.join(DESC, 'meshes/base_link_black.stl')))
    write_stl(os.path.join(OUT, 'base_link_black_no_lidar.stl'), rest)
    write_stl(os.path.join(OUT, 'lidar_red.stl'), red)
    print(f'base_link_black.stl: {len(red)} LiDAR triangles in red, {len(rest)} kept black')
    u = open(os.path.join(DESC, 'urdf/v75_robot.urdf')).read()
    u = u.replace('package://v75_robot_description/meshes/base_link_black.stl',
                  'file://' + os.path.join(OUT, 'base_link_black_no_lidar.stl'))
    u = u.replace('package://v75_robot_description/', 'file://' + DESC + '/')
    u = u.replace('</robot>', f'''  <link name="lidar_red_vis">
    <visual name="lidar_red">
      <geometry><mesh filename="file://{os.path.join(OUT, 'lidar_red.stl')}" scale="1 1 1" /></geometry>
      <material name="lidar_red"><color rgba="1.0 0.05 0.05 1.0" /></material>
    </visual>
  </link>
  <joint name="lidar_red_vis_joint" type="fixed">
    <parent link="base_link" />
    <child link="lidar_red_vis" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>
</robot>''')
    urdf = os.path.join(OUT, 'v75_lidar_red.urdf'); open(urdf, 'w').write(u)
    rviz = os.path.join(OUT, 'lidar_red.rviz'); open(rviz, 'w').write(RVIZ)
    import yaml                                      # the URDF is too long for a -p argument: params file
    params = os.path.join(OUT, 'rsp.yaml')
    yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': u}}}, open(params, 'w'))
    procs = [subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                               '--ros-args', '--params-file', params]),
             subprocess.Popen(['ros2', 'run', 'joint_state_publisher', 'joint_state_publisher']),
             subprocess.Popen(['ros2', 'run', 'rviz2', 'rviz2', '-d', rviz])]
    try:
        procs[-1].wait()
    except KeyboardInterrupt:
        pass
    finally:
        for p in procs:
            p.terminate()


if __name__ == '__main__':
    sys.exit(main())
