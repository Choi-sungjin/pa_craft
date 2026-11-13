#!/usr/bin/python3
"""Robot side: upper-cam JPEG + perception_master's puck_survey_observations -> drawn frame -> stdout.

Length-prefixed JPEG frames (live_view.py --no-model shows them on the PC), so nothing but ssh crosses the
network. Draws boxes, labels (red_prism_tags = tag rule confirmed), tag count and the axis (60 mm, S/L).
  ssh craft@<ip> 'source ~/craft/install/setup.bash && ROS_DOMAIN_ID=95 python3 survey_stream.py' \
    | python live_view.py --no-model
"""
import argparse
import struct
import sys

import cv2
import numpy as np
import rclpy
from craft_perception.msg import PuckObservationArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

ap = argparse.ArgumentParser()
ap.add_argument('--intrinsics-1080', type=float, nargs=4, default=[1362.574, 1361.790, 965.518, 583.265],
                help='perception.yaml uppercam.intrinsics_1920x1080')
ap.add_argument('--width', type=int, default=1280, help='streamed frame width')
a = ap.parse_args()
COLORS = {'red_cylinder': (0, 0, 255), 'red_prism': (60, 60, 255), 'green_cylinder': (0, 190, 0),
          'green_prism': (90, 255, 90)}
out = sys.stdout.buffer
images, results = {}, {}


def key(h):
    return (h.stamp.sec, h.stamp.nanosec)


def show(k):
    img = cv2.imdecode(np.frombuffer(images.pop(k), np.uint8), cv2.IMREAD_COLOR)
    obs = results.pop(k)
    sx, sy = img.shape[1] / 1920.0, img.shape[0] / 1080.0
    fx, fy, cx, cy = a.intrinsics_1080
    K = np.array([[fx * sx, 0, (cx + .5) * sx - .5], [0, fy * sy, (cy + .5) * sy - .5], [0, 0, 1]])

    def project(p):
        q = K @ p
        return int(q[0] / q[2]), int(q[1] / q[2])
    for o in obs.observations:
        x, y, w, h = o.detection.bbox
        color = COLORS.get(o.detection.label.replace('_tags', ''), (200, 200, 200))
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), color, 3)
        cv2.putText(img, f'{o.detection.label} {o.detection.confidence:.2f} tags={o.tags}', (int(x), max(22, int(y) - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if o.axis_valid:
            c = np.array([o.center.x, o.center.y, o.center.z])
            d = np.array([o.axis.x, o.axis.y, o.axis.z])
            cv2.line(img, project(c - d * 0.03), project(c + d * 0.03), (255, 0, 255), 4)
            cv2.circle(img, project(c), 6, (255, 0, 255), -1)
            cv2.putText(img, 'STANDING' if o.standing else 'LYING', (int(x), int(y + h) + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    cv2.putText(img, f'perception_master: {len(obs.observations)} pucks', (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (255, 255, 255), 2)
    small = cv2.resize(img, (a.width, int(a.width * img.shape[0] / img.shape[1])), interpolation=cv2.INTER_AREA)
    ok, jpg = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if ok:
        out.write(struct.pack('<I', len(jpg)) + jpg.tobytes())
        out.flush()


def on_image(m):
    images[key(m.header)] = bytes(m.data)
    for k in [k for k in images if k < key(m.header)][:-10]:
        images.pop(k, None)  # keep the newest few only
    if key(m.header) in results:
        show(key(m.header))


def on_obs(m):
    results[key(m.header)] = m
    if key(m.header) in images:
        show(key(m.header))


rclpy.init()
node = rclpy.create_node('survey_stream')
node.create_subscription(CompressedImage, '/perception/uppercam/image_raw/compressed', on_image, qos_profile_sensor_data)
node.create_subscription(PuckObservationArray, '/perception/vision/uppercam/puck_survey_observations', on_obs, 10)
try:
    rclpy.spin(node)
except (KeyboardInterrupt, BrokenPipeError):
    pass
