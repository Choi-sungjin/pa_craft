#!/usr/bin/python3
"""Show what perception_master's upper-cam puck survey outputs, on recorded frames.

Publishes a res_record.py session as /perception/uppercam/image_raw/compressed (JPEG 80, like the bridge), one
frame at a time, waits for /perception/vision/uppercam/puck_survey_observations with that stamp, and draws it:
box + label (red_prism_tags = the tag rule confirmed the prism), tag count, and the axis as a 60 mm line through
the puck center (S standing / L lying). Run perception_master first (survey_view.sh does both).
Keys: space pause/play, d / a next / previous frame (paused), q quit. Loops.
"""
import glob
import json
import sys
import time

import cv2
import numpy as np
import rclpy
from craft_perception.msg import PuckObservationArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

session = sys.argv[1]
meta = json.load(open(session + '/meta.json'))
K = np.array([[meta['fx'], 0, meta['cx']], [0, meta['fy'], meta['cy']], [0, 0, 1]])
frames = [f for f in sorted(glob.glob(session + '/frames/*.png')) if not f.endswith('_depth.png')]
COLORS = {'red_cylinder': (0, 0, 255), 'red_prism': (60, 60, 255), 'green_cylinder': (0, 190, 0),
          'green_prism': (90, 255, 90)}

rclpy.init()
node = rclpy.create_node('survey_view')
pub = node.create_publisher(CompressedImage, '/perception/uppercam/image_raw/compressed', qos_profile_sensor_data)
latest = {}
node.create_subscription(PuckObservationArray, '/perception/vision/uppercam/puck_survey_observations',
                         lambda m: latest.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m), 10)


def project(p):
    q = K @ p
    return int(q[0] / q[2]), int(q[1] / q[2])


def draw(img, obs):
    for o in obs.observations if obs else []:
        x, y, w, h = o.detection.bbox
        base = o.detection.label.replace('_tags', '')
        color = COLORS.get(base, (200, 200, 200))
        cv2.rectangle(img, (int(x), int(y)), (int(x + w), int(y + h)), color, 3)
        label = f'{o.detection.label} {o.detection.confidence:.2f} tags={o.tags} id={o.puck_id}'
        cv2.putText(img, label, (int(x), max(22, int(y) - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if o.axis_valid:
            c = np.array([o.center.x, o.center.y, o.center.z])
            a = np.array([o.axis.x, o.axis.y, o.axis.z])
            cv2.line(img, project(c - a * 0.03), project(c + a * 0.03), (255, 0, 255), 4)
            cv2.circle(img, project(c), 6, (255, 0, 255), -1)
            txt = f"axis {'STANDING' if o.standing else 'LYING'}  fit {o.tag_fit_error_deg:.1f}deg"
            cv2.putText(img, txt, (int(x), int(y + h) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
        else:
            cv2.putText(img, 'axis: none', (int(x), int(y + h) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (160, 160, 160), 2)
    return img


window = 'perception_master upper-cam puck survey (space, a/d, q)'
cv2.namedWindow(window, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window, 1600, 900)
time.sleep(2.0)  # discovery
i, paused = 0, False
while True:
    img = cv2.imread(frames[i])
    ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    msg = CompressedImage()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'camera_top_color_optical_frame'
    msg.format, msg.data = 'jpeg', jpg.tobytes()
    key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
    pub.publish(msg)
    end = time.time() + 1.5
    while key not in latest and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
    obs = latest.pop(key, None)
    shown = draw(img, obs)
    status = 'master: ' + (f'{len(obs.observations)} pucks' if obs else 'no answer (dropped / too slow)')
    cv2.putText(shown, f'{frames[i].split("/")[-1]}  {status}', (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.imshow(window, shown)
    k = cv2.waitKey(0 if paused else 1) & 0xFF
    if k == ord('q') or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
        break
    if k == ord(' '):
        paused = not paused
    elif k == ord('a'):
        i = (i - 1) % len(frames)
    elif k == ord('d') or not paused:
        i = (i + 1) % len(frames)
node.destroy_node()
rclpy.shutdown()
