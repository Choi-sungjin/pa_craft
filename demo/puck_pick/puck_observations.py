"""Convert C++ detector output to the project's existing observation message (display only)."""
import re
from pathlib import Path

from craft_perception.msg import PuckObservation, PuckObservationArray

NUMBER = r'([+-]?\d+(?:\.\d+)?)'


def upper_observations(text, pose_path, ahead_bias=0., lateral_bias=0.):
    out = PuckObservationArray(); out.header.frame_id = 'base_link'
    p = [float(v) for v in Path(pose_path).read_text().splitlines()[0].split()]
    floor, laser_x, laser_y, face, sign = p[12], p[13], p[14], p[16], p[18]
    for line in text.splitlines():
        m = re.match(rf'PUCK (\S+) {NUMBER} ahead {NUMBER} lateral {NUMBER} top {NUMBER}',line)
        if not m:
            continue
        label,score,ahead,lateral,top = m.groups()
        o = PuckObservation(); o.puck_id = -1
        state = re.search(r' posture (standing|lying|unknown)\b',line)
        posture = 'standing' if label.endswith('cylinder') else state[1] if state else 'unknown'
        o.standing = posture == 'standing'
        o.detection.label = label + ('_posture_unknown' if posture == 'unknown' else '')
        o.detection.confidence = float(score)
        o.axis_valid = True  # position measured; posture explicitly unknown in the label
        o.center.x = laser_x + face + (float(ahead)-ahead_bias)/1000
        o.center.y = laser_y + (float(lateral)-lateral_bias)/1000/sign
        o.center.z = floor + float(top)/1000
        out.observations.append(o)
    return out


def grip_observations(text):
    out = PuckObservationArray(); out.header.frame_id = 'camera_wrist_color_optical_frame'
    m = re.search(rf': (LYING )?YAW {NUMBER} deg CENTRE {NUMBER} {NUMBER} {NUMBER} mm',text)
    if not m:
        return out
    lying,yaw,x,y,z = m.groups()
    o = PuckObservation(); o.puck_id = -1; o.axis_valid = True
    o.standing = not bool(lying); o.detection.label = 'unknown_cylinder' if 'VIZ_SHAPE cylinder' in text else 'unknown_prism'
    frames = re.search(r'(\d+)/(\d+) frames',text)
    o.detection.confidence = int(frames[1])/int(frames[2]) if frames and int(frames[2]) else 0.
    o.center.x,o.center.y,o.center.z = float(x)/1000,float(y)/1000,float(z)/1000
    axis = re.search(rf'VIZ_AXIS {NUMBER} {NUMBER} {NUMBER}',text)
    if axis:
        o.axis.x,o.axis.y,o.axis.z = map(float,axis.groups())
    o.height_m = .060 if o.standing else .030
    out.observations.append(o)
    return out
