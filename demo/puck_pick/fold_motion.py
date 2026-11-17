"""Conservative, model-only check for shoulder-last waypoint insertion.

Visual mesh BVH checks modeled surfaces; open shells do not guarantee
solid containment. Discrete samples and the assumed box and carousel are not a
physical collision guarantee.
"""
import math
import struct
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
import coal
import yaml
import plan

SHOULDER_EARLY_FRACTION = 0.0
_NAMES = plan.Chain.NAMES


def candidate(name):
    return (name.startswith(('standfind 1:', 'standfind 2:', 'standfind 4:',
                             'stand pick: descend', 'stand pick: up', 'stand pick: approach',
                             'lie ', 'rise above the puck height, LM stays',
                             'LM back to the start position', 'arm back to the start pose',
                             'via pose before the nominal pose', 'wrist_pitch to')) or
            any(name.startswith('stand load %d:' % i) for i in range(2, 10)))


def _box(T, bounds, geometry):
    lo, hi = bounds
    corners = np.array([[x, y, z, 1.] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    pts = (T @ corners.T).T[:, :3]
    return pts.min(axis=0), pts.max(axis=0), T[:3, :3], (hi-lo)/2, geometry, coal.Transform3s(T[:3,:3], T[:3,3])


def _axis_box(lo, hi):
    lo, hi = np.asarray(lo), np.asarray(hi)
    return lo, hi, np.eye(3), (hi-lo)/2, coal.Box(*(hi-lo)), coal.Transform3s((lo+hi)/2)


def _overlap(a, b):
    # 15-axis SAT for oriented boxes; tiny epsilon makes touching conservative.
    if not (np.all(a[0] <= b[1]) and np.all(b[0] <= a[1])): return False
    A, B, ea, eb = a[2], b[2], a[3], b[3]
    R = A.T @ B; absR = np.abs(R) + 1e-12
    t = A.T @ ((b[0]+b[1]-a[0]-a[1])/2)
    for i in range(3):
        if abs(t[i]) > ea[i] + np.dot(eb, absR[i,:]): return False
    for j in range(3):
        if abs(np.dot(t, R[:,j])) > eb[j] + np.dot(ea, absR[:,j]): return False
    for i in range(3):
        for j in range(3):
            i1,i2,j1,j2=(i+1)%3,(i+2)%3,(j+1)%3,(j+2)%3
            if abs(t[i2]*R[i1,j]-t[i1]*R[i2,j]) > ea[i1]*absR[i2,j]+ea[i2]*absR[i1,j]+eb[j1]*absR[i,j2]+eb[j2]*absR[i,j1]: return False
    request, result = coal.CollisionRequest(), coal.CollisionResult()
    return coal.collide(a[4], a[5], b[4], b[5], request, result) > 0


def _mesh(path):
    data = path.read_bytes()
    if len(data) < 84: raise ValueError('short mesh: ' + str(path))
    count = struct.unpack_from('<I', data, 80)[0]
    if len(data) != 84 + 50 * count or not count:
        raise ValueError('mesh is not binary STL: ' + str(path))
    vertices = np.array([struct.unpack_from('<9f', data, 84 + 50*i + 12)
                         for i in range(count)]).reshape(-1, 3)
    if not np.isfinite(vertices).all(): raise ValueError('invalid mesh: ' + str(path))
    return vertices.min(axis=0), vertices.max(axis=0)


@lru_cache(maxsize=1)
def _control():
    with open(plan.CONTROL_YAML) as f:
        return yaml.safe_load(f)['control_master']['ros__parameters']


def _motor_bad(q):
    return plan.motor_bad(dict(zip(_NAMES, q)), _control())


class Clearance:
    def __init__(self):
        root = ET.parse(plan.URDF).getroot()
        self.joints = root.findall('joint')
        self.links = {x.get('name'): x for x in root.findall('link')}
        self.meshes = {}
        loader = coal.MeshLoader()
        mesh_dir = plan.URDF.parent.parent / 'meshes'
        for name, link in self.links.items():
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    raise ValueError('unsupported visual geometry: ' + name)
                path = mesh_dir / Path(mesh.get('filename')).name
                scale = np.array([float(x) for x in mesh.get('scale', '1 1 1').split()])
                if np.any(scale <= 0):
                    raise ValueError('invalid mesh scale: ' + name)
                lo, hi = _mesh(path)
                geometry = loader.load(str(path), scale)
                self.meshes.setdefault(name, []).append((plan.rpy_xyz(visual.find('origin')), (lo * scale, hi * scale), geometry))
        self.chain = plan.Chain(plan.URDF)
        self.box = _axis_box([.128-.104, .034-.205, 0.], [.128+.104, .034, .070])

    def _transforms(self, q, jaw, carousel):
        values = dict(zip(_NAMES, q)); values.update(jaw_upper_joint=jaw, jaw_lower_joint=jaw, carousel_joint=carousel)
        out = {'base_link': np.eye(4)}
        remaining = list(self.joints)
        while remaining:
            next_remaining = []
            for j in remaining:
                parent = j.find('parent').get('link'); child = j.find('child').get('link')
                if parent not in out:
                    next_remaining.append(j); continue
                T = out[parent] @ plan.rpy_xyz(j.find('origin'))
                if j.get('type') != 'fixed':
                    axis = j.find('axis')
                    if axis is None: raise ValueError('missing axis: ' + j.get('name'))
                    T = T @ plan.axis_motion([float(x) for x in axis.get('xyz').split()], j.get('type'), values.get(j.get('name'), 0.))
                out[child] = T
            if len(next_remaining) == len(remaining): raise ValueError('unconnected URDF')
            remaining = next_remaining
        return out

    def _all_boxes(self, transforms, prefix):
        out = []
        for name, visuals in self.meshes.items():
            if name.startswith(prefix):
                for origin, bounds, geometry in visuals:
                    out.append((name, _box(transforms[name] @ origin, bounds, geometry)))
        return out

    def check(self, q, held=False, box_known=True):
        if not box_known: return 'box position unknown after drive'
        for i, name in enumerate(_NAMES):
            lo, hi = self.chain.limits(name)
            if not lo <= q[i] <= hi: return 'joint limit: ' + name
        bad = _motor_bad(q)
        if bad: return 'motor limit: ' + bad[0]
        transforms = self._transforms(q, 0., 0.)
        fixed = self._all_boxes(transforms, ('base_link',))
        # Unknown clock angle: check twelve modeled rotations, not a continuous sweep.
        for angle in np.linspace(0., 2.*math.pi, 12, endpoint=False):
            car = self._all_boxes(self._transforms(q, 0., angle), ('carousel_link',))
            if not car: return 'carousel geometry missing'
            fixed.extend(car)
        for jaw in (-.021, .01):
            T = self._transforms(q, jaw, 0.)
            moving = self._all_boxes(T, ('shoulder_link', 'elbow_link', 'wrist_pitch_link', 'tool_roll_link', 'jaw_upper_link', 'jaw_lower_link'))
            if not moving: return 'arm geometry missing'
            if held:
                # Orientation and grasp offset are uncertain: contain both 30x30x60 and 30x60x30
                # in a conservative 60 mm radius cube around TCP; ignore intended jaw contact.
                tcp = T['tool_tcp'][:3, 3]; moving.append(('held puck envelope', _axis_box(tcp-.06, tcp+.06)))
            for name, bounds in moving:
                if bounds[0][2] < 0.: return name + ' below floor'
                if _overlap(bounds, self.box): return name + ' intersects box'
                for obj, obstacle in fixed:
                    if name.startswith('shoulder_link') and obj.startswith('base_link'): continue
                    if _overlap(bounds, obstacle): return name + ' intersects ' + obj
            for shoulder, a in moving:
                if not shoulder.startswith('shoulder_link'): continue
                for other, b in moving:
                    if other.startswith(('tool_roll_link', 'jaw_upper_link', 'jaw_lower_link')) and _overlap(a, b):
                        return shoulder + ' intersects ' + other
        return ''


@lru_cache(maxsize=1)
def _model():
    return Clearance()


def prepare_motion(name, start, target, seconds, tolerance, held=False, box_known=True):
    """Return ([(target, seconds, tolerance, is_intermediate)], reason).

    Invalid pose/start raises ValueError. A model rejection keeps the original move.
    """
    if not candidate(name): return [(target, seconds, tolerance, False)], ''
    if not math.isfinite(SHOULDER_EARLY_FRACTION) or not 0 <= SHOULDER_EARLY_FRACTION <= 1:
        raise ValueError('invalid shoulder early fraction')
    if start is None or len(start) != 5 or len(target) not in (5, 6): raise ValueError('missing or invalid joint feedback/target')
    if not all(math.isfinite(float(x)) for x in list(start[:5]) + list(target)):
        raise ValueError('nonfinite joint feedback/target')
    ch = plan.Chain(plan.URDF)
    for i, n in enumerate(_NAMES):
        lo, hi = ch.limits(n)
        if not lo <= target[i] <= hi: raise ValueError('target joint limit: ' + n)
    bad = _motor_bad(target)
    if bad: raise ValueError('target motor limit: ' + bad[0])
    if abs(target[1] - start[1]) < 1e-10 or not any(abs(target[i] - start[i]) >= 1e-10 for i in (0, 2, 3, 4)):
        return [(target, seconds, tolerance, False)], 'single-axis group'
    mid = list(target); mid[1] = start[1] + SHOULDER_EARLY_FRACTION*(target[1]-start[1])
    try:
        model = _model()
        for a, b in ((start, mid), (mid, target)):
            count = max(1, math.ceil(max(abs(b[0]-a[0])/.002, *(abs(b[i]-a[i])/math.radians(2) for i in range(1, 5)))))
            if count > 2000: return [(target, seconds, tolerance, False)], 'sampling limit'
            for k in range(count + 1):
                q = [a[i] + (b[i]-a[i])*k/count for i in range(5)]
                reason = model.check(q, held, box_known)
                if reason: return [(target, seconds, tolerance, False)], reason
    except (ValueError, OSError, ET.ParseError) as exc:
        return [(target, seconds, tolerance, False)], 'clearance model unavailable: ' + str(exc)
    return [(mid, seconds, min(tolerance if tolerance > 0 else math.radians(.5), math.radians(.5)), True),
            (target, seconds, tolerance, False)], ''
