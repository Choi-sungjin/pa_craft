#!/usr/bin/env python3
"""Decision-only, hardware-free two-puck policy verification and RViz display."""
import argparse
import json
import math
import random
import time
from pathlib import Path

import pick_policy

FACE_X = 257.67
LASER_Y = .02
TYPES = [(c, s, p) for c in ('red', 'green') for s, p in
         (('cylinder', 'standing'), ('prism', 'standing'), ('prism', 'lying'))]


def puck(name, color, shape, posture, ahead, lateral, score=.8):
    return dict(name=name, color=color, shape=shape, posture=posture,
                ahead=ahead, lateral=lateral, score=score)


# All centres remain within the 205 mm board; A/B are stable identities.
FIXTURES = [
    ('S1', [puck('A','red','prism','standing',175,-50), puck('B','green','prism','lying',35,50)], 'A'),
    ('S2', [puck('A','green','cylinder','standing',40,-50,.45), puck('B','red','prism','standing',175,50,.99)], 'A'),
    ('S3', [puck('A','red','prism','lying',45,-50), puck('B','green','prism','lying',165,50)], 'A'),
    ('S4', [puck('A','red','prism','standing',40,-50), puck('B','green','cylinder','standing',175,50)], 'A'),
    ('S5', [puck('A','red','cylinder','standing',165,-50), puck('B','green','cylinder','standing',45,50)], 'B'),
    ('S6', [puck('A','green','cylinder','standing',175,-50), puck('B','red','prism','lying',35,50)], 'A'),
]


def line(p):
    return (f"PUCK {p['color']}_{p['shape']} {p['score']} ahead {p['ahead']} "
            f"lateral {p['lateral']} posture {p['posture']} height 60")


def survey(items, expected):
    found = pick_policy.candidates('\n'.join(map(line, items)))
    selected = pick_policy.select(found, expected=expected)
    # The fixtures and verification data have unique centres, including same-type pairs.
    matches = [p for p in items if p['color']+'_'+p['shape'] == selected.cls
               and abs(p['ahead']-selected.ahead) < 1e-8
               and abs(p['lateral']-selected.lateral) < 1e-8]
    if len(matches) != 1:
        raise AssertionError('survey identity is ambiguous')
    return matches[0], selected


def oracle(items):
    # Independent imperative oracle: standing first, then base-link planar distance.
    standing = [p for p in items if p['shape'] == 'cylinder' or p['posture'] == 'standing']
    eligible = standing if standing else items
    return min(eligible, key=lambda p: math.hypot(FACE_X+p['ahead'], LASER_Y+p['lateral']))


def fixture_results():
    out = []
    for label, items, expected_first in FIXTURES:
        first, first_candidate = survey(items, 2)
        remaining = [p for p in items if p['name'] != first['name']]
        second, second_candidate = survey(remaining, 1)  # fresh survey, never cached selection
        expected_second = next(p['name'] for p in items if p['name'] != expected_first)
        assert (first['name'], second['name']) == (expected_first, expected_second), label
        reason = ('standing priority' if items[0]['posture'] != items[1]['posture']
                  else 'nearest planar distance')
        out.append(dict(panel=label, order=[first['name'], second['name']], reason=reason,
                        first=first_candidate.record(), second=second_candidate.record()))
    return out


def verify(seed):
    rng = random.Random(seed)
    counts = dict(pairs=0, surveys=0, swapped=0, fresh_surveys=0,
                  blocked=0, cylinder_forced=0, fixtures=0, random_pairs=0)
    failures = []

    def check_pair(items, category):
        counts['pairs'] += 1
        if category == 'random': counts['random_pairs'] += 1
        try:
            expected = oracle(items)['name']
            for seq in (items, list(reversed(items))):
                actual, _ = survey(seq, 2)
                counts['surveys'] += 1
                if seq is not items: counts['swapped'] += 1
                if actual['name'] != expected:
                    raise AssertionError(f"expected {expected}, got {actual['name']}")
                rest = [p for p in items if p['name'] != actual['name']]
                second, _ = survey(rest, 1)
                counts['fresh_surveys'] += 1
                if second['name'] != rest[0]['name']:
                    raise AssertionError('fresh survey chose wrong remainder')
        except Exception as exc:
            failures.append(dict(category=category, items=items, error=str(exc)))

    for a in TYPES:
        for b in TYPES:
            # Distances, lateral extremes and score order vary independently of type.
            for ahead_a, lat_a, ahead_b, lat_b in ((0,-102.5,205,102.5),
                    (180,100,20,-100), (80,-90,100,0), (100,0,80,90)):
                for score_a, score_b in ((.41,.99),(.99,.41)):
                    check_pair([puck('A',*a,ahead_a,lat_a,score_a),
                                puck('B',*b,ahead_b,lat_b,score_b)], 'cartesian')
    for _ in range(1500):
        a, b = rng.choice(TYPES), rng.choice(TYPES)
        check_pair([puck('A',*a,rng.uniform(0,205),rng.uniform(-102.5,102.5),rng.uniform(.4,1)),
                    puck('B',*b,rng.uniform(0,205),rng.uniform(-102.5,102.5),rng.uniform(.4,1))], 'random')
    try:
        results = fixture_results()
        counts['fixtures'] = len(results)
    except Exception as exc:
        failures.append(dict(category='fixture', error=str(exc)))
    # Missed, extra, and unknown detections must block selection.
    base = puck('A','red','prism','standing',50,0)
    unknown = puck('B','green','prism','unknown',100,20)
    for items, expected in (([],2),([base],2),([base,unknown],1),([base,unknown],2),
                            ([base,unknown,dict(base,name='C',ahead=150)],None)):
        try:
            pick_policy.select(pick_policy.candidates('\n'.join(map(line,items))), expected=expected)
            failures.append(dict(category='blocking', items=items, error='selection did not block'))
        except ValueError:
            counts['blocked'] += 1
    for reported in ('lying','unknown'):
        cyl = puck('A','red','cylinder',reported,40,0)
        found = pick_policy.candidates(line(cyl))
        if len(found) == 1 and found[0].posture == 'standing': counts['cylinder_forced'] += 1
        else: failures.append(dict(category='cylinder', items=[cyl], error='not forced standing'))
    return dict(seed=seed, counts=counts, failure_count=len(failures), failures=failures,
                fixtures=fixture_results(), scope='decision only; synthetic perception inputs',
                summary='PASS' if not failures else 'FAIL')


def run_ros(phase_seconds, verification):
    # ROS is intentionally imported only for the display, never --verify-only.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Point, TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray

    class Simulation(Node):
        def __init__(self):
            super().__init__('puck_pick_policy_sim')
            self.frames = StaticTransformBroadcaster(self)
            frame = TransformStamped()
            frame.header.stamp = self.get_clock().now().to_msg()
            frame.header.frame_id = 'pick_sim_world'; frame.child_frame_id = 'pick_sim'
            frame.transform.rotation.w = 1.
            self.frames.sendTransform(frame)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.markers = self.create_publisher(MarkerArray, '/puck_pick_sim/markers', qos)
            self.state = self.create_publisher(String, '/puck_pick_sim/state', qos)
            self.create_subscription(String, '/puck_pick_sim/command', self.command, 10)
            self.results = fixture_results()
            self.phase = 0
            self.playing = True
            self.last = time.monotonic()
            self.create_timer(.2, self.update)
            self.update()

        def command(self, msg):
            cmd = msg.data.strip().lower()
            if cmd == 'pause': self.playing = False
            elif cmd == 'play': self.playing = True; self.last = time.monotonic()
            elif cmd == 'next': self.phase = (self.phase+1)%3; self.last = time.monotonic()
            elif cmd == 'reset': self.phase = 0; self.last = time.monotonic()
            else: self.get_logger().warning('Unknown simulation command: '+cmd); return
            self.publish()

        def update(self):
            now = time.monotonic()
            if self.playing and now-self.last >= phase_seconds:
                steps = int((now-self.last)//phase_seconds)
                self.phase = (self.phase+steps)%3
                self.last += steps*phase_seconds
            self.publish()

        def publish(self):
            markers = [Marker(action=Marker.DELETEALL)]
            def marker(kind, x=0., y=0., z=.01, color=(1.,1.,1.,1.)):
                m = Marker(); m.header.frame_id = 'pick_sim'; m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'simulation'; m.id = len(markers); m.type = kind; m.action = Marker.ADD
                m.pose.orientation.w = 1.; m.pose.position.x=float(x); m.pose.position.y=float(y); m.pose.position.z=float(z)
                m.color.r,m.color.g,m.color.b,m.color.a = color
                markers.append(m); return m
            def text(label,x,y,size=.024,color=(1.,1.,1.,1.)):
                m = marker(Marker.TEXT_VIEW_FACING,x,y,.03,color); m.scale.z=size; m.text=label.replace(' ', '_')
            def line_strip(coords, color=(1.,1.,1.,1.), width=.003, z=.02):
                m=marker(Marker.LINE_STRIP,0,0,z,color); m.scale.x=width
                m.points=[Point(x=float(x),y=float(y),z=0.) for x,y in coords]
            def circle(x,y,r,color,width=.003):
                line_strip([(x+r*math.cos(i*math.tau/48),y+r*math.sin(i*math.tau/48)) for i in range(49)],color,width,.035)
            text(f"POLICY: {verification['summary']} | {verification['counts']['pairs']} PAIRS | 0 FAILURES",
                 0,.68,.025,(.3,1.,.55,1.))
            text('DECISION ONLY  |  NO CAMERA OR MOTION PHYSICS',0,.62,.03,(1.,.82,.25,1.))
            text(['PHASE 1: PICK FIRST','PHASE 2: PICK SECOND','PHASE 3: COMPLETE'][self.phase],0,.57,.025)
            for index, ((panel, items, _), result) in enumerate(zip(FIXTURES,self.results)):
                # Evaluate the production selector on the current simulated observation each refresh.
                present = [p for p in items if result['order'].index(p['name']) >= self.phase]
                if present:
                    current, _ = survey(present, expected=len(present))
                    assert current['name'] == result['order'][self.phase]
                ox=(index%3-1)*.65; oy=-(index//3)*.65
                x0,x1=ox-.1025,ox+.1025; y0,y1=oy+.25767,oy+.46267
                line_strip([(x0,y0),(x1,y0),(x1,y1),(x0,y1),(x0,y0)],(.65,.70,.76,1.))
                text(panel,ox,oy+.52,.032)
                text('ROBOT',ox,oy-.045,.018,(.7,.8,1.,1.))
                circle(ox,oy,.012,(.4,.7,1.,1.))
                line_strip([(ox,oy+.02),(ox,oy+.075),(ox-.008,oy+.06),(ox,oy+.075),(ox+.008,oy+.06)],(.4,.7,1.,1.))
                for p in items:
                    x=ox-(LASER_Y+p['lateral'])/1000.; y=oy+(FACE_X+p['ahead'])/1000.
                    order=result['order'].index(p['name'])
                    removed = self.phase > order
                    rgb=(.45,.45,.45,.5) if removed else ((1.,.14,.12,1.) if p['color']=='red' else (.12,.9,.3,1.))
                    m=marker(Marker.CYLINDER if p['shape']=='cylinder' else Marker.CUBE,x,y,.012,rgb)
                    m.scale.x=.06 if p['posture']=='lying' else .03; m.scale.y=.03; m.scale.z=.006
                    if self.phase < 2 and order == self.phase:
                        circle(x,y,.038,(1.,.8,.05,1.),.004)
                    dist=math.hypot(FACE_X+p['ahead'],LASER_Y+p['lateral'])
                    # Put the two labels on opposite sides of the board to avoid overlap.
                    lx=ox+(-.23 if p['name']=='A' else .23)
                    text(f"{p['name']} {p['color']}\n{p['posture']} {p['shape']}\n{dist:.0f} mm\nPICK {order+1}",lx,y,.020,
                         (.65,.65,.65,1.) if removed else (1.,1.,1.,1.))
                text(result['reason'],ox,oy+.19,.020,(.85,.85,.85,1.))
            self.markers.publish(MarkerArray(markers=markers))
            payload = dict(phase=self.phase+1, playing=self.playing, panels=self.results,
                           verification=verification, scope='decision only; no camera or motion physics')
            self.state.publish(String(data=json.dumps(payload,sort_keys=True)))

    rclpy.init()
    node = Simulation()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.try_shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verify-only', action='store_true')
    parser.add_argument('--seed', type=int, default=16)
    parser.add_argument('--phase-seconds', type=float, default=5.)
    parser.add_argument('--report', type=Path)
    args=parser.parse_args()
    if not math.isfinite(args.phase_seconds) or args.phase_seconds <= 0:
        parser.error('--phase-seconds must be positive and finite')
    result=verify(args.seed)
    rendered=json.dumps(result,indent=2,sort_keys=True)
    if args.report: args.report.write_text(rendered+'\n',encoding='utf-8')
    if args.verify_only or result['failure_count']:
        print(rendered)
        return 1 if result['failure_count'] else 0
    run_ros(args.phase_seconds,result)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
