import importlib.util
import math
from pathlib import Path

import pytest
import rclpy
from craft_perception.msg import PuckObservation, PuckObservationArray
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

ROOT = Path(__file__).resolve().parents[3]


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


viz=module('puck_marker',ROOT/'src/craft_bringup/scripts/puck_marker.py')
parse=module('puck_observations',ROOT/'demo/puck_pick/puck_observations.py')


@pytest.mark.parametrize('shape,posture,width,height', [
    ('prism','standing',.03,.03),('prism','lying',.06,.03),
    ('cylinder','standing',.03,.03),('cylinder','lying',.06,.03)])
def test_footprint_dimensions(shape,posture,width,height):
    pts=viz.outline(shape,posture)
    assert pts[0]==pytest.approx(pts[-1])
    assert max(x for x,y in pts)-min(x for x,y in pts)==pytest.approx(width)
    assert max(y for x,y in pts)-min(y for x,y in pts)==pytest.approx(height)


def test_lying_color_shape_label_and_flat_rotation():
    o=PuckObservation();o.detection.label='red_prism';o.standing=False;o.tag_valid=True;o.puck_id=7
    fill,edge,label=viz.markers_for('gripcam',0,'map',o,(1.,2.,3.),(0.,1.,0.),.04,3.)
    assert fill.color.r==1. and fill.color.g<.2
    assert '#7 lying | red prism' in label.text
    assert all(p.z==.04 for p in fill.points)
    assert max(p.y for p in edge.points)-min(p.y for p in edge.points)==pytest.approx(.06)


def test_detector_parsing_does_not_guess_upper_posture():
    msg=parse.upper_observations('PUCK green_cylinder 0.93 ahead 100.0 lateral -5.0 top 120',
                                 ROOT/'tools/realsense_check/upper_cam_pose.txt',20.,-12.)
    o=msg.observations[0]
    assert o.center.x==pytest.approx(.23867+.019+.080)
    assert o.center.y==pytest.approx(.00002+.007)
    assert o.standing and 'posture_unknown' not in o.detection.label
    grip=parse.grip_observations('live: LYING YAW +30.0 deg CENTRE +1.0 -2.0 220.0 mm (10/10 frames)\nVIZ_AXIS 0.000000 1.000000 0.000000')
    assert not grip.observations[0].standing
    assert grip.observations[0].axis.y==1.
    assert grip.observations[0].center.z==.22
    assert not parse.grip_observations('live: YAW NONE (no prism)').observations


class Sink:
    def publish(self,m): self.msg=m


def test_transform_freezes_observation_and_empty_or_stale_clears(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','219');monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    rclpy.init();node=viz.PuckMarker();sink=Sink();node.pub=sink
    try:
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_link'
        t.transform.translation.x=10.;t.transform.rotation.w=1.
        node.tf.set_transform_static(t,'test')
        msg=PuckObservationArray();msg.header.frame_id='base_link'
        o=PuckObservation();o.axis_valid=True;o.center.x=1.;o.detection.label='green_cylinder';o.standing=True
        msg.observations=[o]
        node.receive('uppercam',msg);node.refresh()
        fill=next(m for m in sink.msg.markers if m.type==Marker.TRIANGLE_LIST)
        assert fill.points[0].x==11.
        t.transform.translation.x=20.;node.tf.set_transform_static(t,'test');node.refresh()
        fill=next(m for m in sink.msg.markers if m.type==Marker.TRIANGLE_LIST)
        assert fill.points[0].x==11.  # old detection must not follow the robot
        node.receive('uppercam',PuckObservationArray());node.refresh()
        assert all(m.action==Marker.DELETEALL for m in sink.msg.markers)
        node.receive('uppercam',msg);node.refresh()
        items,received,ttl=node.views['uppercam'];node.views['uppercam']=(items,received-int(4e9),ttl)
        node.refresh();assert all(m.action==Marker.DELETEALL for m in sink.msg.markers)
    finally:
        node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize('upper_label,red,footprints', [('red_prism',1.,1),('green_cylinder',.65,2)])
def test_color_association_requires_matching_shape(monkeypatch,upper_label,red,footprints):
    monkeypatch.setenv('ROS_DOMAIN_ID','219');monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    rclpy.init();node=viz.PuckMarker();sink=Sink();node.pub=sink
    try:
        upper=PuckObservationArray();upper.header.frame_id='map'
        a=PuckObservation();a.axis_valid=True;a.center.x=1.;a.detection.label=upper_label+'_posture_unknown'
        upper.observations=[a]
        grip=PuckObservationArray();grip.header.frame_id='map'
        b=PuckObservation();b.axis_valid=True;b.center.x=1.01;b.detection.label='unknown_prism';b.standing=False
        grip.observations=[b]
        node.receive('picker_upper',upper);node.receive('picker_grip',grip);node.refresh()
        fills=[m for m in sink.msg.markers if m.type==Marker.TRIANGLE_LIST]
        assert len(fills)==footprints
        precise=next(m for m in fills if m.ns.startswith('picker_grip/'))
        assert precise.color.r==pytest.approx(red)
    finally:
        node.destroy_node();rclpy.shutdown()
