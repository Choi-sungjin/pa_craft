"""Real generated ROS message types, read-only callbacks; no hardware publishers."""
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from craft_voice import voice_node as mod
from craft_interfaces.msg import BoxObservation, BoxObservationRequest
from std_msgs.msg import String


def test_mission_and_box_callbacks_keep_real_message_evidence():
    subscriptions = {}; received = []
    node = NS(get_logger=lambda: NS(warning=lambda _: None),
        create_subscription=lambda kind, topic, callback, qos: subscriptions.update({topic: (kind, callback)}),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)),
        narrator=NS(box=lambda *a, **k: received.append(('box', a, k)),
                    box_request=lambda *a: received.append(('request', a)),
                    mission_state=lambda *a: received.append(('mission', a))))
    p = {k: '' for k in ('task_result_topic', 'task_command_topic', 'barcode_topic',
                         'traffic_light_topic', 'obstacles_topic')}
    p.update(box_topic='/box', box_request_topic='/request', mission_state_topic='/mission')
    mod.VoiceNode._subscribe_signals(node, p, 10)
    assert set(subscriptions) == {'/box', '/request', '/mission'}
    req = BoxObservationRequest(); req.request_id = 7; req.source = 'lidar'; req.box_id = 'S1'; req.box_side = .205
    subscriptions['/request'][1](req)
    assert received[-1] == ('request', (7, 'lidar', 'S1', .205))
    m = BoxObservation(); m.header.stamp.sec = 99; m.header.stamp.nanosec = 500_000_000
    m.request_id = 7; m.source = 'lidar'; m.valid = True
    m.face_centre_observed = True; m.extrapolated = False
    m.gap_uncertainty_m = .01; m.face_centre_uncertainty_m = .02
    subscriptions['/box'][1](m)
    assert received[-1][2] == dict(request_id=7, age_s=.5, face_centre_observed=True,
        extrapolated=False, gap_uncertainty_m=.01, face_centre_uncertainty_m=.02)
    state = String(); state.data = 'Collect: approach_box S1'
    subscriptions['/mission'][1](state)
    assert received[-1] == ('mission', ('Collect: approach_box S1',))
