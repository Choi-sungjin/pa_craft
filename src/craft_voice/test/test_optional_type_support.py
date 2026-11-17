"""선택 구독의 메시지 라이브러리가 없거나 깨져도 상태 안내를 유지한다."""
import sys
from types import ModuleType, SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from craft_voice import voice_node as mod

@pytest.mark.parametrize('failure', [None, ImportError('missing symbol'), mod.UnsupportedTypeSupport('craft_perception')])
def test_perception_support_failure_is_optional(monkeypatch, failure):
    package=ModuleType('craft_perception'); messages=ModuleType('craft_perception.msg')
    messages.DetectionArray=type('DetectionArray',(),{})
    messages.PuckObservationArray=type('PuckObservationArray',(),{})
    package.msg=messages
    monkeypatch.setitem(sys.modules,'craft_perception',package)
    monkeypatch.setitem(sys.modules,'craft_perception.msg',messages)
    checks=[]; subscriptions=[]; warnings=[]
    def check(kind):
        checks.append(kind)
        if failure:raise failure
    monkeypatch.setattr(mod,'check_for_type_support',check)
    node=SimpleNamespace(get_logger=lambda:SimpleNamespace(warning=warnings.append),
                         create_subscription=lambda kind,topic,callback,qos:subscriptions.append(topic))
    mod.VoiceNode._subscribe_perception(node,{'odom_topic':'','tags_topic':'','objects_topic':'/objects','pucks_topic':'/pucks'})
    if failure:
        assert not subscriptions
        assert warnings and 'type support unavailable' in warnings[0]
    else:
        assert subscriptions==['/objects','/pucks']
        assert len(checks)==2 and not warnings

def test_no_optional_topics_does_not_load_support(monkeypatch):
    def unexpected(_):raise AssertionError('disabled perception must not load support')
    monkeypatch.setattr(mod,'check_for_type_support',unexpected)
    node=SimpleNamespace(get_logger=lambda:None)
    mod.VoiceNode._subscribe_perception(node,{'odom_topic':'','tags_topic':'','objects_topic':'','pucks_topic':''})
