import sys
from types import ModuleType, SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from craft_voice import voice_node as mod


@pytest.mark.parametrize('failure',[None,ImportError(),mod.UnsupportedTypeSupport('slam_toolbox')])
def test_optional_loop_subscription_and_real_message_callback(monkeypatch,failure):
    from slam_toolbox.msg import LoopClosureEvent
    subscriptions=[];warnings=[];called=[]
    def check(kind):
        if failure:raise failure
    monkeypatch.setattr(mod,'check_for_type_support',check)
    node=NS(get_logger=lambda:NS(warning=warnings.append),
            create_subscription=lambda kind,topic,callback,qos:subscriptions.append((kind,topic,callback)),
            get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
            narrator=NS(loop_closed=lambda:called.append(1)))
    mod.VoiceNode._subscribe_loop_closure(node,{'loop_closure_topic':'/loop'})
    if failure:
        assert warnings and not subscriptions
    else:
        assert len(subscriptions)==1 and subscriptions[0][0] is LoopClosureEvent
        msg=LoopClosureEvent();msg.stamp.sec=100
        subscriptions[0][2](msg);subscriptions[0][2](msg)
        assert called==[1]


def test_disabled_topic_does_not_subscribe():
    mod.VoiceNode._subscribe_loop_closure(NS(),{'loop_closure_topic':''})
