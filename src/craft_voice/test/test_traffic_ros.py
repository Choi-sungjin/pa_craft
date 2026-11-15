from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from craft_voice import voice_node as mod
from craft_perception.msg import TrafficLight
from test_narrator import Backend, make, run


SEQUENCE = [0, 2, 0, 2, 1, 2, 0, 2, 1, 2, 0, 2]
START = ('thought', '신호등 깜빡인다! 몇 번인지 세어 볼게.')
DONE = ('thought', '신호등 다 셌어! 빨강 4번, 노랑 2번, 초록 6번이야.')


def wired(monkeypatch):
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.tracker.style = 'cute'
    subscriptions = {}
    node = NS(get_logger=lambda: NS(warning=lambda _: None), narrator=n,
        create_subscription=lambda kind, topic, callback, qos: subscriptions.update({topic: callback}),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=int(n.clock.now * 1e9))))
    monkeypatch.setattr(mod, 'time', NS(monotonic=n.clock))
    params = {key: '' for key in ('task_result_topic', 'task_command_topic', 'box_topic',
                                  'barcode_topic', 'obstacles_topic')}
    params['traffic_light_topic'] = '/voice_test/traffic_startup'
    mod.VoiceNode._subscribe_signals(node, params, 10)

    def receive(sequence, stamp_s, at, complete=False, valid=False, discarded=0):
        n.clock.now = at
        msg = TrafficLight()
        stamp = int(stamp_s * 1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        msg.sequence = sequence
        msg.red, msg.yellow, msg.green = (sequence.count(c) for c in range(3))
        msg.complete, msg.valid, msg.discarded_cycles = complete, valid, discarded
        subscriptions[params['traffic_light_topic']](msg)
        n.tick(); run(n)
    return n, texts, receive


def test_new_first_blink_is_announced_during_startup_once(monkeypatch):
    n, texts, receive = wired(monkeypatch)
    receive([], 99, 100.1)
    receive([0], 101, 101.1)
    receive([0, 2], 104, 104.1)
    assert texts == [START] and len(n.player.played) == 1
    assert not n.backend.calls


def test_new_completion_after_retained_partial_count_is_announced_once(monkeypatch):
    n, texts, receive = wired(monkeypatch)
    receive(SEQUENCE[:-1], 99, 100.1)
    receive(SEQUENCE, 101, 101.1, complete=True, valid=True)
    receive(SEQUENCE, 101, 103.5, complete=True, valid=True)
    assert texts == [DONE] and len(n.player.played) == 1
    assert n.tracker.signals.traffic['sequence'] == tuple(SEQUENCE)
    assert not n.backend.calls


def test_first_message_can_be_a_new_completion(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive(SEQUENCE, 100.5, 100.6, complete=True, valid=True)
    assert texts == [DONE]


@pytest.mark.parametrize('received_at', [100.1, 104.0])
def test_retained_completion_is_known_but_not_announced(monkeypatch, received_at):
    n, texts, receive = wired(monkeypatch)
    receive(SEQUENCE, 99.9, received_at, complete=True, valid=True)
    assert not texts
    assert n.tracker.signals.traffic['sequence'] == tuple(SEQUENCE)


@pytest.mark.parametrize('stamp', [0, 103])
def test_invalid_timestamp_is_rejected_without_consuming_new_completion(monkeypatch, stamp):
    n, texts, receive = wired(monkeypatch)
    receive(SEQUENCE, stamp, 100.1, complete=True, valid=True)
    assert n.tracker.signals.traffic is None and not texts
    receive(SEQUENCE, 101, 101.1, complete=True, valid=True)
    assert texts == [DONE]


def test_reversed_timestamp_does_not_replace_a_newer_partial_count(monkeypatch):
    n, texts, receive = wired(monkeypatch)
    receive(SEQUENCE[:-1], 101, 101.1)
    receive(SEQUENCE, 100.5, 101.2, complete=True, valid=True)
    assert not texts and n.tracker.signals.traffic['blinks'] == 11
    receive(SEQUENCE, 102, 102.1, complete=True, valid=True)
    assert texts == [DONE]


def test_stale_new_completion_remains_quiet(monkeypatch):
    n, texts, receive = wired(monkeypatch)
    receive(SEQUENCE, 101, 105, complete=True, valid=True)
    assert not texts and n.tracker.signals.traffic['complete']


def test_subscription_timestamp_boundary_is_new(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive([0], 100, 100.1)
    assert texts == [START]


def test_reset_rearms_a_new_count_without_repeating_previous_completion(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive(SEQUENCE, 99, 100.1, complete=True, valid=True)
    receive([], 100.2, 100.3)
    receive([0], 100.4, 100.5)
    receive(SEQUENCE, 101, 101.1, complete=True, valid=True)
    receive(SEQUENCE, 101, 101.2, complete=True, valid=True)
    assert texts == [START, DONE]


def test_real_message_callback_retains_sequence_and_original_age():
    subscriptions=[];called=[]
    node=NS(get_logger=lambda:NS(warning=lambda _:None),
        create_subscription=lambda kind,topic,callback,qos:subscriptions.append((kind,topic,callback)),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=100_000_000_000)),
        narrator=NS(traffic_light=lambda *a,**kw:called.append((a,kw))))
    p={k:'' for k in ('task_result_topic','task_command_topic','box_topic','barcode_topic','obstacles_topic')}
    p['traffic_light_topic']='/traffic'
    mod.VoiceNode._subscribe_signals(node,p,10)
    callback=[cb for kind,topic,cb in subscriptions if topic=='/traffic'][0]
    m=TrafficLight();m.header.stamp.sec=95
    m.sequence=[2,0,1,2];m.red=1;m.yellow=1;m.green=2
    callback(m)
    assert called[-1][0]==(1,1,2,4,False,False,0)
    assert called[-1][1]['sequence']==(2,0,1,2) and called[-1][1]['age_s']==5
    count=len(called)
    for stamp in (0,94,102):
        m.header.stamp.sec=stamp;callback(m)
    assert len(called)==count
    m.header.stamp.sec=100;m.sequence=[0,1,2]*4;m.red=m.yellow=m.green=4;m.valid=True
    callback(m)
    assert called[-1][0][4:6]==(False,True)  # 12th blink, waiting for final dark interval
