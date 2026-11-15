"""Retained barcode snapshots stay quiet; new results are announced even during startup."""
import inspect
from types import SimpleNamespace as NS

import pytest

pytest.importorskip('rclpy')
from std_msgs.msg import String
from craft_voice import voice_node as mod
from test_narrator import Backend, make, run


def wired(monkeypatch):
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.tracker.style = 'cute'
    subscriptions = {}
    node = NS(get_logger=lambda: NS(warning=lambda _: None), narrator=n,
        create_subscription=lambda kind, topic, callback, qos: subscriptions.update({topic: callback}))
    monkeypatch.setattr(mod, 'time', NS(monotonic=n.clock, time_ns=lambda: int(n.clock.now * 1e9)))
    params = {key: '' for key in ('task_result_topic', 'task_command_topic', 'box_topic',
                                  'traffic_light_topic', 'obstacles_topic')}
    params['barcode_topic'] = '/barcode_test'
    mod.VoiceNode._subscribe_signals(node, params, 10)
    callback = subscriptions['/barcode_test']

    def receive(text, source_s, at):
        n.clock.now = at
        msg = String(); msg.data = text
        info = {} if source_s is None else {'source_timestamp': int(source_s * 1e9)}
        # Follow rclpy's one-argument/two-argument callback dispatch, including the old callback.
        if len(inspect.signature(callback).parameters) == 1:
            callback(msg)
        else:
            callback(msg, info)
        n.tick(); run(n)
    return n, texts, receive


@pytest.mark.parametrize('code,order', [('P2-01', '오름차순'), ('P2-02', '내림차순'),
                                       ('P2-03', '오름차순'), ('P2-04', '내림차순')])
def test_first_message_published_after_subscription_is_announced(monkeypatch, code, order):
    _, texts, receive = wired(monkeypatch)
    receive(code, 100.5, 100.6)
    receive(code, 101, 101.1)
    assert texts == [('thought', f'좋아, 바코드 읽었어! {order}이야.')]


def test_initial_empty_snapshot_does_not_hide_new_result_in_first_three_seconds(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive('', 99, 100.1)
    receive('P2-01', 101, 101.1)
    receive('P2-01', 104, 104.1)
    assert texts == [('thought', '좋아, 바코드 읽었어! 오름차순이야.')]


def test_delayed_retained_result_is_known_but_not_announced(monkeypatch):
    n, texts, receive = wired(monkeypatch)
    receive('P2-02', 99, 110)
    assert not texts and n.tracker.signals.barcode[0] == 'P2-02'
    receive('P2-02', 111, 111.1)
    assert not texts  # the same result again is still not a new reading


def test_new_result_replaces_initial_snapshot_without_startup_mute(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive('P2-01', 99, 100.1)
    receive('P2-02', 101, 101.1)
    assert texts == [('thought', '좋아, 바코드 읽었어! 내림차순이야.')]


def test_reset_rearms_the_same_code_and_duplicates_remain_silent(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive('P2-01', 100.2, 100.3)
    receive('', 101, 101.1)
    receive('P2-01', 102, 102.1)
    receive('P2-01', 103, 103.1)
    assert texts == [('thought', '좋아, 바코드 읽었어! 오름차순이야.')] * 2


@pytest.mark.parametrize('missing_timestamp', [None, 0])
def test_missing_metadata_keeps_first_snapshot_quiet_but_allows_later_changes(monkeypatch, missing_timestamp):
    _, texts, receive = wired(monkeypatch)
    receive('', missing_timestamp, 100.1)
    receive('P2-02', missing_timestamp, 101)
    receive('P2-02', missing_timestamp, 104)
    assert texts == [('thought', '좋아, 바코드 읽었어! 내림차순이야.')]


def test_timestamp_at_subscription_boundary_is_new(monkeypatch):
    _, texts, receive = wired(monkeypatch)
    receive('P2-01', 100, 100.1)
    assert texts == [('thought', '좋아, 바코드 읽었어! 오름차순이야.')]
