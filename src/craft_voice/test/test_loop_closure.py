import pytest
from craft_voice.loop_closure import LoopClosureGate
from test_narrator import Backend, Clock, make, run


@pytest.mark.parametrize('stamp,now', [(0,100),(-1,100),(90,100),(101,100),(float('nan'),100)])
def test_stale_future_invalid_events_ignored(stamp,now):
    gate=LoopClosureGate()
    assert not gate.accept(int(stamp*1e9) if stamp==stamp else stamp,int(now*1e9),10)


def test_first_duplicate_out_of_order_and_burst():
    gate=LoopClosureGate()
    assert gate.accept(100_000_000_000,100_000_000_000,1)
    assert not gate.accept(100_000_000_000,100_000_000_000,30)
    assert not gate.accept(99_000_000_000,100_000_000_000,30)
    assert not gate.accept(101_000_000_000,101_000_000_000,2)
    assert not gate.accept(101_000_000_000,101_000_000_000,30)
    assert gate.accept(121_000_000_000,121_000_000_000,21)


def test_completed_event_uses_ai_even_in_template_mode_and_survives_state_change():
    b=Backend('전에 왔던 곳이네! 지도를 다시 맞췄어.')
    n,texts=make(b,state_mode='template',use_image=False)
    n.loop_closed()
    assert n.think_once()
    n.tracker.generation+=1
    run(n)
    assert texts==[('event',b.reply)]
    assert len(b.calls)==1 and '시스템이 확인한 완료 사실' in b.calls[0][1]
    assert '지도 전체 완성 여부' in b.calls[0][1]
    assert '루프 폐쇄' in n.tracker.memory_text(n.clock())
    assert not n._dialogue
    n.close()


@pytest.mark.parametrize('backend',[None,Backend(fail=True)])
def test_offline_event_has_truthful_fixed_fallback(backend):
    n,texts=make(backend)
    n.loop_closed();run(n)
    assert texts==[('event','전에 지나온 곳을 다시 찾아서 지도를 맞췄어!')]
    n.close()


def test_mute_stop_stale_and_duplicate_queue():
    b=Backend(); clock=Clock();n,texts=make(b,clock=clock)
    n.command('mute');n.loop_closed();run(n)
    assert not b.calls
    n.command('unmute');n.loop_closed();n.command('stop');run(n)
    assert not b.calls
    n.loop_closed();n.loop_closed()
    assert len(n._jobs)==1
    n.think_once();clock.now+=21;run(n)
    assert not texts
    n.close()


def test_even_slightly_future_event_ignored():
    assert not LoopClosureGate().accept(100_100_000_000,100_000_000_000,10)


def test_expired_event_does_not_call_api_or_speak_fallback():
    b=Backend(fail=True);clock=Clock();n,texts=make(b,clock=clock)
    n.loop_closed();clock.now+=100;run(n)
    assert not b.calls and not texts
    n.close()


def test_slow_failed_api_does_not_reset_event_age():
    clock=Clock()
    class SlowBackend(Backend):
        def generate(self,*args):
            clock.now+=25
            raise ConnectionError()
    n,texts=make(SlowBackend(),clock=clock)
    n.loop_closed();run(n)
    assert not texts
    n.close()


def test_mute_cancels_event_during_synthesis():
    import threading
    b=Backend(tts_delay=.2);n,texts=make(b,start=True)
    n.loop_closed();assert b.tts_started.wait(2)
    n.command('mute')
    from test_startup import wait
    wait(lambda:not n._current)
    assert not texts and not n.player.played
    n.close()
