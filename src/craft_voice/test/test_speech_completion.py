"""Automatic observations queue after whole speech; only explicit commands cut it."""
import threading

import pytest

from test_flow_voice import QUALITY, VALID_REASON
from test_narrator import Backend, Player, StreamingBackend, FakeStreamPlayer, make, run


def prepare_notice(n, case):
    if case == 'mission':
        n.mission_state('Idle')
        n.mission_state('ReadBarcode')
        n.clock.now += 1.1
    elif case == 'traffic':
        n.traffic_light(1, 0, 0, 1, False, False, 0, sequence=[0])
    elif case == 'barcode':
        n.barcode('P2-01')
    else:
        n.planning_state('Run.Execute')
        n._jobs.clear(); n._utterances.clear()
        n.box_request(7, 'lidar', box_side=.205)
        for t in (100.1, 100.2, 100.3):
            n.clock.now = t
            n.box(True, 'lidar', VALID_REASON, .4, 0, 0, **QUALITY)
    n.tick()
    assert len(n._utterances) == 1
    return n._utterances[0].text


def change_observation(n, case):
    if case == 'mission':
        n.mission_state('Collect: approach_box S1')
    elif case == 'traffic':
        n.traffic_light(0, 0, 0, 0, False, False, 1, sequence=[])
    elif case == 'barcode':
        n.barcode('P2-02')
    else:
        n.box(False, 'lidar', 'lost', 0, 0, 0, **QUALITY)


@pytest.mark.parametrize('case', ['mission', 'traffic', 'barcode', 'box'])
def test_new_observation_waits_until_current_audio_drains(case):
    player = Player(block=True)
    n, texts = make(backend=Backend(), player=player, notice=True, narrate_period_s=0)
    first = prepare_notice(n, case)
    worker = threading.Thread(target=n.speak_once)
    worker.start()
    try:
        assert player.started.wait(2)
        change_observation(n, case)
        n.say('두 번째 문장이야.')
        n.say('세 번째 문장이야.')
        assert worker.is_alive() and len(player.played) == 1
        assert n._current.playback_started and not n._current.cancelled
        assert player.stops == 0
        assert [u.text for u in n._utterances] == ['두 번째 문장이야.', '세 번째 문장이야.']
        player.finish_current()
        worker.join(2)
        assert not worker.is_alive() and n.stats['spoken'] == 1
        player.block = False
        run(n)
        assert [text for _, text in texts] == [first, '두 번째 문장이야.', '세 번째 문장이야.']
        assert n.stats['spoken'] == 3 and player.stops == 0
    finally:
        n.close()
        worker.join(2)


@pytest.mark.parametrize('case', ['mission', 'traffic', 'barcode', 'box'])
def test_obsolete_notice_during_tts_never_starts_audio(case):
    backend = Backend(tts_delay=.3)
    n, texts = make(backend=backend, notice=True, narrate_period_s=0)
    prepare_notice(n, case)
    worker = threading.Thread(target=n.speak_once)
    worker.start()
    try:
        assert backend.tts_started.wait(2)
        change_observation(n, case)
        worker.join(2)
        assert not worker.is_alive() and not texts and not n.player.played
    finally:
        n.close()


@pytest.mark.parametrize('command', ['stop', 'mute'])
def test_explicit_stop_and_mute_still_interrupt_and_clear_queue(command):
    player = Player(block=True)
    n, texts = make(backend=Backend(), player=player)
    n.say('첫 번째 문장이야.')
    n.say('대기 중인 문장이야.')
    worker = threading.Thread(target=n.speak_once)
    worker.start()
    try:
        assert player.started.wait(2)
        n.command(command)
        worker.join(2)
        assert not worker.is_alive() and player.stops == 1
        assert n.stats['spoken'] == 0 and not n._utterances
        assert len(texts) == 1
    finally:
        n.close()


def test_barcode_fact_survives_unrelated_planning_generation():
    n, texts = make(backend=Backend(), notice=True, narrate_period_s=0)
    n.tracker.style = 'cute'
    n.planning_state('Ready')
    n._jobs.clear(); n._utterances.clear()
    n.barcode('P2-01'); n.tick()
    n.planning_state('Run.PrepareTask')
    run(n)
    assert ('thought', '좋아, 바코드 읽었어! 오름차순이야.') in texts


def test_stop_at_playback_boundary_prevents_start():
    n, _ = make(backend=Backend())
    n.on_text = lambda *_: n.command('stop')
    n.say('재생 경계 검사야.')
    run(n)
    assert not n.player.played and n.stats['spoken'] == 0


@pytest.mark.parametrize('case', ['mission', 'traffic', 'barcode', 'box'])
@pytest.mark.parametrize('streaming', [False, True])
def test_observation_changes_at_device_start_boundary(case, streaming):
    n, _ = make(backend=StreamingBackend() if streaming else Backend(), notice=True, narrate_period_s=0)
    stream = FakeStreamPlayer()
    if streaming:
        n.stream_player = stream
    prepare_notice(n, case)
    n.on_text = lambda *_: change_observation(n, case)
    n.speak_once()
    assert not n.player.played and not stream.opened
    assert n.stats['spoken'] == 0 and n.stats['dropped'] == 1


def test_failed_stream_open_does_not_protect_unstarted_wav_fallback():
    from craft_voice.audio import StreamPlayer

    n, _ = make(backend=StreamingBackend(), notice=True, narrate_period_s=0)
    n.stream_player = StreamPlayer(['/nonexistent/craft-queue-test'])
    prepare_notice(n, 'barcode')
    original = n._play_whole

    def change_before_fallback(*args):
        assert not n._current.playback_started
        change_observation(n, 'barcode')
        return original(*args)

    n._play_whole = change_before_fallback
    n.speak_once()
    assert not n.player.played and n.stats['spoken'] == 0


def test_traffic_answer_is_rechecked_at_device_start():
    n, _ = make(backend=Backend())
    n.traffic_light(1, 0, 0, 1, False, False, 0, sequence=[0], quiet=True)
    n.ask('신호등 빨간색 몇 번이야')
    n.on_text = lambda *_: n.traffic_light(0, 0, 0, 0, False, False, 0, sequence=[], quiet=True)
    n.speak_once()
    assert not n.player.played and n.stats['spoken'] == 0 and n.stats['dropped'] == 1
