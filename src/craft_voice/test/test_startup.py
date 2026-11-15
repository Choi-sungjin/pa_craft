import logging
import threading
import time
from pathlib import Path

import pytest

from craft_voice.startup import StartupAnnouncement, check_voice_api
from test_narrator import Backend, make, run
from test_word_reactions import clip


def wait(predicate):
    end = time.monotonic() + 2
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.005)
    assert predicate()


class API:
    def __init__(self, audio, bad=''):
        self.audio, self.bad, self.calls = audio, bad, []

    def generate(self, *args):
        self.calls.append('text')
        return '' if self.bad == 'text' else '준비'

    def synthesize(self, text):
        self.calls.append('speech')
        return b'broken' if self.bad == 'speech' else self.audio

    def transcribe(self, *args):
        self.calls.append('transcription')
        return '' if self.bad == 'transcription' else '준비됐어'


@pytest.mark.parametrize('bad', ['', 'text', 'speech', 'transcription', 'truncated'])
def test_probe_requires_all_real_response_types(tmp_path, bad):
    audio = Path(clip(tmp_path, 'probe.wav', 100)).read_bytes()
    api = API(audio[:44] if bad == 'truncated' else audio, bad)
    if bad:
        with pytest.raises(Exception):
            check_voice_api(api)
    else:
        check_voice_api(api)
        assert api.calls == ['text', 'speech', 'transcription']


def test_waits_for_readiness_retries_connection_and_playback_then_once():
    ready = threading.Event()
    probes, plays = [], []
    def probe():
        probes.append(1)
        if len(probes) == 1:
            raise ConnectionError()
    def announce(done):
        plays.append(1)
        done(len(plays) == 2)
        return True
    startup = StartupAnnouncement(probe, announce, ready.is_set, logging.getLogger(), .01)
    startup.start()
    startup.start()
    try:
        time.sleep(.03)
        assert not probes and not plays
        ready.set()
        wait(lambda: startup.played)
        startup._thread.join(1)
        assert len(probes) == 2 and len(plays) == 2 and startup.api_ready
    finally:
        startup.close()


def test_shutdown_while_api_pending_never_announces():
    entered, release = threading.Event(), threading.Event()
    plays = []
    def probe():
        entered.set()
        release.wait(2)
    startup = StartupAnnouncement(probe, lambda done: plays.append(done), lambda: True, logging.getLogger(), .01)
    startup.start()
    assert entered.wait(1)
    startup.close()
    release.set()
    startup._thread.join(1)
    assert not plays and not startup.played


def test_cancelled_announcement_is_not_retried():
    calls = []
    def announce(done):
        calls.append(1)
        done(None)
        return True
    startup = StartupAnnouncement(lambda: None, announce, lambda: True, logging.getLogger(), .01)
    startup.start()
    startup._thread.join(1)
    assert calls == [1] and not startup.played
    startup.close()


def test_narrator_plays_exact_original_without_synthesis(tmp_path):
    path = clip(tmp_path, 'melody.wav', 100)
    backend = Backend()
    n, _ = make(backend)
    result = []
    assert n.startup_recording(path, result.append)
    run(n)
    assert result == [True]
    assert n.player.played == [Path(path).read_bytes()]
    assert not backend.spoken and not backend.calls
    n.close()


def test_removed_recording_does_not_fall_back_to_cached_or_synthesized_audio(tmp_path):
    path = Path(clip(tmp_path, 'melody.wav', 100))
    backend = Backend()
    n, _ = make(backend)
    n.speech_cache = type('Cache', (), {'get': lambda *args: b'cached'})()
    result = []
    assert n.startup_recording(str(path), result.append)
    path.unlink()
    run(n)
    assert result == [False] and not n.player.played and not backend.spoken
    n.close()


def test_mute_close_stop_and_player_failure(tmp_path):
    path = clip(tmp_path, 'melody.wav', 100)
    n, _ = make()
    result = []
    n.muted = True
    assert not n.startup_recording(path, result.append)
    n.muted = False
    assert n.startup_recording(path, result.append)
    n.command("stop")
    run(n)
    assert result == [None] and not n.player.played
    n.close()
    n, _ = make()
    n.player.play = lambda *args, **kwargs: False
    assert n.startup_recording(path, result.append)
    run(n)
    assert result == [None, False]
    n.close()
    assert not n.startup_recording(path, result.append)


@pytest.mark.parametrize('command', ['stop', 'mute'])
def test_command_while_probe_pending_cancels_future_startup(tmp_path, command):
    path = clip(tmp_path, 'melody.wav', 100)
    entered, release = threading.Event(), threading.Event()
    n, _ = make(start=True)
    def probe():
        entered.set()
        release.wait(2)
    startup = StartupAnnouncement(probe, lambda done: n.startup_recording(path, done),
                                  lambda: True, logging.getLogger(), .01)
    startup.start()
    assert entered.wait(1)
    n.command(command)
    n.command('unmute')
    release.set()
    startup._thread.join(1)
    assert not n.player.played and not startup.played
    startup.close()
    n.close()


def test_mute_interrupts_running_startup(tmp_path):
    from test_narrator import Player
    path = clip(tmp_path, 'melody.wav', 100)
    player = Player(block=True)
    n, _ = make(player=player, start=True)
    outcome = []
    n.startup_recording(path, outcome.append)
    assert player.started.wait(1)
    n.command('mute')
    wait(lambda: bool(outcome))
    assert outcome == [None] and player.stops > 0
    n.close()
