import os
import subprocess
import time
from pathlib import Path

import pytest

from craft_voice import clock, plainclient, realtime
from craft_voice.listen import Listener
from test_listen import Clock, FakeMic, say

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'craft-clock-restore'


def test_1970_and_the_day_before_the_code_are_not_the_real_time():
    assert clock.clock_unset(34.0)
    assert clock.clock_unset(os.path.getmtime(clock.__file__) - 2 * 86400)
    assert not clock.clock_unset(os.path.getmtime(clock.__file__))
    assert not clock.clock_unset()


def test_nothing_is_sent_while_the_clock_is_unset(monkeypatch):
    def no_network(*args, **kwargs):
        raise AssertionError('a connection was tried')
    monkeypatch.setattr(plainclient, 'clock_unset', lambda: True)
    monkeypatch.setattr(realtime, 'clock_unset', lambda: True)
    monkeypatch.setattr(plainclient.urllib.request, 'urlopen', no_network)
    monkeypatch.setattr(realtime.socket, 'create_connection', no_network)
    client = plainclient.PlainClient(api_key='k', base_url='https://example.invalid/v1')
    with pytest.raises(plainclient.ApiError, match='clock is not set'):
        client.responses.create(model='m', input='hi')
    with pytest.raises(realtime.RealtimeError, match='clock is not set'):
        realtime.WebSocket('wss://example.invalid/v1/realtime')


def test_speech_is_dropped_until_the_clock_is_set():
    ticks, ready, woke, asked = Clock(), [False], [], []

    def transcribe(wav):
        asked.append(wav)
        return '크래프트, 지금 뭐 해?'
    lst = Listener(FakeMic(), transcribe, woke.append, clock=ticks, ready=lambda: ready[0])
    say(lst, ticks)
    assert asked == [] and woke == [] and lst.stats['segments'] == 0 and lst.stats['stt_errors'] == 0
    ready[0] = True
    say(lst, ticks)
    assert len(asked) == 1 and woke == ['지금 뭐 해']


def run_restore(tmp_path, stamp_time, synced=False):
    stamp = tmp_path / 'clock'
    stamp.touch()
    os.utime(stamp, (stamp_time, stamp_time))
    flag = tmp_path / 'synchronized'
    if synced:
        flag.touch()
    env = dict(os.environ, CRAFT_CLOCK_STAMP=str(stamp), CRAFT_CLOCK_SYNCED=str(flag), CRAFT_CLOCK_SECONDS='1',
               CRAFT_CLOCK_DRY='1')
    return subprocess.run(['sh', str(SCRIPT)], env=env, capture_output=True, text=True, timeout=10)


def test_restore_sets_a_clock_that_is_behind_the_saved_time(tmp_path):
    done = run_restore(tmp_path, time.time() + 3600)
    assert done.returncode == 0 and 'would set the clock' in done.stdout


def test_restore_never_moves_the_clock_back_and_stops_once_synchronized(tmp_path):
    done = run_restore(tmp_path, time.time() - 3600)
    assert done.returncode == 0 and done.stdout == ''
    done = run_restore(tmp_path, time.time() + 3600, synced=True)
    assert done.returncode == 0 and done.stdout == ''
