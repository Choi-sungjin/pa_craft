import glob
import os
import sys
import tempfile
import threading
import time

import pytest

from craft_voice.audio import CommandPlayer, CommandTts, split_command

PY = sys.executable


def leftovers():
    return set(glob.glob(os.path.join(tempfile.gettempdir(), 'craft_voice_*.wav')))


def test_player_passes_the_file(tmp_path):
    out = tmp_path / 'copy.wav'
    before = leftovers()
    player = CommandPlayer([PY, '-c', f'import shutil,sys; shutil.copy(sys.argv[1], {str(out)!r})', '{file}'])
    assert player.play(b'RIFFdata') is True
    assert out.read_bytes() == b'RIFFdata'
    assert leftovers() == before                               # temp file removed


def test_player_appends_file_without_placeholder(tmp_path):
    out = tmp_path / 'copy.wav'
    player = CommandPlayer([PY, '-c', f'import shutil,sys; shutil.copy(sys.argv[-1], {str(out)!r})'])
    assert player.play(b'abc') and out.read_bytes() == b'abc'


def test_stop_cuts_playback_short():
    player = CommandPlayer([PY, '-c', 'import time; time.sleep(10)', '{file}'])
    result = {}
    thread = threading.Thread(target=lambda: result.setdefault('ok', player.play(b'x')))
    started = time.monotonic()
    thread.start()
    deadline = time.monotonic() + 3
    while not player.playing and time.monotonic() < deadline:
        time.sleep(0.01)
    player.stop()
    thread.join(3)
    assert result == {'ok': False} and time.monotonic() - started < 3


def test_cancelled_before_start_never_runs(tmp_path):
    out = tmp_path / 'copy.wav'
    player = CommandPlayer([PY, '-c', f'import shutil,sys; shutil.copy(sys.argv[1], {str(out)!r})', '{file}'])
    assert player.play(b'x', cancelled=lambda: True) is False and not out.exists()


def test_player_errors():
    with pytest.raises(RuntimeError, match='exited 3'):
        CommandPlayer([PY, '-c', 'import sys; sys.exit(3)', '{file}']).play(b'x')
    with pytest.raises(RuntimeError, match='cannot run player'):
        CommandPlayer(['/nonexistent/player', '{file}']).play(b'x')
    with pytest.raises(ValueError):
        CommandPlayer([])


def test_command_tts():
    tts = CommandTts([PY, '-c', 'import sys; sys.stdout.buffer.write(sys.argv[1].encode())', '{text}'])
    assert tts.synthesize('안녕; rm -rf /') == '안녕; rm -rf /'.encode()   # argv, never a shell
    assert tts.synthesize(' --w/home/x/.bashrc') == b'w/home/x/.bashrc'   # never an option
    assert tts.synthesize('- -f/etc/x') == b'f/etc/x'
    with pytest.raises(ValueError):
        tts.synthesize(' -- ')
    with pytest.raises(RuntimeError):
        CommandTts([PY, '-c', 'import sys; sys.exit(1)', '{text}']).synthesize('x')
    with pytest.raises(ValueError):
        CommandTts(['espeak-ng'])


def test_split_command():
    assert split_command('aplay -q -D "plughw:1,0" {file}') == ['aplay', '-q', '-D', 'plughw:1,0', '{file}']
    assert split_command('  ') == []
