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


def test_normalize_wav_fixes_streaming_header():
    from craft_voice.audio import normalize_wav, pcm_to_wav, wav_seconds
    good = pcm_to_wav(b'\x01\x00' * 22050, 22050)
    broken = bytearray(good)
    broken[4:8] = b'\xff\xff\xff\xff'                          # RIFF size placeholder
    broken[40:44] = b'\xff\xff\xff\x7f'                        # data size placeholder, like espeak-ng --stdout
    assert wav_seconds(bytes(broken)) > 1000
    fixed = normalize_wav(bytes(broken))
    assert wav_seconds(fixed) == pytest.approx(1.0) and fixed == good
    assert normalize_wav(b'not a wav') == b'not a wav'


@pytest.mark.skipif(not __import__('shutil').which('espeak-ng'), reason='espeak-ng not installed')
def test_real_espeak_ng_korean_with_dashes():
    from craft_voice.audio import wav_seconds
    tts = CommandTts(split_command('espeak-ng -v ko --stdout -- {text}'))
    seconds = wav_seconds(tts.synthesize('-w/tmp/craft_voice_should_not_exist 문제가 생겨서 멈췄어요.'))
    assert 0.5 < seconds < 30 and not os.path.exists('/tmp/craft_voice_should_not_exist')


def test_modules_import_without_fcntl():
    """Windows has no fcntl: the laptop demo must still import (only the shm reader needs it)."""
    import subprocess
    code = ('import sys; sys.modules["fcntl"] = None; '
            'import craft_voice.demo, craft_voice.narrator, craft_voice.gpt; '
            'from craft_voice.frames import encode_jpeg; import numpy as np; '
            'print(len(encode_jpeg(np.zeros((8, 8, 3), np.uint8))) > 0)')
    package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    result = subprocess.run([PY, '-c', code], capture_output=True, text=True, cwd=package, timeout=60)
    assert result.returncode == 0 and result.stdout.strip() == 'True', result.stderr


def test_split_command_windows_paths():
    assert split_command(r'C:\ffmpeg\bin\ffplay.exe -nodisp -autoexit {file}', windows=True) == \
        [r'C:\ffmpeg\bin\ffplay.exe', '-nodisp', '-autoexit', '{file}']
    assert split_command(r'"C:\Program Files\VLC\vlc.exe" --play-and-exit {file}', windows=True) == \
        [r'C:\Program Files\VLC\vlc.exe', '--play-and-exit', '{file}']


def test_split_command_windows_quotes_inside_an_argument():
    assert split_command('espeak-ng --voice="Microsoft Heami" -- {text}', windows=True) == \
        ['espeak-ng', '--voice=Microsoft Heami', '--', '{text}']
    assert split_command('a "" b', windows=True) == ['a', '', 'b']
