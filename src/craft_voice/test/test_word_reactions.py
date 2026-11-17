import logging
import random
import wave

from craft_voice.listen import Listener
from craft_voice.wiring import load_word_reactions
from test_narrator import Backend, make, run


def clip(tmp_path, name, sample):
    path = tmp_path / name
    with wave.open(str(path), 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(24000)
        wav.writeframes(sample.to_bytes(2, 'little', signed=True) * 2400)
    return str(path)


def test_word_pool_filters_missing_corrupt_empty_and_duplicate_files(tmp_path, caplog):
    good = clip(tmp_path, 'good.wav', 100)
    broken = tmp_path / 'broken.wav'
    broken.write_bytes(b'not audio')
    spec = f'젯슨|제슨={good}|{broken}|{tmp_path}/missing.wav||{good};bad={broken};invalid'
    words, pools = load_word_reactions(spec, logging.getLogger('test'))
    assert words == {'젯슨': ('젯슨', '제슨')}
    assert pools == {'젯슨': (good,)}
    assert 'not used' in caplog.text


def test_empty_and_legacy_single_file_specs(tmp_path):
    log = logging.getLogger('test')
    assert load_word_reactions('', log) == ({}, {})
    path = clip(tmp_path, 'one.wav', 100)
    words, pools = load_word_reactions(f'젯슨={path}', log)
    narrator, _ = make(word_clips=pools)
    listener = Listener(None, lambda wav: '젯슨', narrator.wake,
                        words=words, on_word=narrator.word)
    listener.process(b'\0\0' * 16000, 0)
    run(narrator)
    assert narrator.player.played == [open(path, 'rb').read()]


def test_truncated_wav_does_not_register_a_word(tmp_path):
    from pathlib import Path
    path = Path(clip(tmp_path, 'truncated.wav', 100))
    path.write_bytes(path.read_bytes()[:44])
    assert load_word_reactions(f'젯슨={path}', logging.getLogger('test')) == ({}, {})


def test_random_originals_only_on_word_call_not_state_or_craft(tmp_path, monkeypatch):
    first = clip(tmp_path, 'melody.wav', 100)
    second = clip(tmp_path, 'short.wav', 200)
    wake = clip(tmp_path, 'wake.wav', 300)
    ready = clip(tmp_path, 'ready.wav', 400)
    words, pools = load_word_reactions(f'젯슨|제슨={first}|{second}', logging.getLogger('test'))
    chooser = random.Random(7)
    monkeypatch.setattr('craft_voice.narrator.random.choice', chooser.choice)
    backend = Backend()
    narrator, _ = make(backend=backend, state_mode='template',
                       word_clips=pools, reactions={'wake': wake, 'ready': ready})
    listener = Listener(None, lambda wav: '젯슨', narrator.wake,
                        words=words, on_word=narrator.word)
    for _ in range(20):
        before = len(narrator.player.played)
        listener.process(b'\0\0' * 16000, 0)
        run(narrator)
        assert len(narrator.player.played) == before + 1
    originals = {open(first, 'rb').read(), open(second, 'rb').read()}
    assert set(narrator.player.played) == originals
    assert backend.spoken == [] and backend.calls == []
    for text in ('레슨', '잭슨', '', '여기 봐'):
        listener.transcribe = lambda wav, text=text: text
        listener.process(b'\0\0' * 16000, 0)
        run(narrator)
    assert len(narrator.player.played) == 20
    narrator.planning_state('Ready')
    run(narrator)
    assert narrator.player.played[-1] == open(ready, 'rb').read()
    listener.transcribe = lambda wav: '크래프트'
    listener.process(b'\0\0' * 16000, 0)
    run(narrator)
    assert open(wake, 'rb').read() in narrator.player.played[21:]
    assert not originals.intersection(narrator.player.played[20:])


def test_word_pool_respects_mute_and_close(tmp_path):
    path = clip(tmp_path, 'one.wav', 100)
    narrator, _ = make(word_clips={'젯슨': (path,)})
    narrator.muted = True
    narrator.word('젯슨')
    run(narrator)
    assert not narrator.player.played
    narrator.muted = False
    narrator.word('unknown')
    run(narrator)
    assert not narrator.player.played
    narrator.close()
    narrator.word('젯슨')
    assert not narrator.player.played
