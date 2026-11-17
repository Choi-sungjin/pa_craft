from pathlib import Path
import os

import pytest
from craft_voice.audio import SpeechCache, pcm_to_wav


def test_original_survives_eviction_and_synthetic_replacement(tmp_path):
    original = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    synthetic = pcm_to_wav(b'\x02\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'existing-voice-settings', max_files=2)
    key = Path(cache.path('갑니다!')).name
    pins = tmp_path / 'protected'; pins.mkdir()
    (pins / key).write_bytes(original)
    cache.put('갑니다!', synthetic)
    for i in range(10):
        cache.put(f'새 안내 {i}', synthetic)
    assert cache.get('갑니다!') == original
    assert (pins / key).read_bytes() == original
    assert len(list(tmp_path.glob('*.wav'))) == 2
    assert cache.get('새 안내 9') == synthetic


@pytest.fixture
def tied_timestamps(monkeypatch):
    real_utime = os.utime
    real_replace = os.replace

    def tie(path, *args, **kwargs):
        real_utime(path, ns=(1_700_000_000_000_000_000,) * 2)

    def replace(src, dst):
        real_replace(src, dst)
        tie(dst)

    monkeypatch.setattr(os, 'utime', tie)
    monkeypatch.setattr(os, 'replace', replace)


def test_latest_write_survives_equal_timestamps(tmp_path, tied_timestamps):
    wav = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'tied', max_files=1)
    for i in range(12):
        cache.put(str(i), wav)
        assert cache.get(str(i)) == wav
        assert len(list(tmp_path.glob('*.wav'))) == 1


def test_recent_read_survives_equal_timestamps(tmp_path, tied_timestamps):
    wav = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'tied', max_files=2)
    cache.put('first', wav)
    cache.put('second', wav)
    assert cache.get('first') == wav
    cache.put('third', wav)
    assert cache.get('first') == wav
    assert cache.get('second') is None
    assert cache.get('third') == wav


def test_zero_limit_keeps_only_protected_recording(tmp_path, tied_timestamps):
    wav = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'zero', max_files=0)
    pins = tmp_path / 'protected'
    pins.mkdir()
    (pins / Path(cache.path('original')).name).write_bytes(wav)
    cache.put('temporary', wav)
    assert cache.get('temporary') is None
    assert cache.get('original') == wav


def test_existing_cache_uses_disk_recency_after_restart(tmp_path):
    wav = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'restart', max_files=2)
    cache.put('old', wav)
    cache.put('newer', wav)
    os.utime(cache.path('old'), ns=(1_000_000_000,) * 2)
    os.utime(cache.path('newer'), ns=(2_000_000_000,) * 2)
    reopened = SpeechCache(str(tmp_path), 'restart', max_files=2)
    reopened.put('latest', wav)
    assert reopened.get('old') is None
    assert reopened.get('newer') == wav
    assert reopened.get('latest') == wav


def test_unpinned_cache_and_different_voice_signature_are_unchanged(tmp_path):
    wav = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'old')
    cache.put('일반 문장', wav)
    assert cache.get('일반 문장') == wav
    pins = tmp_path / 'protected'; pins.mkdir()
    Path(pins, Path(cache.path('갑니다!')).name).write_bytes(wav)
    assert SpeechCache(str(tmp_path), 'different').get('갑니다!') is None
