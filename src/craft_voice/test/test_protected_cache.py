from pathlib import Path
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


def test_unpinned_cache_and_different_voice_signature_are_unchanged(tmp_path):
    wav = pcm_to_wav(b'\x01\x00' * 2400, 24000)
    cache = SpeechCache(str(tmp_path), 'old')
    cache.put('일반 문장', wav)
    assert cache.get('일반 문장') == wav
    pins = tmp_path / 'protected'; pins.mkdir()
    Path(pins, Path(cache.path('갑니다!')).name).write_bytes(wav)
    assert SpeechCache(str(tmp_path), 'different').get('갑니다!') is None
