"""Where each spoken sentence came from (recording / fixed words / GPT) and how it was voiced, the
transcript check before a streamed sentence is cached, and the early speech connection."""
import logging
import os

import pytest

from craft_voice.audio import SpeechCache, pcm_to_wav
from craft_voice.narrator import Narrator, NarratorConfig, Utterance, source_note
from craft_voice.realtime import RealtimeError
from craft_voice.robot_state import StateTracker

from test_narrator import Backend, FakeStreamPlayer, Player, StreamingBackend, run
from test_realtime import FakeConnection, events, speech

READY = '준비됐어요. 출발 신호를 기다리고 있어요.'


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class HeardBackend(StreamingBackend):
    """Realtime-like: reports what the model said for this call through heard=."""

    def __init__(self, said=None, **kwargs):
        super().__init__(**kwargs)
        self.said = said      # None: says the sentence as given
        self.warms = 0

    def stream_speech(self, text, heard=None):
        for chunk in super().stream_speech(text):
            yield chunk
        if heard is not None:
            heard['transcript'] = text if self.said is None else self.said

    def warm(self):
        self.warms += 1


def ready_again(narrator):
    """Ready once more, past the window in which the same sentence is not repeated."""
    narrator.clock.now += 40.0
    narrator.planning_state('Run.Execute')
    run(narrator)
    narrator.clock.now += 40.0
    narrator.planning_state('Ready')
    run(narrator)


def make(backend, tmp_path, stream=True, **config):
    cache = SpeechCache(str(tmp_path / 'speech'), 'sig')
    sources = []
    narrator = Narrator(NarratorConfig(**config), StateTracker(), backend=backend, player=Player(),
                        stream_player=FakeStreamPlayer() if stream else None, clock=Clock(), start=False,
                        speech_cache=cache, logger=logging.getLogger('test_speech_source'),
                        on_source=lambda kind, text, note: sources.append((kind, text, note)))
    return narrator, cache, sources


# ---- (1) the transcript is checked before a streamed sentence is kept --------------------------------------
def test_streamed_fixed_sentence_is_cached_when_the_model_read_it(tmp_path, caplog):
    narrator, cache, sources = make(HeardBackend(), tmp_path, state_mode='template')
    with caplog.at_level(logging.INFO, logger='test_speech_source'):
        narrator.planning_state('Ready')
        run(narrator)
    assert cache.contains(READY) and narrator.stats['tts_mismatch'] == 0
    kind, text, note = sources[0]
    assert (kind, text) == ('state', READY) and note.startswith('말: 고정 문장 · 소리: 실시간 합성 · 첫 소리 ')
    assert any(r.message.startswith(f'say [state] {READY} ‹말: 고정 문장 · 소리: 실시간 합성') for r in caplog.records)
    assert any('받아쓰기 일치' in r.message for r in caplog.records)


def test_streamed_sentence_read_wrongly_is_played_but_not_cached(tmp_path, caplog):
    backend = HeardBackend(said='네, 출발 신호를 기다리는 중입니다. 무엇을 도와드릴까요?')
    narrator, cache, _ = make(backend, tmp_path, state_mode='template')
    with caplog.at_level(logging.INFO, logger='test_speech_source'):
        narrator.planning_state('Ready')
        run(narrator)
    assert narrator.stream_player.data == [b'aa', b'bb', b'cc']     # already out: it streams as it arrives
    assert not cache.contains(READY) and narrator.stats['tts_mismatch'] == 1
    assert any(r.levelno == logging.WARNING and 'not cached' in r.message for r in caplog.records)
    # next time it is synthesized again (and kept once it is read right)
    backend.said = None
    ready_again(narrator)
    assert backend.streams.count(READY) == 2 and cache.contains(READY)


def test_no_transcript_is_not_cached(tmp_path):
    narrator, cache, _ = make(HeardBackend(said=''), tmp_path, state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    assert not cache.contains(READY) and narrator.stats['tts_mismatch'] == 0


def test_a_backend_without_transcripts_keeps_the_old_caching(tmp_path):
    narrator, cache, _ = make(StreamingBackend(), tmp_path, state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    assert cache.contains(READY)


def test_gpt_sentence_read_wrongly_is_reported_and_never_cached(tmp_path, caplog):
    backend = HeardBackend(reply='상자 쪽으로 가는 중이에요.', said='전혀 다른 말이에요 정말로요')
    narrator, cache, sources = make(backend, tmp_path)
    with caplog.at_level(logging.INFO, logger='test_speech_source'):
        narrator.ask('뭐 해?')
        run(narrator)
    assert sources[0][2].startswith('말: GPT 대답 · 소리: 실시간 합성')
    assert narrator.stats['tts_mismatch'] == 1 and not cache.contains('상자 쪽으로 가는 중이에요.')


# ---- where the words and the sound came from --------------------------------------------------------------
def test_cache_and_protected_cache_are_told_apart(tmp_path):
    narrator, cache, sources = make(HeardBackend(), tmp_path, state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    ready_again(narrator)
    assert [s[2] for s in sources if s[1] == READY][1] == '말: 고정 문장 · 소리: 캐시'
    protected = os.path.join(cache.directory, 'protected')
    os.makedirs(protected)
    os.replace(cache.path(READY), os.path.join(protected, os.path.basename(cache.path(READY))))
    assert cache.lookup(READY)[1] == 'protected' and cache.contains(READY)
    ready_again(narrator)
    assert sources[-1][2] == '말: 고정 문장 · 소리: 보호 캐시'


def test_gpt_situation_answer_failure_typed_and_recording_are_labelled(tmp_path):
    narrator, _, sources = make(HeardBackend(reply='출발할게요. 앞이 비어 있어요.'), tmp_path)
    narrator.planning_state('Run.Execute')                     # state_mode gpt: GPT judges the situation
    run(narrator)
    assert sources[-1][2].startswith('말: GPT 상황 판단 · 소리: 실시간 합성')

    failing, _, sources = make(Backend(fail=True), tmp_path / 'b', stream=False)
    failing.ask('지금 어디야?')
    run(failing)
    assert sources[-1][2].startswith('말: GPT 실패→고정 문장 · 소리: 통째 합성 · 합성 ')

    typed, _, sources = make(HeardBackend(), tmp_path / 'c')
    typed.say('안녕하세요.')
    run(typed)
    assert sources[-1][2].startswith('말: 직접 입력 · 소리: 실시간 합성')

    clip = tmp_path / 'jetson.wav'
    clip.write_bytes(pcm_to_wav(b'\x01\x00' * 240, 24000))
    words, _, sources = make(HeardBackend(), tmp_path / 'd', word_clips={'젯슨': [str(clip)]})
    words.word('젯슨')
    run(words)
    assert sources[-1][2] == '말: 녹음 · 소리: 녹음'


def test_no_audio_is_said_too():
    u = Utterance('fault', '정지했어요.', 0.0)
    assert source_note(u) == '말: 고정 문장 · 소리: 없음'
    u = Utterance('ask', '네.', 0.0, origin='quick', trace={'sound': 'cache'})
    assert source_note(u) == '말: 정해진 답 · 소리: 캐시'


# ---- (3) the speech connection opens when a sentence will need it ----------------------------------------
def test_warm_only_for_sentences_that_need_synthesis(tmp_path):
    backend = HeardBackend()
    narrator, cache, _ = make(backend, tmp_path, state_mode='template')
    narrator.planning_state('Ready')                           # not cached yet: open a connection now
    assert backend.warms == 1
    run(narrator)
    narrator.clock.now += 40.0
    narrator.planning_state('Run.Execute')
    run(narrator)
    warms = backend.warms
    narrator.clock.now += 40.0
    narrator.planning_state('Ready')                           # cached now: nothing to open
    assert backend.warms == warms
    clip = tmp_path / 'r.wav'
    clip.write_bytes(pcm_to_wav(b'\x01\x00' * 240, 24000))
    with narrator._lock:
        narrator._enqueue(Utterance('ack', '응!', 0.0, clip=str(clip)))  # a recording: nothing to open
    assert backend.warms == warms


def test_after_a_sentence_the_next_one_waiting_gets_a_connection(tmp_path):
    backend = HeardBackend()
    narrator, _, _ = make(backend, tmp_path)
    with narrator._lock:
        narrator._enqueue(Utterance('say', '첫 번째 문장.', 100.0, origin='typed'))
        narrator._enqueue(Utterance('say', '두 번째 문장.', 100.0, origin='typed'))
    assert backend.warms == 2
    narrator.speak_once()
    assert backend.warms == 3                                   # opened again for the second sentence


# ---- the Realtime model reports its transcript per call ---------------------------------------------------
def test_realtime_stream_fills_heard_for_this_call():
    tts, _ = speech([FakeConnection(events(text='다른 말'))])
    heard = {}
    assert len(list(tts.stream('지금 가는 중이야!', heard=heard))) == 2
    assert heard == {'transcript': '다른 말'}


def test_realtime_synthesize_uses_its_own_transcript():
    tts, _ = speech([FakeConnection(events(text='엉뚱한 대답입니다 여러분')),
                     FakeConnection(events(text='엉뚱한 대답입니다 여러분'))])
    with pytest.raises(RealtimeError, match='did not read'):
        tts.synthesize('지금 가는 중이야!')
