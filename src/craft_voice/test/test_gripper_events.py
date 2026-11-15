"""그리퍼 동작에 맞춘 대사: /voice/event 로 집기 시나리오(execute.py)가 알려 준다.

gripcam_measured top=61 yaw=-22.9 -> "벡터 계산 완료"(녹음)만. 높이·각도는 말하지 않고 기억(사실 줄)에만 남김.
"""

from craft_voice.robot_state import StateTracker
from test_narrator import Backend, Clock, make, run


def notices(tracker, now):
    out = []
    while (n := tracker.notice(now)) is not None:
        out.append(n)
    return out


def test_measured_says_only_the_vector_line_and_remembers_the_numbers():
    t = StateTracker(style='cute')
    t.update_event('gripcam_measured top=61.4 yaw=-22.9', 100.0)
    assert [n.facts for n in notices(t, 100.1)] == ['벡터 계산 완료.']
    assert '그리퍼 카메라로 퍽을 잼 (윗면 높이 61 mm, 각도 -22.9도)' in t.memory_text(100.2)
    t = StateTracker(style='cute')
    t.update_event('gripcam_measured', 200.0)       # no numbers: the same line
    assert [n.facts for n in notices(t, 200.1)] == ['벡터 계산 완료.']


def test_none_loaded_and_unknown_events():
    t = StateTracker(style='cute')
    t.update_event('gripcam_none', 100.0)
    assert [n.facts for n in notices(t, 100.1)] == ['어? 퍽이 안 보여.']
    t.update_event('puck_loaded slot=7', 110.0)
    assert [n.facts for n in notices(t, 110.1)] == ['회전판 7번 칸에 실었어!']
    t.update_event('puck_loaded', 120.0)
    assert [n.facts for n in notices(t, 120.1)] == ['회전판에 실었어!']
    t.update_event('rm -rf top=1', 130.0)            # not an event name: nothing
    t.update_event('gripcam_measured top=abc yaw=nan', 140.0)
    assert [n.facts for n in notices(t, 140.1)] == ['벡터 계산 완료.']


def test_the_same_event_twice_within_3_s_is_said_once():
    t = StateTracker(style='cute')
    t.update_event('gripcam_measured top=61', 100.0)
    t.update_event('gripcam_measured top=62', 101.0)
    assert [n.key for n in notices(t, 101.1)] == ['gripcam_measured']
    t.update_event('gripcam_measured top=62', 104.0)
    assert [n.key for n in notices(t, 104.1)] == ['gripcam_measured']


def test_the_narrator_plays_only_the_recording(tmp_path):
    import wave
    clip = tmp_path / 'vector.wav'
    with wave.open(str(clip), 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(b'\0\0' * 2400)
    clock = Clock()
    narrator, texts = make(Backend(), clock=clock, notice=True, reactions={'gripcam_measured': str(clip)})
    narrator.tracker.style = 'cute'
    narrator.event('gripcam_measured top=61 yaw=-22.9')
    for _ in range(4):
        narrator.tick(); run(narrator); clock.now += 0.5
    assert texts == [('thought', '벡터 계산 완료.')]


def test_every_number_sentence_is_prepared():
    from craft_voice.prepare import fixed_sentences
    t = StateTracker(style='cute')
    have = set(fixed_sentences('cute', {}, {}, variety=True))
    t.update_event('gripcam_measured top=61 yaw=-22.9', 100.0)
    t.update_event('puck_loaded slot=7', 104.0)
    said = [n.facts for n in notices(t, 104.1)]
    assert said and all(s in have for s in said), [s for s in said if s not in have]
    for board in ('바퀴 모터', '팔 모터'):
        assert f'{board} 배터리가 부족해! 지금 10.9볼트야. 곧 충전해 줘.' in have
        assert f'{board} 배터리 거의 바닥이야! 9.9볼트야. 지금 바로 충전해 줘!' in have


class FakeBackend:
    def __init__(self):
        self.said, self.warmed, self.closed = [], 0, False

    def synthesize(self, text, urgent=False):
        if text == 'bad':
            raise RuntimeError('read something else')
        self.said.append(text)
        return b'RIFF' + text.encode()

    def warm(self):
        self.warmed += 1

    def close(self):
        self.closed = True


def test_prepare_in_background_fills_only_what_is_missing(tmp_path):
    import logging
    from craft_voice.audio import SpeechCache
    from craft_voice.wiring import prepare_in_background
    cache = SpeechCache(str(tmp_path), 'sig')
    import wave, io
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(b'\0\0' * 240)
    cache.put('있던 문장', buf.getvalue())
    made = []
    def make():
        b = FakeBackend(); made.append(b); return b
    thread = prepare_in_background(make, cache, ['있던 문장', '새 문장', 'bad'], logging.getLogger('t'), lambda: False, pause_s=0)
    thread.join(5)
    assert made[0].said == ['새 문장'] and made[0].closed
    assert cache.contains('새 문장') and not cache.contains('bad')
    assert prepare_in_background(make, cache, ['있던 문장'], logging.getLogger('t'), lambda: False) is None


def test_keep_warm_asks_the_backend_every_tick():
    backend = FakeBackend()
    narrator, _ = make(backend, keep_warm=True)
    narrator.tick(); narrator.tick()
    assert backend.warmed >= 2
    backend2 = FakeBackend()
    narrator2, _ = make(backend2)
    narrator2.tick()
    assert backend2.warmed == 0
