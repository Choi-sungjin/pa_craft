import math
import struct
import sys
import time

import pytest

from craft_voice.listen import (FRAME_BYTES, RATE, Listener, Microphone, Segmenter, distance, find_wake, jamo,
                                name_distance)


# ---- the name -------------------------------------------------------------------------------------------
def test_jamo_and_distance():
    assert jamo('크래프트') == 'ㅋㅡㄹㅐㅍㅡㅌㅡ' and jamo('Craft') == 'craft'
    assert distance('abc', 'abc') == 0 and distance('abc', 'axc') == 1 and distance('abc', 'a') == 2
    assert name_distance('크래프트', '크래프트') == 0 and name_distance('CRAFT', '크래프트') == 0


@pytest.mark.parametrize('heard, question', [
    ('크래프트', ''),
    ('크래프트!', ''),
    ('크래프트, 지금 뭐 해?', '지금 뭐 해'),
    ('크래프트야, 왜 멈췄어?', '왜 멈췄어'),
    ('저기, 크래프트 지금 상태 어때?', '지금 상태 어때'),
    ('지금 뭐 해 크래프트?', '지금 뭐 해'),
    # what speech-to-text really returned for "크래프트" (2026-11-11, synthetic voices)
    ('크립트, 지금 뭐 해?', '지금 뭐 해'),
    ('크래프티야, 왜 멈췄어?', '왜 멈췄어'),
    ('저기 케프트 지금 상태 어때?', '지금 상태 어때'),
    ('크레프트, 어디 가는 중이야?', '어디 가는 중이야'),
    ('크라프트 뭐 해', '뭐 해'),
    ('그래프트 어디야', '어디야'),
    ('크래 프트 어디 가', '어디 가'),
    ('Craft, what are you doing', 'what are you doing'),
    ('헤이 크래프트', ''),
])
def test_the_name_is_heard_with_its_question(heard, question):
    assert find_wake(heard) == (True, question)


@pytest.mark.parametrize('heard', [
    '그래프 좀 다시 그려줄래?', '오늘 점심 뭐 먹을까?', '크래프톤 주가 봤어?', '이 로봇 지금 뭐 하는 거야?',
    '클래스 끝나고 프로토타입 보자.', '그 트랙 다시 깔아야 돼.', '시청해 주셔서 감사합니다.', '', '   ',
])
def test_other_talk_is_not_the_name(heard):
    assert find_wake(heard) == (False, '')


def test_other_names_can_be_set():
    assert find_wake('로봇아 뭐 해', names=('로봇',)) == (True, '뭐 해')
    assert find_wake('크래프트 뭐 해', names=('로봇',)) == (False, '')


# ---- speech segments ----------------------------------------------------------------------------------------
def tone(seconds, amplitude, frequency=220.0):
    count = int(seconds * RATE)
    return b''.join(struct.pack('<h', int(amplitude * math.sin(2 * math.pi * frequency * i / RATE)))
                    for i in range(count))


def frames(pcm):
    return [pcm[i:i + FRAME_BYTES] for i in range(0, len(pcm) - FRAME_BYTES + 1, FRAME_BYTES)]


def feed(segmenter, pcm):
    return [segment for segment in (segmenter.feed(frame) for frame in frames(pcm)) if segment is not None]


def test_a_sentence_between_silence_comes_out_once():
    segmenter = Segmenter()
    out = feed(segmenter, tone(1.0, 0) + tone(1.2, 6000) + tone(1.0, 0))
    assert len(out) == 1
    seconds = len(out[0]) / (RATE * 2)
    assert 1.2 <= seconds <= 2.2          # the sentence, a little before it and the pause that ended it


def test_clicks_and_silence_are_not_a_sentence():
    segmenter = Segmenter()
    assert feed(segmenter, tone(2.0, 0)) == []
    assert feed(segmenter, tone(0.5, 0) + tone(0.06, 8000) + tone(1.5, 0)) == []   # 60 ms: a click, not speech


def test_steady_noise_becomes_the_floor_and_speech_over_it_is_found():
    """Driving noise: after a moment it is the room, not speech; a voice clearly above it still is."""
    segmenter = Segmenter()
    noise = tone(6.0, 1500, frequency=90.0)
    assert len(feed(segmenter, tone(1.0, 0) + noise)) <= 1  # when it starts it may pass once as a "sentence"
    assert not segmenter.active                           # after that the steady noise is the room
    assert feed(segmenter, noise + noise) == []           # 12 s of driving noise: nothing goes to speech-to-text
    out = feed(segmenter, tone(1.0, 9000) + tone(1.5, 1500, frequency=90.0))
    assert len(out) == 1                                  # a voice over the noise is still found


def test_a_long_sentence_with_word_gaps_is_not_mistaken_for_noise():
    segmenter = Segmenter()
    words = b''.join(tone(0.35, 6000) + tone(0.08, 0) for _ in range(14))   # 6 s of talk with short gaps
    out = feed(segmenter, tone(1.0, 0) + words + tone(1.0, 0))
    assert len(out) == 1 and len(out[0]) / (RATE * 2) > 5.5


def test_an_endless_sentence_is_cut_at_the_limit():
    segmenter = Segmenter(max_s=2.0)
    out = feed(segmenter, tone(0.3, 0) + tone(5.0, 6000))
    assert out and len(out[0]) == 2 * RATE * 2


def test_vad_veto():
    segmenter = Segmenter(vad=lambda frame: False)          # loud, but the VAD says it is not a voice
    assert feed(segmenter, tone(0.5, 0) + tone(1.0, 6000) + tone(1.0, 0)) == []


# ---- microphone -----------------------------------------------------------------------------------------------
def test_microphone_reads_frames_from_a_command():
    code = f"import sys; sys.stdout.buffer.write(b'\\x01\\x00' * {FRAME_BYTES}); sys.stdout.flush()"
    mic = Microphone([sys.executable, '-c', code])
    mic.open()
    try:
        assert mic.read() == b'\x01\x00' * (FRAME_BYTES // 2)
        assert mic.read() == b'\x01\x00' * (FRAME_BYTES // 2)
        assert mic.read() is None                           # the command ended
    finally:
        mic.close()
    with pytest.raises(ValueError):
        Microphone([])


# ---- listener ---------------------------------------------------------------------------------------------------
class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeMic:
    def open(self):
        pass

    def read(self):
        return None

    def close(self):
        pass


def listener(texts, **kwargs):
    """Listener whose speech-to-text returns the given texts in turn."""
    heard = {'wake': [], 'followup': [], 'stt': 0}
    answers = iter(texts)

    def transcribe(wav):
        heard['stt'] += 1
        assert wav[:4] == b'RIFF'
        value = next(answers)
        if isinstance(value, Exception):
            raise value
        return value

    clock = kwargs.pop('clock', None) or Clock()
    speaking = kwargs.pop('speaking', None) or [False]
    lst = Listener(FakeMic(), transcribe, heard['wake'].append, on_followup=heard['followup'].append,
                   is_speaking=lambda: speaking[0], clock=clock, **kwargs)
    return lst, heard, clock, speaking


def say(lst, clock, loud_s=1.0):
    """Feed a sentence and the pause after it; run speech-to-text for whatever came out."""
    for frame in frames(tone(0.3, 0) + tone(loud_s, 6000) + tone(0.9, 0)):
        clock.now += 0.03
        lst.hear(frame)
    done = 0
    while not lst._segments.empty():
        lst.process(*lst._segments.get())
        done += 1
    return done


def test_name_with_a_question_calls_on_wake():
    lst, heard, clock, _ = listener(['크래프트, 지금 뭐 해?', '크래프트'])
    assert say(lst, clock) == 1 and heard['wake'] == ['지금 뭐 해']
    assert say(lst, clock) == 1 and heard['wake'] == ['지금 뭐 해', '']
    assert lst.stats['wakes'] == 2 and lst.stats['ignored'] == 0


def test_other_talk_is_ignored_and_not_logged(caplog):
    lst, heard, clock, _ = listener(['오늘 점심 뭐 먹을까?'])
    with caplog.at_level('INFO', logger='craft_voice'):
        say(lst, clock)
    assert heard['wake'] == [] and heard['followup'] == [] and lst.stats['ignored'] == 1
    assert '점심' not in caplog.text                         # other people's talk stays out of the normal log


def test_nothing_is_heard_while_the_robot_speaks_or_right_after():
    lst, heard, clock, speaking = listener(['크래프트'])
    speaking[0] = True
    say(lst, clock)
    assert heard['stt'] == 0                                 # its own voice never reaches speech-to-text
    speaking[0] = False
    for frame in frames(tone(0.5, 6000)):                    # the echo right after it stopped (tail_s 0.6)
        clock.now += 0.03
        lst.hear(frame)
    assert heard['stt'] == 0 and not lst.segmenter.active
    clock.now += 1.0
    say(lst, clock)
    assert heard['wake'] == ['']


def test_follow_up_without_the_name_only_right_after_an_answer():
    lst, heard, clock, speaking = listener(['왜?', '크래프트', '왜 멈췄어?', '그건 언제 끝나?'], follow_up_s=6.0)
    say(lst, clock)
    assert heard['followup'] == []                           # nobody called the robot: not for it
    say(lst, clock)
    assert heard['wake'] == ['']
    speaking[0] = True                                       # the robot answers ...
    lst.hear(b'\x00' * FRAME_BYTES)
    clock.now += 2.0
    speaking[0] = False                                      # ... and is done
    lst.hear(b'\x00' * FRAME_BYTES)
    clock.now += 1.0
    say(lst, clock)
    assert heard['followup'] == ['왜 멈췄어?']
    clock.now += 20.0                                        # long after: the window is closed again
    say(lst, clock)
    assert heard['followup'] == ['왜 멈췄어?'] and lst.stats['ignored'] == 2


def test_follow_up_can_be_switched_off():
    lst, heard, clock, speaking = listener(['크래프트', '왜?'], follow_up_s=0.0)
    lst.on_followup = None
    say(lst, clock)
    speaking[0] = True
    lst.hear(b'\x00' * FRAME_BYTES)
    speaking[0] = False
    lst.hear(b'\x00' * FRAME_BYTES)
    clock.now += 1.0
    say(lst, clock)
    assert heard['followup'] == [] and lst.stats['ignored'] == 1


def test_speech_to_text_budget_and_errors():
    lst, heard, clock, _ = listener([ConnectionError('offline'), '크래프트'], max_stt_per_min=2)
    say(lst, clock)
    assert lst.stats['stt_errors'] == 1 and heard['wake'] == []   # a failed request is not a crash
    say(lst, clock)
    assert heard['wake'] == ['']
    say(lst, clock)                                          # third sentence within a minute: over the budget
    assert heard['stt'] == 2 and lst.stats['over_budget'] == 1
    clock.now += 61.0
    lst.transcribe = lambda wav: '크래프트 뭐 해'
    say(lst, clock)
    assert heard['wake'] == ['', '뭐 해']


def test_disabled_listener_hears_nothing():
    lst, heard, clock, _ = listener(['크래프트'])
    lst.enabled = False
    say(lst, clock)
    assert heard['stt'] == 0


def test_threads_run_a_real_command_and_stop(tmp_path):
    """Capture + speech-to-text threads with a command as the microphone; close() ends both quickly."""
    pcm = tone(0.3, 0) + tone(1.0, 6000) + tone(1.0, 0)
    raw = tmp_path / 'voice.raw'
    raw.write_bytes(pcm)
    code = ('import sys,time\n'
            f'data=open(r"{raw}","rb").read()\n'
            f'for i in range(0,len(data),{FRAME_BYTES}):\n'
            f'    sys.stdout.buffer.write(data[i:i+{FRAME_BYTES}]); sys.stdout.flush(); time.sleep(0.002)\n'
            'time.sleep(30)\n')
    woke = []
    lst = Listener(Microphone([sys.executable, '-c', code]), lambda wav: '크래프트, 지금 뭐 해?', woke.append,
                   retry_s=0.2)
    lst.start()
    deadline = time.monotonic() + 10.0
    while not woke and time.monotonic() < deadline:
        time.sleep(0.02)
    started = time.monotonic()
    lst.close()
    assert woke == ['지금 뭐 해'] and time.monotonic() - started < 4.0


JETSON = {'젯슨': ('젯슨', '제슨', '제트슨', '젯선', 'jetson')}


def test_a_reaction_word_plays_its_recording_not_a_conversation():
    words = []
    lst, heard, clock, _ = listener(['젯슨!', 'Jetson', '레슨 몇 시야?', '잭슨', '크래프트, 젯슨이 뭐야?'],
                                    words=JETSON, on_word=words.append)
    for _ in range(5):
        say(lst, clock)
    assert words == ['젯슨', '젯슨']                       # 레슨, 잭슨: not the word
    assert heard['wake'] == ['젯슨이 뭐야']                  # called by name: a conversation, as before
    assert lst.stats['words'] == 2


def test_the_word_reaction_is_spoken_as_its_recording(tmp_path):
    import wave
    from test_narrator import make, run
    clip = tmp_path / 'yeap.wav'
    with wave.open(str(clip), 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(24000); w.writeframes(b'\x00\x01' * 2400)
    narrator, texts = make(reactions={'word.젯슨': str(clip)})
    narrator.word('젯슨')
    narrator.word('없는단어')                                # no recording: nothing (a warning)
    run(narrator)
    assert texts == [('ack', '젯슨')] and len(narrator.player.played) == 1
