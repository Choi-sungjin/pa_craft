"""Hear the robot's name and the question that follows. No ROS in here.

    microphone command (raw 16 kHz mono PCM on stdout)
      -> speech segments (energy over the noise floor, plus WebRTC VAD when it is installed)
      -> speech-to-text
      -> "크래프트, 지금 뭐 해?"  ->  on_wake('지금 뭐 해?')      ("크래프트" alone -> on_wake(''))

The microphone stays open all the time; what it picks up while the robot itself is speaking (and for
tail_s after) is thrown away, so the robot never answers its own voice. After the robot answered, a
short follow-up window lets the next sentence through without the name ("왜?", "얼마나 걸려?").
"""

from __future__ import annotations

import logging
import math
import queue
import re
import subprocess
import threading
import time
from array import array
from collections import deque

from .audio import pcm_to_wav

RATE = 16000
FRAME_MS = 30
FRAME_BYTES = RATE * FRAME_MS // 1000 * 2

# ---- the name --------------------------------------------------------------------------------------
_CHO = 'ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ'
_JUNG = 'ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ'
_JONG = ('', 'ㄱ', 'ㄲ', 'ㄳ', 'ㄴ', 'ㄵ', 'ㄶ', 'ㄷ', 'ㄹ', 'ㄺ', 'ㄻ', 'ㄼ', 'ㄽ', 'ㄾ', 'ㄿ', 'ㅀ', 'ㅁ', 'ㅂ', 'ㅄ',
         'ㅅ', 'ㅆ', 'ㅇ', 'ㅈ', 'ㅊ', 'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ')
# Speech-to-text hears the same sound as different letters: treat these as one when comparing.
_SOUND = str.maketrans({'ㄲ': 'ㄱ', 'ㅋ': 'ㄱ', 'ㄸ': 'ㄷ', 'ㅌ': 'ㄷ', 'ㅃ': 'ㅂ', 'ㅍ': 'ㅂ', 'ㅆ': 'ㅅ', 'ㅉ': 'ㅈ',
                        'ㅊ': 'ㅈ', 'ㅐ': 'ㅔ', 'ㅒ': 'ㅖ', 'ㅙ': 'ㅞ', 'ㅚ': 'ㅞ'})
ENGLISH_NAMES = {'크래프트': ('craft', 'kraft', 'crafts')}
# Words that sound close but are something else.
NOT_THE_NAME = ('크래프톤', '그래프', '그래프도', '그래프는', '그래프를', '그래프가')
_PARTICLES = ('야', '아', '님', '씨', '이', '요', '여', '는', '은', '가', '도', '한테', '에게')
_FILLERS = ('저기', '저기요', '어이', '헤이', '야', '있잖아', '음', '어', '자', '오케이', '헤이', '안녕')
_TOKEN = re.compile(r'[0-9A-Za-z가-힣]+')


def jamo(text: str) -> str:
    """'크래프트' -> 'ㅋㅡㄹㅐㅍㅡㅌㅡ' (letters other than Hangul syllables are kept, lowercased)."""
    out = []
    for char in text:
        code = ord(char) - 0xAC00
        if 0 <= code < 11172:
            out.append(_CHO[code // 588] + _JUNG[code % 588 // 28] + _JONG[code % 28])
        else:
            out.append(char.lower())
    return ''.join(out)


def distance(a: str, b: str) -> int:
    """Edit distance (insert, delete, replace)."""
    if len(a) < len(b):
        a, b = b, a
    row = list(range(len(b) + 1))
    for i, x in enumerate(a, 1):
        previous, row[0] = row[0], i
        for j, y in enumerate(b, 1):
            previous, row[j] = row[j], min(row[j] + 1, row[j - 1] + 1, previous + (x != y))
    return row[-1]


def name_distance(word: str, name: str) -> int:
    """How far a heard word is from the name, in letters that sound different (0 = the name)."""
    lowered = word.lower()
    if lowered in ENGLISH_NAMES.get(name, ()) or lowered == name.lower():
        return 0
    if word in NOT_THE_NAME:
        return 99
    return distance(jamo(word).translate(_SOUND), jamo(name).translate(_SOUND))


def find_wake(text: str, names=('크래프트',), tolerance: int = 2) -> tuple[bool, str]:
    """(the robot was called by name, the rest of the sentence without the name and fillers)."""
    tokens = [(m.group(), m.start(), m.end()) for m in _TOKEN.finditer(text)]
    best = None  # (distance, first token, last token)
    for index, (word, _, _) in enumerate(tokens):
        candidates = [(word, index)]
        for particle in _PARTICLES:  # "크래프트야", "크래프트는"
            if word.endswith(particle) and len(word) > len(particle) + 1:
                candidates.append((word[:-len(particle)], index))
        if index + 1 < len(tokens):  # heard as two words: "크래 프트"
            candidates.append((word + tokens[index + 1][0], index + 1))
        for candidate, last in candidates:
            for name in names:
                if abs(len(candidate) - len(name)) > 2 and candidate.isascii() == name.isascii():
                    continue
                d = name_distance(candidate, name)
                if d <= tolerance and (best is None or d < best[0]):
                    best = (d, index, last)
    if best is None:
        return False, ''
    _, first, last = best
    before = text[:tokens[first][1]]
    after = text[tokens[last][2]:]
    return True, _question(after) or _question(before)


def _question(text: str) -> str:
    words = _TOKEN.findall(text)
    while words and words[0] in _FILLERS:
        words.pop(0)
    while words and words[-1] in _FILLERS:
        words.pop()
    if not words:
        return ''
    return re.sub(r'\s+', ' ', text).strip(' ,.!?~…·-') if len(words) == len(_TOKEN.findall(text)) else ' '.join(words)


# ---- speech segments ---------------------------------------------------------------------------------
def rms(frame: bytes) -> float:
    samples = array('h', frame[:len(frame) - len(frame) % 2])
    if not samples:
        return 0.0
    return math.sqrt(sum(s * s for s in samples) / len(samples))


def webrtc_vad(aggressiveness: int = 2):
    """is_speech(frame) from the webrtcvad package, or None if it is not installed."""
    try:
        import webrtcvad
    except ImportError:
        return None
    vad = webrtcvad.Vad(aggressiveness)
    return lambda frame: vad.is_speech(frame, RATE)


class Segmenter:
    """30 ms frames in, one finished utterance (PCM bytes) out when the speaker pauses.

    A frame counts as speech when it is clearly louder than the noise floor and, if a VAD is given, the
    VAD agrees. The floor is the quietest tenth of the last few seconds: a voice has gaps between words,
    so it stays at the room's level, while a steady noise (motors while driving) has none and becomes the
    floor after about floor_window_s. A voice over that noise is still found if it is loud_ratio above it.
    """

    def __init__(self, vad=None, start_frames: int = 3, end_silence_s: float = 0.6, max_s: float = 10.0,
                 pre_roll_s: float = 0.3, min_speech_s: float = 0.25, loud_ratio: float = 2.5,
                 min_rms: float = 150.0, floor_window_s: float = 4.0):
        self.vad = vad
        self.start_frames = start_frames
        self.end_frames = max(1, int(end_silence_s * 1000 / FRAME_MS))
        self.max_bytes = int(max_s * RATE) * 2
        self.min_speech_frames = max(1, int(min_speech_s * 1000 / FRAME_MS))
        self.loud_ratio = loud_ratio
        self.min_rms = min_rms
        self.floor = min_rms / loud_ratio
        self._levels = deque(maxlen=max(10, int(floor_window_s * 1000 / FRAME_MS)))
        self._pre = deque(maxlen=max(start_frames, int(pre_roll_s * 1000 / FRAME_MS)))
        self.reset()

    def reset(self):
        self._pre.clear()
        self._buffer = bytearray()
        self.active = False
        self._run = self._quiet = self._voiced = 0

    def is_speech(self, frame: bytes) -> bool:
        level = rms(frame)
        self._levels.append(level)
        if len(self._levels) >= 10:
            self.floor = sorted(self._levels)[len(self._levels) // 10]
        loud = level > max(self.min_rms, self.floor * self.loud_ratio)
        return loud and (self.vad is None or self.vad(frame))

    def feed(self, frame: bytes) -> bytes | None:
        speech = self.is_speech(frame)
        if not self.active:
            self._pre.append(frame)
            self._run = self._run + 1 if speech else 0
            if self._run >= self.start_frames:
                self.active = True
                self._buffer.extend(b''.join(self._pre))
                self._voiced, self._quiet = self._run, 0
                self._pre.clear()
            return None
        self._buffer.extend(frame)
        self._quiet = 0 if speech else self._quiet + 1
        self._voiced += speech
        if self._quiet < self.end_frames and len(self._buffer) < self.max_bytes:
            return None
        voiced, segment = self._voiced, bytes(self._buffer[:self.max_bytes])
        self.reset()
        return segment if voiced >= self.min_speech_frames else None


# ---- microphone ----------------------------------------------------------------------------------------
class Microphone:
    """Raw 16 kHz mono 16-bit PCM from an external command's stdout, e.g.
    `pw-record --rate 16000 --channels 1 --format s16 -` or `arecord -q -t raw -f S16_LE -r 16000 -c 1 -`."""

    def __init__(self, command: list[str]):
        if not command:
            raise ValueError('microphone command is empty')
        self.command = list(command)
        self._process: subprocess.Popen | None = None
        self._lock = threading.Lock()

    def open(self):
        self.close()
        with self._lock:
            self._process = subprocess.Popen(self.command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                             stderr=subprocess.DEVNULL)

    def read(self) -> bytes | None:
        """One 30 ms frame (blocks); None when the command ended."""
        process = self._process
        if process is None or process.stdout is None:
            return None
        data = b''
        while len(data) < FRAME_BYTES:
            chunk = process.stdout.read(FRAME_BYTES - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def close(self):
        with self._lock:
            process, self._process = self._process, None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        if process.stdout is not None:
            process.stdout.close()


# ---- listener -----------------------------------------------------------------------------------------
class Listener:
    """Microphone -> on_wake(question). Two threads: one reads frames, one waits for speech-to-text."""

    def __init__(self, microphone, transcribe, on_wake, on_followup=None, names=('크래프트',), segmenter=None,
                 is_speaking=lambda: False, tail_s: float = 0.6, follow_up_s: float = 6.0,
                 max_stt_per_min: int = 20, tolerance: int = 2, logger=None, clock=time.monotonic,
                 retry_s: float = 5.0, ready=lambda: True, words=None, on_word=None, word_tolerance: int = 0):
        self.microphone = microphone
        # 반응 단어(이름 말고): {"젯슨": ("젯슨", "제슨", ...)} -> on_word("젯슨"). 짧은 말이라 오차 없이 맞을 때만
        # (오차 1이면 "레슨", "잭슨"도 걸렸다, 2026-11-13). 변형은 별칭으로 받는다.
        self.words = {key: tuple(aliases) for key, aliases in (words or {}).items() if aliases}
        self.on_word = on_word
        self.word_tolerance = word_tolerance
        self.ready = ready                      # False while speech-to-text cannot work: speech is dropped
        self.transcribe = transcribe            # wav bytes -> text
        self.on_wake = on_wake                  # question ('' = only the name)
        self.on_followup = on_followup          # sentence said right after an answer, without the name
        self.names = tuple(names)
        self.segmenter = segmenter or Segmenter(vad=webrtc_vad())
        self.is_speaking = is_speaking
        self.tail_s = tail_s
        self.follow_up_s = follow_up_s
        self.max_stt_per_min = max_stt_per_min
        self.tolerance = tolerance
        self.log = logger or logging.getLogger('craft_voice')
        self.clock = clock
        self.retry_s = retry_s
        self.stats = {'segments': 0, 'transcribed': 0, 'wakes': 0, 'followups': 0, 'ignored': 0,
                      'stt_errors': 0, 'over_budget': 0, 'mic_restarts': 0, 'words': 0}
        self.enabled = True
        self._segments: queue.Queue = queue.Queue(maxsize=2)
        self._stt_times: deque = deque()
        self._deaf_until = -math.inf      # robot speech still ringing in the microphone
        self._was_speaking = False
        self._conversation_until = -math.inf   # an answer is expected or was just given
        self._follow_until = -math.inf
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []

    def start(self):
        for target, name in ((self._capture_loop, 'voice-listen'), (self._stt_loop, 'voice-stt')):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def close(self):
        self._stop.set()
        self.microphone.close()
        try:
            self._segments.put_nowait(None)
        except queue.Full:
            pass
        for thread in self._threads:
            thread.join(timeout=3.0)

    # ---- capture thread ---------------------------------------------------------------------------------
    def _capture_loop(self):
        while not self._stop.is_set():
            try:
                self.microphone.open()
            except OSError as error:
                self.log.error(f'microphone command failed: {error}')
                self._stop.wait(self.retry_s)
                continue
            self.segmenter.reset()
            while not self._stop.is_set():
                frame = self.microphone.read()
                if frame is None:
                    break
                self.hear(frame)
            if not self._stop.is_set():
                self.stats['mic_restarts'] += 1
                self.log.warning(f'microphone stopped; trying again in {self.retry_s:.0f} s')
                self._stop.wait(self.retry_s)
        self.microphone.close()

    def hear(self, frame: bytes):
        """One microphone frame (called by the capture thread; tests call it directly)."""
        now = self.clock()
        speaking = bool(self.is_speaking())
        if speaking or self._was_speaking:
            self._deaf_until = now + self.tail_s
            if self._was_speaking and not speaking and now < self._conversation_until:
                # the robot just finished talking to someone: the next sentence may come without the name
                self._follow_until = now + self.tail_s + self.follow_up_s
            self._was_speaking = speaking
        if not self.enabled or now < self._deaf_until or not self.ready():
            if self.segmenter.active or self.segmenter._run:
                self.segmenter.reset()
            return
        segment = self.segmenter.feed(frame)
        if segment is None:
            return
        self.stats['segments'] += 1
        while self._stt_times and now - self._stt_times[0] >= 60.0:
            self._stt_times.popleft()
        if len(self._stt_times) >= self.max_stt_per_min:
            self.stats['over_budget'] += 1
            return
        self._stt_times.append(now)
        item = (segment, now)
        try:
            self._segments.put_nowait(item)
        except queue.Full:  # speech-to-text is behind: the newest sentence matters, drop the oldest
            try:
                self._segments.get_nowait()
            except queue.Empty:
                pass
            try:
                self._segments.put_nowait(item)
            except queue.Full:
                pass

    # ---- speech-to-text thread -----------------------------------------------------------------------
    def _stt_loop(self):
        while not self._stop.is_set():
            item = self._segments.get()
            if item is None:
                return
            self.process(*item)

    def process(self, segment: bytes, heard_at: float):
        """One utterance: transcribe it and act on it (tests call this directly)."""
        started = time.monotonic()
        try:
            text = (self.transcribe(pcm_to_wav(segment, RATE)) or '').strip()
        except Exception as error:
            self.stats['stt_errors'] += 1
            self.log.error(f'speech-to-text failed: {type(error).__name__}: {error}')
            return
        self.stats['transcribed'] += 1
        took = time.monotonic() - started
        if not text:
            return
        called, question = find_wake(text, self.names, self.tolerance)
        now = self.clock()
        seconds = len(segment) / (RATE * 2)
        if not called and self.on_word is not None:
            for key, aliases in self.words.items():  # "젯슨" -> its recording, not a conversation
                if find_wake(text, aliases, self.word_tolerance)[0]:
                    self.stats['words'] += 1
                    self.log.info(f'heard {key!r} ({seconds:.1f} s of speech, stt {time.monotonic() - started:.2f} s): '
                                  f'{text!r}')
                    self.on_word(key)
                    return
        if called:
            self.stats['wakes'] += 1
            self._conversation_until = now + 30.0
            self.log.info(f'heard my name ({seconds:.1f} s of speech, stt {took:.2f} s): {text!r}')
            self.on_wake(question)
        elif self.on_followup is not None and heard_at <= self._follow_until:
            self.stats['followups'] += 1
            self._conversation_until = now + 30.0
            self.log.info(f'follow-up ({seconds:.1f} s of speech, stt {took:.2f} s): {text!r}')
            self.on_followup(text)
        else:
            self.stats['ignored'] += 1
            self.log.debug(f'not for me: {text!r}')  # other people's talk is not logged at the normal level
