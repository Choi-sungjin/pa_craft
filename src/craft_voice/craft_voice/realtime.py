"""Speech through the OpenAI Realtime API, e.g. the model gpt-realtime-2.1 with the voice marin.

The model is asked to read one sentence aloud exactly as written. The audio comes back as base64
PCM pieces (16-bit mono, 24 kHz) over a WebSocket, so it can be played while it arrives.

The WebSocket client below is only what this needs from RFC 6455 (masked text frames, fragments,
ping/pong, close) on the standard library, so no extra package has to be installed on the robot.
"""

from __future__ import annotations

import base64
import difflib
import hashlib
import json
import os
import re
import socket
import ssl
import struct
import threading
import time
import urllib.parse

from .audio import pcm_to_wav

GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
REALTIME_PREFIXES = ('gpt-realtime', 'gpt-live')  # tts_model names served by the Realtime API


class RealtimeError(RuntimeError):
    def __init__(self, message: str, status_code: int | None = None):
        super().__init__(message)
        self.status_code = status_code


def is_realtime_model(name: str) -> bool:
    return name.startswith(REALTIME_PREFIXES)


class WebSocket:
    """A client WebSocket: send(text), recv(timeout) -> text | None once the server closed."""

    def __init__(self, url: str, headers: dict | None = None, timeout: float = 10.0):
        parts = urllib.parse.urlsplit(url)
        secure = parts.scheme == 'wss'
        host = parts.hostname or ''
        port = parts.port or (443 if secure else 80)
        raw = socket.create_connection((host, port), timeout=timeout)
        try:
            raw.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = ssl.create_default_context().wrap_socket(raw, server_hostname=host) if secure else raw
            key = base64.b64encode(os.urandom(16)).decode('ascii')
            target = (parts.path or '/') + ('?' + parts.query if parts.query else '')
            lines = [f'GET {target} HTTP/1.1', f'Host: {host}' + ('' if port in (80, 443) else f':{port}'),
                     'Upgrade: websocket', 'Connection: Upgrade', f'Sec-WebSocket-Key: {key}',
                     'Sec-WebSocket-Version: 13']
            lines += [f'{name}: {value}' for name, value in (headers or {}).items()]
            self._sock.sendall(('\r\n'.join(lines) + '\r\n\r\n').encode('utf-8'))
            self._buffer = b''
            head = self._read_until(b'\r\n\r\n', 16384).decode('latin-1')
            status = head.split('\r\n', 1)[0]
            if ' 101' not in status:
                code = re.search(r' (\d{3})', status)
                # e.g. 401 with a JSON body: keep the status, never echo request headers
                raise RealtimeError(f'WebSocket refused: {status.strip()}', int(code.group(1)) if code else None)
            accept = base64.b64encode(hashlib.sha1((key + GUID).encode('ascii')).digest()).decode('ascii')
            if accept.lower() not in head.lower():
                raise RealtimeError('WebSocket handshake: wrong Sec-WebSocket-Accept')
        except BaseException:
            raw.close()
            raise
        self._closed = False

    # ---- bytes --------------------------------------------------------------------------------
    def _fill(self):
        chunk = self._sock.recv(65536)
        if not chunk:
            raise RealtimeError('connection closed')
        self._buffer += chunk

    def _read_until(self, marker: bytes, limit: int) -> bytes:
        while marker not in self._buffer:
            if len(self._buffer) > limit:
                raise RealtimeError('WebSocket handshake: header too long')
            self._fill()
        head, _, self._buffer = self._buffer.partition(marker)
        return head

    def _read(self, count: int) -> bytes:
        while len(self._buffer) < count:
            self._fill()
        data, self._buffer = self._buffer[:count], self._buffer[count:]
        return data

    def _frame(self, opcode: int, payload: bytes):
        mask = os.urandom(4)
        length = len(payload)
        if length < 126:
            head = struct.pack('!BB', 0x80 | opcode, 0x80 | length)
        elif length < 65536:
            head = struct.pack('!BBH', 0x80 | opcode, 0x80 | 126, length)
        else:
            head = struct.pack('!BBQ', 0x80 | opcode, 0x80 | 127, length)
        masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload)) if length < 4096 else \
            (int.from_bytes(payload, 'big') ^ int.from_bytes((mask * (length // 4 + 1))[:length], 'big')
             ).to_bytes(length, 'big')
        self._sock.sendall(head + mask + masked)

    # ---- messages -----------------------------------------------------------------------------
    def send(self, text: str):
        self._frame(0x1, text.encode('utf-8'))

    def recv(self, timeout: float | None = None) -> str | None:
        """Next text message; None when the server closed. socket.timeout (TimeoutError) if none in time."""
        deadline = None if timeout is None else time.monotonic() + timeout
        message = b''
        while True:
            if deadline is not None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('no WebSocket message in time')
                self._sock.settimeout(remaining)
            first, second = self._read(2)
            opcode, length = first & 0x0F, second & 0x7F
            if length == 126:
                length = struct.unpack('!H', self._read(2))[0]
            elif length == 127:
                length = struct.unpack('!Q', self._read(8))[0]
            if length > 16_000_000:
                raise RealtimeError('WebSocket frame too large')
            mask = self._read(4) if second & 0x80 else b''
            payload = self._read(length)
            if mask:
                payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
            if opcode == 0x9:        # ping
                self._frame(0xA, payload)
            elif opcode == 0x8:      # close
                self._closed = True
                return None
            elif opcode in (0x0, 0x1, 0x2):
                message += payload
                if first & 0x80:     # FIN
                    return message.decode('utf-8', errors='replace')

    def close(self):
        try:
            if not self._closed:
                self._sock.settimeout(1.0)
                self._frame(0x8, b'')
        except OSError:
            pass
        finally:
            self._closed = True
            try:
                self._sock.close()
            except OSError:
                pass


def _normalized(text: str) -> str:
    return re.sub(r'[^\w가-힣]', '', text, flags=re.UNICODE).lower()


def same_sentence(spoken: str, wanted: str) -> bool:
    """The model read the sentence (a small difference in spelling or a number is fine), not something else."""
    a, b = _normalized(spoken), _normalized(wanted)
    if not a or not b:
        return not b
    return a == b or difflib.SequenceMatcher(None, a, b).ratio() >= 0.8


class RealtimeSpeech:
    """sentence -> PCM pieces as the Realtime model speaks them. One short session per sentence.

    The sentence is given as the model's own line (in the instructions, with no user message): asked
    that way it says the line as written even when the line is a question, instead of answering it.
    warm() opens the connection ahead of time, which brings the first sound about 0.6 s sooner.
    """

    sample_rate = 24000
    warm_ttl_s = 40.0   # an idle connection older than this is not trusted

    def __init__(self, model: str = 'gpt-realtime-2.1', voice: str = 'marin', instructions: str = '',
                 api_key: str | None = None, base_url: str = '', timeout_s: float = 15.0,
                 urgent_timeout_s: float = 4.0, connect=None):
        self.model = model
        self.voice = voice
        self.instructions = instructions
        self.api_key = os.environ.get('OPENAI_API_KEY', '') if api_key is None else api_key
        base = (base_url or os.environ.get('OPENAI_BASE_URL') or 'https://api.openai.com/v1').rstrip('/')
        self.url = re.sub(r'^http', 'ws', base) + '/realtime?model=' + urllib.parse.quote(model, safe='')
        self.timeout_s = timeout_s
        self.urgent_timeout_s = urgent_timeout_s
        self.connect = connect or WebSocket
        self.last_transcript = ''   # what the model says it spoke (for logs and checks)
        self._lock = threading.Lock()
        self._ready = None          # (connection, opened at)
        self._warming = False

    # ---- connection -----------------------------------------------------------------------------
    def _open(self, timeout: float):
        headers = {'Authorization': 'Bearer ' + self.api_key} if self.api_key else {}
        try:
            return self.connect(self.url, headers, timeout)
        except RealtimeError:
            raise
        except (OSError, ssl.SSLError) as error:
            raise RealtimeError(f'connection failed: {type(error).__name__}: {error}') from None

    def warm(self):
        """Open a connection in the background, ready for the next sentence. Never blocks or raises."""
        with self._lock:
            fresh = self._ready is not None and time.monotonic() - self._ready[1] < self.warm_ttl_s / 2
            if self._warming or fresh:
                return
            self._warming = True

        def work():
            connection = None
            try:
                connection = self._open(self.timeout_s)
            except Exception:  # the sentence itself will report a dead network
                pass
            with self._lock:
                self._warming = False
                old, self._ready = self._ready, ((connection, time.monotonic()) if connection else self._ready)
            if connection is not None and old is not None:
                old[0].close()

        threading.Thread(target=work, name='voice-rt-warm', daemon=True).start()

    def _take(self, timeout: float):
        """(connection, was it opened earlier)."""
        with self._lock:
            ready, self._ready = self._ready, None
        if ready is not None:
            if time.monotonic() - ready[1] < self.warm_ttl_s:
                return ready[0], True
            ready[0].close()
        return self._open(timeout), False

    def close(self):
        with self._lock:
            ready, self._ready = self._ready, None
        if ready is not None:
            ready[0].close()

    # ---- speech ---------------------------------------------------------------------------------
    def _line_instructions(self, text: str) -> str:
        return ('너는 로봇의 목소리야. 지금 네가 할 말은 정해져 있어. 아래 대사를 한 글자도 바꾸지 말고 그대로 한 번만 말해. '
                f'대사 앞뒤에 다른 말을 붙이지 마. {self.instructions}'.strip() + f'\n대사: {text}')

    def stream(self, text: str, timeout_s: float | None = None):
        """Yield raw PCM (16-bit mono, 24 kHz) while the model speaks `text`."""
        text = re.sub(r'\s+', ' ', text).strip()
        if not text:
            raise RealtimeError('nothing to say')
        limit = self.timeout_s if timeout_s is None else timeout_s
        self.last_transcript = ''
        for attempt in (0, 1):
            ws, reused = self._take(limit)
            got_audio = False
            try:
                for piece in self._speak(ws, text, limit):
                    got_audio = True
                    yield piece
                return
            except RealtimeError:
                # A connection opened earlier may have been dropped by the server meanwhile: once more, fresh.
                if not reused or got_audio or attempt:
                    raise
            finally:
                ws.close()

    def _speak(self, ws, text: str, limit: float):
        instructions = self._line_instructions(text)
        deadline = time.monotonic() + max(limit, 1.0) + 30.0  # the whole sentence; each wait is `limit`
        try:
            # Both at once: waiting for 'session.updated' before asking costs another round trip (~0.4 s).
            ws.send(json.dumps({'type': 'session.update', 'session': {
                'type': 'realtime', 'model': self.model, 'output_modalities': ['audio'],
                'audio': {'input': {'turn_detection': None},
                          'output': {'voice': self.voice, 'format': {'type': 'audio/pcm', 'rate': self.sample_rate}}},
                'instructions': instructions}}, ensure_ascii=False))
            ws.send(json.dumps({'type': 'response.create', 'response': {
                'conversation': 'none', 'output_modalities': ['audio'], 'max_output_tokens': 1024,
                'instructions': instructions, 'input': []}}, ensure_ascii=False))
        except (OSError, ssl.SSLError) as error:
            raise RealtimeError(f'connection lost: {type(error).__name__}: {error}') from None
        got_audio = False
        while True:
            if time.monotonic() > deadline:
                raise RealtimeError('speech took too long')
            try:
                raw = ws.recv(limit)
            except (TimeoutError, socket.timeout):
                raise RealtimeError(f'no answer from the speech model in {limit:.0f} s') from None
            except (OSError, ssl.SSLError) as error:
                raise RealtimeError(f'connection lost: {type(error).__name__}: {error}') from None
            if raw is None:
                raise RealtimeError('the speech model closed the connection')
            event = json.loads(raw)
            kind = event.get('type', '')
            if kind == 'error':
                detail = event.get('error') or {}
                raise RealtimeError(f"speech model error: {detail.get('code') or ''} "
                                    f"{detail.get('message') or ''}".strip())
            if kind in ('response.output_audio.delta', 'response.audio.delta'):
                piece = base64.b64decode(event.get('delta') or '')
                if piece:
                    got_audio = True
                    yield piece
            elif kind in ('response.output_audio_transcript.done', 'response.audio_transcript.done'):
                self.last_transcript = event.get('transcript') or ''
            elif kind == 'response.done':
                status = (event.get('response') or {}).get('status')
                if status != 'completed' or not got_audio:
                    raise RealtimeError(f'speech not completed (status {status})')
                return

    def synthesize(self, text: str, urgent: bool = False) -> bytes:
        """Whole sentence -> WAV bytes. One more try if the model said something else than the sentence."""
        limit = self.urgent_timeout_s if urgent else self.timeout_s
        for attempt in (0, 1):
            pcm = b''.join(self.stream(text, limit))
            if same_sentence(self.last_transcript, text) or not self.last_transcript:
                return pcm_to_wav(pcm, self.sample_rate)
            if urgent or attempt:
                break
        raise RealtimeError(f'the speech model did not read the sentence (said: {self.last_transcript[:60]!r})')
