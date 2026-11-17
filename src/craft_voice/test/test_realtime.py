"""Realtime speech (WebSocket) and the standard-library OpenAI client, against local fake servers."""
import base64
import hashlib
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from craft_voice.audio import wav_seconds
from craft_voice.gpt import OpenAIBackend, model_unusable
from craft_voice.plainclient import ApiError, PlainClient
from craft_voice.realtime import GUID, RealtimeError, RealtimeSpeech, WebSocket, is_realtime_model, same_sentence


# ---- a tiny WebSocket server -------------------------------------------------------------------------------
class WsServer:
    """Accepts connections; `script(send, receive)` plays the server side of each one."""

    def __init__(self, script, status='101 Switching Protocols'):
        self.script, self.status = script, status
        self.requests = []          # handshake headers of each connection
        self.sock = socket.socket()
        self.sock.bind(('127.0.0.1', 0))
        self.sock.listen(5)
        self.url = f'ws://127.0.0.1:{self.sock.getsockname()[1]}'
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        while True:
            try:
                connection, _ = self.sock.accept()
            except OSError:
                return
            threading.Thread(target=self._handle, args=(connection,), daemon=True).start()

    def _handle(self, connection):
        try:
            data = b''
            while b'\r\n\r\n' not in data:
                data += connection.recv(4096)
            head = data.split(b'\r\n\r\n')[0].decode()
            self.requests.append(head)
            key = next(line.split(': ', 1)[1] for line in head.split('\r\n') if line.lower().startswith('sec-websocket-key'))
            accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
            connection.sendall(f'HTTP/1.1 {self.status}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                               f'Sec-WebSocket-Accept: {accept}\r\n\r\n'.encode())
            if not self.status.startswith('101'):
                return

            def send(payload, opcode=0x1, fin=True):
                payload = payload.encode() if isinstance(payload, str) else payload
                first = (0x80 if fin else 0) | opcode
                if len(payload) < 126:
                    header = struct.pack('!BB', first, len(payload))
                elif len(payload) < 65536:
                    header = struct.pack('!BBH', first, 126, len(payload))
                else:
                    header = struct.pack('!BBQ', first, 127, len(payload))
                connection.sendall(header + payload)

            def read(count):
                buffer = b''
                while len(buffer) < count:
                    chunk = connection.recv(count - len(buffer))
                    if not chunk:
                        raise ConnectionError('client closed')
                    buffer += chunk
                return buffer

            def receive():
                first, second = read(2)
                length = second & 0x7F
                if length == 126:
                    length = struct.unpack('!H', read(2))[0]
                elif length == 127:
                    length = struct.unpack('!Q', read(8))[0]
                assert second & 0x80, 'client frames must be masked'
                mask = read(4)
                payload = bytes(b ^ mask[i % 4] for i, b in enumerate(read(length)))
                return first & 0x0F, payload

            self.script(send, receive)
        except (ConnectionError, OSError, StopIteration):
            pass
        finally:
            connection.close()

    def close(self):
        self.sock.close()


def test_websocket_text_fragments_ping_and_close():
    seen = []

    def script(send, receive):
        seen.append(receive())                       # the client's first message
        send('{"a":', fin=False)                     # one message in two fragments
        send(b'x', opcode=0x9)                       # a ping in between
        send('1}', opcode=0x0)
        seen.append(receive())                       # the pong
        send('z' * 70000)                            # a long frame (64-bit length)
        seen.append(receive())                       # a long masked frame from the client
        send(b'', opcode=0x8)

    server = WsServer(script)
    ws = WebSocket(server.url + '/realtime?model=m', {'Authorization': 'Bearer k'}, timeout=3.0)
    try:
        ws.send('안녕')
        assert ws.recv(3.0) == '{"a":1}'
        assert ws.recv(3.0) == 'z' * 70000
        ws.send('y' * 5000)
        assert ws.recv(3.0) is None                  # the server closed
    finally:
        ws.close()
        server.close()
    assert seen[0] == (0x1, '안녕'.encode()) and seen[1] == (0xA, b'x') and seen[2] == (0x1, b'y' * 5000)
    assert 'GET /realtime?model=m HTTP/1.1' in server.requests[0] and 'Authorization: Bearer k' in server.requests[0]


def test_websocket_refused_keeps_the_status_and_times_out_when_silent():
    server = WsServer(lambda send, receive: None, status='401 Unauthorized')
    with pytest.raises(RealtimeError) as error:
        WebSocket(server.url, {'Authorization': 'Bearer secret-key'}, timeout=3.0)
    assert error.value.status_code == 401 and 'secret-key' not in str(error.value)
    server.close()
    quiet = WsServer(lambda send, receive: time.sleep(1.0))
    ws = WebSocket(quiet.url, timeout=3.0)
    with pytest.raises(TimeoutError):
        ws.recv(0.2)
    ws.close()
    quiet.close()


# ---- Realtime speech -----------------------------------------------------------------------------------------
def events(text='지금 가는 중이야!', pieces=(b'\x01\x00' * 1200, b'\x02\x00' * 1200), status='completed'):
    out = [{'type': 'session.created'}, {'type': 'session.updated'}]
    out += [{'type': 'response.output_audio.delta', 'delta': base64.b64encode(p).decode()} for p in pieces]
    out += [{'type': 'response.output_audio_transcript.done', 'transcript': text},
            {'type': 'response.done', 'response': {'status': status}}]
    return out


class FakeConnection:
    def __init__(self, replies):
        self.replies = list(replies)
        self.sent, self.closed = [], False

    def send(self, text):
        self.sent.append(json.loads(text))

    def recv(self, timeout=None):
        if not self.replies:
            raise TimeoutError('nothing more')
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return None if reply is None else json.dumps(reply)

    def close(self):
        self.closed = True


def speech(connections, **kwargs):
    made = []

    def connect(url, headers, timeout):
        connection = connections.pop(0)
        if isinstance(connection, Exception):
            raise connection
        made.append((url, headers, connection))
        return connection

    return RealtimeSpeech(api_key='k', connect=connect, **kwargs), made


def test_the_line_is_spoken_as_given_and_streamed():
    connection = FakeConnection(events())
    tts, made = speech([connection], model='gpt-realtime-2.1', voice='marin', instructions='밝게.')
    pieces = list(tts.stream('지금 가는 중이야!'))
    assert pieces == [b'\x01\x00' * 1200, b'\x02\x00' * 1200] and connection.closed
    url, headers, _ = made[0]
    assert url == 'wss://api.openai.com/v1/realtime?model=gpt-realtime-2.1' and headers == {'Authorization': 'Bearer k'}
    update, create = connection.sent                                # both sent before the first answer was read
    assert update['type'] == 'session.update' and create['type'] == 'response.create'
    assert update['session']['audio']['output'] == {'voice': 'marin', 'format': {'type': 'audio/pcm', 'rate': 24000}}
    assert update['session']['audio']['input'] == {'turn_detection': None}
    assert create['response']['input'] == [] and create['response']['conversation'] == 'none'
    assert create['response']['instructions'].endswith('\n대사: 지금 가는 중이야!') and '밝게.' in create['response']['instructions']
    assert tts.last_transcript == '지금 가는 중이야!'
    wav = speech([FakeConnection(events())])[0].synthesize('지금 가는 중이야!')
    assert wav[:4] == b'RIFF' and abs(wav_seconds(wav) - 0.1) < 1e-6


def test_speech_errors_are_reported_not_played():
    for replies, message in (
            ([{'type': 'error', 'error': {'code': 'model_not_found', 'message': 'no such model'}}], 'no such model'),
            (events(status='failed'), 'not completed'),
            (events(pieces=()), 'not completed'),
            ([{'type': 'session.updated'}, None], 'closed the connection'),
            ([{'type': 'session.updated'}], 'no answer from the speech model'),
    ):
        tts, _ = speech([FakeConnection(replies)], timeout_s=0.5)
        with pytest.raises(RealtimeError, match=message):
            list(tts.stream('문장'))
    tts, _ = speech([OSError('network unreachable')])
    with pytest.raises(RealtimeError, match='connection failed'):
        list(tts.stream('문장'))
    with pytest.raises(RealtimeError, match='nothing to say'):
        list(speech([])[0].stream('  '))


def test_a_model_that_answers_instead_of_reading_is_retried_once():
    tts, made = speech([FakeConnection(events(text='저는 잘 지내요, 고마워요!')), FakeConnection(events(text='뭐 해?'))])
    assert tts.synthesize('뭐 해?')[:4] == b'RIFF' and len(made) == 2
    tts, made = speech([FakeConnection(events(text='저는 잘 지내요!')), FakeConnection(events(text='저는 잘 지내요!'))])
    with pytest.raises(RealtimeError, match='did not read'):
        tts.synthesize('뭐 해?')
    tts, made = speech([FakeConnection(events(text='아무 말'))])          # a fault sentence: no second try
    with pytest.raises(RealtimeError):
        tts.synthesize('지금은 멈춰 있어요.', urgent=True)
    assert len(made) == 1
    assert same_sentence('퍽 두 개 보여!', '퍽 2개 보여!') and not same_sentence('', '문장') and same_sentence('', '')


def test_a_warmed_connection_is_used_and_a_dead_one_replaced():
    warm = FakeConnection(events())
    tts, made = speech([warm])
    tts.warm()
    deadline = time.monotonic() + 2.0
    while tts._ready is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert tts._ready is not None and warm.sent == []                   # opened, nothing asked yet
    assert len(list(tts.stream('문장'))) == 2 and len(made) == 1 and warm.sent
    dead, fresh = FakeConnection([None]), FakeConnection(events())      # the server dropped the idle connection
    tts, made = speech([dead, fresh])
    tts.warm()
    while tts._ready is None and time.monotonic() < deadline + 2.0:
        time.sleep(0.01)
    assert len(list(tts.stream('문장'))) == 2 and len(made) == 2 and dead.closed
    old = FakeConnection(events())
    tts, made = speech([FakeConnection(events())])
    tts._ready = (old, time.monotonic() - 1000.0)                       # too old to trust
    assert len(list(tts.stream('문장'))) == 2 and old.closed and old.sent == [] and len(made) == 1
    tts.close()


def test_backend_speaks_through_realtime_for_realtime_models():
    assert is_realtime_model('gpt-realtime-2.1') and is_realtime_model('gpt-live-1') and not is_realtime_model('gpt-4o-mini-tts')
    tts, _ = speech([FakeConnection(events()), FakeConnection(events())])
    backend = OpenAIBackend(model='m', tts_model='gpt-realtime-2.1', voice='marin', client=object(), realtime=tts)
    assert list(backend.stream_speech('지금 가는 중이야!')) == [b'\x01\x00' * 1200, b'\x02\x00' * 1200]
    assert backend.synthesize('지금 가는 중이야!')[:4] == b'RIFF' and backend.sample_rate == 24000
    assert backend.speech_signature() == 'gpt-realtime-2.1|marin||1'
    plain = OpenAIBackend(model='m', tts_model='gpt-4o-mini-tts', voice='coral', client=object())
    assert plain.realtime is None
    made = OpenAIBackend(model='m', tts_model='gpt-realtime-2.1', voice='marin', tts_instructions='밝게', client=object())
    assert made.realtime is not None and made.realtime.voice == 'marin' and made.realtime.instructions == '밝게'


# ---- the standard-library client -------------------------------------------------------------------------------
class FakeOpenAI:
    def __init__(self):
        self.requests = []
        self.fail = {}          # path -> list of (status, body) served before the normal answer
        outer = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = 'HTTP/1.1'

            def do_POST(self):
                body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
                outer.requests.append((self.path, dict(self.headers), body))
                queue = outer.fail.get(self.path) or []
                if queue:
                    status, data = queue.pop(0)
                    kind = 'application/json'
                elif self.path.endswith('/responses'):
                    request = json.loads(body)
                    if request['model'] == 'gone':
                        status, data = 404, json.dumps({'error': {'message': 'The model `gone` does not exist'}}).encode()
                    else:
                        status, data = 200, json.dumps({'status': 'completed', 'output': [
                            {'type': 'reasoning'},
                            {'type': 'message', 'content': [{'type': 'output_text', 'text': '가는 중이에요.'}]}]}).encode()
                    kind = 'application/json'
                elif self.path.endswith('/audio/speech'):
                    status, data, kind = 200, b'\x03\x00' * 6000, 'audio/pcm'
                else:
                    form = body.decode('latin-1')
                    model = form.split('name="model"\r\n\r\n')[1].split('\r\n')[0]
                    if model == 'old-stt':
                        status, data = 404, json.dumps({'error': {'message': 'model old-stt not found'}}).encode()
                    else:
                        status, data = 200, json.dumps({'text': ' 크래프트, 지금 뭐 해? '}).encode()
                    kind = 'application/json'
                self.send_response(status)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args):
                pass

        self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.httpd.server_address[1]}/v1'


@pytest.fixture
def server():
    fake = FakeOpenAI()
    yield fake
    fake.httpd.shutdown()


def test_plain_client_calls(server):
    client = PlainClient(api_key='test-key', base_url=server.url, timeout=5.0)
    answer = client.responses.create(model='m', input=[], instructions='x')
    assert answer.output_text == '가는 중이에요.' and answer.status == 'completed'
    assert client.audio.speech.create(model='t', voice='v', input='문장').content == b'\x03\x00' * 6000
    with client.audio.speech.with_streaming_response.create(model='t', voice='v', input='문장') as response:
        assert b''.join(response.iter_bytes(1024)) == b'\x03\x00' * 6000
    heard = client.audio.transcriptions.create(file=('speech.wav', b'RIFFdata', 'audio/wav'), model='stt',
                                               language='ko', prompt='', response_format='json')
    assert heard.text == ' 크래프트, 지금 뭐 해? '
    path, headers, body = server.requests[-1]
    assert path == '/v1/audio/transcriptions' and headers['Authorization'] == 'Bearer test-key'
    assert b'name="language"\r\n\r\nko' in body and b'filename="speech.wav"' in body and b'RIFFdata' in body
    assert b'name="prompt"' not in body                                  # empty fields are left out
    assert json.loads(server.requests[0][2]) == {'model': 'm', 'input': [], 'instructions': 'x'}


def test_plain_client_errors_and_retries(server):
    client = PlainClient(api_key='test-key', base_url=server.url, timeout=5.0, max_retries=1)
    with pytest.raises(ApiError) as error:
        client.responses.create(model='gone', input=[])
    assert error.value.status_code == 404 and model_unusable(error.value) and 'test-key' not in str(error.value)
    server.fail['/v1/responses'] = [(500, b'{"error": {"message": "overloaded"}}')]
    assert client.responses.create(model='m', input=[]).output_text == '가는 중이에요.'      # one retry
    server.fail['/v1/responses'] = [(500, b'{}'), (500, b'{}')]
    with pytest.raises(ApiError) as error:
        client.responses.create(model='m', input=[])
    assert error.value.status_code == 500
    assert client.with_options(timeout=1.0, max_retries=0).max_retries == 0 and client.max_retries == 1
    refused = PlainClient(api_key='k', base_url='http://127.0.0.1:9/v1', timeout=1.0, max_retries=0)
    with pytest.raises(ApiError) as error:
        refused.responses.create(model='m', input=[])
    assert error.value.status_code is None and 'connection failed' in str(error.value)
    no_key = PlainClient(api_key='', base_url=server.url)
    no_key.responses.create(model='m', input=[])
    assert 'Authorization' not in server.requests[-1][1]                  # a proxy adds its own


def test_backend_on_the_plain_client_generates_and_transcribes_with_model_fallback(server):
    backend = OpenAIBackend(model='gone,m', tts_model='gpt-4o-mini-tts', voice='coral', stt_model='old-stt,stt',
                            client=PlainClient(api_key='k', base_url=server.url, timeout=5.0))
    assert backend.generate('지시', '사실') == '가는 중이에요.' and backend.model == 'm'
    assert backend.transcribe(b'RIFFdata') == '크래프트, 지금 뭐 해?' and backend.stt_model == 'stt'
    assert backend.synthesize('문장')[:4] == b'RIFF'
    assert b''.join(backend.stream_speech('문장')) == b'\x03\x00' * 6000
