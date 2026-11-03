"""The real openai SDK against a local fake server: checks what actually goes over HTTP."""

import base64
import io
import json
import threading
import wave
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

openai = pytest.importorskip('openai')

from craft_voice.gpt import EmptyResponse, OpenAIBackend, pcm_to_wav  # noqa: E402

PCM = (b'\x01\x00\xff\xff' * 1200)  # 2400 samples, 16-bit mono


def response_json(text):
    return {
        'id': 'resp_test', 'object': 'response', 'created_at': 0, 'status': 'completed', 'model': 'gpt-5.5',
        'output': [{'type': 'message', 'id': 'msg_test', 'status': 'completed', 'role': 'assistant',
                    'content': [{'type': 'output_text', 'text': text, 'annotations': []}]}],
        'parallel_tool_calls': False, 'tool_choice': 'auto', 'tools': [],
    }


class Server:
    def __init__(self):
        self.requests = []
        self.replies = []  # (status, content type, body) per request, last one repeats
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = self.rfile.read(int(self.headers['Content-Length']))
                outer.requests.append((self.path, {k.lower(): v for k, v in self.headers.items()}, json.loads(body)))
                status, kind, data = outer.replies.pop(0) if len(outer.replies) > 1 else outer.replies[0]
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

    def reply_json(self, payload, status=200):
        self.replies.append((status, 'application/json', json.dumps(payload).encode()))


@pytest.fixture
def server(monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY', 'test-key-not-real')
    s = Server()
    yield s
    s.httpd.shutdown()


def backend(server, **kwargs):
    defaults = dict(model='gpt-5.5', tts_model='gpt-4o-mini-tts', voice='coral', tts_instructions='밝게',
                    reasoning_effort='low', timeout_s=5.0, base_url=server.url)
    defaults.update(kwargs)
    return OpenAIBackend(**defaults)


def test_responses_request_carries_image_and_state(server):
    server.reply_json(response_json('앞에 상자가 보여요.'))
    jpeg = b'\xff\xd8\xff\xe0fake-jpeg'
    text = backend(server).generate('규칙', '[로봇 상태]\n- 상태머신: Ready', jpeg)
    assert text == '앞에 상자가 보여요.'
    path, headers, body = server.requests[0]
    assert path == '/v1/responses'
    assert headers['authorization'] == 'Bearer test-key-not-real'
    assert body['model'] == 'gpt-5.5' and body['instructions'] == '규칙' and body['store'] is False
    assert body['reasoning'] == {'effort': 'low'} and body['max_output_tokens'] == 400
    content = body['input'][0]['content']
    assert body['input'][0]['role'] == 'user'
    assert content[0] == {'type': 'input_text', 'text': '[로봇 상태]\n- 상태머신: Ready'}
    assert content[1]['type'] == 'input_image' and content[1]['detail'] == 'low'
    prefix = 'data:image/jpeg;base64,'
    assert content[1]['image_url'].startswith(prefix)
    assert base64.b64decode(content[1]['image_url'][len(prefix):]) == jpeg


def test_reasoning_rejected_is_dropped_and_retried(server):
    server.reply_json({'error': {'message': "Unsupported parameter: 'reasoning.effort'", 'type':
                                 'invalid_request_error', 'param': 'reasoning.effort', 'code': None}}, 400)
    server.reply_json(response_json('네.'))
    b = backend(server)
    assert b.generate('규칙', '질문') == '네.'
    assert 'reasoning' in server.requests[0][2] and 'reasoning' not in server.requests[1][2]
    assert b.reasoning_effort == ''


def test_other_errors_and_empty_answers_raise(server):
    server.reply_json({'error': {'message': 'Invalid API key', 'type': 'invalid_request_error'}}, 401)
    with pytest.raises(openai.AuthenticationError):
        backend(server).generate('규칙', '질문')
    server.replies.clear()
    server.reply_json(dict(response_json(''), status='incomplete', output=[],
                           incomplete_details={'reason': 'max_output_tokens'}))
    with pytest.raises(EmptyResponse):
        backend(server).generate('규칙', '질문')


def test_speech_request_and_wav(server):
    server.replies.append((200, 'audio/pcm', PCM))
    wav = backend(server).synthesize('준비 완료.')
    path, _, body = server.requests[0]
    assert path == '/v1/audio/speech'
    assert body == {'model': 'gpt-4o-mini-tts', 'voice': 'coral', 'input': '준비 완료.',
                    'response_format': 'pcm', 'instructions': '밝게'}
    with wave.open(io.BytesIO(wav)) as w:
        assert (w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()) == (1, 2, 24000, 2400)
        assert w.readframes(2400) == PCM


def test_tts1_gets_no_instructions_and_speed_is_sent(server):
    server.replies.append((200, 'audio/pcm', PCM))
    backend(server, tts_model='tts-1', tts_speed=1.2).synthesize('네')
    body = server.requests[0][2]
    assert 'instructions' not in body and body['speed'] == 1.2


def test_pcm_to_wav_drops_half_sample():
    with wave.open(io.BytesIO(pcm_to_wav(b'\x01\x02\x03', 16000))) as w:
        assert w.getnframes() == 1 and w.getframerate() == 16000


def test_urgent_speech_gives_up_fast(monkeypatch):
    """Fault speech on a dead network: one attempt with urgent_timeout_s, no retry."""
    import socket
    monkeypatch.setenv('OPENAI_API_KEY', 'test-key-not-real')
    silent = socket.socket()
    silent.bind(('127.0.0.1', 0))
    silent.listen(8)                                          # accepts, never answers
    url = f'http://127.0.0.1:{silent.getsockname()[1]}/v1'
    b = OpenAIBackend(model='m', tts_model='gpt-4o-mini-tts', voice='coral', timeout_s=30.0,
                      urgent_timeout_s=0.5, base_url=url)
    import time
    started = time.monotonic()
    with pytest.raises(openai.APITimeoutError):
        b.synthesize('문제가 생겨서 멈췄어요.', urgent=True)
    assert time.monotonic() - started < 2.0
    silent.close()
