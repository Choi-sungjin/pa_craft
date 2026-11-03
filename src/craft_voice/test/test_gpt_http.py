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


# ---- model chain and reasoning step-down (faster models first) ----------------------------------------
class ModelServer:
    """/v1/responses answer per model: reply(body) -> (status, payload)."""

    def __init__(self, reply):
        self.bodies = []
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                outer.bodies.append(body)
                status, payload = reply(body)
                data = json.dumps(payload).encode()
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args):
                pass

        self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.httpd.server_address[1]}/v1'


def error(message, code=None):
    return {'error': {'message': message, 'type': 'invalid_request_error', 'param': None, 'code': code}}


def test_model_chain_skips_unknown_and_imageless_models_and_sticks(monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY', 'test-key-not-real')

    def reply(body):
        if body['model'] == 'gpt-6.1-luna':
            return 404, error('The model `gpt-6.1-luna` does not exist or you do not have access to it.',
                              'model_not_found')
        if body['model'] == 'text-only':
            return 400, error('Invalid content: image_url is only supported by certain models.')
        return 200, response_json(f'{body["model"]} 응답')

    server = ModelServer(reply)
    b = OpenAIBackend(model='gpt-6.1-luna, text-only, gpt-5.4-mini', tts_model='x', voice='coral',
                      timeout_s=5.0, base_url=server.url)
    assert b.generate('규칙', '질문', b'\xff\xd8jpeg') == 'gpt-5.4-mini 응답'
    assert [body['model'] for body in server.bodies] == ['gpt-6.1-luna', 'text-only', 'gpt-5.4-mini']
    assert b.model == 'gpt-5.4-mini'
    b.generate('규칙', '질문')
    assert [body['model'] for body in server.bodies][-1] == 'gpt-5.4-mini' and len(server.bodies) == 4
    server.httpd.shutdown()


def test_reasoning_effort_steps_down_then_unrelated_errors_raise(monkeypatch):
    monkeypatch.setenv('OPENAI_API_KEY', 'test-key-not-real')

    def reply(body):
        effort = body.get('reasoning', {}).get('effort')
        if effort in ('none', 'minimal'):
            return 400, error(f"Unsupported value: 'reasoning.effort' does not support '{effort}' with this model.")
        if body['max_output_tokens'] == 1:
            return 400, error("Invalid 'max_output_tokens': integer below minimum value.")
        return 200, response_json('네.')

    server = ModelServer(reply)
    b = OpenAIBackend(model='m1,m2', tts_model='x', voice='coral', reasoning_effort='none', timeout_s=5.0,
                      base_url=server.url)
    assert b.generate('규칙', '질문') == '네.'
    assert [body.get('reasoning', {}).get('effort') for body in server.bodies] == ['none', 'minimal', 'low']
    assert b.reasoning_effort == 'low' and b.model == 'm1'
    b.max_output_tokens = 1
    with pytest.raises(openai.BadRequestError):
        b.generate('규칙', '질문')                             # not a model problem: no switch to m2
    assert b.model == 'm1'
    server.httpd.shutdown()


def test_bench_compare_against_fake_server(monkeypatch):
    import os
    import subprocess
    import sys
    from pathlib import Path

    def reply(body):
        assert body['input'][0]['content'][1]['type'] == 'input_image'        # the picture is sent
        assert '[센서가 확인한 것]' in body['input'][0]['content'][0]['text']   # the real narration prompt
        if body['model'] == 'missing':
            return 404, error('The model `missing` does not exist.', 'model_not_found')
        return 200, response_json('접근 중이에요. 초록 신호가 보여요.')

    server = ModelServer(reply)
    env = dict(os.environ, OPENAI_API_KEY='test-key-not-real', CRAFT_VOICE_ENV='/nonexistent',
               PYTHONPATH=str(Path(__file__).resolve().parents[1]))
    result = subprocess.run([sys.executable, '-m', 'craft_voice.bench', '--compare', 'missing,fast-one',
                             '--efforts', 'none', '--runs', '2', '--base-url', server.url],
                            capture_output=True, text=True, timeout=60, env=env)
    server.httpd.shutdown()
    assert result.returncode == 0, result.stderr
    assert 'missing          effort none     FAILED  NotFoundError' in result.stdout
    assert 'fast-one' in result.stdout and 'n=2' in result.stdout and '접근 중이에요' in result.stdout


def test_bench_compare_cycles_through_a_picture_folder():
    import base64 as b64
    import os
    import subprocess
    import sys
    from pathlib import Path
    sizes = []

    def reply(body):
        url = body['input'][0]['content'][1]['image_url']
        sizes.append(len(b64.b64decode(url.split(',', 1)[1])))
        return 200, response_json('사무실 바닥이 보여요.')

    server = ModelServer(reply)
    package = Path(__file__).resolve().parents[1]
    env = dict(os.environ, OPENAI_API_KEY='test-key-not-real', CRAFT_VOICE_ENV='/nonexistent',
               PYTHONPATH=str(package))
    result = subprocess.run([sys.executable, '-m', 'craft_voice.bench', '--compare', 'one', '--efforts', 'none',
                             '--runs', '3', '--base-url', server.url, '--camera', str(package / 'test' / 'images')],
                            capture_output=True, text=True, timeout=60, env=env)
    server.httpd.shutdown()
    assert result.returncode == 0, result.stderr
    assert 'pictures: boxes.png' in result.stdout and 'office_floor.jpg' in result.stdout
    assert len(sizes) == 3 and len(set(sizes)) == 3 and all(s > 5000 for s in sizes)   # three different JPEGs


def test_proxy_auth_mode_sends_no_authorization_header(monkeypatch):
    """OPENAI_AUTH=proxy: the egress proxy injects the key; this process must not send one of its own."""
    from craft_voice.gpt import openai_auth_available
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    monkeypatch.setenv('OPENAI_AUTH', 'proxy')
    assert openai_auth_available()
    seen = []

    def reply(body):
        return 200, response_json('네.')

    server = ModelServer(reply)
    original = server.httpd.RequestHandlerClass.do_POST

    def record(handler):
        seen.append({k.lower(): v for k, v in handler.headers.items()})
        original(handler)

    server.httpd.RequestHandlerClass.do_POST = record
    b = OpenAIBackend(model='m', tts_model='x', voice='coral', timeout_s=5.0, base_url=server.url)
    assert b.generate('규칙', '질문') == '네.'
    b.synthesize('네')                                        # speech request too
    assert len(seen) == 2 and all('authorization' not in headers for headers in seen)
    server.httpd.shutdown()
    monkeypatch.delenv('OPENAI_AUTH')
    assert not openai_auth_available()
