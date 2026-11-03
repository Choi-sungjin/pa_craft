"""End to end: `python -m craft_voice.demo` as a real process against a fake OpenAI server."""

import json
import os
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip('openai')
cv2 = pytest.importorskip('cv2')

PACKAGE = Path(__file__).resolve().parents[1]


class FakeOpenAI:
    def __init__(self):
        self.requests = []
        self.count = 0
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                outer.requests.append((self.path, body))
                if self.path.endswith('/responses'):
                    asked = body['input'][0]['content'][0]['text']
                    outer.count += 1
                    if '물었어' in asked:
                        text = '앞에 파란 상자가 보여요.'
                    elif '바뀌었어' in asked:
                        text = f'상태 알림 {outer.count}번이에요.'  # distinct, or repeat suppression drops it
                    else:
                        text = '주변이 밝은 방이에요.'
                    data = json.dumps({
                        'id': 'r', 'object': 'response', 'created_at': 0, 'status': 'completed', 'model': 'm',
                        'output': [{'type': 'message', 'id': 'm', 'status': 'completed', 'role': 'assistant',
                                    'content': [{'type': 'output_text', 'text': text, 'annotations': []}]}],
                        'parallel_tool_calls': False, 'tool_choice': 'auto', 'tools': []}).encode()
                    kind = 'application/json'
                else:
                    data, kind = b'\x00\x00' * 4800, 'audio/pcm'
                self.send_response(200)
                self.send_header('Content-Type', kind)
                self.send_header('Content-Length', str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *args):
                pass

        self.httpd = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.httpd.server_address[1]}/v1'


def test_demo_runs_end_to_end(tmp_path):
    server = FakeOpenAI()
    picture = tmp_path / 'view.jpg'
    image = np.full((360, 640, 3), 180, np.uint8)
    cv2.rectangle(image, (250, 150), (390, 300), (200, 80, 20), -1)
    cv2.imwrite(str(picture), image)
    played = tmp_path / 'played'
    played.mkdir()
    copy = f"import shutil,sys,time; shutil.copy(sys.argv[1], r'{played}/%d.wav' % time.time_ns())"
    env = dict(os.environ, OPENAI_API_KEY='dummy-not-real', PYTHONPATH=str(PACKAGE))
    result = subprocess.run(
        [sys.executable, '-m', 'craft_voice.demo', '--camera', str(picture), '--once', '--seconds', '9',
         '--scenario', 'Idle:0.5, Ready:2.5, Run.PrepareTask@approach:0.3, Run.Execute@approach:2, '
                       'Fault@approach!Run.Execute: execution timeout:4',
         '--base-url', server.url, '--player', f'{sys.executable} -c "{copy}" {{file}}',
         '--keep-running-without-input'],
        input='앞에 뭐 있어?\n/describe\n/nonsense\n', capture_output=True, text=True, timeout=60, env=env)
    server.httpd.shutdown()
    out = result.stdout
    assert result.returncode == 0, result.stderr
    assert out.count('>> [state] 상태 알림') == 2                 # Ready, Run.Execute(approach)
    assert '>> [ask] 앞에 파란 상자가 보여요.' in out
    assert '>> [scene] 주변이 밝은 방이에요.' in out
    assert '>> [fault] 문제가 생겨서 멈췄어요. 원인: 실행 시간 초과.' in out
    assert 'commands:' in out                                   # unknown command shows help
    responses = [body for path, body in server.requests if path.endswith('/responses')]
    assert responses and all(b['input'][0]['content'][1]['image_url'].startswith('data:image/jpeg;base64,')
                             for b in responses)
    assert any('접근' in b['input'][0]['content'][0]['text'] for b in responses)  # step name from diagnostics
    assert len(list(played.iterdir())) == 5                     # ask, scene, 2 states, fault
