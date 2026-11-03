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
         '--stream', 'off', '--keep-running-without-input'],
        input='앞에 뭐 있어?\n/describe\n/nonsense\n', capture_output=True, text=True, timeout=60, env=env)
    server.httpd.shutdown()
    out = result.stdout
    assert result.returncode == 0, result.stderr
    assert out.count('>> [state] 상태 알림') == 2                 # Ready, Run.Execute(approach)
    assert '>> [ask] 앞에 파란 상자가 보여요.' in out
    assert '>> [scene] 주변이 밝은 방이에요.' in out
    assert '>> [fault] 목표까지 너무 오래 걸려서 멈춰 있어요.' in out
    assert 'commands:' in out                                   # unknown command shows help
    responses = [body for path, body in server.requests if path.endswith('/responses')]
    assert responses and all(b['input'][0]['content'][1]['image_url'].startswith('data:image/jpeg;base64,')
                             for b in responses)
    assert any('접근' in b['input'][0]['content'][0]['text'] for b in responses)  # step name from diagnostics
    assert len(list(played.iterdir())) == 5                     # ask, scene, 2 states, fault


def make_video(path, frames=30, fps=10):
    """Frame i is filled with i*8, so the value tells which frame the camera handed out."""
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'MJPG'), fps, (64, 48))
    for i in range(frames):
        writer.write(np.full((48, 64, 3), i * 8, np.uint8))
    writer.release()
    assert path.stat().st_size > 0


def demo(args, stdin, env_extra=None, timeout=30):
    env = dict(os.environ, PYTHONPATH=str(PACKAGE))
    env.pop('OPENAI_API_KEY', None)
    env.update(env_extra or {})
    started = __import__('time').monotonic()
    result = subprocess.run([sys.executable, '-m', 'craft_voice.demo', '--player', 'none', *args], input=stdin,
                            capture_output=True, timeout=timeout, env=env)
    return result, __import__('time').monotonic() - started


def test_camera_close_while_reading_video_never_hangs_or_crashes(tmp_path):
    """QA: release() while the reader thread was inside read() -> hang/SIGABRT ~1 in 5 runs."""
    video = tmp_path / 'clip.avi'
    make_video(video)
    code = (f'import time; from craft_voice.demo import Camera; c = Camera({str(video)!r}); '
            'time.sleep(0.3); c.close(); print("closed")')
    for _ in range(8):
        result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=15,
                                cwd=str(PACKAGE))
        assert result.returncode == 0 and result.stdout.strip() == 'closed', result.stderr


def test_video_plays_at_its_own_speed_and_loops(tmp_path):
    import time
    from craft_voice.demo import Camera
    video = tmp_path / 'clip.avi'
    make_video(video, frames=30, fps=10)                      # 3 s
    camera = Camera(str(video))
    try:
        samples = []
        for _ in range(26):                                   # 2.6 s, every 0.1 s
            frame = camera.frame()
            samples.append(None if frame is None else round(int(frame[24, 32, 0]) / 8))
            time.sleep(0.1)
        seen = [s for s in samples if s is not None]
        steps = [b - a for a, b in zip(seen, seen[1:])]
        assert max(steps) <= 3, samples                       # ~1 frame per 0.1 s, not jumping ahead
        assert 15 <= seen[-1] - seen[0] <= 30, samples
        time.sleep(1.2)                                       # past the end: it must start over
        assert round(int(camera.frame()[24, 32, 0]) / 8) < seen[-1], samples
    finally:
        camera.close()


def test_quit_works_with_keep_running_flag():
    result, took = demo(['--camera', 'none', '--seconds', '15', '--keep-running-without-input'], b'/quit\n')
    assert result.returncode == 0 and took < 6


def test_broken_utf8_input_does_not_kill_input_thread():
    result, took = demo(['--camera', 'none', '--seconds', '15'], b'\xec\x95\n/quit\n',
                        {'PYTHONIOENCODING': 'utf-8:strict'})
    assert result.returncode == 0 and took < 6 and b'Traceback' not in result.stderr


@pytest.mark.parametrize('encoding', ['cp949', 'cp1252'])
def test_non_utf8_console_keeps_talking(encoding):
    """Windows consoles/pipes: Korean output must not crash the main loop or the speak thread."""
    result, _ = demo(['--camera', 'none', '--seconds', '4', '--keep-running-without-input', '--once',
                      '--scenario', 'Ready:1, Fault!Run.Execute: execution timeout:5'], b'',
                     {'PYTHONIOENCODING': encoding})
    out = result.stdout.decode(encoding, errors='replace')
    assert result.returncode == 0 and b'Traceback' not in result.stderr, result.stderr
    assert out.count('>> [') == 2 and 'pending_speech' in out  # Ready and Fault both went out


def test_raw_mjpeg_plays_as_video_and_non_ascii_picture_path(tmp_path):
    import time
    from craft_voice.demo import Camera
    raw = tmp_path / 'clip.mjpeg'                              # concatenated JPEGs, no container
    with open(raw, 'wb') as out:
        for i in range(30):
            out.write(cv2.imencode('.jpg', np.full((48, 64, 3), i * 8, np.uint8))[1].tobytes())
    camera = Camera(str(raw))
    try:
        assert camera._thread is not None                      # not mistaken for a still picture
        time.sleep(1.0)
        first = camera.frame()
        time.sleep(0.5)
        assert first is not None and int(camera.frame()[24, 32, 0]) != int(first[24, 32, 0])
    finally:
        camera.close()
    picture = tmp_path / '로봇 사진.png'
    cv2.imwrite(str(picture), np.full((10, 20, 3), 77, np.uint8))
    still = Camera(str(picture))
    assert still._thread is None and int(still.frame()[5, 5, 0]) == 77


def test_demo_narrates_while_working_without_a_key():
    result, _ = demo(['--camera', 'none', '--once', '--seconds', '8', '--narrate-period', '2',
                      '--keep-running-without-input', '--scenario', 'Run.Execute@approach:9'], b'')
    out = result.stdout.decode()
    assert result.returncode == 0, result.stderr
    assert '>> [state] 작업을 시작할게요.' in out             # first label: step not in diagnostics yet
    assert '>> [scene] 접근 중이에요.' in out                  # periodic: now with the step name
