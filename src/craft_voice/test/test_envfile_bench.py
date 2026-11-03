import os
import subprocess
import sys
from pathlib import Path

from craft_voice.envfile import load_env_file

PACKAGE = Path(__file__).resolve().parents[1]


def test_env_file_sets_only_missing_names_and_skips_placeholder(tmp_path, monkeypatch):
    env = tmp_path / 'openai.env'
    env.write_text('# comment\nexport OPENAI_API_KEY="sk-test-123"\nOPENAI_BASE_URL=\nOTHER=keep-shell\n'
                   'PLACEHOLDER_ONLY=sk-put-your-key-here\nnot a line\n', encoding='utf-8')
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    monkeypatch.setenv('OTHER', 'from-shell')
    monkeypatch.delenv('PLACEHOLDER_ONLY', raising=False)
    used, names = load_env_file(env)
    assert used == env and names == ['OPENAI_API_KEY']
    assert os.environ['OPENAI_API_KEY'] == 'sk-test-123' and os.environ['OTHER'] == 'from-shell'
    assert 'PLACEHOLDER_ONLY' not in os.environ and 'OPENAI_BASE_URL' not in os.environ
    assert load_env_file(tmp_path / 'missing.env') == (None, [])


def test_example_template_is_tracked_and_has_no_real_key():
    text = (PACKAGE / 'env' / 'openai.env.example').read_text(encoding='utf-8')
    assert 'OPENAI_API_KEY=sk-put-your-key-here' in text


def test_bench_runs_and_reports_the_speak_path():
    env = dict(os.environ, PYTHONPATH=str(PACKAGE), CRAFT_VOICE_ENV='/nonexistent')
    env.pop('OPENAI_API_KEY', None)
    result = subprocess.run([sys.executable, '-m', 'craft_voice.bench', '--engine', 'instant', '--runs', '3',
                             '--player', 'none'], capture_output=True, text=True, timeout=60, env=env)
    assert result.returncode == 0, result.stderr
    assert 'to_play: median' in result.stdout and '(n=3)' in result.stdout
    missing = subprocess.run([sys.executable, '-m', 'craft_voice.bench', '--runs', '1', '--player', 'none'],
                             capture_output=True, text=True, timeout=60, env=env)
    assert missing.returncode == 2 and 'nothing to time' in missing.stderr
