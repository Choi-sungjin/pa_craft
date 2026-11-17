"""Run the bash command with fake setup files and a fake interpreter, never ROS."""
import json
import os
from pathlib import Path
import subprocess

import pytest

SCRIPT = Path(__file__).with_name('home.bash')


@pytest.fixture
def shell_env(tmp_path):
    ws = tmp_path / 'workspace with spaces'
    (ws / 'install').mkdir(parents=True)
    (ws / 'tools').mkdir()
    (ws / 'install/setup.bash').write_text('export CRAFT_HOME_TEST_SETUP=loaded\n')
    (ws / 'tools/go_home.py').write_text('# fake; must not execute\n')
    ros = tmp_path / 'ros.bash'
    ros.write_text('export CRAFT_HOME_TEST_ROS=loaded\n')
    interpreter = tmp_path / 'fake-python'
    interpreter.write_text('''#!/usr/bin/python3
import json, os, sys
print(json.dumps({'argv': sys.argv[1:], 'domain': os.environ['ROS_DOMAIN_ID'],
                  'ros': os.environ.get('CRAFT_HOME_TEST_ROS'),
                  'setup': os.environ.get('CRAFT_HOME_TEST_SETUP')}))
if os.environ.get('CRAFT_HOME_TEST_ERROR'):
    print('RTPS_TRANSPORT_SHM Error noise', file=sys.stderr)
    print('real failure', file=sys.stderr)
sys.exit(int(os.environ.get('CRAFT_HOME_TEST_RC', '0')))
''')
    interpreter.chmod(0o755)
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('CRAFT_', 'ROS_'))}
    env.update(CRAFT_WS=str(ws), CRAFT_HOME_ROS_SETUP=str(ros),
               CRAFT_HOME_PYTHON=str(interpreter))
    return env, ws, ros


def run_home(env, *args):
    return subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', '''
source "$1"
shift
home "$@"
rc=$?
printf '\nCALLER:%s:%s\n' "${ROS_DOMAIN_ID-unset}" "${CRAFT_HOME_TEST_SETUP-unset}"
exit "$rc"
''', 'test-home', str(SCRIPT), *args], env=env, text=True, capture_output=True)


@pytest.mark.parametrize('args', [(), ('0.662', '1.081', '-179.6'), ('--unknown', 'a b')])
def test_arguments_setup_and_caller_environment(shell_env, args):
    env, ws, _ = shell_env
    result = run_home(env, *args)
    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout.splitlines()[0])
    assert payload == {'argv': [str(ws / 'tools/go_home.py'), *args],
                       'domain': '61', 'ros': 'loaded', 'setup': 'loaded'}
    assert 'CALLER:unset:unset' in result.stdout


def test_existing_domain_is_preserved(shell_env):
    env, _, _ = shell_env
    env['ROS_DOMAIN_ID'] = '213'
    result = run_home(env)
    assert result.returncode == 0
    assert json.loads(result.stdout.splitlines()[0])['domain'] == '213'
    assert 'CALLER:213:unset' in result.stdout


def test_sourcing_registers_without_running(shell_env):
    env, _, _ = shell_env
    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', 'source "$1"; type -t home',
         'test-home', str(SCRIPT)], env=env, text=True, capture_output=True)
    assert result.returncode == 0
    assert result.stdout.strip() == 'function'


@pytest.mark.parametrize('missing', ['install/setup.bash', 'tools/go_home.py'])
def test_missing_workspace_stops_before_interpreter(shell_env, missing):
    env, ws, _ = shell_env
    (ws / missing).unlink()
    result = run_home(env)
    assert result.returncode == 1
    assert '작업 경로 또는 빌드 결과' in result.stderr
    assert 'argv' not in result.stdout


@pytest.mark.parametrize('which', ['ros', 'workspace'])
def test_setup_failure_stops_before_interpreter(shell_env, which):
    env, ws, ros = shell_env
    setup = ros if which == 'ros' else ws / 'install/setup.bash'
    setup.write_text('return 9\n')
    result = run_home(env)
    assert result.returncode == 9
    assert 'argv' not in result.stdout


def test_exit_code_and_real_errors_are_preserved(shell_env):
    env, _, _ = shell_env
    env.update(CRAFT_HOME_TEST_RC='7', CRAFT_HOME_TEST_ERROR='1')
    result = run_home(env)
    assert result.returncode == 7
    assert 'real failure' in result.stderr
    assert 'RTPS_TRANSPORT_SHM Error' not in result.stderr
