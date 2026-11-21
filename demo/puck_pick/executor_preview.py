"""Read offline previews from the C++ puck executor."""
import json
import os
from pathlib import Path
import subprocess


def preview(*args):
    env = os.environ.copy()
    env.setdefault('CRAFT_REPO', str(Path(__file__).resolve().parents[2]))
    output = subprocess.check_output(
        ['ros2', 'run', 'craft_planning', 'puck_execute', '--dry-run', '--json', *args],
        text=True, env=env,
    )
    return json.loads(output)
