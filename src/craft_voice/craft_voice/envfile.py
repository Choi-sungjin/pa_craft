"""Load the OpenAI key from a small KEY=VALUE file, so it never has to live in a launch file or a chat.

$CRAFT_VOICE_ENV names the file when it is set (no other file is tried). Otherwise, the first that exists: <source>/src/craft_voice/env/openai.env,
~/.config/craft_voice/openai.env, ~/.config/craft-voice/openai.key (a file named *.key holds only the key). Values already in the environment are never overwritten, and values are
never printed or logged.
"""

from __future__ import annotations

import os
from pathlib import Path

PLACEHOLDER = 'sk-put-your-key-here'


def candidates() -> list[Path]:
    if os.environ.get('CRAFT_VOICE_ENV'):  # named explicitly: only this file, also when it is missing
        return [Path(os.environ['CRAFT_VOICE_ENV']).expanduser()]
    paths = []
    paths.append(Path(__file__).resolve().parents[1] / 'env' / 'openai.env')  # source tree / venv demo
    paths.append(Path.home() / '.config' / 'craft_voice' / 'openai.env')
    paths.append(Path.home() / '.config' / 'craft-voice' / 'openai.key')  # a bare key on one line
    return paths


def load_env_file(path: str | os.PathLike | None = None) -> tuple[Path | None, list[str]]:
    """Returns (file used, names it set). Missing file is fine: (None, [])."""
    files = [Path(path).expanduser()] if path else candidates()
    for file in files:
        if not file.is_file():
            continue
        loaded = []
        for raw in file.read_text(encoding='utf-8').splitlines():
            line = raw.strip()
            if line and '=' not in line and ' ' not in line and file.suffix == '.key':
                line = 'OPENAI_API_KEY=' + line  # a key file holds just the key
            if not line or line.startswith('#') or '=' not in line:
                continue
            name, _, value = line.removeprefix('export ').partition('=')
            name, value = name.strip(), value.strip().strip('"').strip("'")
            if not name or not value or value == PLACEHOLDER or name in os.environ:
                continue
            os.environ[name] = value
            loaded.append(name)
        return file, loaded
    return None, []
