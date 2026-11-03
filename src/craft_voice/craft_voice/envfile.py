"""Load the OpenAI key from a small KEY=VALUE file, so it never has to live in a launch file or a chat.

Looked up in this order (first file that exists): $CRAFT_VOICE_ENV, <source>/src/craft_voice/env/openai.env,
~/.config/craft_voice/openai.env. Values already in the environment are never overwritten, and values are
never printed or logged.
"""

from __future__ import annotations

import os
from pathlib import Path

PLACEHOLDER = 'sk-put-your-key-here'


def candidates() -> list[Path]:
    paths = []
    if os.environ.get('CRAFT_VOICE_ENV'):
        paths.append(Path(os.environ['CRAFT_VOICE_ENV']).expanduser())
    paths.append(Path(__file__).resolve().parents[1] / 'env' / 'openai.env')  # source tree / venv demo
    paths.append(Path.home() / '.config' / 'craft_voice' / 'openai.env')
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
