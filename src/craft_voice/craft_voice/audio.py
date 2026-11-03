"""Speaker output and offline speech through external commands (no shell).

Commands are argv lists; '{file}' is replaced by a temporary WAV path and
'{text}' by the sentence, so nothing is ever interpreted by a shell.
"""

from __future__ import annotations

import os
import re
import shlex
import subprocess
import tempfile
import threading


def split_command(text: str) -> list[str]:
    return shlex.split(text) if text.strip() else []


class CommandPlayer:
    """Plays WAV bytes with e.g. `aplay -q {file}` or `paplay {file}`; stop() cuts it short."""

    def __init__(self, command: list[str], timeout_s: float = 60.0):
        if not command:
            raise ValueError('player command is empty')
        self.command = list(command)
        self.timeout_s = timeout_s
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._stopped = False

    @property
    def playing(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def play(self, wav: bytes, cancelled=None) -> bool:
        """Blocks until playback ends. False if it was stopped (or cancelled() before it began)."""
        handle, path = tempfile.mkstemp(prefix='craft_voice_', suffix='.wav')
        try:
            with os.fdopen(handle, 'wb') as out:
                out.write(wav)
            argv = [path if arg == '{file}' else arg for arg in self.command]
            if '{file}' not in self.command:
                argv.append(path)
            with self._lock:
                # Checked under the same lock stop() takes: a stop() that came before this point
                # found no process, so the caller's cancel flag is what tells us not to start.
                if cancelled is not None and cancelled():
                    return False
                self._stopped = False
                try:
                    self._process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                                     stderr=subprocess.PIPE)
                except OSError as error:
                    raise RuntimeError(f'cannot run player {argv[0]!r}: {error}') from error
                process = self._process
            try:
                _, stderr = process.communicate(timeout=self.timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()
                _, stderr = process.communicate()
                raise RuntimeError('player timed out')
            finally:
                with self._lock:
                    self._process = None
                    stopped = self._stopped
            if stopped:
                return False
            if process.returncode != 0:
                raise RuntimeError(f'player exited {process.returncode}: {stderr.decode(errors="replace").strip()}')
            return True
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

    def stop(self):
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                self._stopped = True
                self._process.terminate()


class CommandTts:
    """Offline fallback, e.g. `espeak-ng -v ko --stdout {text}`: the command prints WAV to stdout."""

    def __init__(self, command: list[str], timeout_s: float = 20.0):
        if '{text}' not in command:
            raise ValueError("fallback TTS command needs a '{text}' argument")
        self.command = list(command)
        self.timeout_s = timeout_s

    def synthesize(self, text: str) -> bytes:
        # Leading dashes would turn the sentence into an option of the TTS program (-w FILE ...).
        text = re.sub(r'^[\s\-\u2010-\u2015]+', '', text)
        if not text:
            raise ValueError('nothing to say')
        argv = [text if arg == '{text}' else arg for arg in self.command]
        result = subprocess.run(argv, stdin=subprocess.DEVNULL, capture_output=True, timeout=self.timeout_s)
        if result.returncode != 0 or not result.stdout:
            raise RuntimeError(f'fallback TTS failed ({result.returncode}): '
                               f'{result.stderr.decode(errors="replace").strip()}')
        return result.stdout
