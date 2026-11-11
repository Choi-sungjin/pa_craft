"""Speaker output and offline speech through external commands (no shell).

Commands are argv lists; '{file}' is replaced by a temporary WAV path and
'{text}' by the sentence, so nothing is ever interpreted by a shell.
"""

from __future__ import annotations

import io
import os
import re
import shlex
import subprocess
import tempfile
import threading
import wave


def split_command(text: str, windows: bool | None = None) -> list[str]:
    """Command text -> argv. On Windows backslashes are path separators, not escapes."""
    if not text.strip():
        return []
    if windows is None:
        windows = os.name == 'nt'
    if not windows:
        return shlex.split(text)
    # Windows rules (simplified): backslashes are literal, "..." groups spaces anywhere in an argument.
    parts, current, quoted, started = [], [], False, False
    for char in text:
        if char == '"':
            quoted, started = not quoted, True
        elif char.isspace() and not quoted:
            if started:
                parts.append(''.join(current))
            current, started = [], False
        else:
            current.append(char)
            started = True
    if started:
        parts.append(''.join(current))
    return parts


def pcm_to_wav(pcm: bytes, sample_rate: int, channels: int = 1, sample_width: int = 2) -> bytes:
    """Raw little-endian PCM -> WAV with a correct header (any player accepts it)."""
    frame = channels * sample_width
    pcm = pcm[:len(pcm) - len(pcm) % frame]
    out = io.BytesIO()
    with wave.open(out, 'wb') as wav:
        wav.setnchannels(channels)
        wav.setsampwidth(sample_width)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm)
    return out.getvalue()


def normalize_wav(data: bytes) -> bytes:
    """Rewrite a PCM WAV header from the data actually present.

    `espeak-ng --stdout` streams, so its header carries a placeholder length (reads as ~13 hours);
    aplay plays until EOF anyway, but players that trust the header may not.
    """
    try:
        with wave.open(io.BytesIO(data)) as wav:
            channels, width, rate = wav.getnchannels(), wav.getsampwidth(), wav.getframerate()
            frames = wav.readframes(wav.getnframes())
    except (wave.Error, EOFError):
        return data  # not PCM WAV: hand it over unchanged
    return pcm_to_wav(frames, rate, channels, width)


def wav_seconds(data: bytes) -> float:
    with wave.open(io.BytesIO(data)) as wav:
        return wav.getnframes() / wav.getframerate()


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


def stream_command_for(player: list[str]) -> list[str] | None:
    """The raw-PCM stdin variant of a WAV player command, keeping its device options; None if there is none.

    aplay -q -D plughw:1,0 {file}  ->  aplay -q -D plughw:1,0 -t raw -f S16_LE -r {rate} -c 1 -
    paplay --device=X {file}      ->  paplay --device=X --raw --format=s16le --rate={rate} --channels=1
    """
    if not player:
        return None
    name = os.path.basename(player[0])
    keep = [arg for arg in player if arg != '{file}']
    if name == 'aplay':
        return keep + ['-t', 'raw', '-f', 'S16_LE', '-r', '{rate}', '-c', '1', '-']
    if name == 'paplay':
        return keep + ['--raw', '--format=s16le', '--rate={rate}', '--channels=1']
    return None


class StreamPlayer:
    """Plays raw 16-bit mono PCM fed chunk by chunk to a command's stdin, e.g.
    `aplay -q -t raw -f S16_LE -r {rate} -c 1 -` (see stream_command_for).
    Same stop/cancel contract as CommandPlayer; writing never holds the lock, so stop() always gets through.
    A write that the player does not take within write_timeout_s (wedged device) stops the player.
    """

    def __init__(self, command: list[str], timeout_s: float = 120.0, write_timeout_s: float = 5.0):
        if not command:
            raise ValueError('stream player command is empty')
        self.command = list(command)
        self.timeout_s = timeout_s
        self.write_timeout_s = write_timeout_s
        self._lock = threading.Lock()
        self._process: subprocess.Popen | None = None
        self._stopped = False
        self._timed_out = False

    @property
    def playing(self) -> bool:
        with self._lock:
            return self._process is not None and self._process.poll() is None

    def open(self, rate: int, cancelled=None) -> bool:
        argv = [arg.replace('{rate}', str(int(rate))) for arg in self.command]
        with self._lock:
            if cancelled is not None and cancelled():
                return False
            self._stopped = self._timed_out = False
            try:
                # bufsize=0: each chunk goes to the player at once, never held in a Python buffer
                self._process = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL,
                                                 stderr=subprocess.PIPE, bufsize=0)
            except OSError as error:
                raise RuntimeError(f'cannot run stream player {argv[0]!r}: {error}') from error
        return True

    def _watchdog(self):
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                self._timed_out = True
                self._process.kill()

    def write(self, chunk: bytes) -> bool:
        """False once the player stopped or died (then the rest is not sent)."""
        process = self._process
        if process is None or self._stopped:
            return False
        timer = threading.Timer(self.write_timeout_s, self._watchdog)
        timer.daemon = True
        timer.start()
        try:
            view = memoryview(chunk)
            while view:
                written = process.stdin.write(view)
                view = view[written or 0:]
            return True
        except (BrokenPipeError, OSError, ValueError):
            return False
        finally:
            timer.cancel()

    def finish(self) -> bool:
        """End of audio: wait until it has been played. False if it was stopped; raises if the player failed."""
        process = self._process
        if process is None:
            return False
        try:
            try:
                process.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            try:
                process.wait(timeout=self.timeout_s)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            error = process.stderr.read().decode(errors='replace').strip() if process.stderr else ''
            if process.stderr:
                process.stderr.close()
        finally:
            with self._lock:
                self._process = None
                stopped, timed_out = self._stopped, self._timed_out
        if stopped:
            return False
        if timed_out:
            raise RuntimeError(f'stream player took no audio for {self.write_timeout_s} s (device stuck?)')
        if process.returncode != 0:
            raise RuntimeError(f'stream player exited {process.returncode}: {error}')
        return True

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
        return normalize_wav(result.stdout)


class WinsoundPlayer:
    """Windows speaker without external programs (winsound, PCM WAV only). Same contract as CommandPlayer."""

    def __init__(self):
        import winsound  # Windows only
        self._winsound = winsound
        self._lock = threading.Lock()
        self._stop = None

    @property
    def playing(self) -> bool:
        with self._lock:
            return self._stop is not None

    def play(self, wav: bytes, cancelled=None) -> bool:
        wav = normalize_wav(wav)
        seconds = wav_seconds(wav)
        handle, path = tempfile.mkstemp(prefix='craft_voice_', suffix='.wav')
        with os.fdopen(handle, 'wb') as out:
            out.write(wav)
        ws = self._winsound
        try:
            with self._lock:
                if cancelled is not None and cancelled():
                    return False
                stop = self._stop = threading.Event()
                ws.PlaySound(path, ws.SND_FILENAME | ws.SND_ASYNC | ws.SND_NODEFAULT)
            stopped = stop.wait(seconds + 0.5)  # margin for device latency before the next sound
            if stopped:
                ws.PlaySound(None, 0)  # stops the asynchronous sound
            return not stopped
        finally:
            with self._lock:
                self._stop = None
            try:
                os.unlink(path)
            except OSError:
                pass

    def stop(self):
        with self._lock:
            if self._stop is not None:
                self._stop.set()


def read_clip(path: str, max_bytes: int = 8_000_000) -> bytes:
    """A recorded reaction (PCM WAV file) as bytes for the player; raises if it is not a usable WAV."""
    with open(os.path.expanduser(path), 'rb') as handle:
        data = handle.read(max_bytes + 1)
    if len(data) > max_bytes:
        raise ValueError(f'{path}: longer than {max_bytes} bytes')
    with wave.open(io.BytesIO(data)) as wav:  # raises wave.Error for anything that is not PCM WAV
        if wav.getnframes() <= 0:
            raise ValueError(f'{path}: empty recording')
    return data


class SpeechCache:
    """Synthesized sentences kept on disk, so fixed sentences play at once and without the network.

    signature: everything that changes how a sentence sounds (model, voice, instructions, tempo).
    """

    def __init__(self, directory: str, signature: str, max_files: int = 500):
        import hashlib
        self._hash = hashlib.sha256
        self.directory = os.path.expanduser(directory)
        self.signature = signature
        self.max_files = max_files
        self._lock = threading.Lock()
        os.makedirs(self.directory, exist_ok=True)

    def path(self, text: str) -> str:
        digest = self._hash(f'{self.signature}\n{text}'.encode('utf-8')).hexdigest()[:40]
        return os.path.join(self.directory, digest + '.wav')

    def get(self, text: str) -> bytes | None:
        path = self.path(text)
        try:
            with open(path, 'rb') as handle:
                data = handle.read()
            wav_seconds(data)  # a truncated file is not played
            os.utime(path)     # most recently used last: pruning removes the oldest
            return data
        except (OSError, wave.Error, EOFError, ZeroDivisionError):
            return None

    def put(self, text: str, wav: bytes):
        if not wav:
            return
        path = self.path(text)
        with self._lock:
            handle, temporary = tempfile.mkstemp(dir=self.directory, suffix='.tmp')
            try:
                with os.fdopen(handle, 'wb') as out:
                    out.write(wav)
                os.replace(temporary, path)
            except OSError:
                try:
                    os.unlink(temporary)
                except OSError:
                    pass
                return
            self._prune()

    def _prune(self):
        try:
            files = [os.path.join(self.directory, name) for name in os.listdir(self.directory)
                     if name.endswith('.wav')]
            if len(files) <= self.max_files:
                return
            files.sort(key=os.path.getmtime)
            for old in files[:len(files) - self.max_files]:
                os.unlink(old)
        except OSError:
            pass
