"""Try the voice on a laptop: no ROS, no robot. Python venv + webcam (or a picture) + speaker.

    python -m craft_voice.demo                       # webcam 0, built-in scenario, GPT + OpenAI speech
    python -m craft_voice.demo --narrate-period 10   # say what it does and sees every 10 s while working
    python -m craft_voice.demo --camera photo.jpg    # a picture or a video file instead of the webcam
    python -m craft_voice.demo --template            # state changes as fixed sentences (no GPT for them)

The same Narrator/StateTracker the robot uses gets a simulated planning state
machine (craft_voice.sim) instead of /planning/state. Type while it runs:
    a question            asked with the current picture and state
    /describe /status     what it sees / what it is doing
    /state Ready          force a state (Label[@step][!reason]); /fault <reason>; /resume back to the scenario
    /say <text>  /mute  /unmute  /stop  /quit
"""

from __future__ import annotations

import argparse
import logging
import os
import platform
import shutil
import sys
import threading
import time

import numpy as np

from .audio import CommandPlayer, CommandTts, StreamPlayer, WinsoundPlayer, split_command, stream_command_for
from .narrator import Narrator, NarratorConfig
from .robot_state import StateTracker, parse_list, parse_mapping
from .mission import DICE_SCENARIO, MISSION_CONTEXT, STEP_NAMES
from .sim import DEFAULT_SCENARIO, SimStateMachine, parse_item, parse_scenario

log = logging.getLogger('craft_voice')
IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff')
SIM_MOTION = {'Run.Execute': (0.12, 0.0), 'Run.Stopping': (0.03, 0.0)}  # simulated odom (m/s, rad/s)


class Camera:
    """Latest frame from a webcam index, a video file (looped) or a still image; None if no source."""

    def __init__(self, source: str):
        import cv2
        self._cv2 = cv2
        self._frame = None
        self._lock = threading.Lock()
        self._capture = None
        self._thread = None
        self._stop = threading.Event()
        self._source = source
        self._is_video = False  # anything but a webcam index: reopen at the end
        if source.lower() in ('', 'none'):
            return
        if source.isdigit():
            self._capture = cv2.VideoCapture(int(source))
        else:
            if os.path.isfile(source) and os.path.splitext(source)[1].lower() in IMAGE_EXTENSIONS:
                # imdecode(fromfile) instead of imread: imread fails on non-ASCII paths on Windows
                image = cv2.imdecode(np.fromfile(source, np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise RuntimeError(f'picture {source!r} could not be read')
                self._frame = image
                return
            self._capture = cv2.VideoCapture(source)  # video file (incl. GIF, raw MJPEG) or stream URL
            self._is_video = True
        if not self._capture.isOpened():
            raise RuntimeError(f'camera source {source!r} could not be opened')
        self._thread = threading.Thread(target=self._read, name='camera', daemon=True)
        self._thread.start()

    def _read(self):
        """Only this thread touches the VideoCapture (it is not thread safe); close() joins it first."""
        cv2 = self._cv2
        # Pace a recording (local file, or a URL with a known length) at its own fps; a live stream
        # and a webcam already deliver in real time, so they are read as they come.
        recording = self._is_video and (os.path.isfile(self._source) or
                                        self._capture.get(cv2.CAP_PROP_FRAME_COUNT) > 0)
        fps = self._capture.get(cv2.CAP_PROP_FPS) if recording else 0
        period = 1.0 / fps if 0 < fps < 240 else (1.0 / 30 if recording else 0.0)
        next_at = time.monotonic()
        while not self._stop.is_set():
            ok, frame = self._capture.read()
            if not ok:
                if self._is_video:
                    # End of a video file: reopen (seeking back fails on MPEG-TS, unknown-length WebM, MJPEG).
                    self._capture.release()
                    self._capture = cv2.VideoCapture(self._source)
                self._stop.wait(0.05)
                continue
            with self._lock:
                self._frame = frame
            if period:  # a video file plays at its own speed, not as fast as it decodes
                next_at = max(next_at + period, time.monotonic() - period)
                self._stop.wait(max(0.0, next_at - time.monotonic()))

    def frame(self):
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            if self._thread.is_alive():
                return  # still inside read(): releasing now could crash; the process exit cleans up
        if self._capture is not None:
            self._capture.release()


def default_player(command: str):
    """--player text -> player object. 'auto' picks what this OS has."""
    if command != 'auto':
        return CommandPlayer(split_command(command)) if command.strip() and command != 'none' else None
    system = platform.system()
    if system == 'Windows':
        return WinsoundPlayer()
    if system == 'Darwin':
        return CommandPlayer(['afplay', '{file}'])
    for candidate in (['paplay', '{file}'], ['aplay', '-q', '{file}'],
                      ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', '{file}']):
        if shutil.which(candidate[0]):
            return CommandPlayer(candidate)
    log.warning('no audio player found (paplay, aplay, ffplay): sentences are printed only')
    return None


def default_stream_player(args, player):
    """Stream OpenAI speech through the raw-PCM variant of the chosen player (aplay/paplay); others: whole WAVs."""
    if args.stream == 'off' or player is None:
        return None
    command = split_command(args.stream_player) or stream_command_for(getattr(player, 'command', []))
    if not command:
        if args.stream == 'on':
            log.warning('--stream on but the player has no raw-PCM variant (give --stream-player)')
        return None
    return StreamPlayer(command)


def build(args) -> tuple[Narrator, SimStateMachine, Camera]:
    backend = None
    from .gpt import openai_auth_available
    if openai_auth_available():
        from .gpt import OpenAIBackend
        backend = OpenAIBackend(model=args.model, tts_model=args.tts_model, voice=args.voice,
                                tts_instructions=args.tts_instructions, reasoning_effort=args.reasoning_effort,
                                image_detail=args.image_detail, timeout_s=args.timeout, base_url=args.base_url)
    else:
        log.warning('OPENAI_API_KEY is not set: fixed sentences only, no GPT and no OpenAI speech')
    fallback = split_command(args.fallback_tts)
    camera = Camera(args.camera)
    tracker = StateTracker(announce=tuple(parse_list(args.announce)), cooldown_s=args.cooldown,
                           step_names=parse_mapping(args.step_names))
    config = NarratorConfig(state_mode='template' if args.template else 'gpt', narrate_period_s=args.narrate_period,
                            narrate_idle_period_s=args.narrate_idle_period,
                            persona=args.persona, use_image=args.camera.lower() not in ('', 'none'),
                            mission_context='' if args.mission_context == 'none' else args.mission_context,
                            camera_description='노트북 카메라 (로봇 대신 시험 중)')
    player = default_player(args.player)
    narrator = Narrator(config, tracker, backend=backend, player=player,
                        stream_player=default_stream_player(args, player) if backend else None,
                        frame_provider=camera.frame, fallback_tts=CommandTts(fallback) if fallback else None,
                        on_text=lambda kind, text: print(f'\n>> [{kind}] {text}', flush=True))
    scenario = {'dice': DICE_SCENARIO, 'default': DEFAULT_SCENARIO}.get(args.scenario, args.scenario)
    return narrator, SimStateMachine(parse_scenario(scenario), loop=not args.once), camera


def handle(line: str, narrator: Narrator, sim: SimStateMachine) -> bool:
    """One typed line. False to quit."""
    line = line.strip()
    if not line:
        return True
    if not line.startswith('/'):
        narrator.ask(line)
        return True
    command, _, rest = line[1:].partition(' ')
    try:
        if command == 'quit':
            return False
        if command == 'state':
            sim.set_override(parse_item(rest, with_duration=False))
        elif command == 'fault':
            sim.fault(rest.strip() or 'execution timeout')
        elif command == 'resume':
            sim.set_override(None)
        elif command == 'say':
            narrator.say(rest)
        elif command in ('describe', 'status', 'mute', 'unmute', 'stop'):
            narrator.command(command)
        else:
            print('commands: <question> /describe /status /state <Label[@step][!reason]> /fault <reason> '
                  '/resume /say <text> /mute /unmute /stop /quit', flush=True)
    except ValueError as error:
        print(f'! {error}', flush=True)
    return True


def run(args) -> int:
    narrator, sim, camera = build(args)
    stop = threading.Event()

    def read_input():
        for line in sys.stdin:
            if not handle(line, narrator, sim):
                stop.set()  # /quit always quits
                return
        if not args.keep_running_without_input:
            stop.set()  # stdin closed (e.g. Ctrl+D)

    threading.Thread(target=read_input, name='stdin', daemon=True).start()
    started = time.monotonic()
    last_diag = -1.0
    shown = None
    try:
        while not stop.is_set() and (args.seconds <= 0 or time.monotonic() - started < args.seconds):
            now = time.monotonic()
            current = sim.at(now - started)
            narrator.planning_state(current.label)            # like /planning/state at 20 Hz
            narrator.motion(*SIM_MOTION.get(current.label, (0.0, 0.0)))  # like /perception/state/odom
            if now - last_diag >= 1.0:                         # like /planning/diagnostics at 1 Hz
                narrator.diagnostics(*sim.diagnostics(current))
                last_diag = now
            narrator.tick()
            if current.label != shown:
                print(f'[{now - started:6.1f}s] state: {current.label}'
                      + (f' ({current.step})' if current.step else '')
                      + (f' - {current.reason}' if current.reason else ''), flush=True)
                shown = current.label
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        narrator.close()
        camera.close()
    print(f'stats: {narrator.status()}', flush=True)
    return 0


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='python -m craft_voice.demo', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--camera', default='0', help='webcam index, image/video path, or none (default 0)')
    p.add_argument('--scenario', default='dice',
                   help="'dice' (the competition mission), 'default', or Label[@step][!reason]:seconds, ...")
    p.add_argument('--mission-context', default=MISSION_CONTEXT, help="background for GPT, 'none' for no mission")
    p.add_argument('--once', action='store_true', help='play the scenario once instead of looping')
    p.add_argument('--template', action='store_true', help='state changes as fixed sentences (no GPT)')
    p.add_argument('--narrate-period', type=float, default=20.0,
                   help='while working, say what it does and sees every N s (0 = off)')
    p.add_argument('--narrate-idle-period', type=float, default=0.0, help='the same while idle/ready/stopped')
    p.add_argument('--announce', default='Ready,Run.Execute,Run.Complete,Run.Stopping,Fault')
    p.add_argument('--cooldown', type=float, default=10.0)
    p.add_argument('--step-names', default=STEP_NAMES + ',approach:접근,grasp_pose:잡기 자세')
    p.add_argument('--persona', default='')
    p.add_argument('--model', default='gpt-5.4-mini,gpt-6-luna,gpt-5.5',
                   help='fastest first, comma separated fallbacks')
    p.add_argument('--reasoning-effort', default='none')
    p.add_argument('--image-detail', default='low')
    p.add_argument('--tts-model', default='gpt-4o-mini-tts')
    p.add_argument('--voice', default='coral')
    p.add_argument('--tts-instructions', default='밝고 또렷한 한국어 안내 로봇 목소리로, 조금 빠르게 말해.')
    p.add_argument('--timeout', type=float, default=15.0)
    p.add_argument('--base-url', default='', help='OpenAI-compatible endpoint ("" = api.openai.com)')
    p.add_argument('--player', default='auto', help="'auto', 'none', or a command with {file}")
    p.add_argument('--stream', choices=['auto', 'on', 'off'], default='auto',
                   help='play OpenAI speech while it arrives (auto: Linux with aplay)')
    p.add_argument('--stream-player', default='',
                   help='raw PCM stdin command with {rate}, e.g. "aplay -q -t raw -f S16_LE -r {rate} -c 1 -"')
    p.add_argument('--fallback-tts', default='', help='offline speech, e.g. "espeak-ng -v ko --stdout -- {text}"')
    p.add_argument('--seconds', type=float, default=0.0, help='stop after N seconds (0 = until /quit)')
    p.add_argument('--keep-running-without-input', action='store_true',
                   help='keep going when stdin closes (for scripted runs)')
    p.add_argument('--log-level', default='INFO')
    return p


def main(argv=None) -> int:
    # Korean text on a console or pipe that is not UTF-8 (Windows cp949/cp1252, C locale): never crash on it.
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors='replace')
        except (AttributeError, ValueError):
            pass
    try:
        sys.stdin.reconfigure(errors='replace')  # a broken byte in typed input must not kill the input thread
    except (AttributeError, ValueError):
        pass
    args = parser().parse_args(argv)
    from .envfile import load_env_file
    used, names = load_env_file()
    logging.basicConfig(level=args.log_level.upper(), format='%(asctime)s %(levelname)s %(message)s',
                        datefmt='%H:%M:%S')
    if names:
        log.info(f'loaded {", ".join(names)} from {used}')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
