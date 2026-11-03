"""How long from "the sentence is ready" (GPT answered) to "the speaker starts"? Measured, not guessed.

    python -m craft_voice.bench                          # OpenAI speech if OPENAI_API_KEY is set
    python -m craft_voice.bench --fallback-tts "espeak-ng -v ko --stdout -- {text}"
    python -m craft_voice.bench --gpt --camera photo.jpg # also time the GPT answer (picture + facts)
    python -m craft_voice.bench --engine instant          # only this code's own overhead (no TTS at all)
    python -m craft_voice.bench --compare "gpt-5.4-mini,gpt-6-luna,gpt-5.5" --efforts none,low
                                                          # GPT models side by side: the robot's real narration
                                                          # prompt + a picture (--camera, else a drawn scene)

Per run it times, through the same Narrator the robot uses:
    tts       sentence -> WAV bytes (OpenAI speech or the fallback command)
    to_play   sentence handed over -> player process started (queue + TTS + WAV + spawn)
    audio     length of the spoken audio
and with --gpt also the GPT answer, and event -> player start (GPT + TTS + overhead).
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import threading
import time

from .audio import CommandPlayer, CommandTts, pcm_to_wav, split_command, wav_seconds
from .narrator import Narrator, NarratorConfig
from .robot_state import StateTracker

SENTENCES = (
    '목표 지점으로 이동하고 있어요. 앞에 초록 신호가 켜져 있어요.',
    '퍽 두 개가 보여요. 가까운 퍽부터 집으러 갈게요.',
    '다음 동작을 계산하고 있어요.',
)


class Instant:
    """TTS that takes no time: what is left is this code's own overhead."""

    def synthesize(self, text, urgent=False):
        return pcm_to_wav(b'\0\0' * 24000, 24000)  # 1 s of silence


class TimedPlayer:
    """Wraps a real player and notes when play() is entered (= the WAV is ready and handed to the player)."""

    def __init__(self, inner):
        self.inner = inner
        self.started = []
        self.event = threading.Event()

    def play(self, wav, cancelled=None):
        self.started.append((time.monotonic(), wav_seconds(wav)))
        self.event.set()
        return self.inner.play(wav, cancelled) if self.inner else True

    def stop(self):
        if self.inner:
            self.inner.stop()


def summary(name: str, values: list[float]) -> str:
    if not values:
        return f'{name:>10}: -'
    return (f'{name:>10}: median {statistics.median(values):6.2f} s   min {min(values):6.2f}   '
            f'max {max(values):6.2f}   (n={len(values)})')


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog='python -m craft_voice.bench', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--runs', type=int, default=5)
    p.add_argument('--engine', choices=['auto', 'openai', 'fallback', 'instant'], default='auto',
                   help='auto: OpenAI if OPENAI_API_KEY is set, else --fallback-tts')
    p.add_argument('--fallback-tts', default='', help='e.g. "espeak-ng -v ko --stdout -- {text}"')
    p.add_argument('--player', default='aplay -q {file}', help="command with {file}; 'none' to skip playback")
    p.add_argument('--gpt', action='store_true', help='also time GPT (needs OPENAI_API_KEY)')
    p.add_argument('--camera', default='', help='picture for --gpt (none = text only)')
    p.add_argument('--model', default='gpt-5.4-mini,gpt-6-luna,gpt-5.5')
    p.add_argument('--reasoning-effort', default='none')
    p.add_argument('--tts-model', default='gpt-4o-mini-tts')
    p.add_argument('--voice', default='coral')
    p.add_argument('--base-url', default='')
    p.add_argument('--compare', default='', help='comma separated GPT models to time side by side')
    p.add_argument('--efforts', default='none,low', help='reasoning efforts for --compare ("" = not sent)')
    args = p.parse_args(argv)
    from .envfile import load_env_file
    used, names = load_env_file()
    if names:
        print(f'loaded {", ".join(names)} from {used}')

    from .gpt import openai_auth_available
    key = openai_auth_available()
    if args.compare:
        if not key:
            print('OPENAI_API_KEY is not set: models cannot be compared', file=sys.stderr)
            return 2
        return compare(args)
    engine = args.engine if args.engine != 'auto' else ('openai' if key else 'fallback')
    backend = fallback = None
    if engine == 'openai' or args.gpt:
        if not key:
            print('OPENAI_API_KEY is not set: OpenAI speech/GPT cannot be timed', file=sys.stderr)
            return 2
        from .gpt import OpenAIBackend
        backend = OpenAIBackend(model=args.model, tts_model=args.tts_model, voice=args.voice,
                                reasoning_effort=args.reasoning_effort, base_url=args.base_url)
    if engine == 'fallback':
        if not args.fallback_tts:
            print('no OPENAI_API_KEY and no --fallback-tts: nothing to time (try --engine instant)',
                  file=sys.stderr)
            return 2
        fallback = CommandTts(split_command(args.fallback_tts))
    tts = {'openai': backend, 'fallback': fallback, 'instant': Instant()}[engine]

    player = TimedPlayer(CommandPlayer(split_command(args.player)) if args.player != 'none' else None)
    # The narrator speaks with `backend` first; give it exactly the engine under test.
    narrator = Narrator(NarratorConfig(state_mode='template', narrate_period_s=0, use_image=False),
                        StateTracker(announce=()), backend=tts if engine != 'fallback' else None,
                        player=player, fallback_tts=fallback)
    print(f'TTS engine: {engine}' + (f' ({args.tts_model}/{args.voice})' if engine == 'openai' else '')
          + f', player: {args.player}')
    tts_times, to_play, audio = [], [], []
    try:
        for i in range(args.runs):
            text = SENTENCES[i % len(SENTENCES)]
            started = time.monotonic()
            wav = tts.synthesize(text)
            tts_times.append(time.monotonic() - started)
            player.event.clear()
            started = time.monotonic()
            narrator.say(text)                                # the whole speak path: queue, TTS, WAV, player
            if not player.event.wait(60):
                print('player never started', file=sys.stderr)
                return 1
            to_play.append(player.started[-1][0] - started)
            audio.append(wav_seconds(wav))
            while narrator.status()['speaking']:
                time.sleep(0.01)
            print(f'run {i + 1}: tts {tts_times[-1]:.2f} s, sentence -> player start {to_play[-1]:.2f} s, '
                  f'audio {audio[-1]:.1f} s', flush=True)
        gpt_times, event_to_play = [], []
        if args.gpt:
            gpt_times, event_to_play = time_gpt(args, backend, player)
    finally:
        narrator.close()
    print()
    print(summary('tts', tts_times))
    print(summary('to_play', to_play))
    print(summary('audio', audio))
    if args.gpt:
        print(summary('gpt', gpt_times))
        print(summary('event->play', event_to_play))
    return 0


PICTURE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp', '.webp')


def picture_files(path: str) -> list[str]:
    """--camera: one picture, or every picture in a folder (sorted); runs cycle through them."""
    if path and os.path.isdir(path):
        files = sorted(os.path.join(path, f) for f in os.listdir(path) if f.lower().endswith(PICTURE_EXTENSIONS))
        if not files:
            raise SystemExit(f'no pictures in {path!r}')
        return files
    return [path] if path else []


def scene_picture(path: str):
    """A picture file, or a drawn arena scene (floor, wall, green signal, two pucks) so images are exercised."""
    import cv2
    import numpy as np
    if path:
        image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f'cannot read picture {path!r}')
        return image
    image = np.full((360, 640, 3), (90, 90, 90), np.uint8)
    cv2.rectangle(image, (0, 0), (640, 150), (200, 200, 205), -1)                 # wall
    cv2.circle(image, (520, 80), 30, (40, 200, 40), -1)                           # green signal
    cv2.ellipse(image, (250, 270), (40, 18), 0, 0, 360, (200, 90, 20), -1)        # lying puck
    cv2.rectangle(image, (380, 200), (420, 280), (200, 90, 20), -1)               # standing puck
    return image


def compare(args) -> int:
    """Each model x effort answers the robot's own narration prompt with a picture; fastest first."""
    from .frames import encode_jpeg
    from .gpt import OpenAIBackend
    tracker = StateTracker(step_names={'approach': '접근'})
    narrator = Narrator(NarratorConfig(), tracker, start=False, clock=lambda: 100.0)
    tracker.update_planning('Run.Execute', 90.0)
    tracker.update_planning('Run.Execute', 100.0)
    tracker.update_diagnostics('planning/state_machine', 0, 'Run.Execute: ', {'step': 'approach'}, 100.0)
    tracker.update_motion(0.12, 0.0, 100.0)
    tracker.update_objects(['sign_green'], 100.0)
    tracker.update_pucks([(False, 0.9), (True, 1.3)], 100.0)
    files = picture_files(args.camera)
    jpegs = [(os.path.basename(f), encode_jpeg(scene_picture(f), 512, 80)) for f in files] or \
        [('drawn scene', encode_jpeg(scene_picture(''), 512, 80))]
    print('pictures: ' + ', '.join(f'{name} ({len(data) // 1024} KB JPEG)' for name, data in jpegs))
    from .narrator import Job
    instructions, prompt = narrator._prompt(Job('scene', 100.0), has_image=True)
    efforts = [e.strip() for e in args.efforts.split(',')] if args.efforts.strip() else ['']
    rows = []
    for model in [m.strip() for m in args.compare.split(',') if m.strip()]:
        for effort in efforts:
            backend = OpenAIBackend(model=model, tts_model=args.tts_model, voice=args.voice,
                                    reasoning_effort=effort, base_url=args.base_url)
            times, answer, error = [], '', ''
            for run in range(args.runs):
                name, jpeg = jpegs[run % len(jpegs)]
                started = time.monotonic()
                try:
                    answer = backend.generate(instructions, prompt, jpeg)
                    print(f'    {model} [{effort or "-"}] {name}: {time.monotonic() - started:.2f} s  "{answer[:70]}"',
                          flush=True)
                except Exception as exc:  # report and move on to the next model
                    error = f'{type(exc).__name__}: {str(exc).splitlines()[0][:90]}'
                    break
                times.append(time.monotonic() - started)
            used = backend.reasoning_effort  # after any step-down
            rows.append((statistics.median(times) if times else float('inf'), model, effort or '-',
                         used or '-', times, answer, error))
            line = (f'{model:<16} effort {effort or "-":<8} ' +
                    (f'median {statistics.median(times):5.2f} s  (min {min(times):.2f}, max {max(times):.2f}, '
                     f'n={len(times)})' if times else f'FAILED  {error}'))
            print(line, flush=True)
    print('\nfastest first (effort actually used in brackets):')
    for median, model, effort, used, times, answer, error in sorted(rows, key=lambda r: r[0]):
        if times:
            print(f'  {median:5.2f} s  {model} effort {effort} [{used}]  "{answer[:60]}"')
        else:
            print(f'    -     {model} effort {effort}  {error}')
    return 0


def time_gpt(args, backend, player):
    """GPT answer, then the event -> player start path with GPT state sentences."""
    from .demo import Camera
    files = picture_files(args.camera)
    camera = Camera(files[0] if files else 'none')
    tracker = StateTracker(cooldown_s=0.0)
    narrator = Narrator(NarratorConfig(state_mode='gpt', narrate_period_s=0, use_image=bool(files)),
                        tracker, backend=backend, player=player, frame_provider=camera.frame)
    gpt_times, event_to_play = [], []
    try:
        labels = ['Ready', 'Run.Execute', 'Run.Complete']
        for i in range(args.runs):
            player.event.clear()
            narrator.planning_state('Idle')
            started = time.monotonic()
            narrator.planning_state(labels[i % len(labels)])
            if not player.event.wait(90):
                print('player never started', file=sys.stderr)
                break
            event_to_play.append(player.started[-1][0] - started)
            while narrator.status()['speaking']:
                time.sleep(0.01)
            print(f'gpt run {i + 1}: event -> player start {event_to_play[-1]:.2f} s', flush=True)
        for i in range(args.runs):
            started = time.monotonic()
            backend.generate('한 문장으로 답해.', f'지금 상황을 한 문장으로 말해 줘. ({i})')
            gpt_times.append(time.monotonic() - started)
    finally:
        narrator.close()
        camera.close()
    return gpt_times, event_to_play


if __name__ == '__main__':
    sys.exit(main())
