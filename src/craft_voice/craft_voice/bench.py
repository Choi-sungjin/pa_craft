"""How long from "the sentence is ready" (GPT answered) to "the speaker starts"? Measured, not guessed.

    python -m craft_voice.bench                          # OpenAI speech if OPENAI_API_KEY is set
    python -m craft_voice.bench --fallback-tts "espeak-ng -v ko --stdout -- {text}"
    python -m craft_voice.bench --gpt --camera photo.jpg # also time the GPT answer (picture + facts)
    python -m craft_voice.bench --engine instant          # only this code's own overhead (no TTS at all)

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
    p.add_argument('--model', default='gpt-5.5')
    p.add_argument('--reasoning-effort', default='low')
    p.add_argument('--tts-model', default='gpt-4o-mini-tts')
    p.add_argument('--voice', default='coral')
    p.add_argument('--base-url', default='')
    args = p.parse_args(argv)
    from .envfile import load_env_file
    used, names = load_env_file()
    if names:
        print(f'loaded {", ".join(names)} from {used}')

    key = bool(os.environ.get('OPENAI_API_KEY'))
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


def time_gpt(args, backend, player):
    """GPT answer, then the event -> player start path with GPT state sentences."""
    from .demo import Camera
    camera = Camera(args.camera or 'none')
    tracker = StateTracker(cooldown_s=0.0)
    narrator = Narrator(NarratorConfig(state_mode='gpt', narrate_period_s=0, use_image=bool(args.camera)),
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
