"""Synthesize every fixed sentence once and keep it in the speech cache.

    python -m craft_voice.prepare --style cute                # all fixed sentences of the persona
    python -m craft_voice.prepare --style cute --list         # only show them

After this the fixed sentences (state changes, blocked path, faults, "응!") play at once and also
when the network is down. Sentences written by GPT are still synthesized when they are said.
"""

from __future__ import annotations

import argparse
import sys

from .gpt import DEFAULT_TTS_MODEL, DEFAULT_TTS_VOICE
from .robot_state import FAULT_CUTE, PHRASES, REASON_KO, STEP_KO, StateTracker, parse_mapping
from .signals import TRAFFIC_BLINKS
from .wiring import DEFAULT_CACHE_DIR, load_lines, make_cache


def fixed_sentences(style: str, lines: dict, step_names: dict, variety: bool = False) -> list[str]:
    """Every sentence the robot can say without GPT, for this style."""
    tracker = StateTracker(style=style, lines=lines, step_names=step_names, variety=variety)
    steps = sorted({*STEP_KO.values(), *step_names.values()})
    sentences = set()
    for key in {*PHRASES['polite'], *PHRASES.get(style, {}), *lines}:
        for text in tracker.phrases(key):  # every wording of the key
            if '{step}' in text:
                sentences.update(text.format(step=step) for step in steps)
            elif '{' not in text and text:
                sentences.add(text)
    # The traffic light result: 12 blinks, each colour 1..6 times (TrafficLight.valid), so every count is known.
    for text in tracker.phrases('notice.traffic_done'):
        for red in range(1, 7):
            for yellow in range(1, 7):
                green = TRAFFIC_BLINKS - red - yellow
                if 1 <= green <= 6 and text:
                    sentences.add(text.format(red=red, yellow=yellow, green=green))
    from .chatter import all_lines as chatter_lines
    sentences.update(chatter_lines(style))   # 혼잣말(켜져 있지 않아도 미리 합성해 둠)
    if variety:
        from . import variety as drive
        sentences.update(drive.all_lines(style))
        going = tracker.phrases('doing.Run.Execute.step')[0]
        for step_id in drive.DRIVE_STEPS:  # "지금 목표 지점 이동 중이야! 3미터쯤 남았어."
            name = step_names.get(step_id, STEP_KO.get(step_id, ''))
            if name and '{step}' in going:
                sentences.update(f'{going.format(step=name)} {drive.left_line(style, d)}' for d in range(drive.MAX_M + 1))
    for needle, _, polite in REASON_KO:
        sentences.add(lines.get('fault.' + needle) or (FAULT_CUTE.get(needle, polite) if style == 'cute' else polite))
    # "what I am doing" answers pair a cannot-answer with a doing sentence only at run time: not prepared
    return sorted(sentences)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog='python -m craft_voice.prepare', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--style', choices=['polite', 'cute'], default='polite')
    p.add_argument('--phrases-file', default='')
    p.add_argument('--variety', action='store_true', help='also the further wordings and the drive sentences')
    p.add_argument('--step-names', default='')
    p.add_argument('--cache-dir', default=DEFAULT_CACHE_DIR)
    p.add_argument('--tts-model', default=DEFAULT_TTS_MODEL)
    p.add_argument('--voice', default=DEFAULT_TTS_VOICE)
    p.add_argument('--tts-instructions', default='밝고 또렷한 한국어 안내 로봇 목소리로, 조금 빠르게 말해.')
    p.add_argument('--base-url', default='')
    p.add_argument('--list', action='store_true', help='print the sentences and stop')
    p.add_argument('--force', action='store_true', help='synthesize again even if already kept')
    args = p.parse_args(argv)

    import logging
    log = logging.getLogger('craft_voice')
    sentences = fixed_sentences(args.style, load_lines(args.phrases_file, log), parse_mapping(args.step_names),
                                args.variety)
    if args.list:
        print('\n'.join(sentences))
        return 0
    from .envfile import load_env_file
    load_env_file()
    from .gpt import OpenAIBackend, openai_auth_available
    if not openai_auth_available():
        print('OPENAI_API_KEY is not set (see env/openai.env.example)', file=sys.stderr)
        return 2
    backend = OpenAIBackend(model='unused', tts_model=args.tts_model, voice=args.voice,
                            tts_instructions=args.tts_instructions, base_url=args.base_url, timeout_s=30.0)
    cache = make_cache(backend, args.cache_dir)
    made = kept = failed = 0
    for index, text in enumerate(sentences, 1):
        if not args.force and cache.get(text) is not None:
            kept += 1
            continue
        try:
            cache.put(text, backend.synthesize(text))
            made += 1
            print(f'[{index}/{len(sentences)}] {text}', flush=True)
        except Exception as error:
            failed += 1
            print(f'[{index}/{len(sentences)}] FAILED {text}: {type(error).__name__}: {error}', file=sys.stderr, flush=True)
    backend.close()
    print(f'{made} synthesized, {kept} already kept, {failed} failed -> {cache.directory}')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
