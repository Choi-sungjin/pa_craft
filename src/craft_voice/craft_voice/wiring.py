"""Pieces shared by the ROS node and the laptop demo: speech cache, recorded reactions, microphone listener."""

from __future__ import annotations

import os

from .audio import SpeechCache, read_clip, split_command
from .robot_state import load_phrases, parse_list, parse_mapping

DEFAULT_MIC_COMMAND = 'pw-record --rate 16000 --channels 1 --format s16 -'   # raw 16 kHz mono PCM on stdout
DEFAULT_CACHE_DIR = '~/.cache/craft_voice/speech'


def make_cache(backend, directory: str):
    """Speech cache for this backend's voice, or None (no backend, or directory '')."""
    if backend is None or not directory.strip():
        return None
    signature = backend.speech_signature() if hasattr(backend, 'speech_signature') else 'speech'
    return SpeechCache(directory, signature)


def load_lines(phrases_file: str, log) -> dict:
    """Sentences from a phrases file ({} if there is none or it cannot be read)."""
    if not phrases_file.strip():
        return {}
    try:
        return load_phrases(phrases_file)
    except (OSError, ValueError) as error:
        log.error(f'phrases file {phrases_file!r} not used: {type(error).__name__}: {error}')
        return {}


def load_reactions(text: str, log) -> dict:
    """'replan=/path/a.wav, run_done=~/b.wav' -> {event: path}; recordings that cannot be played are left out."""
    reactions = {}
    for event, path in parse_mapping(text, '=').items():
        path = os.path.expanduser(path)
        try:
            read_clip(path)
        except Exception as error:
            log.warning(f'reaction {event}: recording {path!r} not used ({type(error).__name__}: {error})')
            continue
        reactions[event] = path
    return reactions


def make_listener(narrator, backend, log, mic_command: str = DEFAULT_MIC_COMMAND, wake_words: str = '크래프트',
                  language: str = 'ko', stt_prompt: str = '', follow_up_s: float = 6.0, tail_s: float = 0.6,
                  max_stt_per_min: int = 20, tolerance: int = 2, min_rms: float = 150.0, loud_ratio: float = 2.5,
                  on_heard=None):
    """Microphone listener that calls narrator.wake(question); None when it cannot work (no key, no command)."""
    from .listen import Listener, Microphone, Segmenter, webrtc_vad
    command = split_command(mic_command)
    if backend is None or not hasattr(backend, 'transcribe'):
        log.warning('listen: no OpenAI backend, so no speech-to-text; the microphone stays off')
        return None
    if not command:
        log.warning('listen: mic_command is empty; the microphone stays off')
        return None
    vad = webrtc_vad()
    if vad is None:
        log.info('listen: webrtcvad not installed, speech is found by loudness over the noise floor only')

    def wake(question: str):
        if on_heard:
            on_heard(question)
        narrator.wake(question)

    return Listener(Microphone(command), lambda wav: backend.transcribe(wav, language, stt_prompt), wake,
                    on_followup=narrator.followup if follow_up_s > 0 else None,
                    names=tuple(parse_list(wake_words)) or ('크래프트',),
                    segmenter=Segmenter(vad=vad, min_rms=min_rms, loud_ratio=loud_ratio),
                    is_speaking=lambda: narrator.audible, tail_s=tail_s, follow_up_s=follow_up_s,
                    max_stt_per_min=max_stt_per_min, tolerance=tolerance, logger=log)
