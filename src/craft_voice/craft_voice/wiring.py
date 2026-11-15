"""Pieces shared by the ROS node and the laptop demo: speech cache, recorded reactions, microphone listener."""

from __future__ import annotations

import os

from .audio import SpeechCache, read_clip, split_command
from .clock import clock_unset
from .robot_state import load_phrases, parse_list, parse_mapping

DEFAULT_MIC_COMMAND = 'pw-record --rate 16000 --channels 1 --format s16 -'   # raw 16 kHz mono PCM on stdout
DEFAULT_CACHE_DIR = '~/.cache/craft_voice/speech'


def make_cache(backend, directory: str, max_files: int = 500):
    """Speech cache for this backend's voice, or None (no backend, or directory '')."""
    if backend is None or not directory.strip():
        return None
    signature = backend.speech_signature() if hasattr(backend, 'speech_signature') else 'speech'
    return SpeechCache(directory, signature, max_files=max_files)


def prepare_in_background(make_backend, cache, sentences, log, stop, pause_s: float = 0.3):
    """Synthesize the fixed sentences the cache lacks, one by one on a thread of its own, with a backend of its
    own (the narrator's warm connection stays the narrator's). Started at boot: after one pass every fixed sentence
    plays from disk. Returns the thread (None when nothing is missing)."""
    import threading
    import time
    missing = [text for text in sentences if not cache.contains(text)]
    log.info(f'speech cache: {len(sentences) - len(missing)}/{len(sentences)} fixed sentences kept, '
             f'{len(missing)} to synthesize in the background')
    if not missing:
        return None

    def work():
        backend, made, failed = make_backend(), 0, 0
        for text in missing:
            if stop():
                break
            if cache.contains(text):  # said (and kept) meanwhile
                continue
            try:
                cache.put(text, backend.synthesize(text))
                made += 1
            except Exception as error:  # the model read something else, the network is down, ...
                failed += 1
                log.warning(f'prepare: {text!r} not kept ({type(error).__name__}: {error})')
            if (made + failed) % 50 == 0:
                log.info(f'prepare: {made} synthesized, {failed} failed, {len(missing) - made - failed} left')
            time.sleep(pause_s)
        close = getattr(backend, 'close', None)
        if close:
            close()
        log.info(f'prepare done: {made} synthesized, {failed} failed')

    thread = threading.Thread(target=work, name='voice-prepare', daemon=True)
    thread.start()
    return thread


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
    """'replan=/path/a.wav, run_done=~/b.wav, trapped=~/c.wav|~/d.wav' -> {event: path ('a|b' = one of them)};
    recordings that cannot be played are left out."""
    reactions = {}
    for event, spec in parse_mapping(text, '=').items():
        usable = []
        for path in spec.split('|'):
            path = os.path.expanduser(path.strip())
            try:
                read_clip(path)
            except Exception as error:
                log.warning(f'reaction {event}: recording {path!r} not used ({type(error).__name__}: {error})')
                continue
            usable.append(path)
        if usable:
            reactions[event] = '|'.join(usable)
    return reactions


def load_word_reactions(spec: str, log) -> tuple[dict, dict]:
    """'젯슨|제슨=~/멜로디.wav|~/대답.wav;다른말=~/c.wav'. 유효한 녹음만 후보로 등록한다."""
    words, pools = {}, {}
    for entry in (spec or '').split(';'):
        names, _, paths = entry.partition('=')
        aliases = tuple(a.strip() for a in names.split('|') if a.strip())
        if not aliases or not paths.strip():
            continue
        key = aliases[0]
        clips = list(pools.get(key, ()))
        for path in paths.split('|'):
            if not path.strip():
                continue
            loaded = load_reactions(f'word.{key}={path.strip()}', log)
            clip = loaded.get('word.' + key)
            if clip and clip not in clips:
                clips.append(clip)
        if clips:
            pools[key] = tuple(clips)
            words[key] = tuple(dict.fromkeys(words.get(key, ()) + aliases))
            log.info(f'word {key!r} ({", ".join(words[key])}) -> {len(clips)} recordings')
    return words, pools


def make_listener(narrator, backend, log, mic_command: str = DEFAULT_MIC_COMMAND, wake_words: str = '크래프트',
                  language: str = 'ko', stt_prompt: str = '', follow_up_s: float = 6.0, tail_s: float = 0.6,
                  max_stt_per_min: int = 20, tolerance: int = 2, min_rms: float = 150.0, loud_ratio: float = 2.5,
                  on_heard=None, words=None, stt_peak: float = 0.0, stt_max_gain: float = 10.0,
                  log_heard: bool = False, save_dir: str = ''):
    """Microphone listener that calls narrator.wake(question); None when it cannot work (no key, no command).

    words: 반응 단어 {'젯슨': ('젯슨', '제슨', ...)} -> narrator.word('젯슨') (녹음 재생)."""
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
                    max_stt_per_min=max_stt_per_min, tolerance=tolerance, logger=log,
                    ready=lambda: not clock_unset(), words=words or {},
                    on_word=narrator.word if words else None,
                    stt_peak=stt_peak, stt_max_gain=stt_max_gain, log_heard=log_heard,
                    save_dir=os.path.expanduser(save_dir) if save_dir else '')
