#!/usr/bin/env python3
"""Put the "아? 예?" of the path search recording in front of two SCV lines in the speech cache.

    python3 scripts/aye_lines.py --recording replanning.cedar.wav --cache-dir ~/.cache/craft_voice/speech \\
        --voice cedar --tts-model gpt-realtime-2.1 --tts-instructions "..."   [--out-dir DIR]

'아? 예? 여기 뭐 있는데요.' is kept as: the recording up to the end of "예" + a short pause + the kept speech of
'여기 뭐 있는데요.' (synthesized first if it is not kept). Run it again after the cache was emptied or the voice
changed; without it the whole sentence is ordinary synthesized speech.
"""
import argparse, io, os, sys, wave
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from craft_voice.envfile import load_env_file
from craft_voice.gpt import DEFAULT_TTS_MODEL, DEFAULT_TTS_VOICE, OpenAIBackend
from craft_voice.wiring import make_cache

LINES = ('여기 뭐 있는데요.', '안 되겠는데요, 막혔어요.')
PREFIX = '아? 예? '


def read(data: bytes):
    w = wave.open(io.BytesIO(data))
    assert w.getnchannels() == 1 and w.getsampwidth() == 2
    return np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(float), w.getframerate()


def write(samples, rate) -> bytes:
    out = io.BytesIO()
    with wave.open(out, 'wb') as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
        w.writeframes(np.round(np.clip(samples, -32767, 32767)).astype(np.int16).tobytes())
    return out.getvalue()


def opening(samples, rate, quiet_s=0.5):
    """The recording up to the end of its second burst of speech ("아", "예"), faded out."""
    step = rate // 100
    loud = np.array([np.sqrt((samples[i:i + step] ** 2).mean()) for i in range(0, len(samples) - step, step)])
    speech = loud > 0.12 * loud.max()
    bursts, start, gap = [], None, 0
    for i, on in enumerate(speech):
        if on:
            start, gap = (i if start is None else start), 0
        elif start is not None:
            gap += 1
            if gap * 0.01 >= quiet_s:
                bursts.append((start, i - gap + 1)); start = None
    if start is not None:
        bursts.append((start, len(speech)))
    assert len(bursts) >= 3, f'expected "아", "예", sentence; found {len(bursts)} parts'
    end = int((bursts[1][1] * 0.01 + 0.08) * rate)
    cut = samples[:end].copy()
    fade = int(0.04 * rate); cut[-fade:] *= np.linspace(1, 0, fade)
    return cut, [(round(a * 0.01, 2), round(b * 0.01, 2)) for a, b in bursts]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--recording', required=True)
    p.add_argument('--cache-dir', required=True)
    p.add_argument('--tts-model', default=DEFAULT_TTS_MODEL)
    p.add_argument('--voice', default=DEFAULT_TTS_VOICE)
    p.add_argument('--tts-instructions', default='')
    p.add_argument('--pause', type=float, default=0.25)
    p.add_argument('--out-dir', default='')
    args = p.parse_args()
    load_env_file()
    backend = OpenAIBackend(model='unused', tts_model=args.tts_model, voice=args.voice,
                            tts_instructions=args.tts_instructions, timeout_s=30.0)
    cache = make_cache(backend, args.cache_dir)
    head, rate = read(open(os.path.expanduser(args.recording), 'rb').read())
    head, bursts = opening(head, rate)
    print(f'recording parts {bursts}, opening {len(head) / rate:.2f}s')
    for line in LINES:
        wav = cache.get(line)
        if wav is None:
            wav = backend.synthesize(line); cache.put(line, wav)
        body, body_rate = read(wav)
        assert body_rate == rate, (body_rate, rate)
        scaled = head * (np.abs(body).max() / np.abs(head).max())      # as loud as the line
        joined = write(np.concatenate([scaled, np.zeros(int(args.pause * rate)), body]), rate)
        cache.put(PREFIX + line, joined)
        if args.out_dir:
            name = os.path.join(os.path.expanduser(args.out_dir), 'aye_' + line.rstrip('.').replace(', ', '_').replace(' ', '_') + '.wav')
            open(name, 'wb').write(joined)
        print(f'{PREFIX + line}  {len(joined) / 2 / rate:.2f}s  -> {os.path.basename(cache.path(PREFIX + line))}  '
              f'heard as {backend.transcribe(joined)!r}')
    backend.close()


if __name__ == '__main__':
    main()
