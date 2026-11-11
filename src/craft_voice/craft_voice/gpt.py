"""OpenAI calls: image + robot state -> sentence (Responses API), sentence -> speech, speech -> text.

Speech comes from the audio/speech endpoint, or from the Realtime API when tts_model is a Realtime
model (e.g. gpt-realtime-2.1 with the voice marin). Without the openai SDK installed a small
standard-library client is used instead (plainclient.py), so the robot needs no extra package.

The key comes from OPENAI_API_KEY (never from a parameter file). Models are
parameters because OpenAI retires them: the SDK this was written against
(openai 3.42.1) marks every audio.speech model, gpt-4o-mini-tts included,
with an announced shutdown date of 2027-01-06.
"""

from __future__ import annotations

import base64

from .audio import pcm_to_wav  # noqa: F401  (re-exported)


import os


def openai_auth_available() -> bool:
    """A key in the environment, or OPENAI_AUTH=proxy: an egress proxy adds the Authorization header itself
    (e.g. a Claude Code cloud "Network secret" for api.openai.com), so the key never enters this machine."""
    return bool(os.environ.get('OPENAI_API_KEY')) or os.environ.get('OPENAI_AUTH', '').lower() == 'proxy'


# Speech: the Realtime model and voice the robot talks with. Speech-to-text: fastest that got every test
# sentence right (2026-11-11: gpt-transcribe 0.62 s, gpt-4o-mini-transcribe 0.70 s, whisper-1 1.48 s median).
DEFAULT_TTS_MODEL = 'gpt-realtime-2.1'
DEFAULT_TTS_VOICE = 'marin'
DEFAULT_STT_MODEL = 'gpt-transcribe,gpt-4o-mini-transcribe,whisper-1'

# A model that rejects reasoning.effort=X is tried with the next lighter-weight value, then without it.
REASONING_FALLBACK = {'none': 'minimal', 'minimal': 'low', 'low': ''}


def model_unusable(error: Exception) -> bool:
    """The model itself cannot serve this request (unknown/not allowed model, no image input)."""
    status = getattr(error, 'status_code', None)
    text = str(error).lower()
    return status in (400, 403, 404) and 'reasoning' not in text and (
        'model' in text or 'image' in text or 'vision' in text)


class EmptyResponse(RuntimeError):
    """The model answered without text (e.g. max_output_tokens spent on reasoning)."""


class OpenAIBackend:
    def __init__(self, model: str, tts_model: str, voice: str, tts_instructions: str = '',
                 image_detail: str = 'low', max_output_tokens: int = 400, reasoning_effort: str = '',
                 tts_speed: float = 1.0, sample_rate: int = 24000, timeout_s: float = 15.0,
                 urgent_timeout_s: float = 4.0, base_url: str = '', client=None,
                 stt_model: str = DEFAULT_STT_MODEL, realtime=None):
        proxy = not os.environ.get('OPENAI_API_KEY') and os.environ.get('OPENAI_AUTH', '').lower() == 'proxy'
        self.request_headers = {}
        if client is None:
            try:
                from openai import OpenAI  # imported here so tests and template-only runs need no SDK
            except ImportError:
                from .plainclient import PlainClient  # no SDK on this machine: standard library only
                client = PlainClient(base_url=base_url, timeout=timeout_s, max_retries=1)
            else:
                extra = {'api_key': 'injected-by-proxy'} if proxy else {}  # never sent: see request_headers
                client = OpenAI(base_url=base_url or None, timeout=timeout_s, max_retries=1, **extra)
                if proxy:
                    # Proxy auth: omit Authorization on every request (the SDK only honours an omit per
                    # request), so the header the egress proxy injects is the only one.
                    from openai import Omit
                    self.request_headers = {'Authorization': Omit()}
        self.client = client
        # "fast-model,fallback-model,...": the first that works is kept for the rest of the run.
        self.models = [m.strip() for m in model.split(',') if m.strip()] or ['gpt-5.5']
        self.model = self.models[0]
        self.tts_model = tts_model
        self.voice = voice
        self.tts_instructions = tts_instructions
        self.image_detail = image_detail
        self.max_output_tokens = max_output_tokens
        self.reasoning_effort = reasoning_effort
        self.tts_speed = tts_speed
        self.sample_rate = sample_rate
        self.urgent_timeout_s = urgent_timeout_s
        self.stt_models = [m.strip() for m in stt_model.split(',') if m.strip()] or ['whisper-1']
        self.stt_model = self.stt_models[0]
        # tts_model gpt-realtime-* / gpt-live-*: the Realtime API speaks (WebSocket), not audio/speech.
        self.realtime = realtime
        if self.realtime is None:
            from .realtime import RealtimeSpeech, is_realtime_model
            if is_realtime_model(tts_model):
                self.realtime = RealtimeSpeech(tts_model, voice, tts_instructions, base_url=base_url,
                                               timeout_s=timeout_s, urgent_timeout_s=urgent_timeout_s)
        if self.realtime is not None:
            self.sample_rate = self.realtime.sample_rate

    def warm(self):
        """A sentence is on its way: open the speech connection now (Realtime speech only)."""
        if self.realtime is not None:
            self.realtime.warm()

    def close(self):
        if self.realtime is not None:
            self.realtime.close()

    def speech_signature(self) -> str:
        """Everything that changes how a sentence sounds, for the speech cache."""
        return '|'.join((self.tts_model, self.voice, self.tts_instructions, f'{self.tts_speed:g}'))

    def transcribe(self, wav: bytes, language: str = 'ko', prompt: str = '') -> str:
        """Speech (WAV bytes) -> text. The next stt model is used if this key cannot use one."""
        while True:
            fields = {'model': self.stt_model, 'file': ('speech.wav', wav, 'audio/wav'), 'response_format': 'json'}
            if language:
                fields['language'] = language
            if prompt:
                fields['prompt'] = prompt
            try:
                result = self.client.audio.transcriptions.create(**fields, extra_headers=self.request_headers or None)
                return (getattr(result, 'text', '') or '').strip()
            except Exception as error:
                index = self.stt_models.index(self.stt_model)
                if not model_unusable(error) or index + 1 >= len(self.stt_models):
                    raise
                self.stt_model = self.stt_models[index + 1]

    def generate(self, instructions: str, text: str, jpeg: bytes | None = None) -> str:
        content = [{'type': 'input_text', 'text': text}]
        if jpeg:
            content.append({'type': 'input_image', 'detail': self.image_detail,
                            'image_url': 'data:image/jpeg;base64,' + base64.b64encode(jpeg).decode('ascii')})
        request = {'instructions': instructions, 'store': False,
                   'input': [{'role': 'user', 'content': content}],
                   'max_output_tokens': self.max_output_tokens}
        while True:
            request['model'] = self.model
            try:
                response = self._create(request)
                break
            except Exception as error:
                index = self.models.index(self.model)
                if not model_unusable(error) or index + 1 >= len(self.models):
                    raise
                self.model = self.models[index + 1]  # e.g. model not available to this key, or no image input
        text = (getattr(response, 'output_text', '') or '').strip()
        if not text:
            raise EmptyResponse(f'no text in response (status {getattr(response, "status", "?")}, '
                                f'{getattr(response, "incomplete_details", None)})')
        return text

    def _create(self, request: dict):
        """responses.create, stepping reasoning.effort down (none -> minimal -> low -> unset) if rejected."""
        while True:
            if self.reasoning_effort:
                request['reasoning'] = {'effort': self.reasoning_effort}
            else:
                request.pop('reasoning', None)
            try:
                return self.client.responses.create(**request, extra_headers=self.request_headers or None)
            except Exception as error:
                if not self.reasoning_effort or 'reasoning' not in str(error).lower():
                    raise
                self.reasoning_effort = REASONING_FALLBACK.get(self.reasoning_effort, '')

    def _speech_request(self, text: str) -> dict:
        request = {'model': self.tts_model, 'voice': self.voice, 'input': text, 'response_format': 'pcm'}
        if self.tts_instructions and not self.tts_model.startswith('tts-1'):
            request['instructions'] = self.tts_instructions  # tts-1 / tts-1-hd do not take instructions
        if self.tts_speed != 1.0:
            request['speed'] = self.tts_speed
        return request

    def stream_speech(self, text: str, chunk_size: int = 4096):
        """Yield raw PCM (16-bit mono, self.sample_rate) as the speech endpoint produces it.

        The first chunk comes well before the whole sentence is synthesized (measured: median 0.80 s vs 1.44 s).
        """
        if self.realtime is not None:
            yield from self.realtime.stream(text)
            return
        speech = self.client.audio.speech.with_streaming_response
        with speech.create(**self._speech_request(text), extra_headers=self.request_headers or None) as response:
            yield from response.iter_bytes(chunk_size)

    def synthesize(self, text: str, urgent: bool = False) -> bytes:
        """Text -> WAV bytes. Asks for raw PCM and writes the header itself.

        urgent (fault sentences): urgent_timeout_s and no retry instead of timeout_s with one retry.
        """
        if self.realtime is not None:
            return self.realtime.synthesize(text, urgent=urgent)
        request = self._speech_request(text)
        client = self.client.with_options(timeout=self.urgent_timeout_s, max_retries=0) if urgent else self.client
        response = client.audio.speech.create(**request, extra_headers=self.request_headers or None)
        pcm = response.content if hasattr(response, 'content') else response.read()
        if not pcm:
            raise EmptyResponse('speech response had no audio')
        return pcm_to_wav(pcm, self.sample_rate)
