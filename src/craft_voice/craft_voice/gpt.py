"""OpenAI calls: image + robot state -> sentence (Responses API), sentence -> speech.

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
                 urgent_timeout_s: float = 4.0, base_url: str = '', client=None):
        if client is None:
            from openai import OpenAI  # imported here so tests and template-only runs need no SDK
            extra = {}
            if not os.environ.get('OPENAI_API_KEY') and os.environ.get('OPENAI_AUTH', '').lower() == 'proxy':
                extra = {'api_key': 'injected-by-proxy'}  # never sent: see self.request_headers
            client = OpenAI(base_url=base_url or None, timeout=timeout_s, max_retries=1, **extra)
        self.client = client
        # Proxy auth: omit Authorization on every request (the SDK only honours an omit per request), so the
        # header the egress proxy injects is the only one.
        self.request_headers = {}
        if not os.environ.get('OPENAI_API_KEY') and os.environ.get('OPENAI_AUTH', '').lower() == 'proxy':
            from openai import Omit
            self.request_headers = {'Authorization': Omit()}
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

    def synthesize(self, text: str, urgent: bool = False) -> bytes:
        """Text -> WAV bytes. Asks for raw PCM and writes the header itself.

        urgent (fault sentences): urgent_timeout_s and no retry instead of timeout_s with one retry.
        """
        request = {'model': self.tts_model, 'voice': self.voice, 'input': text, 'response_format': 'pcm'}
        if self.tts_instructions and not self.tts_model.startswith('tts-1'):
            request['instructions'] = self.tts_instructions  # tts-1 / tts-1-hd do not take instructions
        if self.tts_speed != 1.0:
            request['speed'] = self.tts_speed
        client = self.client.with_options(timeout=self.urgent_timeout_s, max_retries=0) if urgent else self.client
        response = client.audio.speech.create(**request, extra_headers=self.request_headers or None)
        pcm = response.content if hasattr(response, 'content') else response.read()
        if not pcm:
            raise EmptyResponse('speech response had no audio')
        return pcm_to_wav(pcm, self.sample_rate)
