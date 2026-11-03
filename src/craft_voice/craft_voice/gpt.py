"""OpenAI calls: image + robot state -> sentence (Responses API), sentence -> speech.

The key comes from OPENAI_API_KEY (never from a parameter file). Models are
parameters because OpenAI retires them: the SDK this was written against
(openai 3.42.1) marks every audio.speech model, gpt-4o-mini-tts included,
with an announced shutdown date of 2027-01-06.
"""

from __future__ import annotations

import base64
import io
import wave


class EmptyResponse(RuntimeError):
    """The model answered without text (e.g. max_output_tokens spent on reasoning)."""


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


class OpenAIBackend:
    def __init__(self, model: str, tts_model: str, voice: str, tts_instructions: str = '',
                 image_detail: str = 'low', max_output_tokens: int = 400, reasoning_effort: str = '',
                 tts_speed: float = 1.0, sample_rate: int = 24000, timeout_s: float = 15.0,
                 urgent_timeout_s: float = 4.0, base_url: str = '', client=None):
        if client is None:
            from openai import OpenAI  # imported here so tests and template-only runs need no SDK
            client = OpenAI(base_url=base_url or None, timeout=timeout_s, max_retries=1)
        self.client = client
        self.model = model
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
        request = {'model': self.model, 'instructions': instructions, 'store': False,
                   'input': [{'role': 'user', 'content': content}],
                   'max_output_tokens': self.max_output_tokens}
        if self.reasoning_effort:
            request['reasoning'] = {'effort': self.reasoning_effort}
        try:
            response = self.client.responses.create(**request)
        except Exception as error:
            # A model without reasoning support rejects the field: drop it for good, retry once.
            if not self.reasoning_effort or 'reasoning' not in str(error).lower():
                raise
            self.reasoning_effort = ''
            request.pop('reasoning')
            response = self.client.responses.create(**request)
        text = (getattr(response, 'output_text', '') or '').strip()
        if not text:
            raise EmptyResponse(f'no text in response (status {getattr(response, "status", "?")}, '
                                f'{getattr(response, "incomplete_details", None)})')
        return text

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
        response = client.audio.speech.create(**request)
        pcm = response.content if hasattr(response, 'content') else response.read()
        if not pcm:
            raise EmptyResponse('speech response had no audio')
        return pcm_to_wav(pcm, self.sample_rate)
