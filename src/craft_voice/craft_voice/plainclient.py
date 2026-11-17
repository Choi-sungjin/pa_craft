"""A small OpenAI client on the standard library, for machines without the openai SDK.

It has only what OpenAIBackend uses, with the same call shapes as the SDK:
    client.responses.create(**request)                               -> .output_text, .status
    client.audio.speech.create(**request)                            -> .content (bytes)
    client.audio.speech.with_streaming_response.create(**request)    -> context manager, .iter_bytes(n)
    client.audio.transcriptions.create(file=(name, bytes, type), ...) -> .text
    client.with_options(timeout=, max_retries=)
Errors carry .status_code and the server's message, which is what OpenAIBackend's model and
reasoning fallbacks look at. The key is never logged or put in an error text.
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
import uuid

DEFAULT_BASE_URL = 'https://api.openai.com/v1'


class ApiError(RuntimeError):
    def __init__(self, status_code: int | None, message: str):
        super().__init__(message)
        self.status_code = status_code


class _Result:
    def __init__(self, **fields):
        self.__dict__.update(fields)


class _Stream:
    """`with ... as response: response.iter_bytes(n)` like the SDK's streaming response."""

    def __init__(self, response):
        self._response = response

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self._response.close()
        return False

    def iter_bytes(self, chunk_size: int = 4096):
        read = getattr(self._response, 'read1', self._response.read)  # read1: what has arrived, not a full chunk
        while True:
            chunk = read(chunk_size)
            if not chunk:
                return
            yield chunk


class _Responses:
    def __init__(self, client):
        self._client = client

    def create(self, extra_headers=None, **request):
        raw = self._client.post_json('/responses', request)
        text = ''.join(part.get('text', '') for item in raw.get('output') or [] if item.get('type') == 'message'
                       for part in item.get('content') or [] if part.get('type') == 'output_text')
        return _Result(output_text=text, status=raw.get('status'), incomplete_details=raw.get('incomplete_details'))


class _StreamingSpeech:
    def __init__(self, client):
        self._client = client

    def create(self, extra_headers=None, **request):
        return _Stream(self._client.open('/audio/speech', json.dumps(request).encode('utf-8'), 'application/json'))


class _Speech:
    def __init__(self, client):
        self._client = client
        self.with_streaming_response = _StreamingSpeech(client)

    def create(self, extra_headers=None, **request):
        with self._client.open('/audio/speech', json.dumps(request).encode('utf-8'), 'application/json') as response:
            return _Result(content=response.read())


class _Transcriptions:
    def __init__(self, client):
        self._client = client

    def create(self, file, model: str, extra_headers=None, **fields):
        name, data, content_type = file
        boundary = uuid.uuid4().hex
        parts = []
        for key, value in dict(fields, model=model).items():
            if value is None or value == '':
                continue
            parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{key}"\r\n\r\n{value}\r\n'
                         .encode('utf-8'))
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="file"; filename="{name}"\r\n'
                     f'Content-Type: {content_type}\r\n\r\n'.encode('utf-8') + data + b'\r\n')
        parts.append(f'--{boundary}--\r\n'.encode('utf-8'))
        with self._client.open('/audio/transcriptions', b''.join(parts),
                               f'multipart/form-data; boundary={boundary}') as response:
            body = response.read()
        try:
            return _Result(text=json.loads(body).get('text', ''))
        except ValueError:
            return _Result(text=body.decode('utf-8', errors='replace').strip())  # response_format=text


class _Audio:
    def __init__(self, client):
        self.speech = _Speech(client)
        self.transcriptions = _Transcriptions(client)


class PlainClient:
    def __init__(self, api_key: str | None = None, base_url: str = '', timeout: float = 15.0, max_retries: int = 1):
        # No key (OPENAI_AUTH=proxy): no Authorization header, the egress proxy adds its own.
        self.api_key = os.environ.get('OPENAI_API_KEY', '') if api_key is None else api_key
        self.base_url = (base_url or os.environ.get('OPENAI_BASE_URL') or DEFAULT_BASE_URL).rstrip('/')
        self.timeout = timeout
        self.max_retries = max_retries
        self.responses = _Responses(self)
        self.audio = _Audio(self)

    def with_options(self, timeout: float | None = None, max_retries: int | None = None):
        return PlainClient(self.api_key, self.base_url, self.timeout if timeout is None else timeout,
                           self.max_retries if max_retries is None else max_retries)

    def open(self, path: str, body: bytes, content_type: str):
        """POST and return the open response. Retries a refused connection, 429 and 5xx max_retries times."""
        headers = {'Content-Type': content_type, 'User-Agent': 'craft-voice'}
        if self.api_key:
            headers['Authorization'] = 'Bearer ' + self.api_key
        attempt = 0
        while True:
            request = urllib.request.Request(self.base_url + path, data=body, headers=headers, method='POST')
            try:
                return urllib.request.urlopen(request, timeout=self.timeout)
            except urllib.error.HTTPError as error:
                status = error.code
                message = self._message(error)
                retry = status == 429 or status >= 500
                failure = ApiError(status, f'Error code: {status} - {message}')
            except (urllib.error.URLError, TimeoutError, OSError) as error:
                retry = True
                reason = getattr(error, 'reason', error)
                failure = ApiError(None, f'connection failed: {type(reason).__name__}: {reason}')
            if not retry or attempt >= self.max_retries:
                raise failure from None
            attempt += 1
            time.sleep(0.4 * attempt)

    def post_json(self, path: str, payload: dict) -> dict:
        with self.open(path, json.dumps(payload).encode('utf-8'), 'application/json') as response:
            return json.loads(response.read())

    @staticmethod
    def _message(error: urllib.error.HTTPError) -> str:
        try:
            body = error.read(20000).decode('utf-8', errors='replace')
        except OSError:
            return 'no body'
        try:
            detail = json.loads(body).get('error')
            return str(detail.get('message', detail)) if isinstance(detail, dict) else str(detail or body)[:500]
        except ValueError:
            return body[:500]
