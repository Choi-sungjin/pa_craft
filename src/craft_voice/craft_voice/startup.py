"""음성 API가 실제 응답한 뒤 시작 녹음을 한 번 재생한다. ROS/로봇 제어와 독립적이다."""

import io
import threading
import wave


def check_voice_api(backend):
    if not backend.generate('연결 확인이야. 준비 라고만 답해.', '준비됐어?').strip():
        raise ValueError('empty text response')
    audio = backend.synthesize('준비됐어.')
    with wave.open(io.BytesIO(audio)) as wav:
        expected = wav.getnframes() * wav.getnchannels() * wav.getsampwidth()
        if not expected or len(wav.readframes(wav.getnframes())) != expected:
            raise ValueError('empty or truncated speech response')
    if not backend.transcribe(audio, 'ko', '').strip():
        raise ValueError('empty transcription response')


class StartupAnnouncement:
    def __init__(self, probe, announce, ready, logger, retry_s=15.0):
        self.probe, self.announce, self.ready, self.log = probe, announce, ready, logger
        self.retry_s = retry_s
        self.api_ready = False
        self.played = False
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name='voice-startup', daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self):
        while not self._stop.is_set():
            try:
                if self.ready():
                    if not self.api_ready:
                        self.probe()
                        if self._stop.is_set():
                            return
                        self.api_ready = True
                        self.log.info('voice API ready: text, speech and transcription checked')
                    done, outcome = threading.Event(), []

                    def finished(played):
                        outcome.append(played)
                        done.set()

                    if self.announce(finished):
                        while not done.wait(0.1):
                            if self._stop.is_set():
                                return
                        if outcome[0] is None or self._stop.is_set():
                            return  # 명시적인 stop/mute/종료로 취소됨
                        if outcome[0]:
                            self.played = True
                            self.log.info('startup melody played once')
                            return
            except Exception as error:
                self.log.warning(f'voice startup waiting: {type(error).__name__}')
            self._stop.wait(self.retry_s)
