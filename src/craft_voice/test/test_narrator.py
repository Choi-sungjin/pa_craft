import threading
import time

import numpy as np
import pytest

from craft_voice.narrator import Narrator, NarratorConfig, clean_text
from craft_voice.robot_state import StateTracker


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class Backend:
    def __init__(self, reply='앞에 상자가 보여요.', fail=False, tts_fail=False, tts_delay=0.0, urgent_delay=0.0):
        self.reply, self.fail, self.tts_fail = reply, fail, tts_fail
        self.tts_delay, self.urgent_delay = tts_delay, urgent_delay
        self.calls, self.spoken, self.urgent = [], [], []
        self.tts_started = threading.Event()

    def generate(self, instructions, prompt, jpeg=None):
        self.calls.append((instructions, prompt, jpeg))
        if self.fail:
            raise ConnectionError('offline')
        return self.reply

    def synthesize(self, text, urgent=False):
        self.urgent.append(urgent)
        self.tts_started.set()
        time.sleep(self.urgent_delay if urgent else self.tts_delay)
        if self.tts_fail:
            raise ConnectionError('tts offline')
        self.spoken.append(text)
        return b'RIFF' + text.encode()


class Player:
    """Same contract as CommandPlayer: stop() only ends a playback that is running."""

    def __init__(self, block=False):
        self.played, self.stops, self.block = [], 0, block
        self.started = threading.Event()
        self._lock = threading.Lock()
        self._running = None

    def play(self, wav, cancelled=None):
        with self._lock:
            if cancelled is not None and cancelled():
                return False
            running = self._running = threading.Event()
        self.played.append(wav)
        self.started.set()
        stopped = running.wait(5.0) if self.block else False
        with self._lock:
            self._running = None
        return not stopped

    def stop(self):
        with self._lock:
            self.stops += 1
            if self._running is not None:
                self._running.set()

    def wait_playing(self, count, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if len(self.played) >= count and self._running is not None:
                    return True
            time.sleep(0.01)
        return False


def frame():
    return np.zeros((360, 640, 3), np.uint8)


def make(backend=None, player=None, frames=frame, clock=None, start=False, **config):
    texts = []
    narrator = Narrator(NarratorConfig(**config), StateTracker(), backend=backend, player=player or Player(),
                        frame_provider=frames, on_text=lambda kind, text: texts.append((kind, text)),
                        clock=clock or Clock(), start=start)
    return narrator, texts


def run(narrator):
    """Drain both queues on this thread (start=False)."""
    while narrator.think_once() or narrator.speak_once():
        pass


def test_state_change_goes_through_gpt_with_image():
    backend = Backend('준비됐어요. 앞에 벽이 보여요.')
    narrator, texts = make(backend)
    narrator.planning_state('Idle')
    narrator.planning_state('Ready')
    run(narrator)
    instructions, prompt, jpeg = backend.calls[0]
    assert jpeg[:2] == b'\xff\xd8'
    assert 'Idle에서 Ready' in prompt and '[로봇 상태]' in prompt and '이미지 첨부됨' in prompt
    assert '지어내지 마' in instructions
    assert texts == [('state', '준비됐어요. 앞에 벽이 보여요.')]
    assert narrator.player.played == [b'RIFF' + '준비됐어요. 앞에 벽이 보여요.'.encode()]


def test_template_mode_and_no_backend_skip_gpt():
    backend = Backend()
    narrator, texts = make(backend, state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    assert backend.calls == [] and texts == [('state', '준비 완료. 미션 시작을 기다리고 있어요.')]
    narrator, texts = make(None)
    narrator.planning_state('Ready')
    run(narrator)
    assert texts == [('state', '준비 완료. 미션 시작을 기다리고 있어요.')]


def test_gpt_failure_falls_back_to_template():
    backend = Backend(fail=True)
    narrator, texts = make(backend)
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert texts == [('state', '정지하는 중이에요.')] and narrator.stats['gpt_errors'] == 1


def test_no_frame_means_no_image():
    backend = Backend()
    narrator, _ = make(backend, frames=lambda: None)
    narrator.command('describe')
    run(narrator)
    assert backend.calls[0][2] is None and '이미지 없음' in backend.calls[0][1]


def test_fault_beats_pending_state_and_skips_gpt():
    clock = Clock()
    backend = Backend()
    narrator, texts = make(backend, clock=clock)
    narrator.planning_state('Run.Execute')                    # GPT job pending
    narrator.planning_state('Fault')
    clock.now += 0.3
    narrator.diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: execution timeout', {})
    narrator.tick()
    run(narrator)
    assert backend.calls == []                                # the state job was dropped, fault is a template
    assert texts == [('fault', '문제가 생겨서 멈췄어요. 원인: 실행 시간 초과.')]


def test_state_sentence_dropped_when_newer_state_arrives():
    clock = Clock()
    backend = Backend('준비 완료!')
    narrator, texts = make(backend, clock=clock)
    narrator.planning_state('Ready')
    narrator.think_once()                                     # sentence ready, not spoken yet
    narrator.planning_state('Run.Execute')
    backend.reply = '출발할게요.'
    run(narrator)
    assert texts == [('state', '출발할게요.')]


def test_state_sentence_in_flight_is_not_spoken_after_fault():
    clock = Clock()
    narrator, texts = make(Backend('출발할게요.'), clock=clock)
    narrator.planning_state('Run.Execute')
    narrator.think_once()                                     # GPT answered just as the robot faulted
    narrator.planning_state('Fault')
    narrator.speak_once()
    assert texts == []                                        # dropped while the fault reason is pending
    clock.now += 2.0
    narrator.tick()
    run(narrator)
    assert texts == [('fault', '문제가 생겨서 멈췄어요.')]


def test_fault_drops_scene_queued_while_reason_pending():
    clock = Clock()
    narrator, texts = make(Backend('넓은 경기장이에요.'), clock=clock)
    narrator.planning_state('Fault')
    narrator.command('describe')                              # asked during the reason wait
    narrator.think_once()
    clock.now += 2.0
    narrator.tick()                                           # fault sentence (no reason) now
    run(narrator)
    assert texts == [('fault', '문제가 생겨서 멈췄어요.')]


def test_gpt_call_in_flight_when_fault_arrives():
    clock = Clock()

    class FaultDuringCall(Backend):
        def generate(self, instructions, prompt, jpeg=None):
            narrator.planning_state('Fault')                  # ROS callback while the request is out
            return super().generate(instructions, prompt, jpeg)

    narrator, texts = make(FaultDuringCall('작업을 시작할게요.'), clock=clock)
    narrator.planning_state('Run.Execute')
    run(narrator)
    assert texts == []
    clock.now += 2.0
    narrator.tick()
    run(narrator)
    assert texts == [('fault', '문제가 생겨서 멈췄어요.')]


def test_state_sentence_dropped_when_state_moved_on_unannounced():
    clock = Clock()
    narrator, texts = make(Backend('정지하는 중이에요.'), clock=clock)
    narrator.planning_state('Run.Stopping')
    narrator.think_once()
    for label in ('Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute'):  # none announced
        narrator.planning_state(label)
    run(narrator)
    assert texts == []


def test_complete_is_spoken_after_the_state_moved_on():
    backend = Backend('출발!')
    narrator, texts = make(backend)
    narrator.planning_state('Run.Execute')
    run(narrator)
    backend.reply = '작업 끝!'
    narrator.planning_state('Run.Complete')
    narrator.think_once()
    narrator.planning_state('Run.AwaitRequest')               # one loop later
    run(narrator)
    assert texts == [('state', '출발!'), ('state', '작업 끝!')]


def test_late_state_sentence_dropped():
    clock = Clock()
    narrator, texts = make(Backend(), clock=clock, state_max_age_s=8.0)
    narrator.planning_state('Ready')
    narrator.think_once()
    clock.now += 9.0
    run(narrator)
    assert texts == []


def test_budget_limits_gpt_calls():
    clock = Clock()
    backend = Backend('지피티 문장')
    narrator, texts = make(backend, clock=clock, max_gpt_calls_per_min=1, repeat_window_s=0.0)
    narrator.planning_state('Ready')
    run(narrator)
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert len(backend.calls) == 1 and texts[-1] == ('state', '정지하는 중이에요.')
    clock.now += 61.0
    narrator.planning_state('Run.Execute')
    run(narrator)
    assert len(backend.calls) == 2


def test_periodic_scene_only_when_quiet():
    clock = Clock()
    backend = Backend()
    narrator, texts = make(backend, clock=clock, scene_period_s=30.0, min_interval_s=5.0,
                           repeat_window_s=0.0)
    narrator.tick()
    run(narrator)
    assert len(backend.calls) == 1 and texts[0][0] == 'scene'
    clock.now += 10.0
    narrator.tick()
    assert narrator.think_once() is False
    clock.now += 21.0
    narrator.command('mute')
    narrator.tick()
    assert narrator.think_once() is False
    narrator.command('unmute')
    narrator.tick()
    run(narrator)
    assert len(backend.calls) == 2


def test_repeat_suppressed():
    clock = Clock()
    narrator, texts = make(Backend('같은 말'), clock=clock, repeat_window_s=30.0)
    narrator.command('describe')
    run(narrator)
    clock.now += 5.0
    narrator.command('describe')
    run(narrator)
    assert texts == [('scene', '같은 말')]


def test_ask_and_say():
    backend = Backend('앞에 빨간 퍽이 있어요.')
    narrator, texts = make(backend)
    narrator.ask('앞에 뭐 있어?')
    narrator.say('안녕하세요, 저는 CRAFT예요.')
    run(narrator)
    assert '"앞에 뭐 있어?"' in backend.calls[0][1]
    assert sorted(texts) == [('ask', '앞에 빨간 퍽이 있어요.'), ('say', '안녕하세요, 저는 CRAFT예요.')]
    narrator, texts = make(None)
    narrator.ask('뭐 보여?')
    narrator.command('describe')
    run(narrator)
    assert [kind for kind, _ in texts] == ['ask', 'scene'] and all('지금은' in t for _, t in texts)


def test_mute_keeps_faults_and_stop_clears():
    clock = Clock()
    narrator, texts = make(None, clock=clock)
    narrator.command('mute')
    narrator.planning_state('Ready')
    narrator.say('들리면 안 됨')
    narrator.planning_state('Fault')
    clock.now += 2.0
    narrator.tick()
    run(narrator)
    assert texts == [('fault', '문제가 생겨서 멈췄어요.')]
    narrator.command('unmute')
    narrator.say('하나')
    narrator.ask('둘')
    narrator.command('stop')
    run(narrator)
    assert len(texts) == 1 and narrator.player.stops == 1
    assert narrator.command('dance').startswith('unknown command')


def test_status_command_template():
    narrator, texts = make(None)
    narrator.command('status')
    run(narrator)
    assert texts == [('status', '상태머신 데이터가 들어오지 않고 있어요.')]


def test_tts_fallback_chain():
    class Fallback:
        def synthesize(self, text):
            return b'ESPEAK'

    narrator, texts = make(Backend(tts_fail=True), state_mode='template')
    narrator.fallback_tts = Fallback()
    narrator.planning_state('Ready')
    run(narrator)
    assert narrator.player.played == [b'ESPEAK']
    narrator, texts = make(Backend(tts_fail=True), state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    assert texts and narrator.player.played == [] and narrator.stats['tts_errors'] == 1


def test_threads_fault_interrupts_playback():
    """Real worker threads: a long scene sentence is playing when the robot faults."""
    clock = Clock()
    player = Player(block=True)
    narrator, texts = make(Backend('주변이 넓은 경기장이에요.'), player=player, clock=clock, start=True)
    try:
        narrator.command('describe')
        assert player.wait_playing(1)
        narrator.planning_state('Fault')
        clock.now += 2.0
        narrator.tick()
        assert player.wait_playing(2)                         # scene was cut, fault is playing
        player.stop()                                         # let the fault sentence finish
        assert [kind for kind, _ in texts] == ['scene', 'fault']
        assert narrator.backend.urgent == [False, True]       # fault speech: short timeout, no retry
    finally:
        narrator.close()


def test_threads_fault_during_speech_synthesis():
    """The scene sentence is still in TTS (no player process yet) when the fault comes: never played."""
    clock = Clock()
    player = Player(block=True)
    backend = Backend('주변이 넓은 경기장이에요.', tts_delay=0.5)
    narrator, texts = make(backend, player=player, clock=clock, start=True)
    try:
        narrator.command('describe')
        assert backend.tts_started.wait(3.0)
        narrator.planning_state('Fault')
        clock.now += 2.0
        narrator.tick()
        assert player.wait_playing(1)
        player.stop()
        assert player.played == [b'RIFF' + '문제가 생겨서 멈췄어요.'.encode()]
    finally:
        narrator.close()


def test_threads_stop_command_during_synthesis():
    player = Player(block=True)
    backend = Backend('길게 설명할게요.', tts_delay=0.5)
    narrator, texts = make(backend, player=player, start=True)
    try:
        narrator.command('describe')
        assert backend.tts_started.wait(3.0)
        narrator.command('stop')
        time.sleep(1.0)
        assert player.played == [] and texts == []
    finally:
        narrator.close()


class Fallback:
    def __init__(self):
        self.texts = []

    def synthesize(self, text):
        self.texts.append(text)
        return b'FALLBACK'


def test_threads_fault_not_delayed_by_hanging_tts():
    """Dead network: the Ready sentence's TTS hangs. The fault must not wait for it (QA N1)."""
    clock = Clock()
    player, fallback = Player(block=True), Fallback()
    backend = Backend(tts_delay=3.0)
    narrator, texts = make(backend, player=player, clock=clock, start=True, state_mode='template')
    narrator.fallback_tts = fallback
    try:
        narrator.planning_state('Ready')
        assert backend.tts_started.wait(3.0)
        narrator.planning_state('Fault')
        clock.now += 2.0
        started = time.monotonic()
        narrator.tick()
        assert player.wait_playing(1, timeout=2.0)
        assert time.monotonic() - started < 1.0
        player.stop()
        assert player.played == [b'RIFF' + '문제가 생겨서 멈췄어요.'.encode()]
        assert fallback.texts == []                           # the cancelled sentence never reached fallback
        assert texts == [('fault', '문제가 생겨서 멈췄어요.')]
    finally:
        narrator.close()


def test_threads_fault_speech_deadline_then_fallback():
    """The fault's own OpenAI speech hangs (e.g. DNS): fallback after fault_tts_deadline_s."""
    clock = Clock()
    player, fallback = Player(block=True), Fallback()
    narrator, texts = make(Backend(urgent_delay=5.0), player=player, clock=clock, start=True,
                           fault_tts_deadline_s=0.3)
    narrator.fallback_tts = fallback
    try:
        narrator.planning_state('Fault')
        clock.now += 2.0
        started = time.monotonic()
        narrator.tick()
        assert player.wait_playing(1, timeout=2.0)
        assert time.monotonic() - started < 1.0
        player.stop()
        assert player.played == [b'FALLBACK'] and narrator.stats['tts_errors'] == 1
    finally:
        narrator.close()


def test_same_label_after_unannounced_steps_is_not_the_same_state():
    """Execute(approach) queued; robot does Complete -> ... -> Execute(grasp) inside the cooldown (QA N2)."""
    clock = Clock()
    narrator, texts = make(None, clock=clock, state_mode='template')
    narrator.tracker.cooldown_s = 30.0
    narrator.planning_state('Run.Complete')
    run(narrator)
    clock.now += 11.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ', {'step': 'approach'})
    narrator.planning_state('Run.Execute')                    # announced, not spoken yet
    for label in ('Run.Complete', 'Run.AwaitRequest', 'Run.PrepareTask'):
        narrator.planning_state(label)                        # cooldown: not announced
    narrator.diagnostics('planning/state_machine', 0, 'Run.PrepareTask: ', {'step': 'grasp'})
    narrator.planning_state('Run.Compute')
    narrator.planning_state('Run.Execute')                    # cooldown: not announced
    run(narrator)
    assert texts == [('state', '작업을 마쳤어요.')]


def test_state_change_during_tts_drops_the_sentence():
    """Checked again after TTS, right before it is played and published (QA N3)."""
    class Moving(Backend):
        def synthesize(self, text, urgent=False):
            for label in ('Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute'):
                narrator.planning_state(label)
            return super().synthesize(text, urgent)

    narrator, texts = make(Moving(), state_mode='template')
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert texts == [] and narrator.player.played == []


@pytest.mark.parametrize('raw, expected', [
    ('**앞에** 상자가\n- 보여요.', '앞에 상자가 보여요.'),
    ('"따옴표 문장"', '따옴표 문장'),
    ('가' * 80 + '. ' + '나' * 200, '가' * 80 + '.'),
    ('가' * 300, '가' * 150 + '…'),
])
def test_clean_text(raw, expected):
    assert clean_text(raw, 150) == expected
