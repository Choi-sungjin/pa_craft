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
    def __init__(self, reply='앞에 상자가 보여요.', fail=False, tts_fail=False, tts_delay=0.0, urgent_delay=0.0,
                 gpt_delay=0.0):
        self.reply, self.fail, self.tts_fail, self.gpt_delay = reply, fail, tts_fail, gpt_delay
        self.tts_delay, self.urgent_delay = tts_delay, urgent_delay
        self.calls, self.spoken, self.urgent = [], [], []
        self.tts_started = threading.Event()

    def generate(self, instructions, prompt, jpeg=None):
        self.calls.append((instructions, prompt, jpeg))
        time.sleep(self.gpt_delay)
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
    assert 'Idle -> Ready' in prompt and '[로봇 상태]' in prompt and '이미지 첨부됨' in prompt
    assert '[센서가 확인한 것]' in prompt and '- 지금 하는 일: 출발 신호를 기다리고 있어요.' in prompt
    assert '지어내지 마' in instructions and '오류 코드' in instructions
    assert texts == [('state', '준비됐어요. 앞에 벽이 보여요.')]
    assert narrator.player.played == [b'RIFF' + '준비됐어요. 앞에 벽이 보여요.'.encode()]


def test_template_mode_and_no_backend_skip_gpt():
    backend = Backend()
    narrator, texts = make(backend, state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    assert backend.calls == [] and texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')]
    narrator, texts = make(None)
    narrator.planning_state('Ready')
    run(narrator)
    assert texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')]


def test_gpt_failure_falls_back_to_template():
    backend = Backend(fail=True)
    narrator, texts = make(backend)
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert texts == [('state', '멈추는 중이에요.')] and narrator.stats['gpt_errors'] == 1


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
    assert texts == [('fault', '목표까지 너무 오래 걸려서 멈춰 있어요.')]


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
    assert texts == [('fault', '지금은 멈춰 있어요.')]


def test_fault_drops_scene_queued_while_reason_pending():
    clock = Clock()
    narrator, texts = make(Backend('넓은 경기장이에요.'), clock=clock)
    narrator.planning_state('Fault')
    narrator.command('describe')                              # asked during the reason wait
    narrator.think_once()
    clock.now += 2.0
    narrator.tick()                                           # fault sentence (no reason) now
    run(narrator)
    assert texts == [('fault', '지금은 멈춰 있어요.')]


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
    assert texts == [('fault', '지금은 멈춰 있어요.')]


def test_state_sentence_dropped_when_state_moved_on_unannounced():
    clock = Clock()
    narrator, texts = make(Backend('멈추는 중이에요.'), clock=clock)
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
    assert len(backend.calls) == 1 and texts[-1] == ('state', '멈추는 중이에요.')
    clock.now += 61.0
    narrator.planning_state('Run.Execute')
    run(narrator)
    assert len(backend.calls) == 2


def test_periodic_scene_only_when_quiet():
    clock = Clock()
    backend = Backend()
    narrator, texts = make(backend, clock=clock, narrate_idle_period_s=30.0, min_interval_s=5.0,
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


def test_repeat_suppressed_for_announcements_not_for_narration():
    clock = Clock()
    narrator, texts = make(None, clock=clock, repeat_window_s=30.0, state_mode='template')
    narrator.tracker.cooldown_s = 0.0
    narrator.planning_state('Ready')
    run(narrator)
    clock.now += 5.0
    narrator.planning_state('Idle')
    narrator.planning_state('Ready')                          # same announcement again within 30 s
    run(narrator)
    assert texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')]
    narrator.command('describe')
    run(narrator)
    clock.now += 5.0
    narrator.command('describe')                              # asked twice: said twice
    run(narrator)
    assert [kind for kind, _ in texts] == ['state', 'scene', 'scene']


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
    assert texts == [('ask', '지금은 그 질문에 답하기 어려워요. 아직 제 작업 상태를 받지 못했어요. 주행 프로그램이 꺼져 있는 것 같아요.'),
                     ('scene', '아직 제 작업 상태를 받지 못했어요. 주행 프로그램이 꺼져 있는 것 같아요.')]


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
    assert texts == [('fault', '지금은 멈춰 있어요.')]
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
    assert texts == [('status', '아직 제 작업 상태를 받지 못했어요. 주행 프로그램이 꺼져 있는 것 같아요.')]


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
        assert player.played == [b'RIFF' + '지금은 멈춰 있어요.'.encode()]
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
        assert player.played == [b'RIFF' + '지금은 멈춰 있어요.'.encode()]
        assert fallback.texts == []                           # the cancelled sentence never reached fallback
        assert texts == [('fault', '지금은 멈춰 있어요.')]
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
    assert texts == [('state', '작업 하나를 끝냈어요.')]


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


# ---- dead network: slow failures must still end in speech (QA round 3) -----------------------------
def test_dead_network_template_state_goes_out_through_fallback():
    """OpenAI speech fails only after a long wait; the robot is still Ready, so say it via fallback."""
    fallback = Fallback()
    narrator, texts = make(Backend(tts_delay=0.6, tts_fail=True), clock=time.monotonic, state_mode='template',
                           state_max_age_s=0.3)
    narrator.fallback_tts = fallback
    narrator.planning_state('Ready')
    run(narrator)
    assert texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')]
    assert narrator.player.played == [b'FALLBACK']


def test_dead_network_gpt_state_falls_back_to_template_in_time():
    narrator, texts = make(Backend(gpt_delay=0.6, fail=True), clock=time.monotonic, state_max_age_s=0.3)
    narrator.planning_state('Ready')
    run(narrator)
    assert texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')]


def test_hanging_gpt_and_tts_hit_their_deadlines():
    fallback = Fallback()
    narrator, texts = make(Backend(gpt_delay=5.0, tts_delay=5.0), clock=time.monotonic,
                           gpt_state_deadline_s=0.3, tts_deadline_s=0.3)
    narrator.fallback_tts = fallback
    started = time.monotonic()
    narrator.planning_state('Ready')
    run(narrator)
    assert time.monotonic() - started < 2.0
    assert texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')] and narrator.player.played == [b'FALLBACK']


def test_requested_scene_failure_is_still_said():
    narrator, texts = make(Backend(gpt_delay=0.6, fail=True), clock=time.monotonic, scene_max_age_s=0.3)
    narrator.command('describe')
    run(narrator)
    assert texts == [('scene', '아직 제 작업 상태를 받지 못했어요. 주행 프로그램이 꺼져 있는 것 같아요.')]   # what it does, in fixed words


def test_status_answer_refreshed_when_state_changes_during_tts():
    """QA round 3 #3: a status sentence describes "now"; if the state moved on, say the current one."""
    class Moving(Backend):
        move = False

        def synthesize(self, text, urgent=False):
            if self.move:
                self.move = False
                for label in ('Run.Complete', 'Run.AwaitRequest', 'Run.Compute', 'Run.Execute'):
                    narrator.planning_state(label)
            return super().synthesize(text, urgent)

    backend = Moving()
    narrator, texts = make(backend, state_mode='template')
    narrator.tracker.cooldown_s = 60.0
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert texts == [('state', '멈추는 중이에요.')]
    texts.clear()
    narrator.tracker.announce = ()                            # only the status answer is under test
    backend.move = True
    narrator.command('status')                                # "멈추는 중이에요." goes to TTS ...
    run(narrator)
    assert texts == [('status', '작업하고 있어요.')]          # ... robot moved on: the current state is said


def test_close_during_tts_publishes_nothing():
    """QA round 3 #4."""
    narrator, texts = make(Backend(tts_delay=0.5), state_mode='template', start=True)
    narrator.planning_state('Ready')
    assert narrator.backend.tts_started.wait(2.0)
    narrator.close()
    time.sleep(0.8)
    assert texts == []


def test_failing_callbacks_do_not_stop_speech():
    def boom(*args):
        raise UnicodeEncodeError('cp1252', '가', 0, 1, 'cannot encode')

    narrator = Narrator(NarratorConfig(state_mode='template'), StateTracker(), backend=Backend(), player=Player(),
                        on_text=boom, on_speaking=boom, start=False)
    narrator.planning_state('Ready')
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert len(narrator.player.played) == 1 and narrator.stats['spoken'] == 1
    narrator.say('또 말해요')
    run(narrator)
    assert len(narrator.player.played) == 2


def test_narrates_what_it_does_periodically_only_while_working():
    clock = Clock()
    narrator, texts = make(None, clock=clock, narrate_period_s=20.0, min_interval_s=5.0, state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)                                             # "준비됐어요..." announced
    for _ in range(3):
        clock.now += 10.0
        narrator.planning_state('Ready')
        narrator.tick()
        run(narrator)
    assert [kind for kind, _ in texts] == ['state']           # idle: no periodic narration by default
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ', {'step': 'approach'})
    narrator.planning_state('Run.Execute')
    narrator.motion(0.12, 0.0)
    narrator.objects(['sign_green'])
    run(narrator)
    clock.now += 6.0
    narrator.planning_state('Run.Execute')
    narrator.motion(0.12, 0.0)
    narrator.objects(['sign_green'])
    narrator.tick()
    run(narrator)
    assert texts[-1] == ('scene', '작업하고 있어요. 지금 보이는 건 초록 신호 1개예요.')  # step id never read
    clock.now += 10.0                                         # within narrate_period_s: quiet
    narrator.planning_state('Run.Execute')
    narrator.tick()
    assert narrator.think_once() is False and narrator.speak_once() is False


def test_sensors_reach_the_gpt_prompt():
    backend = Backend('목표로 가는 중이고 앞에 초록 신호가 보여요.')
    narrator, texts = make(backend)
    narrator.planning_state('Run.Execute')
    narrator.motion(0.12, 0.0)
    narrator.objects(['sign_green'])
    narrator.pucks([(False, 0.8)])
    narrator.tags([5])
    run(narrator)
    prompt = backend.calls[-1][1]
    assert '[센서가 확인한 것]' in prompt and '앞으로 초속 0.12 m로 이동 중' in prompt
    assert '초록 신호 1개' in prompt and '퍽 1개(누운 것 1' in prompt and 'AprilTag 5번' in prompt
    assert texts == [('state', '목표로 가는 중이고 앞에 초록 신호가 보여요.')]


# ---- QA round on narration (e42da8f) ----------------------------------------------------------------
class SlowNarration(Backend):
    """GPT that takes `scene_delay` seconds for narrations only, and can run a hook while it works."""

    def __init__(self, scene_delay=0.0, hook=None, **kwargs):
        super().__init__(**kwargs)
        self.scene_delay, self.hook = scene_delay, hook

    def generate(self, instructions, prompt, jpeg=None):
        if '지금 무엇을 하고 있는지 먼저' in prompt:
            if self.hook:
                self.hook()
            time.sleep(self.scene_delay)
            self.calls.append((instructions, prompt, jpeg))
            return '목표 지점으로 이동하고 있어요.'
        return super().generate(instructions, prompt, jpeg)


@pytest.mark.parametrize('mode', ['gpt', 'template'])
def test_narration_written_before_a_fault_or_state_change_is_not_spoken(mode):
    clock = Clock()

    def change():
        narrator.diagnostics('planning/state_machine', 2, 'Fault: Run.Execute: localization lost', {})
        narrator.planning_state('Fault')

    def complete():
        narrator.planning_state('Run.Complete')

    backend = SlowNarration(hook=change if mode == 'gpt' else complete, reply='상태 문장')
    narrator, texts = make(backend, clock=clock, state_mode=mode)
    narrator.planning_state('Run.Execute')
    run(narrator)
    texts.clear()
    clock.now += 6.0
    narrator.planning_state('Run.Execute')
    narrator.tick()                                           # periodic narration -> GPT, situation changes
    clock.now += 2.0
    narrator.tick()
    run(narrator)
    assert all(kind != 'scene' for kind, _ in texts), texts
    expected = ('fault', '제 위치를 놓쳐서 멈춰 있어요.') if mode == 'gpt' else ('state', '작업 하나를 끝냈어요.')
    assert expected in texts


def test_threads_state_change_does_not_wait_for_a_slow_narration():
    """QA F2: one think thread; a narration hanging in GPT must not hold the state announcement."""
    clock = Clock()
    player = Player(block=False)
    backend = SlowNarration(scene_delay=5.0, reply='출발할게요.')
    narrator, texts = make(backend, player=player, clock=clock, start=True)   # state sentences via GPT too
    try:
        narrator.planning_state('Run.Execute')
        deadline = time.monotonic() + 3.0
        while ('state', '출발할게요.') not in texts and time.monotonic() < deadline:
            time.sleep(0.01)
        while narrator.status()['speaking'] and time.monotonic() < deadline:
            time.sleep(0.01)
        backend.reply = '이제 멈출게요.'
        clock.now += 6.0
        narrator.planning_state('Run.Execute')
        narrator.tick()                                       # narration starts, GPT hangs 5 s
        time.sleep(0.3)
        started = time.monotonic()
        narrator.planning_state('Run.Stopping')               # its GPT job must not queue behind the narration
        while ('state', '이제 멈출게요.') not in texts and time.monotonic() - started < 4.0:
            time.sleep(0.02)
        assert time.monotonic() - started < 1.0, texts
    finally:
        narrator.close()


def test_threads_state_change_cuts_a_playing_narration():
    clock = Clock()
    player = Player(block=True)
    narrator, texts = make(Backend('목표로 가고 있어요.'), player=player, clock=clock, start=True,
                           state_mode='template')
    try:
        narrator.planning_state('Run.Execute')
        assert player.wait_playing(1)
        player.stop()
        deadline = time.monotonic() + 3.0
        while narrator.status()['speaking'] and time.monotonic() < deadline:
            time.sleep(0.01)                                  # let the speak thread finish the first sentence
        clock.now += 6.0
        narrator.planning_state('Run.Execute')
        narrator.tick()                                       # narration plays
        assert player.wait_playing(2)
        narrator.planning_state('Run.Stopping')               # what I am doing changed: cut it
        assert player.wait_playing(3)
        player.stop()
        assert [kind for kind, _ in texts] == ['state', 'scene', 'state']
    finally:
        narrator.close()


def test_narration_gpt_deadline_falls_back_to_fixed_words():
    narrator, texts = make(SlowNarration(scene_delay=3.0), clock=time.monotonic, gpt_narrate_deadline_s=0.3,
                           state_mode='template')
    narrator.planning_state('Ready')
    run(narrator)
    started = time.monotonic()
    narrator.command('describe')
    run(narrator)
    assert time.monotonic() - started < 1.5 and texts[-1] == ('scene', '출발 신호를 기다리고 있어요.')


def test_fixed_narration_keeps_its_period_without_gpt():
    """QA F4: identical fixed sentences are not dropped as repeats, so it really speaks every period."""
    clock = Clock()
    narrator, texts = make(None, clock=clock, narrate_period_s=20.0, min_interval_s=5.0, state_mode='template')
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ', {'step': 'base_goal'})
    narrator.planning_state('Run.Execute')
    run(narrator)
    for _ in range(1200):                                     # 120 s at 10 Hz
        clock.now += 0.1
        narrator.planning_state('Run.Execute')
        narrator.tick()
        run(narrator)
    assert sum(1 for kind, _ in texts if kind == 'scene') == 6


# ---- QA round 2 on narration (6a76917) ----------------------------------------------------------------
class SlowAnswers(Backend):
    """GPT where describe/status/ask answers take `delay` s; state sentences are quick."""

    def __init__(self, delay, **kwargs):
        super().__init__(**kwargs)
        self.delay = delay

    def generate(self, instructions, prompt, jpeg=None):
        self.calls.append((instructions, prompt, jpeg))
        if '바뀌었어' in prompt:
            return '이제 멈출게요.'
        time.sleep(self.delay)
        return '오래 걸린 답이에요.'


def wait_for(texts, item, seconds=3.0):
    deadline = time.monotonic() + seconds
    while item not in texts and time.monotonic() < deadline:
        time.sleep(0.01)
    return item in texts


@pytest.mark.parametrize('command', ['describe', 'status'])
def test_threads_requested_answer_does_not_block_state_announcement(command):
    clock = Clock()
    narrator, texts = make(SlowAnswers(delay=12.0), player=Player(), clock=clock, start=True)
    try:
        narrator.planning_state('Run.Execute')
        assert wait_for(texts, ('state', '이제 멈출게요.'))
        texts.clear()
        narrator.command(command)                             # GPT hangs 12 s on it
        time.sleep(0.3)
        started = time.monotonic()
        narrator.planning_state('Run.Stopping')
        assert wait_for(texts, ('state', '이제 멈출게요.'), 1.5)
        kind = 'scene' if command == 'describe' else 'status'
        assert (kind, '멈추는 중이에요.') in texts               # answered right away with the new situation
        assert time.monotonic() - started < 1.5 and ('scene', '오래 걸린 답이에요.') not in texts
    finally:
        narrator.close()


def test_threads_question_in_gpt_does_not_hold_the_announcement():
    clock = Clock()
    narrator, texts = make(SlowAnswers(delay=1.5), player=Player(), clock=clock, start=True)
    try:
        narrator.ask('앞에 뭐 있어?')
        time.sleep(0.2)
        narrator.planning_state('Run.Stopping')               # think thread busy with the question
        assert wait_for(texts, ('state', '멈추는 중이에요.'), 1.0)  # fixed words at once
        assert wait_for(texts, ('ask', '오래 걸린 답이에요.'), 3.0)   # the question is still answered
    finally:
        narrator.close()


def test_question_deadline():
    narrator, texts = make(SlowAnswers(delay=3.0), clock=time.monotonic, gpt_ask_deadline_s=0.3)
    started = time.monotonic()
    narrator.ask('앞에 뭐 있어?')
    run(narrator)
    assert time.monotonic() - started < 1.5
    assert texts == [('ask', '지금은 그 질문에 답하기 어려워요. 아직 제 작업 상태를 받지 못했어요. 주행 프로그램이 꺼져 있는 것 같아요.')]


def test_threads_stop_discards_what_gpt_is_writing():
    narrator, texts = make(SlowAnswers(delay=0.8), player=Player(), start=True)
    try:
        narrator.command('describe')
        time.sleep(0.2)
        narrator.command('stop')
        time.sleep(1.5)
        assert texts == []
    finally:
        narrator.close()


def test_refreshed_answer_respects_mute():
    """The status answer is in TTS when the person mutes and the situation changes: no refresh is said."""
    class MuteDuringTts(Backend):
        def synthesize(self, text, urgent=False):
            narrator.command('mute')
            narrator.planning_state('Run.Complete')
            return super().synthesize(text, urgent)

    narrator, texts = make(MuteDuringTts(), state_mode='template')
    narrator.tracker.announce = ()
    narrator.planning_state('Run.Execute')
    narrator.command('status')
    run(narrator)
    assert texts == [] and narrator.player.played == []


def test_state_sentence_is_written_before_waiting_answers():
    backend = Backend('답')
    narrator, texts = make(backend)
    narrator.tracker.announce = ('Ready',)
    narrator.command('status')                                # queued first
    narrator.planning_state('Ready')                          # but the announcement expires: GPT does it first
    narrator.think_once()
    assert '바뀌었어' in backend.calls[0][1]


def test_status_answer_survives_quick_state_changes():
    """Run.Complete -> AwaitRequest -> PrepareTask in 100 ms must not leave the asker without an answer."""
    class Moving(Backend):
        def synthesize(self, text, urgent=False):
            for label in ('Run.Complete', 'Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute'):
                narrator.planning_state(label)
            return super().synthesize(text, urgent)

    narrator, texts = make(Moving(), state_mode='template')
    narrator.tracker.announce = ()
    narrator.planning_state('Run.Execute')
    narrator.command('status')
    run(narrator)
    assert texts and texts[-1][0] == 'status'


def test_refreshed_answer_tells_the_situation_at_the_moment_it_is_said():
    class OnceMoving(Backend):
        moved = False

        def synthesize(self, text, urgent=False):
            if not self.moved:
                self.moved = True
                narrator.planning_state('Run.Complete')        # the first answer goes stale in TTS
            return super().synthesize(text, urgent)

    narrator, texts = make(OnceMoving(), state_mode='template')
    narrator.tracker.announce = ()
    narrator.planning_state('Run.Execute')
    narrator.command('status')
    narrator.speak_once()                                     # stale after TTS -> refresh queued
    narrator.planning_state('Run.Stopping')                   # changes again before the refresh is said
    run(narrator)
    assert texts == [('status', '멈추는 중이에요.')]


# ---- streamed speech (B) ---------------------------------------------------------------------------------
class StreamingBackend(Backend):
    def __init__(self, chunks=(b'aa', b'bb', b'cc'), first_delay=0.0, chunk_delay=0.0, fail=None, **kwargs):
        super().__init__(**kwargs)
        self.chunks, self.first_delay, self.chunk_delay, self.fail = chunks, first_delay, chunk_delay, fail
        self.streams = []
        self.sample_rate = 24000

    def stream_speech(self, text):
        self.streams.append(text)
        time.sleep(self.first_delay)
        if self.fail:
            raise self.fail
        for i, chunk in enumerate(self.chunks):
            if i:
                time.sleep(self.chunk_delay)
            yield chunk


class FakeStreamPlayer:
    def __init__(self, block=False):
        self.opened, self.data, self.finished, self.stops, self.block = [], [], 0, 0, block
        self._stop = threading.Event()

    def open(self, rate, cancelled=None):
        if cancelled is not None and cancelled():
            return False
        self._stop.clear()
        self.opened.append(rate)
        return True

    def write(self, chunk):
        if self._stop.is_set():
            return False
        self.data.append(chunk)
        return True

    def finish(self):
        self.finished += 1
        if self.block:
            return not self._stop.wait(5.0)
        return not self._stop.is_set()

    def stop(self):
        self.stops += 1
        self._stop.set()


def make_streaming(backend, stream, **config):
    texts = []
    narrator = Narrator(NarratorConfig(**config), StateTracker(), backend=backend, player=Player(),
                        stream_player=stream, on_text=lambda kind, text: texts.append((kind, text)),
                        clock=Clock(), start=False)
    return narrator, texts


def test_speech_is_streamed_in_order():
    backend, stream = StreamingBackend(reply='출발할게요.'), FakeStreamPlayer()
    narrator, texts = make_streaming(backend, stream)
    narrator.planning_state('Run.Execute')
    run(narrator)
    assert texts == [('state', '출발할게요.')] and backend.streams == ['출발할게요.']
    assert stream.opened == [24000] and stream.data == [b'aa', b'bb', b'cc'] and stream.finished == 1
    assert narrator.player.played == [] and narrator.stats['spoken'] == 1   # the WAV player was not used


@pytest.mark.parametrize('case', ['timeout', 'error'])
def test_stream_without_first_audio_falls_back(case):
    backend = StreamingBackend(first_delay=1.0) if case == 'timeout' else \
        StreamingBackend(fail=ConnectionError('offline'))
    stream = FakeStreamPlayer()
    narrator, texts = make_streaming(backend, stream, tts_deadline_s=0.2, state_mode='template')
    narrator.fallback_tts = type('F', (), {'synthesize': lambda self, text: b'FALLBACK'})()
    narrator.planning_state('Ready')
    run(narrator)
    assert texts == [('state', '준비됐어요. 출발 신호를 기다리고 있어요.')]
    assert narrator.player.played == [b'FALLBACK'] and stream.opened == [] and narrator.stats['tts_errors'] == 1


def test_state_change_before_first_audio_drops_the_sentence():
    class Moving(StreamingBackend):
        def stream_speech(self, text):
            narrator.planning_state('Run.Compute')            # the robot moved on while TTS started
            yield from super().stream_speech(text)

    stream = FakeStreamPlayer()
    narrator, texts = make_streaming(Moving(), stream, state_mode='template')
    narrator.planning_state('Run.Stopping')
    run(narrator)
    assert texts == [] and stream.opened == []


def test_fault_cuts_a_streaming_sentence_and_is_not_streamed():
    clock = Clock()
    backend = StreamingBackend(chunks=(b'a',) * 50, chunk_delay=0.05)
    stream = FakeStreamPlayer(block=True)
    texts = []
    narrator = Narrator(NarratorConfig(state_mode='template'), StateTracker(), backend=backend, player=Player(),
                        stream_player=stream, on_text=lambda kind, text: texts.append((kind, text)),
                        clock=clock, start=True)
    try:
        narrator.planning_state('Run.Execute')
        deadline = time.monotonic() + 3
        while not stream.data and time.monotonic() < deadline:
            time.sleep(0.01)
        narrator.planning_state('Fault')
        clock.now += 2.0
        narrator.tick()
        deadline = time.monotonic() + 3
        while not narrator.player.played and time.monotonic() < deadline:
            time.sleep(0.01)
        assert stream.stops >= 1 and len(stream.data) < 50          # cut off mid-sentence
        assert [kind for kind, _ in texts] == ['state', 'fault']
        assert narrator.player.played and narrator.player.played[-1].startswith(b'RIFF')  # fault: whole WAV
        assert backend.urgent and backend.urgent[-1] is True
    finally:
        narrator.close()
