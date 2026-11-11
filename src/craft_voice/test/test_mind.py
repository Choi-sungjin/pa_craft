"""Reactions without a state change, memory, noticing, the persona, and answering when called by name."""
import io
import json
import threading
import time
import wave

from test_narrator import Backend, Clock, Player, make

from craft_voice.audio import SpeechCache, pcm_to_wav
from craft_voice.narrator import Narrator, NarratorConfig
from craft_voice.robot_state import StateTracker, fault_situation, load_phrases, waiting_situation

BLOCKED = 'path blocked; waiting before replanning'
REPLAN = 'path still blocked; replanning'


def diag(tracker, now, waiting='', step='base_goal', message='Run.Execute: ', queued='0', task_id='7'):
    return tracker.update_diagnostics('planning/state_machine', 0, message,
                                      {'step': step, 'task_id': task_id, 'queued': queued, 'waiting_for': waiting}, now)


def driving(tracker, now=0.0):
    """Ready -> ... -> Run.Execute with its diagnostics, like a goal being driven to."""
    for offset, label in enumerate(('Ready', 'Run.AwaitRequest', 'Run.PrepareTask', 'Run.Compute', 'Run.Execute')):
        tracker.update_planning(label, now + offset * 0.1)
    diag(tracker, now + 0.6)
    return now + 0.6


class WavBackend(Backend):
    """Like Backend, but the speech is a real (silent) WAV, as the speech cache checks what it keeps."""

    def synthesize(self, text, urgent=False):
        super().synthesize(text, urgent)
        return pcm_to_wav(b'\x00\x00' * 2400, 24000)


def wav_file(path, seconds=0.2):
    with wave.open(str(path), 'wb') as out:
        out.setnchannels(1)
        out.setsampwidth(2)
        out.setframerate(16000)
        out.writeframes(b'\x10\x00' * int(16000 * seconds))
    return str(path)


# ---- situations: things that happen without a label change -------------------------------------------------
def test_waiting_values_map_to_situations():
    assert waiting_situation(BLOCKED)[0] == 'blocked' and waiting_situation(REPLAN)[0] == 'replan'
    assert waiting_situation('box: perceiving')[0] == 'box_perceive'
    assert waiting_situation('route unavailable (no path); waiting to retry')[0] == 'no_route'
    assert waiting_situation('') == ('', '') and waiting_situation('something new') == ('', '')


def test_blocked_then_replanning_are_events_once_each():
    t = StateTracker()
    now = driving(t)
    generation = t.generation
    event = diag(t, now + 1.0, BLOCKED)
    assert event and event.situation == 'blocked' and event.label == 'Run.Execute'
    assert t.template(event) == '길이 막혀서 잠깐 기다리고 있어요.' and t.generation == generation + 1
    assert diag(t, now + 2.0, BLOCKED) is None                    # the same situation again: nothing new
    assert t.doing(now + 2.0) == '길이 막혀서 잠깐 기다리고 있어요.'
    event = diag(t, now + 3.0, REPLAN)
    assert event and event.situation == 'replan' and t.template(event) == '길이 계속 막혀서 다른 길을 찾고 있어요.'
    t.update_planning('Run.Execute', now + 4.0)                    # the label keeps coming at 20 Hz
    assert diag(t, now + 4.0, '') is None and t.current_situation(now + 4.0) == ''   # clear again: not announced
    assert t.doing(now + 4.0) == '목표 지점 이동 중이에요.'
    assert t.generation == generation + 3                         # ... but what it does did change each time
    assert diag(t, now + 5.0, BLOCKED) is None                    # blocked again within the cooldown: not said twice
    assert diag(t, now + 6.0, '') is None
    assert diag(t, now + 30.0, BLOCKED) is not None


def test_other_waiting_values_are_context_not_announcements():
    t = StateTracker()
    now = driving(t)
    generation = t.generation
    assert diag(t, now + 1.0, 'box: perceiving', step='box_approach') is None
    assert t.generation == generation                             # an announcement in the works stays valid
    assert t.doing(now + 1.0) == '상자 위치를 확인하고 있어요.'
    assert '지금 상황: 상자 위치를 센서로 확인하는 중' in t.context_text(now + 1.5)


def test_situation_needs_fresh_diagnostics_of_the_current_step():
    t = StateTracker()
    now = driving(t)
    diag(t, now + 1.0, BLOCKED)
    t.update_planning('Run.Execute', now + 9.0)                    # the label keeps coming, diagnostics stopped
    assert t.current_situation(now + 9.0) == '' and t.doing(now + 9.0) == '목표 지점 이동 중이에요.'
    t.update_planning('Fault', now + 10.0)
    assert diag(t, now + 10.5, BLOCKED, message='Fault: Run.Execute: no progress') is None
    assert t.situation == ''                                       # a fault is told as the fault, not as waiting


def test_clear_can_be_announced_if_wanted():
    t = StateTracker(announce_situations=('replan', 'blocked', 'clear'))
    now = driving(t)
    diag(t, now + 1.0, BLOCKED)
    event = diag(t, now + 3.0, '')
    assert event and event.situation == 'clear' and t.template(event) == '길이 다시 열려서 계속 갈게요.'


# ---- persona -------------------------------------------------------------------------------------------------
def test_cute_style_and_a_phrases_file(tmp_path):
    t = StateTracker(style='cute')
    now = driving(t)
    assert t.doing(now) == '지금 목표 지점 이동 중이야!'
    event = diag(t, now + 1.0, REPLAN)
    assert t.template(event) == '예? 새로운 경로를 찾아볼게!'
    assert fault_situation('Run.Execute: localization lost', 'cute') == '내 위치를 놓쳐서 멈췄어.'
    assert fault_situation('something odd', 'cute') == '지금은 멈춰 있어. 확인이 필요해.'
    assert fault_situation('box L1: the dock or back-out path stays blocked (obstacle or padding)') == \
        '상자 앞 길이 계속 막혀서 멈춰 있어요.'
    assert fault_situation('camera dock ended before target gap') == '카메라로 간격을 맞추지 못해서 멈춰 있어요.'
    phrases = tmp_path / 'lines.json'
    phrases.write_text(json.dumps({'doing.Ready': '출발만 기다려!', 'situation.replan': '새 길 찾는다!',
                                   'fault.localization lost': '어디지? 길을 잃었어.', 'empty': ' '}), encoding='utf-8')
    lines = load_phrases(str(phrases))
    assert 'empty' not in lines
    t = StateTracker(style='cute', lines=lines)
    t.update_planning('Ready', 0.0)
    assert t.doing(0.0) == '출발만 기다려!'
    assert fault_situation('localization lost', 'cute', lines) == '어디지? 길을 잃었어.'
    assert t.phrase('wake') == '짜스!' and StateTracker().phrase('wake') == '네!'


# ---- memory ----------------------------------------------------------------------------------------------------
def test_memory_lists_what_happened_with_how_long_ago():
    t = StateTracker()
    now = driving(t)
    diag(t, now + 20.0, BLOCKED)
    diag(t, now + 25.0, REPLAN)
    diag(t, now + 40.0, '')
    t.update_planning('Run.Complete', now + 100.0)
    text = t.memory_text(now + 130.0)
    lines = text.splitlines()
    assert any('준비 완료' in line for line in lines) and any('길이 막혀서 잠깐 기다리는 중' in line for line in lines)
    assert any(line.startswith('- 30초 전: 작업 하나 완료(목표 지점 이동)') for line in lines)
    assert lines[-1] == '- 이번 실행에서: 끝낸 작업 1번, 길 막힘 1번, 경로 다시 계산 1번'
    assert lines.index(next(line for line in lines if '길이 다시 열림' in line)) > \
        lines.index(next(line for line in lines if '경로를 다시 계산' in line))
    assert t.last_done == {'step': 'base_goal', 'queued': 0, 'at': now + 100.0}


# ---- noticing -----------------------------------------------------------------------------------------------------
def test_notices_fire_once_per_episode():
    t = StateTracker(notice_compute_s=8.0, notice_blocked_s=20.0, notice_replans=3)
    t.update_planning('Run.Compute', 0.0)
    t.update_planning('Run.Compute', 5.0)
    assert t.notice(5.0) is None
    t.update_planning('Run.Compute', 9.0)
    noticed = t.notice(9.0)
    assert noticed and noticed.key == 'long_compute' and '9초째' in noticed.facts
    t.update_planning('Run.Compute', 9.5)
    assert t.notice(10.0) is None                                   # said once for this stretch
    now = driving(t, 20.0)
    diag(t, now + 1.0, BLOCKED)
    for step in range(2, 40):                                        # label at 20 Hz in reality: keep it fresh
        t.update_planning('Run.Execute', now + step)
        diag(t, now + step, BLOCKED)
        noticed = t.notice(now + step)
        if noticed:
            break
    assert noticed.key == 'long_blocked' and 21 <= step <= 23
    assert t.notice(now + step + 1) is None


def test_many_replans_and_lost_state_are_noticed():
    t = StateTracker(notice_replans=3, notice_blocked_s=0.0, notice_lost_s=5.0)
    now = driving(t)
    for i in range(3):
        diag(t, now + 1 + i * 2, REPLAN)
        t.update_planning('Run.Execute', now + 1 + i * 2)
        diag(t, now + 2 + i * 2, '')
    noticed = t.notice(now + 6.5)
    assert noticed and noticed.key == 'many_replans' and '3번' in noticed.facts
    assert t.notice(now + 7.0) is None
    lost = t.notice(now + 6.0 + 6.0)                                 # no /planning/state for 6 s: the stack is gone
    assert lost and lost.key == 'state_lost'
    assert t.notice(now + 30.0) is None                              # ... said once
    t.update_planning('Run.Execute', now + 40.0)
    assert t.notice(now + 40.1).key == 'state_back'


def test_idle_is_noticed_every_period():
    t = StateTracker(notice_idle_s=1800.0, notice_lost_s=0.0)
    t.update_planning('Ready', 0.0)
    for minute in range(1, 61):
        t.update_planning('Ready', minute * 60.0)
        noticed = t.notice(minute * 60.0)
        assert (noticed is not None) == (minute in (30, 60))
        if noticed:
            assert noticed.key == 'idle'


# ---- narrator: called by name ----------------------------------------------------------------------------------------
def test_the_name_alone_gets_an_ack_then_what_it_is_doing_from_gpt():
    backend = Backend(reply='목표 지점으로 가는 중이야!')
    narrator, texts = make(backend=backend)
    narrator.planning_state('Run.Execute')
    narrator._jobs.clear()
    narrator.wake('')
    assert narrator.speak_once() is True and texts == [('ack', '네!')]   # at once, before GPT has answered
    assert narrator.think_once() is True and narrator.speak_once() is True
    assert texts[-1] == ('status', '목표 지점으로 가는 중이야!')
    instructions, prompt, jpeg = backend.calls[-1]
    assert '누가 방금 내 이름을 불렀어' in prompt and jpeg is None       # about the state: no picture, sooner
    assert '[기억]' in prompt and '작업 실행' in prompt
    assert '사람이 "(이름을 부름)"라고 해서 "목표 지점으로 가는 중이야!"라고 대답함' in narrator.tracker.memory_text(narrator.clock())


def test_a_question_by_voice_is_answered_and_remembered_for_the_next_one():
    backend = Backend(reply='길이 막혀서 기다렸어.')
    narrator, texts = make(backend=backend)
    narrator.planning_state('Run.Execute')
    narrator._jobs.clear()
    narrator.wake('왜 멈췄어?')
    assert narrator.speak_once() and texts[-1] == ('ack', '잠깐만요.')
    assert narrator.think_once() and narrator.speak_once()
    assert texts[-1] == ('ask', '길이 막혀서 기다렸어.')
    assert '"왜 멈췄어?"' in backend.calls[-1][1] and '추론해서 답하고' in backend.calls[-1][1]
    backend.reply = '한 십 초쯤.'
    narrator.followup('얼마나?')
    assert narrator.think_once() and narrator.speak_once()
    prompt = backend.calls[-1][1]
    assert '[최근 대화]' in prompt and '- 사람: 왜 멈췄어? / 나: 길이 막혀서 기다렸어.' in prompt
    assert 'SKIP 라고만 답해' in prompt and texts[-1] == ('ask', '한 십 초쯤.')


def test_a_follow_up_not_meant_for_the_robot_gets_no_answer():
    backend = Backend(reply='SKIP')
    narrator, texts = make(backend=backend)
    narrator.followup('아 배고프다')
    assert narrator.think_once() is True and narrator.speak_once() is False and texts == []
    backend.fail = True                                               # nor a non-answer when GPT is unreachable
    narrator.followup('아 배고프다')
    assert narrator.think_once() is True and narrator.speak_once() is False and texts == []


def test_the_name_without_gpt_is_answered_with_fixed_words_at_once():
    narrator, texts = make(backend=None)
    narrator.tracker.style = 'cute'
    narrator.planning_state('Ready')
    narrator._utterances.clear()
    narrator.wake('')
    assert narrator.speak_once() is True and texts == [('status', '준비 끝! 출발 신호 기다리는 중이야.')]
    narrator.wake('왜 멈췄어?')
    assert narrator.speak_once() is True and texts[-1] == ('status', '그건 지금 잘 모르겠어. 준비 끝! 출발 신호 기다리는 중이야.')


def test_template_wake_mode_skips_gpt_for_the_name_alone():
    backend = Backend()
    narrator, texts = make(backend=backend, wake_mode='template')
    narrator.planning_state('Ready')
    narrator._jobs.clear()
    narrator.wake('')
    assert narrator.speak_once() is True and texts == [('status', '출발 신호를 기다리고 있어요.')] and not backend.calls


def test_a_slow_gpt_answer_to_the_name_becomes_fixed_words():
    backend = Backend(gpt_delay=1.0)
    narrator, texts = make(backend=backend, gpt_voice_deadline_s=0.1, wake_ack=False)
    narrator.planning_state('Ready')
    narrator._jobs.clear()
    narrator.wake('')
    started = time.monotonic()
    assert narrator.think_once() is True and time.monotonic() - started < 0.6
    assert narrator.speak_once() is True and texts == [('status', '출발 신호를 기다리고 있어요.')]


def test_muted_robot_does_not_answer_its_name():
    backend = Backend()
    narrator, texts = make(backend=backend)
    narrator.command('mute')
    narrator.wake('뭐 해?')
    assert narrator.think_once() is False and narrator.speak_once() is False and texts == []


# ---- narrator: situations, reactions, cache, thoughts -------------------------------------------------------------------
def test_a_blocked_path_is_said_at_once_without_gpt():
    backend = Backend()
    narrator, texts = make(backend=backend)
    for label in ('Ready', 'Run.PrepareTask', 'Run.Execute'):
        narrator.planning_state(label)
    narrator._jobs.clear()
    narrator.clock.now += 1.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                         {'step': 'base_goal', 'task_id': '3', 'queued': '0', 'waiting_for': BLOCKED})
    assert narrator.speak_once() is True and texts == [('state', '길이 막혀서 잠깐 기다리고 있어요.')]
    assert narrator.think_once() is False and backend.calls == []
    narrator.clock.now += 1.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                         {'step': 'base_goal', 'task_id': '3', 'queued': '0', 'waiting_for': REPLAN})
    narrator.clock.now += 0.2
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                         {'step': 'base_goal', 'task_id': '3', 'queued': '0', 'waiting_for': ''})
    assert narrator.speak_once() is True and len(texts) == 1          # the path cleared first: the sentence is dropped


def test_a_reaction_goes_before_a_pending_answer_but_after_a_fault():
    backend = Backend(reply='목표 지점으로 가는 중이야.')
    narrator, texts = make(backend=backend)
    for label in ('Ready', 'Run.PrepareTask', 'Run.Execute'):
        narrator.planning_state(label)
    narrator._jobs.clear()
    narrator._utterances.clear()
    narrator.wake('어디 가?')
    assert narrator.think_once() is True                               # ack and answer are waiting to be said
    narrator.clock.now += 1.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ',
                         {'step': 'base_goal', 'task_id': '3', 'queued': '0', 'waiting_for': REPLAN})
    for _ in range(3):
        narrator.speak_once()
    assert [kind for kind, _ in texts] == ['state', 'ack', 'ask']
    assert texts[0][1] == '길이 계속 막혀서 다른 길을 찾고 있어요.'


def test_recordings_play_instead_of_sentences_for_their_events(tmp_path):
    replan, cheer, finale = (wav_file(tmp_path / name) for name in ('replan.wav', 'cheer.wav', 'finale.wav'))
    backend = Backend()
    player = Player()
    narrator, texts = make(backend=backend, player=player,
                           reactions={'replan': replan, 'step_done': cheer, 'run_done': finale,
                                      'ready': str(tmp_path / 'missing.wav')})
    narrator.tracker.announce_situations = ('replan',)
    values = {'step': 'base_goal', 'task_id': '3', 'queued': '1', 'waiting_for': ''}
    narrator.planning_state('Ready')                                    # its recording is missing: the sentence
    narrator._jobs.clear()
    narrator.speak_once()
    assert backend.spoken == ['준비됐어요. 출발 신호를 기다리고 있어요.']
    for label in ('Run.PrepareTask', 'Run.Execute'):
        narrator.planning_state(label)
    narrator._jobs.clear()
    narrator.clock.now += 1.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ', dict(values, waiting_for=REPLAN))
    assert narrator.speak_once() is True
    assert player.played[-1] == open(replan, 'rb').read() and texts[-1][0] == 'state'
    narrator.clock.now += 1.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ', values)   # a step with one more queued
    narrator.planning_state('Run.Complete')
    assert narrator.speak_once() is True and player.played[-1] == open(cheer, 'rb').read()
    narrator.clock.now += 20.0
    for label in ('Run.PrepareTask', 'Run.Execute'):
        narrator.planning_state(label)
    narrator._jobs.clear()
    narrator.clock.now += 1.0
    narrator.diagnostics('planning/state_machine', 0, 'Run.Execute: ', dict(values, queued='0'))
    narrator.planning_state('Run.Complete')                             # nothing queued behind it: the run is over
    assert narrator.speak_once() is True and player.played[-1] == open(finale, 'rb').read()
    assert len(backend.spoken) == 1 and backend.calls == []             # recordings need no GPT and no TTS


def test_fixed_sentences_are_synthesized_once_and_then_come_from_the_cache(tmp_path):
    cache = SpeechCache(str(tmp_path / 'speech'), 'model|voice')
    backend = WavBackend()
    narrator = Narrator(NarratorConfig(state_mode='template'), StateTracker(cooldown_s=0.0), backend=backend,
                        player=Player(), clock=Clock(), start=False, speech_cache=cache)
    narrator.planning_state('Ready')
    assert narrator.speak_once() is True and backend.spoken == ['준비됐어요. 출발 신호를 기다리고 있어요.']
    narrator.clock.now += 60.0
    narrator.planning_state('Idle')
    narrator.planning_state('Ready')
    backend.tts_fail = True                                             # the network is gone ...
    assert narrator.speak_once() is True
    assert len(narrator.player.played) == 2 and narrator.stats['tts_errors'] == 0   # ... the sentence still plays
    assert SpeechCache(str(tmp_path / 'speech'), 'other voice').get('준비됐어요. 출발 신호를 기다리고 있어요.') is None


def test_gpt_sentences_are_not_kept_in_the_cache(tmp_path):
    cache = SpeechCache(str(tmp_path / 'speech'), 'sig')
    backend = WavBackend(reply='상자 쪽으로 가고 있어요.')
    narrator = Narrator(NarratorConfig(), StateTracker(), backend=backend, player=Player(), clock=Clock(),
                        start=False, speech_cache=cache)
    narrator.planning_state('Ready')
    assert narrator.think_once() is True and narrator.speak_once() is True
    assert cache.get('상자 쪽으로 가고 있어요.') is None


def test_speech_cache_prunes_the_oldest(tmp_path):
    cache = SpeechCache(str(tmp_path), 'sig', max_files=3)
    out = io.BytesIO()
    with wave.open(out, 'wb') as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(24000)
        wav.writeframes(b'\x00\x00' * 240)
    for index in range(5):
        cache.put(f'문장 {index}', out.getvalue())
        time.sleep(0.01)
    assert cache.get('문장 0') is None and cache.get('문장 4') == out.getvalue()
    assert len(list(tmp_path.glob('*.wav'))) == 3
    (tmp_path / 'broken.wav').write_bytes(b'RIFFxxxx')
    assert cache.get('nothing') is None


def test_a_noticed_long_compute_becomes_a_remark():
    backend = Backend(reply='계산이 좀 오래 걸리네, 잠깐만!')
    narrator, texts = make(backend=backend, notice=True, narrate_period_s=0.0)
    narrator.tracker.notice_compute_s = 8.0
    narrator.tracker.announce = ()
    narrator.planning_state('Run.Compute')
    for _ in range(9):
        narrator.clock.now += 1.0
        narrator.planning_state('Run.Compute')
        narrator.tick()
    assert narrator.think_once() is True and narrator.speak_once() is True
    assert texts == [('thought', '계산이 좀 오래 걸리네, 잠깐만!')]
    assert '방금 스스로 알아챈 게 있어: 계산이 8초째 이어짐' in backend.calls[-1][1] and backend.calls[-1][2] is None
    narrator.clock.now += 1.0
    narrator.planning_state('Run.Compute')
    narrator.tick()
    assert narrator.think_once() is False                               # once


def test_a_remark_is_dropped_when_the_state_moved_on_and_lost_state_is_fixed_words():
    backend = Backend(reply='계산이 오래 걸려요.')
    narrator, texts = make(backend=backend, notice=True)
    narrator.tracker.announce = ()
    narrator.planning_state('Run.Compute')
    narrator.clock.now += 9.0
    narrator.planning_state('Run.Compute')
    narrator.tick()
    assert narrator.think_once() is True
    narrator.planning_state('Run.Execute')                              # the compute ended meanwhile
    assert narrator.speak_once() is True and texts == []
    narrator.clock.now += 6.0                                           # /planning/state stopped arriving
    narrator.tick()
    assert narrator.think_once() is False and narrator.speak_once() is True
    assert texts == [('thought', '작업 상태 신호가 끊겼어요.')]


def test_audible_flag_follows_playback():
    seen = []
    backend = Backend()
    player = Player(block=True)
    narrator = Narrator(NarratorConfig(state_mode='template'), StateTracker(), backend=backend, player=player,
                        on_speaking=seen.append, clock=Clock(), start=False)
    narrator.planning_state('Ready')
    thread = threading.Thread(target=narrator.speak_once)
    thread.start()
    assert player.wait_playing(1) and narrator.audible is True
    player.stop()
    thread.join(timeout=3.0)
    assert narrator.audible is False and seen == [True, False]


def test_the_name_alone_is_answered_with_the_wake_recording(tmp_path):
    cheer = wav_file(tmp_path / 'jjasseu.wav')
    backend = Backend(reply='준비 끝내고 기다리는 중이야!')
    player = Player()
    narrator, texts = make(backend=backend, player=player, reactions={'wake': cheer})
    narrator.tracker.style = 'cute'
    narrator.planning_state('Ready')
    narrator._jobs.clear()
    narrator.wake('')
    assert narrator.speak_once() is True and texts == [('ack', '짜스!')]
    assert player.played == [open(cheer, 'rb').read()] and backend.spoken == []   # the recording, no synthesis
    narrator.wake('어디 가?')                                              # with a question: the spoken "어디 보자!"
    assert narrator.speak_once() is True and texts[-1] == ('ack', '어디 보자!') and backend.spoken == ['어디 보자!']


def test_a_question_about_the_state_is_answered_by_the_state_machine_at_once():
    backend = Backend(reply='GPT가 지어낸 답')
    narrator, texts = make(backend=backend)
    narrator.tracker.style = 'cute'
    for label in ('Ready', 'Run.PrepareTask', 'Run.Execute'):
        narrator.planning_state(label)
    narrator._jobs.clear()
    narrator._utterances.clear()
    narrator.wake('뭐 하는 상태야?')
    assert not narrator._jobs                                           # nothing for GPT to do
    assert narrator.speak_once() is True and texts == [('status', '지금 작업하는 중이야!')]   # no "어디 보자!" first
    narrator.clock.now += 60.0                                          # planning stopped publishing
    narrator.wake('지금 뭐 해?')
    assert narrator.speak_once() is True
    assert texts[-1] == ('status', '지금은 상태 신호가 안 들어와. 주행 프로그램이 꺼진 것 같아.')
    narrator.wake('왜 멈췄어?')                                          # a reasoning question still goes to GPT
    assert narrator.speak_once() is True and texts[-1] == ('ack', '어디 보자!')
