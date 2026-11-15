"""NPC 소개 대사 교체와 젯슨 호출/실제 질문의 구분."""
from pathlib import Path

import pytest

from craft_voice.listen import Listener
from craft_voice.narrator import Job, RAP_LINE, Utterance
from craft_voice.robot_state import Notice
from test_narrator import Backend, make, run
from test_word_reactions import clip


INTRO = '젯슨은 엔비디아의 소형 컴퓨터야.'


@pytest.mark.parametrize('kind', ['thought', 'scene'])
@pytest.mark.parametrize('line', [INTRO, '젯슨은 엔비디아의 작은 AI 컴퓨터 보드야.',
                                  'Jetson은 NVIDIA의 소형 컴퓨터야.'])
def test_npc_generated_intro_plays_existing_rap_without_tts(tmp_path, kind, line):
    path = Path(clip(tmp_path, 'rap.wav', 800))
    backend = Backend(line)
    n, texts = make(backend, reactions={'chatter_rap': str(path)})
    notice = Notice('idle', '쉬는 중', n.tracker.generation)
    n._submit(Job(kind, n.clock(), notice=notice))
    run(n)
    assert len(backend.calls) == 1
    assert n.player.played == [path.read_bytes()]
    assert texts == [('thought', RAP_LINE)]
    assert backend.spoken == []


@pytest.mark.parametrize('kind,requested,line', [
    ('ask', False, INTRO), ('status', False, INTRO), ('say', False, INTRO),
    ('scene', True, INTRO), ('thought', False, '엔비디아 소식이 궁금하네.'),
    ('thought', False, '젯슨이라고 부르면 대답할게.'),
])
def test_requested_answers_and_other_npc_lines_remain(tmp_path, kind, requested, line):
    backend = Backend()
    n, texts = make(backend, reactions={'chatter_rap': clip(tmp_path, 'rap.wav', 800)})
    n._enqueue(Utterance(kind, line, n.clock(), requested=requested))
    run(n)
    assert texts == [(kind, line)]
    assert backend.spoken == [line]


@pytest.mark.parametrize('problem', ['unconfigured', 'missing', 'corrupt', 'moving', 'working', 'muted'])
def test_npc_substitution_cannot_fall_back_to_intro_or_speak_during_work(tmp_path, problem):
    path = Path(clip(tmp_path, 'rap.wav', 800))
    backend = Backend()
    n, texts = make(backend, reactions={'chatter_rap': str(path)})
    if problem == 'unconfigured':
        n.config.reactions.clear()
    elif problem == 'missing':
        path.unlink()
    elif problem == 'corrupt':
        path.write_bytes(b'broken')
    elif problem == 'moving':
        n.tracker.update_motion(.2, 0, n.clock())
    elif problem == 'working':
        n.tracker.update_planning('Run.Execute', n.clock())
    elif problem == 'muted':
        n.command('mute')
    n._enqueue(Utterance('thought', INTRO, n.clock(), n.tracker.generation))
    run(n)
    assert texts == [] and n.player.played == [] and backend.spoken == []


@pytest.mark.parametrize('heard', ['젯슨', '젯슨!', '크래프트, 젯슨.', '크래프트야, 젯슨!',
                                  '크래프트 제슨', 'craft, jetson'])
def test_jetson_calls_use_tank_recordings_even_with_craft_prefix(tmp_path, heard):
    tank = Path(clip(tmp_path, 'tank.wav', 100))
    backend = Backend(INTRO)
    n, texts = make(backend, word_clips={'젯슨': (str(tank),)},
                    reactions={'chatter_rap': clip(tmp_path, 'rap.wav', 800)})
    listener = Listener(None, lambda _: heard, n.wake,
                        words={'젯슨': ('젯슨', '제슨', 'jetson')}, on_word=n.word)
    listener.process(b'\0\0' * 16000, 0)
    run(n)
    assert n.player.played == [tank.read_bytes()]
    assert backend.calls == [] and backend.spoken == []
    assert texts == [('ack', '젯슨')]


@pytest.mark.parametrize('heard', ['크래프트, 젯슨이 뭐야?', '크래프트 젯슨은 어떤 컴퓨터야?',
                                  '크래프트, 젯슨 설명해 줘'])
def test_real_jetson_questions_still_receive_ai_answers(tmp_path, heard):
    backend = Backend(INTRO)
    n, texts = make(backend, wake_ack=False,
                    word_clips={'젯슨': (clip(tmp_path, 'tank.wav', 100),)},
                    reactions={'chatter_rap': clip(tmp_path, 'rap.wav', 800)})
    listener = Listener(None, lambda _: heard, n.wake,
                        words={'젯슨': ('젯슨',)}, on_word=n.word)
    listener.process(b'\0\0' * 16000, 0)
    run(n)
    assert len(backend.calls) == 1 and backend.spoken == [INTRO]
    assert texts == [('ask', INTRO)]
