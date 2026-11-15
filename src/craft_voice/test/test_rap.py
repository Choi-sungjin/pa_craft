"""희귀 힙합 녹음은 혼잣말 기회당 1%, 제어와 독립적으로 재생한다."""
import threading
from pathlib import Path
import pytest
from craft_voice.narrator import RAP_LINE
from craft_voice.chatter import all_lines
from test_chatter import idle_narrator
from test_narrator import Player, run
from test_word_reactions import clip


def due(n):
    n.chatter._next_at = n.clock()
    n._last_activity = n.clock() - 100


def test_exactly_one_of_100_draws_uses_the_recording(monkeypatch, tmp_path):
    path = Path(clip(tmp_path, 'rap', 800))
    selected=[]
    for draw in range(100):
        n, texts=idle_narrator(reactions={'chatter_rap':str(path)})
        due(n)
        def pick(limit):
            assert limit == 100
            return draw
        monkeypatch.setattr('craft_voice.narrator.random.randrange',pick)
        n.tick()
        assert n.chatter._next_at > n.clock()
        u=n._utterances[0]
        selected.append(u.label == 'chatter_rap')
        if draw == 0:
            assert u.clip == str(path) and u.text == RAP_LINE
            assert n.chatter._said == set()
            run(n)
            assert n.player.played == [path.read_bytes()]
            assert not n.backend.spoken and not n.backend.calls
        else:
            assert u.text.startswith('내 상태는? ') and not u.clip   # 혼잣말 = 상태 읽기 (11-15)
    assert sum(selected) == 1


@pytest.mark.parametrize('blocked',['muted','moving','working','recent','not_due','disabled','missing'])
def test_no_draw_when_ineligible(monkeypatch, tmp_path, blocked):
    n,_=idle_narrator(reactions={'chatter_rap':str(clip(tmp_path,'rap',800))})
    due(n)
    if blocked=='muted':n.command('mute')
    elif blocked=='moving':n.tracker.update_motion(.2,0,n.clock())
    elif blocked=='working':n.planning_state('Run.Execute')
    elif blocked=='recent':n._last_activity=n.clock()
    elif blocked=='not_due':n.chatter._next_at=n.clock()+1
    elif blocked=='disabled':n.chatter.period_s=0
    elif blocked=='missing':n.config.reactions.clear()
    def forbidden(*a):raise AssertionError('draw while ineligible')
    monkeypatch.setattr('craft_voice.narrator.random.randrange',forbidden)
    n.tick()
    assert not any(u.label=='chatter_rap' for u in n._utterances)


@pytest.mark.parametrize('action',['delete','corrupt','move','state','stop','mute'])
def test_queued_rap_never_becomes_normal_tts(monkeypatch,tmp_path,action):
    path=Path(clip(tmp_path,'rap',800))
    n,texts=idle_narrator(reactions={'chatter_rap':str(path)})
    monkeypatch.setattr('craft_voice.narrator.random.randrange',lambda _:0)
    due(n);n.tick()
    if action=='delete':path.unlink()
    elif action=='corrupt':path.write_bytes(b'broken')
    elif action=='move':n.tracker.update_motion(.2,0,n.clock())
    elif action=='state':n.planning_state('Run.Execute')
    else:n.command(action)
    run(n)
    assert RAP_LINE not in n.backend.spoken
    assert not any(text==RAP_LINE for kind,text in texts)


def test_mute_interrupts_running_rap(monkeypatch,tmp_path):
    player=Player(block=True)
    n,_=idle_narrator(player=player,reactions={'chatter_rap':str(clip(tmp_path,'rap',800))})
    monkeypatch.setattr('craft_voice.narrator.random.randrange',lambda _:0)
    due(n);n.tick()
    thread=threading.Thread(target=n.speak_once);thread.start()
    assert player.started.wait(1)
    n.command('mute');thread.join(1)
    assert not thread.is_alive() and player.stops


@pytest.mark.parametrize('change',['motion','planning'])
def test_running_rap_finishes_when_work_or_motion_begins(monkeypatch,tmp_path,change):
    player=Player(block=True)
    n,_=idle_narrator(player=player,reactions={'chatter_rap':str(clip(tmp_path,'rap',800))})
    monkeypatch.setattr('craft_voice.narrator.random.randrange',lambda _:0)
    due(n);n.tick()
    thread=threading.Thread(target=n.speak_once);thread.start()
    assert player.started.wait(1)
    if change=='motion':n.motion(.2,0)
    else:n.planning_state('Run.Execute')
    assert thread.is_alive() and player.stops == 0
    assert not n._current.cancelled
    player.finish_current();thread.join(1)
    assert not thread.is_alive() and n.stats['spoken'] == 1
