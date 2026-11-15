"""색 순서와 횟수는 수신 자료와 일치해야 하며 현재 색을 추측하면 안 된다."""
import random
import re
import threading
import pytest
from craft_voice.traffic import COLORS, answer, observation
from craft_voice.narrator import Job
from test_narrator import Backend, make, run


def record(seq, **kwargs):
    defaults=dict(complete=len(seq)==12,valid=len(seq)==12,discarded=0,at=100,sequence=seq)
    defaults.update(kwargs)
    return observation(*(seq.count(c) for c in range(3)),len(seq),**defaults)


def feed(n,seq,**kwargs):
    n.traffic_light(*(seq.count(c) for c in range(3)),len(seq),len(seq)==12,len(seq)==12,0,
                    sequence=seq,quiet=True,**kwargs)


def test_500_random_sequences_exact_order_counts_and_every_ordinal():
    rng=random.Random(20)
    for _ in range(500):
        while True:
            seq=[rng.randrange(3) for _ in range(12)]
            if all(1<=seq.count(c)<=6 for c in range(3)):break
        r=record(seq)
        assert not r['error']
        assert answer('신호등 순서 알려줘',r,100).split('순서는 ')[1] == ', '.join(COLORS[c] for c in seq)+'이야.'
        text=answer('신호등 색별 횟수 알려줘',r,100)
        for c,name in enumerate(COLORS):assert f'{name} {seq.count(c)}번' in text
        for i,c in enumerate(seq):assert f'{i+1}번째는 {COLORS[c]}이야.' in answer(f'신호등 {i+1}번째 색',r,100)


@pytest.mark.parametrize('seq,overrides',[
    ([0]*12,{}),([0,1,2]*4,{'sequence':[0]*12}),([0,1,2]*4,{'sequence':[0,1,8]*4}),
    ([0,1,2]*4,{'sequence':[0,1]}),([0,1,2]*4,{'discarded':-1}),
    ([0,1],{'complete':True,'valid':True}),([0,1,2]*4,{'valid':False}),
])
def test_invalid_record_is_never_reported_as_a_color(seq,overrides):
    r=record(seq,**overrides)
    assert r['error']
    for q in ('신호등 순서 알려줘','신호등 색별 횟수 알려줘'):
        text=answer(q,r,100)
        assert '맞지 않아' in text and not any(c in text for c in COLORS)


def test_partial_missing_and_stale_results():
    seq=[2,0,0,1]
    r=record(seq)
    assert '4번까지' in answer('신호등 순서 알려줘',r,100)
    assert '초록, 빨강, 빨강, 노랑' in answer('신호등 순서 알려줘',r,100)
    assert '확인하지 못했어' in answer('신호등 5번째 색',r,100)
    assert '기록이 없어' in answer('신호등 횟수 알려줘',r,1901)
    assert '기록이 없어' in answer('신호등 순서 알려줘',None,100)
    assert '점멸 순서 기록은 없어' in answer('신호등 순서 알려줘',record(seq,sequence=None),100)
    assert '점멸이 없어' in answer('신호등 순서 알려줘',record([]),100)
    assert not record([0,1,2]*4,complete=False,valid=True)['error']


def test_current_color_is_not_the_last_closed_blink():
    for r in (None,record([0,1,2]*4),record([0])):
        for q in ('지금 신호등 무슨 색이야','신호등 뭐가 켜졌어','현재 신호등 색 알려줘'):
            assert answer(q,r,100)=='지금 켜진 색은 점멸 기록만으로는 알 수 없어.'


def test_specific_colors_and_spoken_ordinals():
    r=record([0,0,0,0,0,0,1,1,2,2,2,2])
    assert '빨강 6번' in answer('신호등 빨간색 몇 번이야',r,100)
    assert '노랑' not in answer('신호등 빨간색 몇 번이야',r,100)
    assert '초록 4번' in answer('신호등 녹색은 몇 번?',r,100)
    assert '12번째는 초록' in answer('신호등 열두 번째 색',r,100)
    assert '1번째는 빨강' in answer('신호등 첫 번째 색',r,100)
    assert answer('신호등은 왜 필요한 거야',r,100) is None
    assert answer('사과 3개에 2개를 더하면',r,100) is None


@pytest.mark.parametrize('entry',['ask','wake','followup'])
def test_voice_entry_reads_complete_sequence_without_gpt_or_truncation(entry):
    n,texts=make(backend=Backend(),max_chars=20)
    seq=[0,0,1,2,1,2,0,2,1,2,0,1]
    feed(n,seq)
    n._quick_at=n.clock()
    getattr(n,entry)('신호등 순서 알려줘');run(n)
    assert ', '.join(COLORS[c] for c in seq) in texts[-1][1]
    assert not n.backend.calls
    assert n.backend.spoken==[texts[-1][1]]


def test_general_ai_question_keeps_verified_order_in_context():
    n,_=make(backend=Backend())
    feed(n,[0,1,2]*4)
    _,prompt=n._prompt(Job('ask',n.clock(),text='신호등을 어떻게 셌니'),False)
    assert '신호등 점멸 순서: 빨강, 노랑, 초록' in prompt


def test_old_latched_result_is_not_rejuvenated_or_announced():
    n,texts=make(backend=Backend(),notice=True)
    feed(n,[0,1,2]*4,age_s=2000)
    n.tick();n.wake('신호등 횟수 알려줘');run(n)
    assert texts==[('ask','최근에 확인한 신호등 점멸 기록이 없어.')]


def test_bad_update_cannot_reuse_pending_completion_with_invalid_counts():
    n,texts=make(backend=Backend(),notice=True)
    n.traffic_light(4,4,4,12,True,True,0,sequence=[0,1,2]*4)
    n.traffic_light(9,1,2,12,True,True,0,sequence=[0,1,2]*4)
    n.tick();run(n)
    assert not texts
    context=n.tracker.context_text(n.clock())
    assert '관측 불일치' in context and '빨강 9번' not in context


def test_changed_record_during_tts_does_not_say_obsolete_order():
    n,texts=make(backend=Backend(tts_delay=.15))
    feed(n,[0,1,2]*4);n.ask('신호등 순서 알려줘')
    thread=threading.Thread(target=n.speak_once);thread.start()
    assert n.backend.tts_started.wait(1)
    feed(n,[2,1,0]*4)
    thread.join(1)
    assert not texts and not n.player.played


def test_new_cycle_drops_queued_old_completion():
    n,texts=make(backend=Backend(),notice=True)
    n.traffic_light(4,4,4,12,True,True,0,sequence=[0,1,2]*4)
    n.tick()
    assert any(u.label=='traffic_notice:traffic_done' for u in n._utterances)
    n.traffic_light(0,0,0,0,False,False,0,sequence=[])
    run(n)
    assert not texts


@pytest.mark.parametrize('ordinal',['열세','스물두','-1','0','13','1.5'])
def test_out_of_range_ordinal_never_matches_a_suffix(ordinal):
    text=answer(f'신호등 {ordinal}번째 색',record([0,1,2]*4),100)
    assert not any(c in text for c in COLORS)
    assert '확인' in text


def test_signal_to_noise_question_keeps_general_ai_answer():
    n,texts=make(backend=Backend())
    feed(n,[0,1,2]*4)
    n.wake("신호 대 잡음비 계산 순서 알려줘.");run(n)
    assert n.backend.calls
