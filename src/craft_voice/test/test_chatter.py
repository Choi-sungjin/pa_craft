"""혼잣말: 정해 둔 문장을 돌아가며, 일하지 않고 서 있고 조용할 때만."""

from craft_voice.chatter import LINES, Chatter, all_lines, time_slot
from craft_voice.prepare import fixed_sentences
from test_narrator import Backend, make, run


def test_time_slots():
    assert [time_slot(h) for h in (6, 11.6, 14, 19, 2)] == ['morning', 'lunch', 'afternoon', 'evening', 'night']


def test_lines_rotate_without_repeating_and_follow_the_time_of_day():
    chatter = Chatter(100.0, style='cute', seed=1, hour=lambda: 12.0)
    pool = set(LINES['cute']['any']) | set(LINES['cute']['lunch'])
    said = [chatter.next_line(0.0) for _ in range(len(pool))]
    assert len(set(said)) == len(pool) and set(said) == pool          # every line once before any repeat
    assert not set(said) & set(LINES['cute']['morning'])
    no_clock = Chatter(100.0, style='cute', seed=1, hour=lambda: None)
    assert all(no_clock.next_line(0.0) in LINES['cute']['any'] for _ in range(30))


def test_the_gap_is_irregular_and_postponed():
    chatter = Chatter(100.0, jitter=0.4, seed=3, hour=lambda: None)
    assert not chatter.due(0.0) and not chatter.due(59.0)
    gaps = []
    for _ in range(20):
        start = chatter._next_at
        chatter.next_line(start)
        gaps.append(chatter._next_at - start)
    assert min(gaps) >= 60.0 and max(gaps) <= 140.0 and max(gaps) - min(gaps) > 20.0
    chatter.postpone(1000.0)
    assert not chatter.due(1059.0)
    assert not Chatter(0.0).due(1e9)


def test_every_line_is_prepared():
    sentences = fixed_sentences('cute', {}, {})
    assert set(all_lines('cute')) <= set(sentences)


def idle_narrator(**config):
    narrator, texts = make(backend=Backend(reply='GPT'), chatter_period_s=100.0, chatter_quiet_s=60.0, **config)
    narrator.tracker.style = 'cute'
    narrator.chatter.hour = lambda: None
    return narrator, texts


def test_it_talks_to_itself_only_when_idle():
    narrator, texts = idle_narrator()
    narrator.tick()                                    # first gap starts now
    narrator.clock.now += 150.0
    narrator.tick()
    run(narrator)
    assert texts and texts[-1][0] == 'thought' and texts[-1][1] in LINES['cute']['any']


def test_no_chatter_while_working_or_moving():
    narrator, texts = idle_narrator()
    narrator.tick()
    narrator.planning_state('Run.Execute')
    run(narrator)
    texts.clear()
    for _ in range(30):                                # working for 300 s
        narrator.clock.now += 10.0
        narrator.planning_state('Run.Execute')
        narrator.tick()
    run(narrator)
    assert not [t for t in texts if t[1] in all_lines('cute')]
    narrator2, texts2 = idle_narrator()
    narrator2.tick()
    for _ in range(30):                                # driving without planning for 300 s
        narrator2.clock.now += 10.0
        narrator2.tracker.update_motion(0.2, 0.0, narrator2.clock.now)
        narrator2.tick()
    run(narrator2)
    assert not [t for t in texts2 if t[1] in all_lines('cute')]


def test_off_by_default():
    narrator, texts = make(backend=Backend(reply='GPT'))
    narrator.tick()
    narrator.clock.now += 10000.0
    narrator.tick()
    run(narrator)
    assert texts == []
