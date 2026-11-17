"""Rules are lexicographic, independent of colour, shape, detector ordering and score."""
import importlib.util
import itertools
from pathlib import Path
import sys

import pytest

path=Path(__file__).resolve().parents[3]/'demo/puck_pick/pick_policy.py'
spec=importlib.util.spec_from_file_location('pick_policy',path)
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)


def puck(color='red',shape='prism',posture='standing',ahead=100,lat=0,score=.7):
    return f'PUCK {color}_{shape} {score} ahead {ahead} lateral {lat} top 100 px 1 1 depth 0.4 posture {posture} height 60'


@pytest.mark.parametrize('color1,color2,shape',itertools.product(('red','green'),('red','green'),('prism','cylinder')))
def test_far_standing_beats_near_lying_for_all_colors_shapes(color1,color2,shape):
    a=puck(color1,shape,'standing',ahead=170,score=.5)
    b=puck(color2,'prism','lying',ahead=60,score=.99)
    for text in (a+'\n'+b,b+'\n'+a):
        selected=m.select(m.candidates(text),expected=2)
        assert selected.ahead==170 and selected.posture=='standing'


@pytest.mark.parametrize('posture',('standing','lying'))
def test_same_posture_nearest_beats_score(posture):
    a=puck(posture=posture,ahead=160,score=.99)
    b=puck(posture=posture,ahead=80,score=.41)
    assert m.select(m.candidates(a+'\n'+b)).ahead==80


def test_planar_robot_distance_accounts_for_lateral_offset():
    a=puck(ahead=100,lat=90)
    b=puck(ahead=105,lat=0)
    assert m.select(m.candidates(a+'\n'+b)).ahead==105


def test_cylinder_always_stands():
    c=m.candidates(puck(shape='cylinder',posture='lying'))[0]
    assert c.posture=='standing'


def test_unknown_posture_or_missing_detection_prevents_wrong_priority():
    with pytest.raises(ValueError,match='posture unknown'):
        m.select(m.candidates(puck(posture='unknown')+'\n'+puck(ahead=60)))
    with pytest.raises(ValueError,match='expected 2'):
        m.select(m.candidates(puck()),expected=2)
    with pytest.raises(ValueError):
        m.select([])


def test_second_pick_uses_fresh_survey_not_first_snapshot():
    first=m.candidates(puck(ahead=150)+'\n'+puck(posture='lying',ahead=60))
    assert m.select(first,2).ahead==150
    second=m.candidates(puck(color='green',posture='lying',ahead=65))
    assert m.select(second,1).ahead==65


def test_low_score_rejected_and_tie_is_deterministic():
    assert not m.candidates(puck(score=.2))
    text=puck(color='red')+'\n'+puck(color='green')
    assert m.select(m.candidates(text))==m.select(m.candidates('\n'.join(reversed(text.splitlines()))))


def test_detections_off_the_docked_box_do_not_affect_count_or_priority():
    text=puck(ahead=100)+'\n'+puck(shape='cylinder',ahead=230,lat=-328)
    assert m.select(m.candidates(text),expected=1).ahead==100
