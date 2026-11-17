"""Exercise actual executor selection/dispatch helpers without starting ROS/control."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[3]


@pytest.fixture
def executor(monkeypatch,tmp_path):
    monkeypatch.syspath_prepend(str(ROOT/'demo/puck_pick'))
    monkeypatch.setattr(sys,'argv',['execute.py','--collect','--expected-pucks','2','--cell','5','--pick'])
    spec=importlib.util.spec_from_file_location('pick_execute',ROOT/'demo/puck_pick/execute.py')
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    monkeypatch.setattr(m,'find_log',lambda **kw: None)
    monkeypatch.setattr(m,'find_save_dir',lambda:tmp_path)
    return m


def scene(state1='standing',state2='lying'):
    return (f'PUCK red_prism 0.90 ahead 175 lateral 0 top 130 posture {state1} height 60\n'
            f'PUCK green_prism 0.99 ahead 115 lateral 0 top 100 posture {state2} height 30')


def test_survey_selects_standing_and_cached_target_is_used(executor,monkeypatch):
    m=executor
    monkeypatch.setattr(m,'camera_run',lambda *a,**kw:SimpleNamespace(stdout=scene()))
    m.POLICY_TARGET=m.survey_pick()
    assert m.POLICY_TARGET.cls=='red_prism'
    def no_second_survey(*a,**kw): raise AssertionError('must use the ranked observation')
    monkeypatch.setattr(m,'camera_run',no_second_survey)
    steps=m.stage(m.find_steps(),None)
    ok,_=m.find_step(steps,0,{'what':'upper'})
    assert ok and m.FIND['cls']=='red_prism'
    assert '--avoid' in m.grip_target_args(m.FIND['ahead'])


def test_later_survey_replaces_target_after_first_pick(executor,monkeypatch):
    m=executor
    responses=iter([scene(), 'PUCK green_prism 0.99 ahead 115 lateral 0 top 100 posture lying height 30'])
    monkeypatch.setattr(m,'camera_run',lambda *a,**kw:SimpleNamespace(stdout=next(responses)))
    assert m.survey_pick().posture=='standing'
    sys.argv[sys.argv.index('--expected-pucks')+1]='1'
    m.POLICY_TARGET=m.survey_pick()
    assert m.POLICY_TARGET.posture=='lying'
    steps=m.lie_steps(start=m.nominal_pose())
    assert any(kind==m.LIEYAW for _,kind,_ in steps)
    assert any(name.startswith('lie regrasp 1:') for name,_,_ in steps)


def test_missed_puck_retries_then_refuses_to_choose(executor,monkeypatch):
    m=executor;calls=[]
    def capture(*a,**kw):
        calls.append(a)
        return SimpleNamespace(stdout=scene().splitlines()[0])
    monkeypatch.setattr(m,'camera_run',capture)
    with pytest.raises(ValueError,match='expected 2'):
        m.survey_pick()
    assert len(calls)==2


def test_legacy_find_also_keeps_the_other_puck_as_a_rival(executor,monkeypatch):
    m=executor
    monkeypatch.setattr(m,'camera_run',lambda *a,**kw:SimpleNamespace(stdout=scene()))
    steps=m.stage(m.find_steps(),None)
    ok,_=m.find_step(steps,0,{'what':'upper'})
    assert ok and m.POLICY_TARGET.cls=='red_prism'
    assert len(m.POLICY_CANDIDATES)==2
    assert '--avoid' in m.grip_target_args(m.FIND['ahead'])
