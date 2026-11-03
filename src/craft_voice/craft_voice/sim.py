"""Simulated planning state machine for trying the voice without the robot.

Plays a scripted timeline of planning_master labels and builds diagnostics in
the same format planning_master.cpp report() publishes. Scenario text:

    Label[@step][!reason]:seconds, ...

e.g. "Ready:5, Run.Execute@approach:8, Fault@approach!Run.Execute: execution timeout:10".
The duration is taken after the last ':', so a reason may contain ':'.
"""

from __future__ import annotations

from dataclasses import dataclass

from .robot_state import DIAGNOSTIC_NAME, STATE_KO

DEFAULT_SCENARIO = (
    'Idle:3, Ready:6, Run.AwaitRequest:0.5, Run.PrepareTask@base_goal:0.5, Run.Compute@base_goal:0.5, '
    'Run.Execute@base_goal:10, Run.Complete@base_goal:0.05, Run.AwaitRequest:0.5, '
    'Run.PrepareTask@approach:0.5, Run.Compute@approach:1, Run.Execute@approach:8, '
    'Fault@approach!Run.Execute: execution timeout:12'
)


@dataclass(frozen=True)
class SimStep:
    label: str
    step: str = ''
    reason: str = ''
    seconds: float = 0.0


def parse_item(text: str, with_duration: bool = True) -> SimStep:
    """'Fault@approach!Run.Execute: execution timeout:10' -> SimStep."""
    text = text.strip()
    seconds = 0.0
    if with_duration:
        text, sep, number = text.rpartition(':')
        if not sep:
            raise ValueError(f'scenario item needs ":seconds": {text!r}')
        seconds = float(number)
        if not seconds > 0:
            raise ValueError(f'duration must be > 0: {text!r}')
    head, _, reason = text.partition('!')
    label, _, step = head.partition('@')
    label = label.strip()
    if label not in STATE_KO:
        raise ValueError(f'unknown state {label!r} (one of {", ".join(STATE_KO)})')
    return SimStep(label, step.strip(), reason.strip(), seconds)


def parse_scenario(text: str) -> list[SimStep]:
    steps = [parse_item(item) for item in text.split(',') if item.strip()]
    if not steps:
        raise ValueError('empty scenario')
    return steps


class SimStateMachine:
    """Timeline of SimSteps with a manual override; feed it the elapsed time."""

    def __init__(self, steps: list[SimStep], loop: bool = True):
        self.steps = steps
        self.loop = loop
        self.total = sum(step.seconds for step in steps)
        self.override: SimStep | None = None
        self.task_id = 0
        self._last: SimStep | None = None
        self._index = -1

    def set_override(self, step: SimStep | None):
        self.override = step

    def at(self, elapsed: float) -> SimStep:
        if self.override is not None:
            current, index = self.override, -2
        else:
            t = elapsed % self.total if self.loop else min(elapsed, self.total - 1e-9)
            index = 0
            while index < len(self.steps) - 1 and t >= self.steps[index].seconds:
                t -= self.steps[index].seconds
                index += 1
            current = self.steps[index]
        if (index, current) != (self._index, self._last):
            if current.label == 'Run.PrepareTask':
                self.task_id += 1  # planning_master hands out a new task id per step
            self._index, self._last = index, current
        return current

    def diagnostics(self, current: SimStep) -> tuple[str, int, str, dict]:
        """(name, level, message, values) as planning_master publishes them (1 Hz)."""
        values = {'task_id': str(self.task_id) if current.step else '', 'step': current.step,
                  'queued': '0', 'waiting_for': ''}
        level = 2 if current.label == 'Fault' else 0
        return DIAGNOSTIC_NAME, level, f'{current.label}: {current.reason}', values
