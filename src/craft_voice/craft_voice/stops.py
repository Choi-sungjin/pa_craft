"""Fresh wheel odometry confirms a stop; only current robot signals explain it.

When no signal gives a reason the robot does not say "I am finding out why": it says the state it is
stopped in (driving to the goal, preparing, just finished, no state signal), which is what it knows."""
import math

STOP_LINES = {
    'driving': '목표 지점으로 가는 중인데 지금은 멈춰 있어.',
    'executing': '작업하는 중인데 지금은 멈춰 있어.',
    'preparing': '다음 작업을 준비하느라 멈춰 있어.',
    'finished': '이번 단계를 끝내고 멈춰 있어.',
    'no_signal': '멈춰 있어. 지금은 상태 신호가 안 들어와.',
    'still': '지금은 멈춰 있어.',
    'blocked': '길이 막혀서 잠깐 멈춰 있어.',
    'replan': '길이 막혀서 멈추고, 새로운 경로를 찾는 중이야.',
    'no_route': '지나갈 길을 못 찾아서 기다리고 있어.',
    'box_perceive': '상자 위치를 확인하느라 잠깐 기다리고 있어.',
    'box_settle': '정확히 맞추려고 멈춰서 흔들림을 가라앉히는 중이야.',
    'target': '목표물이 다시 보이기를 기다리고 있어.',
    'control': '제어기 응답을 기다리는 중이야.',
    'sensors': '새 위치 정보를 기다리느라 멈춰 있어.',
    'compute': '다음 동작을 계산하며 기다리고 있어.',
    'hold': '일시정지 신호가 들어와서 기다리고 있어.',
    'control_fault': '제어기에서 오류 신호가 와서 멈춰 있어.',
    'done': '작업이 끝나서 다음 지시를 기다리고 있어.',
    'waiting': '다음 지시를 기다리는 중이야.',
}


DEPART_STILL_S = 3.0   # a start counts as a departure after standing still this long
DEPART_GAP_S = 10.0    # and at most one departure remark in this time


class StopMonitor:
    def __init__(self):
        self.epoch = 0
        self.departed = False          # the wheels started after standing still (taken by the tracker)
        self._depart_at = -math.inf
        self.reset()

    def reset(self):
        self.last = None
        self.moving_since = None
        self.armed = False
        self.since = None
        self.notified = False
        self.still_from = None         # standing still since (None: moving, or not seen still since the reset)
        self.moving = False

    def update(self, v, w, now):
        if not all(math.isfinite(x) for x in (v, w, now)):
            self.reset()
            return
        if self.last is not None and not 0 <= now-self.last <= 0.6:
            self.reset()
        self.last = now
        if abs(v) >= .006 or abs(w) >= .04:
            self.since = None
            if self.moving_since is None:
                self.moving_since = now
            if now-self.moving_since >= .25:
                self.armed = True
                self.notified = False
                if not self.moving:
                    self.moving = True
                    if (self.still_from is not None and self.moving_since-self.still_from >= DEPART_STILL_S and
                            now-self._depart_at >= DEPART_GAP_S):
                        self.departed = True
                        self._depart_at = now
                    self.still_from = None
        elif abs(v) <= .003 and abs(w) <= .02:
            self.moving_since = None
            if self.moving or self.still_from is None:
                self.moving = False
                self.still_from = now
            if self.armed and self.since is None:
                self.since = now
                self.epoch += 1
        else:
            self.moving_since = None
            self.since = None

    def stopped(self, now):
        return (self.last is not None and 0 <= now-self.last <= .6 and
                self.since is not None and now-self.since >= 1.5)


def stop_reason(tracker, now):
    if not tracker.wheel_stop.stopped(now):
        return None
    if tracker.label is None or not 0 <= now-tracker.label_at <= tracker.stale_s:
        return 'no_signal'
    if tracker.label in ('Fault', 'Run.Stopping'):
        return None  # the existing state announcement already explains these
    c = tracker.control
    if 0 <= now-c.at <= tracker.stale_s:
        if c.fault:
            return 'control_fault'
        if c.held:
            return 'hold'
    if (0 <= now-tracker.diag_at <= tracker.stale_s and
            tracker.diag_message.partition(':')[0] == tracker.label):
        situation = tracker.current_situation(now)
        if situation in STOP_LINES:
            return situation
    if tracker.label == 'Run.Compute':
        return 'compute'
    if tracker.label in ('Ready', 'Run.AwaitRequest', 'Idle'):
        done = tracker.last_done or {}
        return 'done' if 0 <= now-done.get('at', -math.inf) <= 5 else 'waiting'
    # No signal gives a reason: say the state it is stopped in.
    if tracker.label == 'Run.Execute':
        fresh = 0 <= now-tracker.diag_at <= tracker.stale_s and tracker.step_known()
        return 'driving' if fresh and tracker.diag.get('step', '') == 'base_goal' else 'executing'
    if tracker.label == 'Run.PrepareTask':
        return 'preparing'
    if tracker.label == 'Run.Complete':
        return 'finished'
    return 'still'
