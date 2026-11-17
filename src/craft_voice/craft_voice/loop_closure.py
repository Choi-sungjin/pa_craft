"""완료 이벤트만 받는다. 지도 모양이나 위치 변화를 성공으로 추측하지 않는다."""
import math


class LoopClosureGate:
    def __init__(self, cooldown_s=20.0, max_age_s=5.0):
        self.cooldown_s = cooldown_s
        self.max_age_s = max_age_s
        self.last_stamp_ns = 0
        self.last_announced = -math.inf

    def accept(self, stamp_ns, now_ns, monotonic):
        if not isinstance(stamp_ns, int) or stamp_ns <= 0:
            return False
        age = (now_ns - stamp_ns) / 1e9
        if not math.isfinite(age) or age < 0 or age > self.max_age_s:
            return False
        if stamp_ns <= self.last_stamp_ns:
            return False
        self.last_stamp_ns = stamp_ns
        if monotonic - self.last_announced < self.cooldown_s:
            return False
        self.last_announced = monotonic
        return True
