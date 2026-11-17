"""Latest-frame distance with a jump gate, for the upper cam stop trigger (2026-11-10).

CausalMedian3 reported the middle of the last three frames: while the robot closes in, that is the previous frame
(+1 frame, up to +2 of lag), and any invalid frame cleared it so three fresh valid frames were needed again (about
40 % of the frames fail in a dock). This reports the newest valid frame itself and only gates it:

- a frame within jump_mm (+ max_rate_mm_s per second since the last accepted one) of the last accepted value is
  accepted and reported at once (no added lag);
- a frame outside that is held back as an outlier; a second outlier agreeing with it within jump_mm means the
  distance really moved (re-anchor), so it is accepted;
- after a reset (start, or nothing accepted for max_gap_s) two agreeing frames are needed before the first value;
- invalid frames are skipped, they do not clear anything (miss()).
"""
from collections import deque
import math


class LatestGate:
    def __init__(self, jump_mm=8.0, max_rate_mm_s=120.0, max_gap_s=0.3):
        self.jump_mm = float(jump_mm)
        self.max_rate_mm_s = float(max_rate_mm_s)
        self.max_gap_s = float(max_gap_s)
        self.anchor = None      # last accepted frame
        self.pending = None     # first frame after a reset, or the last outlier
        self.history = deque(maxlen=3)  # recent frames (indices kept for the geometry lookup)

    def reset(self):
        self.anchor = None; self.pending = None; self.history.clear()

    def miss(self, timestamp_s=None):
        """An invalid frame: nothing changes; a long silence is handled by the next update."""

    def _close(self, a, b):
        allowed = self.jump_mm + self.max_rate_mm_s * max(b['time'] - a['time'], 0.0)
        return abs(b['value'] - a['value']) <= allowed

    def update(self, value, frame_index, timestamp_s, valid=True, extrapolated=False):
        if not valid or value is None or not math.isfinite(float(value)):
            self.miss(timestamp_s)
            return dict(valid=False, value=None, state='invalid', history_n=len(self.history))
        ts = float(timestamp_s)
        if not math.isfinite(ts):
            raise ValueError('Timestamp must be finite')
        item = dict(value=float(value), index=int(frame_index), time=ts, extrapolated=bool(extrapolated))
        last = self.anchor or self.pending
        if last is not None and (ts <= last['time'] or ts - last['time'] > self.max_gap_s):
            self.reset()
        self.history.append(item)
        if self.anchor is not None and self._close(self.anchor, item):
            self.anchor = item; self.pending = None
        elif self.pending is not None and self._close(self.pending, item):
            self.anchor = item; self.pending = None   # two agreeing frames: start, or the distance really moved
        else:
            self.pending = item
            return dict(valid=False, value=None, state='warmup' if self.anchor is None else 'outlier',
                        history_n=len(self.history))
        return dict(valid=True, value=item['value'], state='ready', history_n=len(self.history),
                    source_frame_index=item['index'], sample_age_s=0.0, extrapolated=item['extrapolated'])
