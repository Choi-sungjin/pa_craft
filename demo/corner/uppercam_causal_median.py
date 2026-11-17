"""Trailing median of exactly three consecutive valid distance estimates.

No future samples, hold-last-value, or zero padding. Invalid input, out-of-order
frame/time, or a long timestamp gap clears history. First two valid frames after
reset report warmup; geometry can still be displayed, but no filtered range.
"""
from collections import deque
import math


class CausalMedian3:
    def __init__(self,max_gap_s=.15):
        self.max_gap_s=float(max_gap_s)
        self.history=deque(maxlen=3)

    def reset(self):
        self.history.clear()

    def update(self,value,frame_index,timestamp_s,valid=True,extrapolated=False):
        if not valid or value is None or not math.isfinite(float(value)):
            self.reset()
            return dict(valid=False,value=None,state='invalid',history_n=0)
        index=int(frame_index);ts=float(timestamp_s)
        if not math.isfinite(ts):raise ValueError('Timestamp must be finite')
        if self.history:
            previous=self.history[-1]
            if index!=previous['index']+1 or ts<=previous['time'] or ts-previous['time']>self.max_gap_s:
                self.reset()
        self.history.append(dict(value=float(value),index=index,time=ts,extrapolated=bool(extrapolated)))
        if len(self.history)<3:
            return dict(valid=False,value=None,state='warmup',history_n=len(self.history))
        selected=sorted(self.history,key=lambda item:item['value'])[1]
        return dict(valid=True,value=selected['value'],state='ready',history_n=3,
                    source_frame_index=selected['index'],sample_age_s=ts-selected['time'],
                    extrapolated=selected['extrapolated'])
