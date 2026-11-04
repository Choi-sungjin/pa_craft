"""Which camera frame the narrator looks at: the one the planning state machine's input came from.

planning_master acts on perception inputs (YOLO detections, puck and knob observations). Each of those
messages carries the exact camera stamp of the frame it was computed from: perception_master copies
frame_meta[4] into header.stamp. So the picture behind what the state machine is acting on can be found
by stamp. craft_sensors reuses each of its 4 shared-memory slots after ~0.13 s, so the matching frame is
copied as soon as the input arrives and kept for max_age_s. At most KEEP_MAX copies are kept, so copies are
at least max(min_interval_s, max_age_s / (KEEP_MAX - 1)) apart: those always cover max_age_s.

pick(at) returns the input frame the state machine had at time `at` (a state change), the newest input
frame when `at` is now, or the newest camera frame when there is no input frame: no perception running,
stamps that match no frame, the slot already overwritten, or image_source 'latest'.
"""

from __future__ import annotations

import math
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass

import numpy as np

from .frames import compressed_to_bgr, image_to_bgr

SLACK_S = 0.1     # an input that arrives just after the state change (another topic's callback) still counts
RECENT = 16       # camera frames remembered by stamp (~0.5 s at 30 fps; the shm keeps only 4 anyway)
KEEP_MAX = 64     # input copies kept at most (memory bound if min_interval_s is set very small)
COPY_TRIES = 3    # craft_sensors holds an exclusive lock while it writes a slot: retry a busy copy briefly


@dataclass(frozen=True)
class Picture:
    bgr: np.ndarray
    source: str        # 'input': same frame as a state machine input | 'latest': newest camera frame
    received: float    # clock time the input (or, for 'latest', the frame) arrived
    stamp_ns: int = 0  # camera stamp, 0 if unknown
    topic: str = ''    # perception topic of the input


def stamp_ns(stamp) -> int:
    """builtin_interfaces/Time -> integer nanoseconds (0 when missing)."""
    if stamp is None:
        return 0
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CameraFrames:
    """Thread safe: on_* run in ROS callbacks, pick() in the narrator's think thread."""

    def __init__(self, reader=None, mode: str = 'input', max_age_s: float = 2.0, min_interval_s: float = 0.2,
                 clock=time.monotonic):
        if mode not in ('input', 'latest'):
            raise ValueError("image_source must be 'input' or 'latest'")
        self.reader = reader  # ShmFrameReader for d435/c310, None for image topics
        self.mode = mode
        self.max_age_s = max_age_s
        self.min_interval_s = max(min_interval_s, max_age_s / (KEEP_MAX - 1))
        self.clock = clock
        # lost: the slot was reused before it could be copied; busy: still locked by the writer after retries
        self.stats = {'input_frames': 0, 'input_unmatched': 0, 'input_lost': 0, 'input_busy': 0}
        self._lock = threading.Lock()
        self._read_lock = threading.Lock()   # ShmFrameReader keeps one fd/mmap: one copy at a time
        self._latest = None                  # (received, frame_meta or image message, stamp_ns)
        self._recent: OrderedDict[int, object] = OrderedDict()  # stamp_ns -> frame_meta / image message
        self._inputs: deque[Picture] = deque(maxlen=KEEP_MAX)  # pruned by age, so max_age_s decides
        self._seen: deque[int] = deque(maxlen=64)               # input stamps already handled
        self._captured_at = -math.inf

    # ---- camera ------------------------------------------------------------------------------
    def on_meta(self, meta):
        """Shared-memory camera: one frame_meta message ([offset, w, h, seq, stamp_ns, ...])."""
        meta = [int(v) for v in meta]
        stamp = meta[4] if len(meta) >= 5 else 0
        self._store(meta, stamp)

    def on_image(self, msg, stamp: int = 0):
        """Image topic: one sensor_msgs Image or CompressedImage (kept by reference, converted on use)."""
        self._store(msg, stamp)

    def _store(self, item, stamp: int):
        with self._lock:
            self._latest = (self.clock(), item, stamp)
            if self.mode == 'input' and stamp > 0:
                self._recent[stamp] = item
                self._recent.move_to_end(stamp)
                while len(self._recent) > RECENT:
                    self._recent.popitem(last=False)

    # ---- state machine inputs ------------------------------------------------------------------
    def on_input(self, stamp: int, topic: str = '') -> bool:
        """A perception input computed from the camera frame with this stamp arrived: keep that frame."""
        if self.mode != 'input' or stamp <= 0:
            return False
        now = self.clock()
        with self._lock:
            if stamp in self._seen:  # detections and puck observations of one frame share the stamp
                return False
            if now - self._captured_at < self.min_interval_s:
                return False
            item = self._recent.get(stamp)
            if item is None:
                self._seen.append(stamp)
                self.stats['input_unmatched'] += 1
                return False
            if self._reused(item):  # a newer frame went into the same slot: no copy can succeed
                self._seen.append(stamp)
                self.stats['input_lost'] += 1
                return False
        bgr = self._to_bgr(item, tries=COPY_TRIES)  # outside the lock: shm copy or image conversion
        with self._lock:
            if bgr is None:
                # Still locked by the writer, reused meanwhile, or the image is unreadable. Not marked as
                # seen: the same frame's input on the other topic may still get it.
                self.stats['input_busy'] += 1
                return False
            if stamp in self._seen:
                return False
            self._seen.append(stamp)
            while self._inputs and now - self._inputs[0].received > self.max_age_s:
                self._inputs.popleft()
            self._inputs.append(Picture(bgr, 'input', now, stamp, topic))
            self._captured_at = now
            self.stats['input_frames'] += 1
        return True

    def _reused(self, item) -> bool:
        """Lock held. Shared memory: a newer frame_meta points at the same slot offset. Newer means a later
        stamp too: craft_sensors counts sequences from 0 again after a restart, stamps keep increasing."""
        if self.reader is None or not isinstance(item, list):
            return False
        return any(isinstance(m, list) and m[0] == item[0] and m[3] > item[3] and m[4] > item[4]
                   for m in self._recent.values())

    # ---- for the narrator ----------------------------------------------------------------------
    def pick(self, at: float | None = None) -> Picture | None:
        """The input frame the state machine had at `at` (default now), else the newest camera frame."""
        now = self.clock()
        at = now if at is None else at
        with self._lock:
            fresh = [p for p in self._inputs if now - p.received <= self.max_age_s]
        if fresh:
            before = [p for p in fresh if p.received <= at + SLACK_S]
            if before:
                return max(before, key=lambda p: p.received)
            return min(fresh, key=lambda p: p.received)  # all newer than `at`: the closest one
        return self._newest()

    def _newest(self) -> Picture | None:
        for _ in range(3):
            with self._lock:
                latest = self._latest
            if latest is None or self.clock() - latest[0] > self.max_age_s:
                return None
            received, item, stamp = latest
            bgr = self._to_bgr(item)
            if bgr is not None:
                return Picture(bgr, 'latest', received, stamp)
            if self.reader is None:
                return None
            time.sleep(0.04)  # slot busy or overwritten: take the next frame_meta
        return None

    def ages(self) -> tuple[float, float]:
        """(seconds since the newest camera frame, since the newest input frame); inf if none."""
        now = self.clock()
        with self._lock:
            latest = now - self._latest[0] if self._latest else math.inf
            newest = now - self._inputs[-1].received if self._inputs else math.inf
        return latest, newest

    def close(self):
        if self.reader is not None:
            with self._read_lock:
                self.reader.close()

    def _to_bgr(self, item, tries: int = 1):
        """BGR of a frame_meta or image message; None if it is gone or unreadable (never raises)."""
        try:
            if self.reader is not None:
                for attempt in range(tries):
                    if attempt:
                        time.sleep(0.003)
                    with self._read_lock:
                        frame = self.reader.snapshot(item)
                    if frame is not None:
                        return frame.bgr
                return None
            if hasattr(item, 'format'):
                return compressed_to_bgr(item.data)
            return image_to_bgr(item.encoding, item.height, item.width, item.step, item.data)
        except Exception:  # cv2.error is not a ValueError; a bad image must not reach a ROS callback
            return None
