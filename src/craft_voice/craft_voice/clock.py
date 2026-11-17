"""Is the system clock set?

The robot's computer has no clock battery. It powers on in 1970 and the network time service sets the clock
about a minute later, half a minute after the Wi-Fi is up. Until then every certificate looks "not yet valid",
so every request would fail. A clock more than a day earlier than this file was written cannot be the real
time: while that is so nothing is sent and the microphone is not listened to. scripts/craft-clock-restore
(a system service) puts the clock back within a second of power-on, so with it installed this wait is not seen.
"""

from __future__ import annotations

import os
import time

_EARLIEST = os.path.getmtime(__file__) - 86400.0


def clock_unset(now: float | None = None) -> bool:
    return (time.time() if now is None else now) < _EARLIEST


NOT_SET = 'the system clock is not set yet'
