"""Two-puck collection policy. Colour/shape do not break posture/distance priority."""
from dataclasses import dataclass, asdict
import math
import re

N = r'([+-]?\d+(?:\.\d+)?)'


@dataclass(frozen=True)
class Candidate:
    cls: str
    score: float
    ahead: float
    lateral: float
    posture: str
    height: float | None
    distance: float

    def record(self):
        return asdict(self)


def candidates(text, ahead_bias=0., lateral_bias=0., face_x=257.67, laser_y=.02, min_score=.4):
    found = []
    for line in text.splitlines():
        m = re.match(rf'PUCK (\S+) {N} ahead {N} lateral {N}', line)
        if not m:
            continue
        cls, score, ahead, lateral = m.groups()
        score, ahead, lateral = float(score), float(ahead)-ahead_bias, float(lateral)-lateral_bias
        if score < min_score or cls not in ('red_cylinder','green_cylinder','red_prism','green_prism'):
            continue
        # Survey belongs to the docked 205 mm box, not other detections in the room.
        if not 0 <= ahead <= 205 or abs(lateral) > 102.5:
            continue
        state = re.search(r' posture (standing|lying|unknown)\b', line)
        height = re.search(rf' height {N}', line)
        posture = 'standing' if cls.endswith('cylinder') else state[1] if state else 'unknown'
        found.append(Candidate(cls,score,ahead,lateral,posture,float(height[1]) if height else None,
                               math.hypot(face_x+ahead,laser_y+lateral)))
    return found


def select(found, expected=None):
    if expected is not None and len(found) != expected:
        raise ValueError(f'expected {expected} puck(s), detected {len(found)}; rescan before choosing')
    if not found:
        raise ValueError('no confidently detected puck on the box')
    if len(found) > 2:
        raise ValueError(f'expected at most two pucks on this box, detected {len(found)}')
    if any(p.posture == 'unknown' for p in found):
        raise ValueError('prism posture unknown: cannot enforce standing-first priority; rescan')
    return min(found,key=lambda p:(p.posture != 'standing',p.distance,-p.score,p.cls,p.lateral))
