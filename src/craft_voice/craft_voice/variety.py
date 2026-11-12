"""More than one way to say the same thing, and real facts in the sentence.

A tour is the same two labels over and over (Run.Execute, Run.Complete, step base_goal): with one sentence per
label the robot repeats itself. Two things change that, both in fixed words (no GPT, kept in the speech cache):
  - VARIANTS: further wordings of a sentence, said in turn;
  - DRIVE: sentences about a drive to a goal, filled with what the robot knows at that moment: how far the goal
    is and on which side (/planning/target_base_pose against /planning/odom), which goal of the run this is, how
    long the leg took, whether the path was blocked or searched again on the way.
Numbers are rounded (whole metres up to MAX_M, seconds to 5 up to MAX_S, goals up to the 20th) so the set of
sentences is finite and can be synthesized beforehand (all_lines); outside that range the plain wording is used.
"""

from __future__ import annotations

import math

VARIANTS = {
    'cute': {
        # Lines of the StarCraft II SCV (건설로봇, Korean dub) are mixed in where they fit the situation.
        # The two about a blocked path come after the "아? 예?" of the path search recording (see
        # scripts/aye_lines.py: their kept speech is that recording's opening followed by the line).
        'change.Ready': ['준비 완료! 신호만 주면 바로 출발할게.', '건설로봇 준비 완료!', '나 준비됐어! 출발 신호 기다릴게.'],
        'change.Run.Execute': ['자, 다음 길로 가 볼게!', '시키면 해야죠.', '다음 경로로 출발!', '라져!'],
        'change.Run.Execute.step': ['{step} 출발!', '자, {step} 시작!', '{step} 들어갈게!'],
        'change.Run.Complete': ['한 단계 클리어!', '작업 끝!', '이번 것도 끝!'],
        'change.Run.Complete.step': ['{step} 완료!', '좋아, {step} 끝!', '{step} 마쳤어!'],
        'change.Run.Stopping': ['이제 멈출게.'],
        'situation.blocked': ['앞이 막혔네, 잠깐 기다릴게.', '아? 예? 여기 뭐 있는데요.', '아, 깜짝이야!', '길이 막혀 있어. 조금만 기다려 볼게.'],
        'situation.replan': ['이 길은 안 되겠다, 다른 길 찾아볼게!', '돌아가는 길을 찾는 중이야!'],
        'situation.clear': ['길이 뚫렸어! 다시 출발!', '이제 지나갈 수 있어, 계속 갈게!'],
        'situation.no_route': ['지나갈 길이 안 보여. 조금 기다려 볼게.', '아주 훌륭하군요.'],
        'notice.long_blocked': ['아직도 막혀 있어. 길 좀 터 줄래?', '아? 예? 안 되겠는데요, 막혔어요.', '계속 기다리는 중이야. 비켜 주면 바로 갈게!'],
        'notice.many_replans': ['또 길을 다시 찾고 있어. 오늘 길이 복잡하네.', '내 팔자야.'],
        'notice.long_execute': ['이번 건 시간이 좀 걸리네!', '이야, 야근이다!'],
        'ack': ['잠깐만, 확인해 볼게!', '무슨 일이죠?', '음, 어디 보자!'],
        'notice.grasped': ['좋아, 꽉 잡았어!'],
        'notice.box_found': ['상자 발견!'],
        # Departure without the state machine (wheels start after standing still): the SCV lines of a start, the
        # same words as above so their kept speech (original recordings) is played.
        'notice.depart': ['갑니다!', '시키면 해야죠.', '예, 알았다고요.', '라져!'],
    },
    'polite': {
        'change.Ready': ['준비를 마쳤어요. 신호를 주시면 출발할게요.'],
        'change.Run.Execute.step': ['{step}, 출발할게요.', '이제 {step} 시작합니다.'],
        'change.Run.Complete.step': ['{step}, 마쳤어요.', '{step} 완료했어요.'],
        'situation.blocked': ['앞이 막혀 있어서 잠깐 기다릴게요.'],
        'situation.replan': ['다른 길을 찾아보고 있어요.'],
        'situation.clear': ['길이 열렸어요. 다시 출발할게요.'],
        'notice.long_blocked': ['아직 길이 막혀 있어요. 비켜 주시면 바로 갈게요.'],
        'ack': ['확인해 볼게요.'],
    },
}

DRIVE = {
    'cute': {
        'start.plain': ['다음 지점으로 출발!', '갑니다!', '자, 다음 지점 간다!', '예, 알았다고요.', '다음 지점으로 이동할게!'],
        'done.plain': ['도착!', '작업 끝!', '이번 지점도 도착!', '여기까지 왔어!'],
        'start.near': ['바로 앞 지점으로 갈게!', '코앞이야, 금방 갈게!'],
        'start.dist': ['다음 지점까지 {d}미터, 출발!', '{d}미터 앞으로 가 볼게!'],
        'start.dir': ['{dir} {d}미터 지점으로 갈게!'],
        'done.count': ['{n} 지점 도착!', '{n} 지점 찍었어!'],
        'done.time': ['도착! {s}초쯤 걸렸어.', '{s}초 만에 도착했어!'],
        'done.blocked': ['길이 막혔었지만 {n} 지점 도착!'],
        'done.replan': ['길을 다시 찾아서 {n} 지점에 왔어!'],
        'left': ['{d}미터쯤 남았어.'],
        'left.near': ['거의 다 왔어!'],
    },
    'polite': {
        'start.plain': ['다음 지점으로 출발할게요.'],
        'done.plain': ['도착했어요.'],
        'start.near': ['바로 앞 지점으로 갈게요.'],
        'start.dist': ['다음 지점까지 {d}미터예요. 출발할게요.'],
        'start.dir': ['{dir} {d}미터 지점으로 갈게요.'],
        'done.count': ['{n} 지점에 도착했어요.'],
        'done.time': ['도착했어요. {s}초쯤 걸렸어요.'],
        'done.blocked': ['길이 막혔었지만 {n} 지점에 도착했어요.'],
        'done.replan': ['길을 다시 찾아서 {n} 지점에 도착했어요.'],
        'left': ['{d}미터쯤 남았어요.'],
        'left.near': ['거의 다 왔어요.'],
    },
}

DIRECTIONS = ('앞쪽', '왼쪽', '뒤쪽', '오른쪽')  # quarter turns counter-clockwise from straight ahead
_ORDINALS = ('첫', '두', '세', '네', '다섯', '여섯', '일곱', '여덟', '아홉', '열', '열한', '열두', '열세', '열네',
             '열다섯', '열여섯', '열일곱', '열여덟', '열아홉', '스무')
MAX_M = 12   # farther goals are announced in plain words
MAX_S = 60
DRIVE_STEPS = ('base_goal',)  # step ids that are a drive to a goal pose


def ordinal(n: int) -> str:
    """'세 번째' for 3; '' outside 1..20."""
    return f'{_ORDINALS[n - 1]} 번째' if 1 <= n <= len(_ORDINALS) else ''


def metres(distance: float | None) -> int | None:
    """Whole metres (0 = right in front); None when unknown or farther than MAX_M."""
    if distance is None or not math.isfinite(distance) or distance < 0:
        return None
    rounded = int(distance + 0.5)
    return rounded if rounded <= MAX_M else None


def seconds(duration: float | None) -> int | None:
    """Rounded to 5 s (at least 5); None when unknown or longer than MAX_S."""
    if duration is None or not math.isfinite(duration) or duration < 0:
        return None
    rounded = max(5, 5 * int(duration / 5 + 0.5))
    return rounded if rounded <= MAX_S else None


def direction(bearing: float | None) -> str:
    """Side of the goal seen from the robot (bearing in rad, counter-clockwise from its heading)."""
    if bearing is None or not math.isfinite(bearing):
        return ''
    return DIRECTIONS[int(((bearing + math.pi / 4) % (2 * math.pi)) // (math.pi / 2))]


def _lines(style: str, key: str) -> list[str]:
    return DRIVE.get(style, DRIVE['polite']).get(key, [])


def start_lines(style: str, distance: float | None, bearing: float | None) -> list[str]:
    """Ways to announce a drive that starts now, with what is known ([] = nothing known)."""
    d = metres(distance)
    if d is None:
        return []
    if d == 0:
        return list(_lines(style, 'start.near'))
    lines = [text.format(d=d) for text in _lines(style, 'start.dist')]
    side = direction(bearing)
    if side:
        lines += [text.format(d=d, dir=side) for text in _lines(style, 'start.dir')]
    return lines


def done_lines(style: str, count: int, duration: float | None, blocked: bool, replanned: bool) -> list[str]:
    """Ways to announce an arrival. A leg that was blocked or searched again is said as such."""
    n = ordinal(count)
    if n and blocked:
        return [text.format(n=n) for text in _lines(style, 'done.blocked')]
    if n and replanned:
        return [text.format(n=n) for text in _lines(style, 'done.replan')]
    lines = [text.format(n=n) for text in _lines(style, 'done.count')] if n else []
    s = seconds(duration)
    if s is not None:
        lines += [text.format(s=s) for text in _lines(style, 'done.time')]
    return lines


def plain_lines(style: str, arrived: bool) -> list[str]:
    """Wordings of a drive without numbers (the step's own name reads badly in a sentence)."""
    return list(_lines(style, 'done.plain' if arrived else 'start.plain'))


def left_line(style: str, distance: float | None) -> str:
    """'3미터쯤 남았어.' for the answer to "what are you doing"; '' when unknown."""
    d = metres(distance)
    if d is None:
        return ''
    lines = _lines(style, 'left.near' if d == 0 else 'left')
    return lines[0].format(d=d) if lines else ''


def all_lines(style: str) -> list[str]:
    """Every DRIVE sentence that can be said, for synthesizing them beforehand."""
    table = DRIVE.get(style, DRIVE['polite'])
    lines = list(table.get('start.near', [])) + list(table.get('left.near', []))
    lines += list(table.get('start.plain', [])) + list(table.get('done.plain', []))
    for d in range(1, MAX_M + 1):
        lines += [text.format(d=d) for text in table.get('start.dist', []) + table.get('left', [])]
        lines += [text.format(d=d, dir=side) for text in table.get('start.dir', []) for side in DIRECTIONS]
    for count in range(1, len(_ORDINALS) + 1):
        for key in ('done.count', 'done.blocked', 'done.replan'):
            lines += [text.format(n=ordinal(count)) for text in table.get(key, [])]
    for s in range(5, MAX_S + 1, 5):
        lines += [text.format(s=s) for text in table.get('done.time', [])]
    return lines
