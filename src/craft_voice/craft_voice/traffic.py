"""인식 노드가 확정한 신호등 점멸 기록. 현재 켜진 램프 색으로 해석하지 않는다."""
from __future__ import annotations

import re

COLORS = ('빨강', '노랑', '초록')
ALIASES = (('빨강', '빨간', '적색'), ('노랑', '노란', '황색'), ('초록', '녹색', '파란', '파랑'))
ORDINALS = ('첫', '두', '세', '네', '다섯', '여섯', '일곱', '여덟', '아홉', '열', '열한', '열두')


def observation(red, yellow, green, blinks, complete, valid, discarded, at, sequence=None):
    counts = (int(red), int(yellow), int(green))
    seq = tuple(sequence) if sequence is not None else None
    error = (any(c < 0 for c in counts) or not 0 <= blinks <= 12 or sum(counts) != blinks or
             discarded < 0 or (seq is not None and (len(seq) != blinks or
             any(c not in (0, 1, 2) for c in seq) or
             tuple(seq.count(c) for c in range(3)) != counts)) or
             (complete and (not valid or blinks != 12 or any(not 1 <= c <= 6 for c in counts))))
    return dict(red=counts[0], yellow=counts[1], green=counts[2], blinks=int(blinks), complete=bool(complete),
                valid=bool(valid) and not error, discarded=int(discarded), at=at, sequence=seq, error=bool(error))


def question_kind(question):
    q = re.sub(r'\s+', '', question)
    if not any(word in q for word in ('신호등', '신호색', '불빛')):
        return None
    if any(word in q for word in ('순서', '번째', '처음', '마지막', '첫색')):
        return 'sequence'
    if re.search(r'몇.*(번|회|개)|횟수|각각|총몇', q):
        return 'counts'
    if re.search(r'무슨색|어떤색|색.*(뭐|알려)|켜|지금|현재', q):
        return 'current'
    return None


def answer(question, record, now, keep_s=1800.0):
    kind = question_kind(question)
    if kind is None:
        return None
    if kind == 'current':
        return '지금 켜진 색은 점멸 기록만으로는 알 수 없어.'
    if record is None or now - record['at'] > keep_s:
        return '최근에 확인한 신호등 점멸 기록이 없어.'
    if record.get('error'):
        return '신호등 색 순서와 횟수가 맞지 않아. 다시 확인해야 해.'
    q = re.sub(r'\s+', '', question)
    prefix = '마지막으로 센 결과야. ' if record['complete'] else f"아직 {record['blinks']}번까지 확인한 기록이야. "
    if kind == 'counts':
        selected = [i for i, names in enumerate(ALIASES) if any(name in q for name in names)]
        if not selected:
            selected = range(3)
        return prefix + ', '.join(f"{COLORS[i]} {record[('red', 'yellow', 'green')[i]]}번" for i in selected) + '이야.'
    seq = record.get('sequence')
    if seq is None:
        return '색별 횟수는 있지만 점멸 순서 기록은 없어.'
    if not seq:
        return '아직 확인한 신호등 점멸이 없어.'
    # 숫자 전체를 읽는다. 열세 -> 세, 스물두 -> 두, -1 -> 1처럼 일부만 맞추지 않는다.
    ordinal_text = re.sub(r'신호등(?:의|은|에서)?|신호색|불빛', ' ', question)
    ordinal = re.search(r'([+-]?\d+(?:\.\d+)?|[가-힣]+)\s*번\s*째', ordinal_text)
    index = None
    if ordinal:
        word = ordinal.group(1)
        if re.fullmatch(r'[+-]?\d+', word):
            index = int(word) - 1
        elif word in ORDINALS:
            index = ORDINALS.index(word)
        else:
            return '그 순번은 확인할 수 없어. 1번째부터 12번째 사이로 물어봐 줘.'
    if index is None and ('처음' in q or '첫색' in q):
        index = 0
    if index is None and '마지막' in q:
        index = len(seq) - 1
    if index is not None:
        if not 0 <= index < len(seq):
            return f"{index + 1}번째 색은 확인하지 못했어. 기록은 {len(seq)}번까지야."
        return prefix + f'{index + 1}번째는 {COLORS[seq[index]]}이야.'
    return prefix + '순서는 ' + ', '.join(COLORS[c] for c in seq) + '이야.'


def facts(record):
    if record.get('error'):
        return '신호등 관측 불일치: 색 순서나 횟수가 잘못되어 결과를 확정할 수 없음.'
    seq = record.get('sequence')
    order = ', '.join(COLORS[c] for c in seq) if seq is not None else '전달되지 않음'
    return f'신호등 점멸 순서: {order}. 이 기록은 끝난 점멸이며 현재 켜진 램프 색을 뜻하지 않음.'
