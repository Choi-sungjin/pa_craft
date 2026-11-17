# path_compare

기존 A*+arc와 8방향 A* + Spline, 8방향 A* + Spline + 곡선경로를 같은 맵·같은 C-space·같은 구간에서 비교한다.

```bash
/usr/bin/python3 ~/craft/tools/path_compare/compare_planners.py [--map ~/maps/arena5_legend_arena.yaml] [--approach-len 0.25]
# -> out/leg1..3.png (경로·곡률·속도·표), out/summary.md / .csv
```

- C-space: `planning.yaml` path_planner의 `r_robot + margin`(0.243 + 0.05 = 0.293 m), 같은 벽 근접 비용. 격자 = 맵 해상도
  (C++ 플래너와 같게. Python 레퍼런스 `astar_arc`는 원래 `Params.res` 0.05로 세므로 여기서 맞춰 준다).
- 목표 접근 방향: 세 방법 모두 목표 직전 `approach_len`은 목표 heading 방향 직선(A*+arc의 ALIGN). 8방향 두 방법은 진입점
  E까지 탐색하고 직선을 붙이며, 꺾여 들어오면(>10°) 진입점 제자리 회전으로 센다(A*+arc의 STOP_TURN과 같게).
- 시간 모델(`astar8_spline.profile`): v ≤ v_ref, √(a_lat/κ), w_max/κ, 가감속 a_lon, 제자리 회전은 |Δyaw|/w_max.
- 방법(`src/craft_planning/src/astar8_spline.py`):
  - 8방향+Spline: 8방향 A* 셀 경로 → cubic B-spline(반 셀 RMS 허용, 충돌 시 smoothing 절반).
  - 8방향+Spline+곡선경로: 8방향 A* → line-of-sight 가지치기 → 장애물 여유(pad+0.05)·곡률(κ ≤ min(a_lat/v_ref², w_max/v_ref))
    반복 스무딩(점은 free C-space 안, 시작·접근 직선 고정) → cubic spline.
- 표의 계획 ms / Hz는 세 방법 모두 Python 구현 기준(공정 비교용). 실제 주행에 쓰는 C++ A*+arc는 젯슨(Orin Nano Super)에서
  같은 구간·목표 heading 포함 median 20.5 / 21.6 / 33.2 ms = **48.7 / 46.3 / 30.1 Hz**.

## 초레지 결과 (arena5_legend_arena, 2026-11-07, approach_len 0.25 m)

| 구간 | 방법 | 길이 m | 예상시간 s | 정지 | 최대 κ | 최대 Δκ | 최소여유 m | 계획 ms (Python) | Hz (Python) | Hz (C++ 젯슨) |
|---|---|---|---|---|---|---|---|---|---|---|
| start → L2 (남쪽 보며) | A*+arc (기존) | 6.88 | 30.7 | 3 | 8.3 | 8.70 | 0.296 | 435 | 2.3 Hz | 48.7 Hz |
| | 8방향+Spline | 6.41 | 28.2 | 2 | 5.2 | 1.31 | 0.310 | 163 | 6.1 Hz | - |
| | 8방향+Spline+곡선경로 | 6.00 | 26.2 | 2 | 4.9 | 1.29 | 0.300 | 562 | 1.8 Hz | - |
| start → L3 (북쪽 보며) | A*+arc (기존) | 7.37 | 32.6 | 3 | 8.3 | 8.33 | 0.295 | 456 | 2.2 Hz | 46.3 Hz |
| | 8방향+Spline | 7.06 | 30.4 | 2 | 5.1 | 3.30 | 0.295 | 211 | 4.7 Hz | - |
| | 8방향+Spline+곡선경로 | 6.63 | 28.4 | 2 | 9.3 | 8.92 | 0.295 | 642 | 1.6 Hz | - |
| L3 → L1 (동쪽 보며) | A*+arc (기존) | 5.41 | 23.0 | 1 | 4.5 | 6.25 | 0.295 | 653 | 1.5 Hz | 30.1 Hz |
| | 8방향+Spline | 5.50 | 24.2 | 2 | 13.8 | 7.99 | 0.295 | 141 | 7.1 Hz | - |
| | 8방향+Spline+곡선경로 | 5.05 | 21.5 | 1 | 6.3 | 0.78 | 0.295 | 519 | 1.9 Hz | - |

- L3를 남쪽에서 접근하면 목표 뒤 곧은 여유가 0.28 m뿐이다(pad 0.293). approach_len 0.30이면 세 방법 모두(C++ 포함) 거부한다.
- 정지 수에는 출발 제자리 회전이 들어 있다. 도착 heading 오차는 세 방법 모두 0°.
