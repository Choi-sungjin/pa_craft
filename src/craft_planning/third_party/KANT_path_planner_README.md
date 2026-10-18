# A* + arc Path Planner (정적 환경 1단계)

`astar_arc.py` 한 파일:
- `run(map)` — 맵 받아서 경로(`PlanResult`) 리턴. print·저장·시각화 없음. 맵의 r/l/u/d 목표를 순서대로 방문
- `visualize(res)` — 리포트 print + matplotlib 그림 (matplotlib은 이 함수 안에서만 import)
- `__main__` — `run()` → (SAVE) `save_result()` → (VISUALIZE) `visualize()`

## 프로젝트에서 사용
```python
from astar_arc import run, Params, MAP

res = run(MAP, start=(2, 2), goal=(17, 17), p=Params(lam=0.6))   # 문자열/행 리스트/GridMap
res.legs         # 목표별 구간 [Leg(goal, goal_h, waypoints, path), ...]
res.waypoints    # [Waypoint(x, y, theta, mode(PASS/STOP_TURN/ALIGN), R, v_max, leg), ...]
res.path         # (N,5) [x, y, yaw, kappa, v_limit]
res.ok           # padding 검증 통과 여부
```

## 단독 실행
```bash
cd ~/craft/path_planner
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python astar_arc.py
```
새 터미널에서는 `source .venv/bin/activate`로 가상환경을 활성화합니다. 종료는 `deactivate`입니다.

`__main__` 블록에서 설정:
- `VISUALIZE = True/False` — 시각화 on/off
- `SAVE = True/False` — `out/` 에 json, csv, png 저장 on/off
- `params = Params(...)` — 파라미터 조정
- `map_data = gen_random_map(seed=3)` — 랜덤 맵 사용 (주석 해제)

## 맵 만들기
- `astar_arc.py` 상단 `MAP` 문자열 수정: `#` 장애물, `.` 빈칸
- **`r` `l` `u` `d` = 미션 목표(puck / knob 앞 정지 위치) + 진입 방향**
  - r: 오른쪽(+x)을 보며 진입, l: 왼쪽(-x), u: 위(+y), d: 아래(-y)
  - 글자 칸 = 로봇 중심 정지 위치. 도착 직전 `approach_len`(0.3 m)은 그 방향으로 **직진만** (원호·회전 없음)
  - 목표물(`#`)과 글자 사이 빈칸 1칸 이상, 글자 뒤쪽(진입 반대편)에 approach 공간 필요 — 부족하면 에러 메시지로 알려줌
  - 여러 개면 읽는 순서(위→아래, 왼→오)로 방문. 순서 지정: `run(targets=[(17, 10, 'r'), (8, 7, 'u')])`
- 시작: `START = (col, row)` (row는 첫 줄이 0) 또는 맵에 `S`
- `G` / `GOAL` = 방향 무관 목표 (r/l/u/d 가 없을 때만 사용)
- 1칸 = 0.25 m, A*는 0.05 m 격자 → 20x20 = 5 m × 5 m

## 미션 목표 경로 계획 (plan_leg)
1. 진입점 E = 목표 − L·u(dir). E 까지 A* 를 돌리되 **E 에서 heading = dir 인 상태만 도착으로 인정**
2. E → 목표 직진 구간을 붙임. 이 구간(approach_len)에는 원호가 못 들어가도록 직전 코너의 R 상한을 제한
3. L = approach_len + {R_hi, R_hi/2, 0} 을 모두 시도 → 예상 시간(주행 + 회전 + 정지 가감속)이 가장 짧은 것 채택
   (L 이 길면 진입 전 코너를 원호로 통과 가능, 짧으면 진입점에서 제자리 회전)
4. 도착 waypoint mode = `ALIGN` → Motion FSM 에서 RealSense 정렬로 연결
5. 다음 목표는 도착 heading 에서 출발 (제자리 회전 포함)

결과: `res.legs[i]` (목표별 waypoints / path), `res.waypoints` / `res.path` (전체 이어붙임, `Waypoint.leg` 로 구간 구분)

## 처리 순서
1. **C-space 변환**: ESDF(장애물 거리장) < r_robot + margin 인 셀을 점유로 padding → 로봇 = 질점
2. **A\***: 상태 (x, y, heading∈E/N/W/S), 전진 = res·(1+벽근접비용), 90° 회전 = λ, h = 맨해튼 + 최소회전수·λ
3. **코너 압축**: heading이 바뀌는 셀만 waypoint로
4. **fillet**: R 상한 = min(R_hi_global, len_in − R_prev, len_out(다음 코너도 PASS 후보면 ½)) → 원호가 C-space free인 최대 R 이분 탐색
5. **mode**: 시작/목표/180° 반전은 강제 STOP_TURN, R ≥ R_MIN → PASS, 아니면 STOP_TURN
6. **v_max** = min(v_ref, ω_max·R, √(a_lat·R))
7. **검증**: 최종 경로 전 구간 연속 ESDF 최소값 ≥ padding

## 파라미터 (`Params`)
| 이름 | 기본 | 의미 |
|---|---|---|
| r_robot / margin | 0.21 / 0.03 m | padding = 0.24 m (외접원, 실측 후 확정) |
| lam | 0.6 m | 90° 회전 비용 |
| w_prox / prox_band | 1.0 / 0.15 m | 벽 근접 비용 → 통로 중앙 선호 |
| r_min / r_hi_global | 0.08 / 0.40 m | PASS 최소 반경 / 원호 반경 상한 |
| v_ref / w_max / a_lat_max | 0.26 m/s / 1.82 rad/s / 0.5 m/s² | Waffle Pi 스펙 기반 |
| approach_len | 0.30 m | 미션 목표 직전 직진 구간 |
| a_lon_max | 0.5 m/s² | 정지 1회 시간 손해(v_ref/a) — 진입 방식 선택에 사용 |
