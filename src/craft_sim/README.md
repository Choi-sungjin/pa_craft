# craft_sim — 2D 플래닝 시뮬레이터

플래닝 코드를 바꿀 때마다 실물 주행을 하지 않고 PC에서 먼저 확인하기 위한 가벼운 2D 시뮬레이터.

- **실제 `planning_master`·`control_master` 바이너리를 그대로 실행**한다. 경로 생성(global_local), 상태기계, 박스 접근·도킹 단계, Pure Pursuit·룩어헤드, task ID·hold·정지 확인까지 로봇과 같은 코드다.
- 시뮬레이터는 **바퀴 OpenCR 보드(가상 시리얼 PTY)** 와 **`/perception/*` 입력**만 대신한다.
- 기본 가정: 이상적 차동구동(명령한 바퀴 속도 = 실제 속도), 위치·LiDAR·카메라 참값. 시나리오 옵션으로 실측 조건(LiDAR 간격 어긋남, 카메라 지연·무효 프레임)을 넣을 수 있다.
- 두 master 가 실제 시계 타이머를 쓰므로 **1배속**으로 돈다(박스 도킹 한 번 약 70 초).

## 빠른 사용

저장소 루트에서:

```bash
src/craft_sim/sim.sh build                  # 처음 한 번, 코드 바꾼 뒤마다
src/craft_sim/sim.sh run 02_box_L1 --rviz   # 시나리오 하나 + RViz
src/craft_sim/sim.sh all                    # scenarios/*.yaml 전부 + 요약표
src/craft_sim/sim.sh rviz                   # 같은 ROS 환경으로 RViz 만
src/craft_sim/sim.sh test                   # 단위 시험
```

- 결과: `~/craft_sim_runs/<시나리오>_<시각>/` (`CRAFT_SIM_OUT` 으로 변경). `report.html`(지표·지도·시간 그래프·단계 기록·로그), `result.json`, `record.json`(원시 기록), `planning.log`, `control.log`, `sim.log`, `overrides.yaml`. 요약표는 `~/craft_sim_runs/summary.html`.
- 종료 코드: 모든 요청이 기대(`expect`, 기본 `complete`)대로 끝나면 0, 아니면 1.
- `run` 에는 시나리오 이름(`02_box_L1`) 또는 아무 yaml 경로를 준다.
- `sim.sh` 는 ROS 환경을 직접 맞춘다: `ROS_DOMAIN_ID=63`(`CRAFT_SIM_DOMAIN`), `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, Fast DDS. 로봇이나 다른 PC와 섞이지 않는다. 같은 컴퓨터에서 다른 프로그램이 같은 topic 을 내고 있으면 실행 직후 멈추고 알려 준다.

### 빌드 메모

- **ncnn 이 있는 컴퓨터(젯슨)**: 저장소 작업공간에 그대로 `colcon build` 한다.
- **ncnn 이 없는 PC**: `craft_perception` 대신 메시지 + `tag_registry` 만 만드는 `perception_stub/` 으로 시뮬레이터 전용 작업공간 `.sim_ws/` 를 따로 만든다(로봇 코드는 그대로, `.gitignore` 에 등록됨).
- `craft_planning` 이 팔 IK 용으로 pinocchio 를 링크한다. 없으면 `sudo apt install ros-$ROS_DISTRO-pinocchio`. 다른 곳에 설치된 pinocchio 를 쓰려면 `CRAFT_SIM_PINOCCHIO=<prefix>`.
- yaml(planning.yaml, control.yaml)만 바꿨으면 다시 빌드하지 않아도 된다. 시뮬레이터는 저장소 `src/` 의 yaml 을 바로 읽는다(`CRAFT_SIM_SRC` 로 다른 트리 지정).

## 구성

| 파일 | 역할 |
|---|---|
| `sim.sh` | 환경 고정 + build/run/all/rviz/test |
| `craft_sim/sim_node.py` | 가짜 바퀴 보드(PTY, 50 Hz) + 참값 센서 + 박스·카메라 관측 + RViz 마커 |
| `craft_sim/world.py` | 맵(yaml+pgm), 이상적 차동구동 원호 적분, 광선 투사, 박스 기하(`fit_box_face` 계약), 장애물 필터 |
| `craft_sim/run_scenario.py` | sim_node → control_master → planning_master 실행, run_corner.py 와 같은 순서(RESET → START → 요청), 기록 |
| `craft_sim/report.py` | 지표·그림·HTML |
| `craft_sim/summary.py` | 시나리오별 최신 결과 요약표 |
| `perception_stub/` | ncnn 없는 PC 용 craft_perception 대체 (`COLCON_IGNORE`: 저장소 빌드에는 안 들어감) |
| `rviz/sim.rviz` | 맵·추종 경로·룩어헤드 점·장애물 점·차체·박스·궤적 |
| `scenarios/` | 시나리오 yaml |
| `maps/` | 경기장 맵 `arena5_legend_opt_arena1cm` (corner_demo 기본 계획 맵) |
| `test/` | 단위 시험: 차동구동, 맵, 카메라·LiDAR 관측이 `demo/corner` 의 브리지 함수와 같은지 |

### 시뮬레이터가 대신 내는 것

| topic | 실제 발행자 | 시뮬레이터 |
|---|---|---|
| 바퀴 보드 시리얼 | OpenCR | PTY. `wl,wr,clock` 받아 그대로 실행, 20 ms 마다 `wl,wr,clock,wheel_state,clock_state,imu_ok,gx,gy,gz,ax,ay,az`. 정지 판정 0.05 rad/s, 명령 250 ms 끊기면 바퀴 0·두 축 state 2 (펌웨어와 같음) |
| `/perception/state/odom` (map) | perception_master(EKF+SLAM) | 참값 50 Hz |
| `/perception/lidar/odom`, `/perception/imu/data` | perception_master | 참값 10 Hz / 50 Hz, stamp 증가 (`require_lidar_localization: true` 감시 통과) |
| `/perception/lidar/obstacle` (map) | LiDAR 장애물 필터 | 앞(x 0.155)·뒤(x −0.1295) 180° 라이다 광선이 맵에 없는 물체(장애물·박스)에 맞은 점 → 실제 필터 단계: 맵 벽 6 cm 이내 제거, 3 m 이내, 5 cm 칸 3프레임 지속(이웃 2칸), 묶음 3점, 기억(광선 3프레임 연속 통과 시 삭제, 새 Navigate EXECUTE 때 초기화) |
| `/perception/box/observation` (LiDAR, map) | uppercam_sensor_bridge (`fit_box_face`) | 요청이 lidar 일 때 10 Hz. **`fit_box_face` 와 같은 계약**: 면 기울기·법선 방향 위치는 참값, 접선 방향 중심은 요청의 설정 위치(prior), 간격은 전방축이 면과 만나는 거리. 앞 라이다 1° 광선이 원점 0.10~1.0 m 에서 면에 6점 이상·8 cm 이상 닿을 때만 valid. 불확실도 10 mm |
| `/perception/uppercam/observation` (base_link) | uppercam_sensor_bridge | 요청이 uppercam 일 때 15 Hz, 간격(앞면 중점의 base x) 0~200 mm 에서만 valid, 불확실도 3 mm. 플래닝은 간격만 씀 |
| `/lidar/merged`, TF `map→odom→base_link` | 드라이버·EKF·SLAM | RViz 표시용 병합 스캔, 참값 TF |
| `/planning/map` | map_server | 같은 맵 |

`--fake-detect`(박스 위치만 `/perception/box/pose`)와 달리, 현재 기본값인 `camera_enabled: true` 흐름(stage → LiDAR 도킹 → 100 mm 전환 → 카메라 도킹 → 최종 검사)을 끝까지 돈다. 노드 파라미터는 corner_demo launch 와 같게 덮어쓴다: `arm_enabled: false`, `require_arm_joints: false`, `require_lidar_localization: true`, `map_file`.

## 시나리오 형식

```yaml
name: box_L1_offset
description: 설명 (보고서에 표시)
map: maps/arena5_legend_opt_arena1cm.yaml   # 패키지 기준 상대 경로 또는 절대 경로
start: [0.527, 5.889, -4.5]                 # x, y [m], yaw [deg]
timeout_s: 240                              # 요청별 기본 시간 제한
settle_s: 2.0                               # 시작 후 대기
box_offsets:                                # 실제 박스를 planning.yaml 위치에서 옮김 (dx, dy [m], dyaw [deg])
  L1: [0.030, -0.020, 2.0]
obstacles:
  - name: 상자A
    shape: circle            # circle | rect
    center: [1.97, 3.45]
    radius: 0.12             # rect 는 size: [w, h], yaw_deg
    appear: {robot_within: [1.97, 3.45, 0.9]}   # 생략 = 처음부터 / {after_s: 10} = 첫 요청 후 10 초
    disappear: {after_s: 5}                     # 나타난 뒤 5 초에 사라짐 (생략 = 계속)
params:                      # 노드 파라미터 덮어쓰기 (로봇 yaml 뒤에 적용, 적은 값만 바뀜)
  planning:
    box_approach: {lateral_tolerance: 0.02}
  control:
    lookahead_distances: [0.21, 0.21, 0.25]
sim:                         # 시뮬레이터 옵션 (모두 생략 가능)
  footprint: [-0.20, 0.165, -0.165, 0.165]   # 충돌 검사용 차체 사각형 base_link (추정값)
  boxes_as_obstacles: true
  camera_hz: 15
  # 실측 조건 재현 (2026-11-09 실주행·bag 기준)
  lidar_gap_drift: [0.53, 0.024]   # [기준 간격 m, 기울기]: 가까워질수록 LiDAR 간격이 이동량보다 덜 줄어듦
  camera_latency_s: 0.19           # 촬영 → 도착 지연
  camera_burst_rate_hz: 0.36       # 무효 프레임 묶음(nonfinite camera geometry) 빈도
  camera_burst_frames: 2
  camera_valid_max_gap: 0.10       # 이 간격보다 멀면 카메라 무효 (0 = 끔)
  camera_seed: 1
actions:                     # 차례로 실행, 기대와 다르면 중단
  - {type: goal, pose: [2.6, 2.9]}          # [x, y] 또는 [x, y, yaw_deg]
  - {type: box, name: L1}                   # /planning/approach_box
  - {type: box_near, name: L1, expect: complete, timeout_s: 90}   # /planning/approach_box_near
```

## 지표 (report.html)

- 결과(완료 / Fault / 시간 초과)와 소요 시간, 주행 거리, 최고 속도
- **횡오차**: control 이 실제로 추종 중인 경로(`/control/tracking_path`) 기준, 이동 중(|v|>0.01 m/s)만, RMSE·최대·p95. 진행 방향 왼쪽 +
- 방향 오차(경로 접선 기준) RMSE
- 목표점: 목표까지 남은 거리(도착 허용 0.10 m 때문에 보통 약 100 mm)
- 박스: 최종 간격(LiDAR 앞면→박스 면, 참값)·횡오차·방향 오차와 허용치, 박스 단계별 시간, box_status 기록, 간격-시간 그림
- EXECUTE/UPDATE_PATH/STOP 수, 추종 경로 갱신 수, 장애물 최소 간격, 충돌(차체 사각형이 맵 벽·장애물·박스와 겹침, 맵에 그려진 박스 외곽은 벽으로 치지 않음)

## 시나리오

| 시나리오 | 내용 |
|---|---|
| 01_goal_basic | L1 시험 출발점 → (2.6, 2.9) 전역 경로 주행 |
| 02_box_L1 | 박스 L1 전체 흐름 (이상적 센서) |
| 03_box_near_L1 | 80 mm 앞 정렬 정지에서 근거리 도킹 |
| 04_goal_blocked_replan | 주행 중 경로 위 장애물 출현 → 2 s 대기 → 재계획 |
| 05_box_L1_offset | 실제 박스 +30 mm, −20 mm, +2° 어긋남 |
| 06_box_near_L1_offset | 근거리 출발 횡 +6 mm, +0.5° |
| 07_box_L1_skew | 실제 박스 −20 mm, +2° (x 그대로) |
| 08_box_L1_lidar_drift | 실주행 재현: 가까워질수록 LiDAR 간격이 덜 줄어듦 |
| 09_box_L1_camera_real | 08 + 실측 카메라(7.7 Hz, 190 ms 지연, 무효 묶음) |
| 10_box_L1_camera_real100 | 09 + 100 mm 보다 멀면 카메라 무효 |

### 이 브랜치 기준(control-test 7ef54d4) 결과 (2026-11-10)

| 시나리오 | 판정 | 참값 최종 검사 | 비고 |
|---|---|---|---|
| 01 · 04 | 통과 | - | 횡오차 RMSE 7~9 mm, 04 는 2 s 대기 후 우회 |
| 02 · 08 · 09 · 10 | 통과 | 허용 | 박스 도킹 약 47 s, 최종 간격 12.4~12.7 mm |
| 03 · 06 근거리 도킹 | **실패** (1.5 s) | - | control 이 도킹 task 거부: `invalid task target/calibration` |
| 05 박스 +30/−20 mm, +2° | 실행마다 다름 | **초과** | 최종 간격 12.8~13.1 mm(허용 상한 13 mm 경계), 참값 횡오차 23 mm |
| 07 박스 −20 mm, +2° | 통과 | **초과** | 플래닝은 정렬 완료로 판단, 참값 횡오차 23 mm |

- 03·06: 근거리 진입에서 planning 이 `camera_distance_control=true` 인 EXECUTE 를 속도 0.10 m/s 로 보내는데, control 은 카메라 모드 task 의 속도를 `camera_distance_max_speed`(0.005 m/s) 이하만 받는다. 메시지 계약은 그대로라 시뮬레이터가 아니라 planning·control 사이의 어긋남이다.
- 05·07: LiDAR 면 맞춤은 박스의 접선 방향 위치를 알 수 없어(설정 위치를 prior 로 씀), 실제 박스가 옆으로 20 mm 밀려 있으면 플래닝은 정렬됐다고 보지만 참값으로는 허용 20 mm 를 넘는다. 보고서의 "참값 기준 최종 검사"가 이 경우를 따로 표시한다.
- 이전 코드(11-09)에서 본 현상: 근거리 출발 횡 +6 mm 이면 최종 방향이 1° 를 넘음, 계획 맵의 L1 그림이 설정 면보다 3.5 cm 서쪽까지 칠해져 있어 박스가 동쪽으로 어긋나면 도킹 경로가 막힘.

## 한계

- 1배속(두 master 가 실제 시계 타이머 사용). 카메라 프레임 시점 같은 수 ms 차이로 허용 경계에 걸린 경우는 실행마다 판정이 바뀔 수 있다.
- 차체 동역학·미끄럼·바퀴 지연·위치 점프 없음. 센서 잡음은 시나리오 옵션으로 넣는 것만 있다.
- 장애물 필터·LiDAR 면 맞춤은 단계와 계약은 같지만 입력이 잡음 없는 참값이다(평행면 모호성·가림 없음).
- 충돌 검사 차체 사각형은 추정값(`sim.footprint`).
- 팔·주사위·퍽 미션은 범위 밖(arm:=false 구성).
