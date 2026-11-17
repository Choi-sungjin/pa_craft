# 작업 인계: LiDAR SLAM / 위치 추정 / Corner demo (2026-11-01)

다음 세션이 이 파일 하나로 이어갈 수 있게 정리했다. 마지막 상태(2026-11-01 밤): **Corner demo 실주행 성공**(목표 구석 도착 확인). 남은 과제는 ㄷ자 벽 안쪽 출발 시 벽으로 돌진하는 문제.

## 0. 사용자와 일하는 방식

- 답은 한국어로, 전문용어는 영어 그대로(power cable, USB hub, fault, launch, TF, FOV, scan matching 등).
  "사신 전력선" 같은 직역을 싫어한다.
- **하드웨어 값은 추측하지 말고 물어본다.** 부품(모터, 센서)은 순정이라 사양을 써도 되지만, 조립 방식과 판 크기는
  순정 Waffle이 아니라서 배치 값은 실측만 쓴다.
- 사용자는 Jetson(ssh)에서 실행하고 RViz는 노트북(`pa29@...Legion`)에서 본다. 명령은 **기기별로, 터미널별로** 준다.
  `ros2 launch`를 직접 칠 때는 `cd ~/craft && source install/setup.bash`를 꼭 붙인다.
- 커밋은 요청이 있을 때만 한다(아래 6절 참고).

## 1. 환경

| 항목 | 값 |
|---|---|
| 로봇 컴퓨터 | Jetson, Ubuntu 24.04, ROS 2 Lyrical, Fast DDS, `ROS_DOMAIN_ID` 61 (~/.bashrc), Wi-Fi `10.2.17.163` |
| 노트북 RViz | `export ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS=10.2.17.163` 후 `rviz2 -d ~/craft_slam.rviz` (`src/craft_bringup/rviz/slam.rviz`, LiDAR만 볼 땐 `lidar_check.rviz`) |
| USB hub `1-2.3` | 앞 LiDAR `1-2.3.4`, 뒤 LiDAR `1-2.3.1`, 바퀴 OpenCR `1-2.3.2`, 팔 OpenCR `1-2.3.3`. udev 규칙 `src/lidar_processor/udev/99-craft-lidar-jetson.rules` (적용됨) |
| 차체 | 축 간격 0.16 m, 좌우 0.289 m, 바퀴 반지름 0.033 m(순정), base_link = 앞뒤 축 가운데 |
| LiDAR (LDS-03 ×2) | **업사이드다운** 장착, 바깥쪽으로 약 6° 들림. TF: 앞 (x 0.155, yaw 180°, roll 180°), 뒤 (x −0.1295, yaw 0°, roll 180°), y 0, z 0.021. 스캔 0°가 로봇 안쪽을 향함. 두 LiDAR 가운데 `body_center_x` 0.01275. 병합 FOV 각 180°(바깥쪽 ±90°), `fov_near_m` 10 |
| IMU | 두 OpenCR 보드, 배율 0.999(5바퀴 기준선 실측, 보정 불필요), 기울기 1.5° 이내 |

## 2. 지금 하던 일: Corner demo (`demo/corner/`)

목표: 저장한 맵 `~/maps/arena3_clean` 안 **아무 곳에** 로봇을 놓아도 스스로 위치를 찾고 **오른쪽 아래 구석**
(map +x, −y 끝, 벽에서 0.30 m → (4.925, −3.363))까지 경로를 짜서 주행. 도착 방향은 상관없음.

구성: `ros2 launch craft_bringup corner_demo.launch.py`(control + LiDAR/EKF + slam_toolbox **localization mode** +
perception + planning, 맵은 posegraph/yaml 같은 base) → `demo/corner/run_corner.py`(global localization →
`/initialpose` → slam_toolbox 수렴 확인(TF map→base_link) → planning `START` → `/planning/target_base_pose` → `Run.Complete` 대기).
자세한 설명은 `demo/corner/README.md`.

**확인된 것**
- 오프라인: `demo/corner/test_map_tools.py` 가상 scan 6/6, 오차 ≤ 0.5 cm / 0.2°, 약 6초(6 cm × 3° 탐색).
- 오프라인: planning 알고리즘(Python 구현)으로 여러 시작점 → 구석 경로 생성 (`demo/corner/planned_paths.png`).
- 실기 `--dry-run` 1회: 로봇이 **map 원점 근처**에 있을 때 global (−0.315, −0.171, −1.9°) 점수 0.99 / 2등 0.74,
  slam_toolbox (−0.254, −0.185, −1.5°)와 6 cm 차이로 일치.

- 실기 global localization: 원점에서 먼 곳 (−0.64, −3.00, 93°) 0.97/0.76, (0.54, −1.15, 97°) 1.00/0.76,
  (3.57, −2.89, 270°) 0.94~0.96/0.64 모두 slam_toolbox와 4 cm 이내로 일치. `/initialpose`로 바로잡힘 확인.
- 실기 주행: 아래 수정 후 **목표 구석까지 도착 확인**(사용자 확인).

**다음 할 일: ㄷ자 벽 안쪽 출발 시 벽으로 돌진 (내일 1순위, 사용자 요청)**
- 증상: 로봇을 ㄷ자 벽 안쪽(맵 (0.15~0.9, −1.35~−0.6) 근처의 위가 열린 주머니, 예: (0.55, −1.19), 97°)에 놓으면
  벽으로 돌진한다. 사용자 관찰이며 로그/bag은 아직 없음 → 재현 때 `ros2 bag record --topics /corner/path /corner/trail
  /control/tracking_path /planning/diagnostics /tf /tf_static /lidar/merged /perception/lidar/obstacle` 권장.
- 알고 있는 것: 주머니 폭 약 0.75 m, planner padding 0.24 m(r_robot 0.21 + margin 0.03) → 자유 띠가 약 0.27 m뿐.
  앞선 실행에서 이 자리에서 `Run.Compute`가 약 12초 재시도(route unavailable) 뒤 출발했다. 그 뒤
  `path_planner.cpp`에 **start 탈출**(margin 안이어도 몸체 0.21 m가 비면 가장 가까운 자유 칸으로 나감, 첫 구간만 r_robot
  검사)을 넣었으므로, 탈출 칸이 벽 쪽/뒤쪽으로 잡히는지, 첫 구간 heading과 Pure Pursuit(lookahead 0.3) 제자리 회전,
  control spline 재샘플링이 벽 쪽으로 튀는지부터 의심. RViz 초록(경로)/노랑(궤적)으로 경로 문제인지 추종 문제인지 먼저 가른다.
- 사용자가 원하면 `~/.bashrc`에 `cornerdemo`/`cornerrun` 함수 추가(제안만 한 상태).

**2026-11-01 오후: ㄷ자 돌진 대응 (커밋 안 됨, 실기 미검증)**
- 원인(오프라인 확인): control Pure Pursuit가 경로 전체에서 '차체 앞쪽 첫 점'을 찾아, 근처 경로가 뒤에 있으면
  몇 m 떨어진 경로 후반부(구석 쪽 = 아래)를 집고 제자리 회전 없이 직진했다.
- planner: 사용자의 A*+arc(`astar_arc.py`)를 C++로 포팅해 기존 8방향 A* + shortcut을 교체
  (heading 상태 4방향 A*, fillet 이분 탐색, PASS/STOP_TURN/ALIGN). Python과 waypoint 40/40 일치,
  계획 시간 중앙값 24 ms / 최대 49 ms(LiDAR 점 400개 포함) vs Python 560 ms / 1058 ms. 출발점이 margin 안이면
  0.10 m 안의 자유 칸으로 직선 탈출(몸체 r_robot은 비어야 함).
- planning: 경로를 첫 STOP_TURN 코너까지만 보내고, 도착(SUCCEEDED)하면 그 자리에서 다시 계획해 새 task id로 보냄.
  주행 중 주기 재계획은 없애고, 0.2 s마다 남은 구간 막힘 검사(~5 ms)만 해서 막혔을 때만 재계획.
  diagnostics에 `plan_ms`, `plan_ms_max`, `more_segments`, 로그 `path plan ... ms`.
- control: 내 노드 = 직전 노드 ±3 샘플 안 최근접(동점은 앞쪽), lookahead는 경로를 따라 0.3 m, 목표점이 0.5 rad
  넘게 어긋나면 먼저 제자리 회전(0.25 rad 아래에서 추종 재개, 진행 방향 바뀌면 초기화). UPDATE_PATH 끝점 일치 조건 제거.
- 테스트: planning 76/76, control 52/52. 남은 것: 실기 ㄷ자 6방향, 근접 복구(직선 후진), 주행 중 안전 계층(차체 치수 필요).

**추종 under steer 보정 (2026-11-02 오후, 커밋 안 됨, 실기 A/B 대기)**
- bag `roi_1422`(1.63, −3.34, 0.7° → br 구석): 횡오차 RMSE 33 mm, 최대 77 mm. 코너(우회전) 뒤 직선에서 바깥쪽(+, 왼쪽)으로
  +40~70 mm가 몇 초 동안 이어짐 → under steer로 봄. map→odom 보정 점프(최대 62 mm)도 오차에 섞여 있다.
- 사용자 결정: Pure Pursuit 회전 명령에 계수 1.2. `control.yaml` `wheel_tracking_gain`(1.0 = 기존), 제자리 회전에는 적용 안 함.
  yaml은 install에 symlink라 값만 바꾸고 launch를 다시 띄우면 된다. 분석: `demo/corner/lateral_error.py <bag>`.
- 1.2 결과(`g12_1506`): 주행 RMSE 34.6 → 28.7 mm, 코너 뒤 직선 평균 +44~57 → +33~41 mm. 최대 76 mm는 S자(2.0, −2.1)에서
  lookahead 0.3 m가 다음 호를 보고 안쪽으로 질러감(1.0 때는 제자리 회전이 걸려서 없었음). 다음 시도: lookahead 0.18 m, 게인 1.2.
- lookahead 0.18 m + 게인 1.2(`la18_1521`): 주행 RMSE 16.8 mm, 최대 46.7 mm(S자 (2.06, −2.13)), 코너 뒤 직선 평균 +15~22 mm.
  흔들림·제자리 회전 증가 없음, task 3/3 성공. 비교 그림은 lateral_error.py CSV로 그림.
- 다음 시험(사용자 요청): planning `cruise_speed` 0.12 → 0.26 m/s(= max_v), lookahead 0.25 m, 게인 1.2. 펌웨어 Profile
  Acceleration 0.124 m/s²는 감속에도 걸리고 control은 도착 10 cm 안에서야 0을 보내므로 정지 거리 약 0.27 m → 끝점·STOP_TURN
  overshoot 약 0.17 m 예상(벽까지 0.30 m인 구석 목표에서 위험). 감속 구간은 아직 없음.
- 결과(`v26_1545`): 구간 1 끝점(4.49, −3.44)을 0.18 m 지나침 → 180° 돌아 복귀 → 구간 2에서 다시 180°. 코너(1.98, −1.83)에서도
  제동 지나침 87 mm. 바퀴 최고 0.24 m/s, 0.2 m/s 이상 9 s뿐, 가속 0.03~0.05 / 감속 0.10 m/s². 총 81 s(0.12 m/s 때 77 s)로
  오히려 느림. 주행 RMSE 49 mm(직선 구간만 28 mm). 결론: 끝점·코너 앞 감속 없이는 속도 올려도 이득 없음.

**LiDAR 바닥 보정과 비대칭 ROI (2026-11-02 새벽, 커밋 안 됨, 실기 미검증)**
- 22:00 전 bag은 LiDAR 각도 작업 전이라 쓰지 않는다. 평평한 바닥 `floor_cal_0034`(높이 앞 1.2 cm, 뒤 1.1 cm 실측):
  바닥 점이 V자 → 평면(pitch+roll) 모델 실패(rms 33~113 cm), 원뿔 0.6° + pitch 1.65°(바깥쪽 들림) + roll 0.1~0.2° 모델은
  rms 0.9~1.4 cm, 시간 분리 검증 OK, 다른 각도로 외삽은 부정확 → 경기장 bag 실측으로 ROI를 정함.
- 병합기(`scan_merger`) `floor_roi`: LiDAR별·좌우별 clear 각도(바깥쪽 기준) 안은 전부, 그 밖 ~90°는 5° 구간별 최대 거리.
  값은 perception.yaml `lidar.floor_roi` (fov180_2359, pocket_2309, pocket_2248). 교차 검증: 보지 않은 bag에서 바닥 점
  75~90 % 제거, 맵 벽 점 80~97 % 유지. `/lidar/merged` 자체에 적용되므로 SLAM·위치 찾기·장애물 모두 영향. 화각은 180° 기본.
- 분석 도구 복구(2026-11-09): `tools/lidar_floor/` (`derive_roi.py`가 위 표를 그대로 재현). 새 bag은 raw 스캔(`/lidar/front|rear/scan`) 포함해서 찍을 것.

**실기 bag 분석 후 수정 (2026-11-01 밤, 커밋 안 됨, 실기 미검증)**
- bag `~/bags/astar_arc_1804`, `pocket_2309`, `fov90_2321`: (1) 출발점이 margin 안이면 2~6 cm 탈출 구간 → control 도착 허용 10 cm에
  즉시 SUCCEEDED → 같은 자리 재계획 무한 루프, (2) 장애물 점 하나(잡음)·맵 벽을 4 cm 어긋나게 본 점·지나가는 사람으로 즉시 재계획.
- perception `LidarObstacleFilter`(`lidar_obstacle.filter`): 맵 벽 6 cm 안 점 제거, 3프레임 지속(주변 2셀), 3점 미만 군집 제거.
  맵 TF 없으면 raw 점 그대로.
- planning: 막히면 즉시 정지, `block_wait_s` 2 s 지나도 막혀 있으면 재계획(풀리면 같은 경로 재개). 시작점 `min_segment_m` 0.15 m 안
  STOP_TURN은 구간 끝으로 안 씀. 진행 없는(2 cm 미만) 구간 도착 3연속이면 Fault "no progress".
- 실행 편의: `corner_demo.launch.py arm:=false`(팔 보드 없이 주행), `lidar_fov_deg:=90`, `run_corner.py --near X Y`.
  udev: hub 소켓 번호로 구분(`2.?.N`), hub를 다른 Jetson 포트로 옮겨도 유지. 팔 OpenCR은 부팅 시 모터 초기화 fault(상태 2, 위치 0).
- LiDAR pitch를 올려서 180°에서 바닥을 찍음 → 현재 90°로 운용. 화각 확장 아이디어는 아래 대화 참고(하드웨어 변경 없이).

**퍽 graspable 판정 (2026-11-01 오후, 커밋 안 됨)**
- 사용자 결정: IK 대신 실험으로 구한 퍽 종류별(누움/섬) 3D 영역. **정렬이 먼저**(누운 퍽 장축이 차체 전방과
  `axis_tolerance_deg` 안, 섬은 방향 무관), 정렬된 경우에만 퍽 중심(base_link)이 영역(margin 축소) 안인지 판정,
  `confirm_count`회 연속이면 확정. `src/craft_planning/src/grasp_region.cpp`, `planning.yaml` `grasp_region`.
- 영역 값은 아직 미측정(전부 0 = 항상 불가). 실험 때 diagnostics `puck_base_link`로 퍽 좌표를 읽어 채운다.
- **TODO (사용자 실측 대기)**: 잡기 가능 영역은 임시로 base_link 축 정렬 직육면체로 구현했다. 사용자가 퍽 종류별
  (누움/섬)로 실측해서 알려 주면 **모양(직육면체가 맞는지, 다면체·원통 등)과 값**을 반영한다. 받기 전에는 추측해서 넣지 않는다.
  고칠 곳: `grasp_region.hpp`의 `GraspBox`·`check_graspable()`, `planning.yaml` `grasp_region`, `test_grasp_region.cpp`.
- 정지 위치 비용(사용자 결정): `J = w_box/(d_short − 0.03) + w_grasp·((s − s_good)/σ)²`. d_short = 앞면(LM 가이드)↔박스
  최단거리, s = 앞면→퍽 중심, 안전거리 3 cm에서 무한대. `plan_stop()`. 가중치는 사용자가 실험으로 조정.
  앞면 위치·폭, s_good은 실측 대기(TODO). 2차 장벽(d_ref)과 비교: 연산량 차이는 무의미(7 ns vs 49 ns).
- 퍽 centerpose(perception)는 TODO라 실제 판정은 그 뒤에 의미가 있다. 다음: control `APPROACH`, `pick_puck_from_box`
  하위 미션(A1 staging → A2 observe → A3 approach → A4 graspable 정지 → A5 verify → A6 pick&load).

**이번에 고친 것 (2026-11-01 밤, 커밋 안 됨)**
- timeout: control `task_timeout_s`, planning `fsm.execution_timeout_s` 60 → 180 s (0.12 m/s로 ~7 m 경로가 60 s 초과,
  1.4 m 남기고 멈춤), `run_corner.py --timeout` 240 s.
- `planning_master.cpp` `receive_points()`: TF 변환 실패(scan이 TF보다 몇 ms 앞섬) 시 직전 장애물 sample을 지우지 않음.
  예전엔 그 순간 Execute가 Fault. Execute Fault 메시지도 남은 `readiness_reason` 대신 `stale odom/lidar input` /
  `stale control task result`.
- `path_planner.cpp`: shortcut이 벽에서 딱 padding(0.24 m)으로 붙어 지나가 추종 오차로 start가 margin 안에 들어가면
  재계획이 영원히 실패(벽 끝 (3.81, −2.82)에서 0.240 m, 1.4 m 남기고 정지) → shortcut은 먼저 padding + prox_band/2로
  시도, start 탈출 추가, goal 막힘 메시지 분리. `colcon test` 75/75 통과.
- `run_corner.py`: RViz용 latched `/corner/goal`, `/corner/path`(`/control/tracking_path` 복사), `/corner/trail`
  (map→base_link 2 cm 간격). `slam.rviz`에 3개 display 추가. Ctrl+C가 rclpy 메시지 수신 중에 걸리면 RuntimeError로
  STOP을 건너뛰던 문제 → SIGINT는 flag만 세우고 `spin()`에서 KeyboardInterrupt (dry-run에서 STOP 전송 확인).
- 경로 알고리즘 참고: 실행되는 건 C++ 8방향 A*(heading/근접 비용) + line-of-sight shortcut(직선 꺾은선)이고,
  `astar_arc.py`(arc fillet)는 참고용으로만 남아 있다.

**알아 둘 것**
- slam_toolbox는 로봇이 서 있으면 scan을 처리하지 않아 `/perception/lidar/pose`가 안 나온다. 위치는 TF로 읽는다.
- `craft.launch.py slam_map:=<base>`가 비어 있으면 기존 mapping 그대로, 값이 있으면 `localization_slam_toolbox_node`
  + `config/slam_localization.yaml`(map_start_pose 0,0,0, stack_size_to_use).

## 3. 이번 세션에서 바꾼 것 (요약)

| 영역 | 내용 | 위치 |
|---|---|---|
| 휠 odom | 실측 yaw rate 맵 `wheel_yaw_rate_coeffs: [0.099025]`(1차, turn_map 실험), `skid_steer_factor` 1.153, wheelbase 0.16 | `src/craft_control/config/control.yaml`, `wheel_odometry.cpp` |
| 슬립 검출 | 휠 vs 바퀴 보드 gyro yaw rate 차 > 0.1 rad/s → 휠 twist 분산 ×100 (0.5 s 유지) | `src/craft_control/src/slip_monitor.cpp` |
| LiDAR odom | `ScanRelocalizer` 기반 keyframe scan matching → `/perception/lidar/odom` | `src/craft_perception/src/scan_odometry.cpp`, perception.yaml `lidar_odometry` |
| EKF | `lidar_odom:=true` 시 `ekf_lidar.yaml` 덧씌움(LiDAR vx/vy 주, IMU yaw, 휠 큰 공분산) | `src/craft_perception/config/ekf_lidar.yaml` |
| SLAM | `slam_search:=wide`(±25 cm, 기본), `slam_map:=`(localization) | `slam_wide.yaml`, `slam_localization.yaml`, `craft.launch.py` |
| 업사이드다운 | TF roll 180°를 2D 변환에서 인식(스캔 각도 부호 반전) | `lidar_processor/scan_merger.hpp`, perception, `lidar_calibrator` |
| 실행 편의 | `slam.launch.py`(+bag 자동 `loop_<n>`), `save_map.sh`, `corner_demo.launch.py`; `~/.bashrc`의 `slamall`, `slamsave`(고침: `map_saver_cli -t`) | `src/craft_bringup/` |
| 분석 도구 | `test/drive/`: `turn_map.py`, `gyro_spin.py`, `odom_compare.py`, `odom_trajectories.py`, `slam_graph.py`; `test/lidar/lidar_check.py` | 각 README |
| posegraph | 들어 올린 구간 삭제 도구 → `arena3_clean.*`(scan 0~1175) | `tools/mapmaker/posegraph/` |
| 맵 | `~/maps/arena3.*`(원본), `arena3_clean.*`(데모용) | |

## 4. 보류 중인 문제

- (2026-11-01 밤) 앞 LiDAR CP2102가 `-110` timeout으로 멈춰 launch가 `Failed to open lidar port` → `map` frame 없음.
  `sudo sh -c 'echo 0 > /sys/bus/usb/devices/1-2.3.4/authorized; sleep 2; echo 1 > ...'` 또는 재연결. 팔 OpenCR도 hub 끊김 뒤
  feedback 정지(`arm_lm_state` 2) → `resetarm`/power cycle. 실행 전 `arm_lm_state` 0, `/lidar/front/scan` 10 Hz 확인.

1. **USB hub 끊김(최우선)**: hub 전체가 수시로 끊김(커널 `clear tt error -71`, `EMI?`). 벽에 부딪혀 바퀴가 막힐 때도
   발생. 끊기면 LiDAR driver가 `ioctl return value is -1`을 내며 죽은 포트를 붙잡고 있어 launch 재시작 필요.
   사용자가 **power cable을 사서 hub 전원 분리 예정** → 장착 후 `journalctl -k`로 끊김 0건 확인.
2. **resetwheel 후 LATCHED FAULT**: Ctrl+C 종료 문제 아님(모든 프로세스 정상 종료 확인). 펌웨어가 boot 시 Dynamixel
   hardware error가 남아 있으면 fault로 고정 → power cycle 필요. 제안(미착수): boot 시 hardware error 모터에 reboot
   instruction 후 재확인, fault 원인(motor ID, 단계)을 feedback으로 보고; `resetwheel`이 upload 후 포트 재등장을 최대
   15초 기다리게(`FileNotFoundError` 방지). `~/.cache/craft_fw/log/`에 기록.
3. **LiDAR 기울기**: 바깥쪽 6° 들림 → 40 cm 미로 벽은 약 3.5 m, 7 cm 상자는 약 0.45 m까지만 보임. 1.5~2° 권장(차체 숙임 1.4°보다 큼).
   차체를 살짝 높인 뒤 `laser_z`(현재 0.021) 재측정 요청했으나 답 없음.
4. **LiDAR odom `min_score` 0.4**, 공분산은 초기값. 실기 `LiDAR odometry score` 로그 확인 필요.
5. 휠 odom CW/CCW 비대칭(약 ±2%)은 하중 바뀔 때 다시 보기로 함. LiDAR는 driver 종료 후에도 계속 돈다(정상, 정지 명령 없음).

## 5. 함정 모음

- 맵 저장은 **SLAM을 끄기 전에, Jetson에서**: `slamsave <name>` (내부 `map_saver_cli -t /perception/lidar/map`; `-r map:=`은 무시됨).
  놓쳤으면 bag의 `/perception/lidar/map`에서 꺼낼 수 있다(그래서 `slamall`이 bag을 기본 기록).
- `ros2 bag record`는 `--topics` 필요(Lyrical).
- slam_toolbox Karto 링크 시 `${slam_toolbox_LIBRARIES}` 쓰지 말 것(ld 20분+). `tools/mapmaker/posegraph/CMakeLists.txt` 참고.
- `resetwheel`/`bro`는 보드 잠금(`_craft_locked`)을 쓴다. launch(`slamall`, `corner_demo`)는 보드 점검을 안 한다.
- 오늘 아침 udev를 잘못 바꿨다가 되돌림: 앞뒤가 바뀌어 보인 원인은 포트가 아니라 yaw 180° 오류였다. 판정은 손으로 LiDAR를
  가려 어느 장치가 막히는지 보는 방식이 확실하다.

## 6. Git 상태

마지막 커밋 `be55493` 이후 **커밋 안 됨**. 이번 세션 변경(3절) 외에 이전부터 있던 수정도 섞여 있다:
`firmware/*`, `config/udev/*`, `test/slip/slip_test.py`, `test/test_motor_axes.cpp`, `firmware/imu_stream/` 등.
`test/slip/logs/*.csv` 삭제는 하드웨어 변경으로 사용자가 요청한 것. 커밋 요청이 오면 영역별로 나눠서, 이전부터 있던
수정은 사용자에게 확인한 뒤 포함한다.
