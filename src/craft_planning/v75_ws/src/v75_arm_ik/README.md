# v75_arm_ik

URDF 기반 팔 IK (C++ / Pinocchio, ROS 없이 단독 실행).

**로봇 치수는 코드에 없다.** 링크 길이, 관절 위치·축·한계, TCP 위치를 실행할 때마다 URDF에서 읽는다.
로봇이 바뀌면 **URDF만 교체**하면 되고 재빌드도 필요 없다.

### 수직 하향 잡기 전략: 앞뒤는 바퀴, 좌우는 LM, 높이·각도는 팔

그리퍼는 항상 수직 아래(pitch −90°)를 보게 하고, 물체가 로봇 기준 y = −110 mm(기준 위치)에 오도록 바퀴로 앞뒤를 맞춘다.
좌우(x)는 LM 가이드(77~181 mm), 높이(z)는 팔이 맡는다. 기준 위치 y = −110 mm는 `downmap`에서 수직 하향 높이 범위가
가장 넓은 곳(z −45~130 mm, 바닥~노브 대기 높이)이고, 차체 앞면(y ≈ −55 mm)과 55 mm 떨어져 있다.

```bash
./build/arm_ik downmap            # y 별 수직 하향 도달 높이 범위표
./build/arm_ik plan 80 -140 116 0 # 물체 (x y z) roll -> 바퀴 이동량 + LM/팔 관절값
./build/arm_ik plan 80 -140 116 0 --yref -100   # 기준 위치 바꾸기
```

## ROS2: 수직 하향 잡기 + 바퀴 이동 시뮬레이션 (RViz)

노브 받침대는 바닥(world)에 고정되고, 로봇은 바퀴로 world 위를 앞뒤(y)로 움직인다.
world 는 로봇 시작 위치의 base_link 와 같다.

```bash
ros2 launch v75_arm_ik ik_demo.launch.py
# 다른 터미널
ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 0      # 노브 잡기: 바퀴 -30 mm, LM -69 mm, 팔 수직 하향
ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 -90    # 잡은 채 roll -90 -> 노브 +90 deg
ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 0      # 노브 0 deg 로 되돌리기
ros2 run v75_arm_ik ik_goal 150 300 250 0 0             # 팔만 이동(바퀴 그대로) -> 노브 놓음
```

- `--grasp X Y Z [ROLL]`: world 기준 물체 위치. 물체가 로봇 기준 y = y_ref(-110 mm)에 오도록 바퀴가 먼저 움직이고(동작 시간의 앞 40%),
  LM 가이드가 x, 팔이 z 를 맞추며 그리퍼는 항상 수직 아래
- `ik_goal X Y Z PITCH ROLL`: 팔만 움직임 (base_link 기준)
- launch 인자: `move_time:=2.0`, `y_ref:=-0.110`

## ROS2: 카메라 정렬 노브 돌리기 시나리오

그리퍼 옆 카메라(노란 상자, `camera_optical_frame`: 비틀기 축에서 x로 −65 mm, 아래를 봄)를 이용한다.

```bash
ros2 run v75_arm_ik ik_goal --knob 90    # 노브를 90°로 (기본 90)
ros2 run v75_arm_ik ik_goal --knob 0     # 다시 0°로
```

1. 정렬: 바퀴로 노브를 y_ref에, LM으로 카메라 광축을 노브 위에 (그리퍼 수직 하향, roll 0, 끝점 z = `observe_tcp_z` 130 mm)
2. 축 추출: 그 자세의 카메라 광축 = 노브 축 (RViz에 주황 막대로 표시). 실물에선 영상 중앙 정렬로 얻는 값
3. LM 이동: LM 가이드만 카메라-그리퍼 간격(−65 mm)만큼 → 그리퍼 축이 노브 축 위
4. 하강: 끝점 z = `grasp_tcp_z`(116 mm)까지 내리고 집게 닫기 (LM 변화 0)
5. 돌리기: roll = −(목표 − 현재 노브 각도) → 노브 회전
6. 놓기: 집게 열고 정렬 높이로 복귀

각 단계는 `move_time` 동안 움직이고 터미널에 단계 이름이 찍힌다. 다른 목표를 보내면 진행 중인 시나리오는 취소된다.

## ROS2: IK 결과를 RViz에서 보기 (팔만)

`v75_robot_description`과 함께 colcon 워크스페이스에 넣고 빌드하면 `ik_rviz_node`도 같이 빌드된다.

```bash
ros2 launch v75_arm_ik ik_demo.launch.py
# 다른 터미널에서 목표 보내기: [x, y, z (mm), pitch, roll (deg)]
ros2 run v75_arm_ik ik_goal 150 390 195 -30 20
# 또는 (값은 반드시 소수점 포함: 정수로 쓰면 float64 배열 검사에서 거부됨)
ros2 topic pub --once /ik_goal std_msgs/msg/Float64MultiArray "{data: [150.0, 390.0, 195.0, -30.0, 20.0]}"
```

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/ik_goal` (입력) | `std_msgs/Float64MultiArray` | `[x, y, z, pitch, roll]` (mm, deg) 또는 `[x, y, z]` (위치만) |
| `/ik_target` (입력) | `geometry_msgs/PoseStamped` | m, 쿼터니언, base_link 기준 |
| `/joint_states` (출력) | `sensor_msgs/JointState` | RViz 표시용. 현재 자세에서 해까지 `move_time`초 동안 부드럽게 이동 |
| `/ik_solution` (출력) | `sensor_msgs/JointState` | IK 체인 관절의 해 (m, rad). 나중에 실물 제어로 넘길 값 |
| `/ik_target_marker` (출력) | `visualization_msgs/MarkerArray` | 목표 위치와 접근 방향. 초록 = 성공, 빨강 = 도달 불가 |

파라미터: `urdf` (기본: v75_robot_description의 URDF), `tip` (tool_tcp), `base` (base_link), `move_time` (1.5 s), `rate` (30 Hz)

도달할 수 없는 목표면 팔은 움직이지 않고 경고와 빨간 마커만 표시된다.
`ik_demo.launch.py`는 `/joint_states`를 이 노드가 보내므로 `display.launch.py`(슬라이더)와 동시에 실행하지 않는다.


### 노브 잡기 시뮬레이션 (표시용)

집는 점이 노브 회전축 위(반경 6 mm 이내, 노브 관절 원점 위 0~40 mm)에 오고 그리퍼 비틀기 축이 노브 축과
나란하면(10° 이내) "잡음" 상태가 된다. 잡으면 집게가 닫히고, 그 뒤 roll을 바꾸면 노브가 같은 각도만큼 돈다.
노브 관절 한계(URDF, 0~90°)를 넘으면 노브는 한계에서 멈추고 경고가 나온다. 노브에서 벗어나면 집게가 열린다.

```bash
ros2 run v75_arm_ik ik_goal 80 -140 116 -90 0     # 잡기
ros2 run v75_arm_ik ik_goal 80 -140 116 -90 -90   # 노브 +90° 회전 (위에서 보면 비틀기 방향과 노브 방향이 반대)
ros2 run v75_arm_ik ik_goal 80 -140 116 -90 0     # 노브 0° 로 되돌리기
ros2 run v75_arm_ik ik_goal 150 300 250 0 0       # 놓고 빠지기
```

## 폴더 배치 (권장)

```
ws/
├─ v75_robot_description/   ← URDF 원본 (RViz용 패키지와 공유)
│   └─ urdf/v75_robot.urdf
└─ v75_arm_ik/              ← 이 프로젝트
```

이렇게 나란히 두고 빌드하면 `arm_ik`가 `v75_robot_description/urdf/v75_robot.urdf`를 기본으로 읽는다.
URDF 원본이 하나라서 RViz와 IK가 항상 같은 값을 쓴다. 옆에 없으면 `v75_arm_ik/urdf/` 안의 사본을 쓴다.

URDF 경로 우선순위: `--urdf PATH` > 환경변수 `ARM_IK_URDF` > 빌드 시 기본 경로

## 빌드

### Ubuntu 22.04 + ROS2 Humble (노트북, Orin Nano 공통)

```bash
sudo apt install ros-humble-pinocchio
source /opt/ros/humble/setup.bash
cmake -S . -B build && cmake --build build -j
```

### ROS 없이 pip로

```bash
pip install pin cmeel-eigen cmeel-urdfdom-headers
PREFIX=$(python3 -c "import sysconfig;print(sysconfig.get_paths()['purelib'])")/cmeel.prefix
cmake -S . -B build -DCMAKE_PREFIX_PATH=$PREFIX && cmake --build build -j
export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
```

Pinocchio 2.6 이상 (4.1에서 빌드·테스트 확인). ROS 노드 빌드에는 `ros-$ROS_DISTRO-pinocchio` 필요.

## 사용법

단위: 길이 mm, 각도 deg. 좌표는 `base_link` 기준.

```bash
./build/arm_ik info                          # 사용 중인 URDF, IK 관절과 한계, q=0 끝점 위치
./build/arm_ik fk 10 -20 15 -25 20           # 관절값(info 순서) -> 끝점 자세
./build/arm_ik ik 150 390 195 -30 20         # 끝점 목표(x y z pitch roll) -> 관절값
./build/arm_ik ik 150 390 195 -30 20 --seed 0 0 0 0 0   # 현재 관절값 근처의 해 우선
./build/arm_ik ik 150 390 195 0 0 --pos-only # 위치만 맞추기
./build/arm_ik selftest 1000                 # 무작위 왕복 테스트
./build/arm_ik --urdf new.urdf info          # 다른 URDF로 실행 (재빌드 불필요)
```

옵션: `--urdf PATH`, `--tip LINK` (끝점 링크, 기본 `tool_tcp`), `--base LINK` (기본 `base_link`)

종료 코드: 0 성공, 2 해 없음(도달 불가), 3 selftest 실패 포함

## URDF에서 읽는 것

| 항목 | URDF 위치 |
|---|---|
| 끝점(TCP) | `tool_tcp` 링크. 없으면 `tool_roll_link` 원점을 쓰고 알림 출력 |
| IK에 쓰는 관절 | `base_link`에서 끝점까지 이어진 관절을 자동으로 찾음 |
| 링크 길이, 관절 위치·축 | 각 `<joint>`의 `<origin>`, `<axis>` |
| 관절 한계 | 각 `<joint>`의 `<limit lower upper>` |
| 고정하는 관절 | 체인 밖 관절(carousel, selector, idler, 집게)은 q=0으로 고정 |

예: 팔꿈치 위치를 실측값으로 고치려면 URDF의 `elbow_joint` `<origin>`만 바꾸고,
집는 점이 바뀌면 `tool_tcp_joint` `<origin>`만 바꾸면 된다.
현재 `tool_tcp`는 `tool_roll_link` 기준 (0, 99.4, −28.1) mm로, 집게 메시에서 잰 두 패드 사이 중심이다.

## 자세 정의

`R = Rx(pitch) · Ry(roll)` (끝점 좌표계의 방향)

- pitch = 0, roll = 0: q=0(CAD 저장 자세)의 끝점 방향. 접근 방향은 +y
- pitch: x축 회전. 양수면 그리퍼 끝이 위로 들림
- roll: 그리퍼 접근 방향(y)을 축으로 비틀기
- v75 팔은 회전축이 x, x, x, y 순서라 이 두 각도만 바꿀 수 있다. 접근 방향의 yaw는 차체 회전으로 맞춘다.
- 이 정의는 관절 **축 방향 구성**이 바뀔 때만 다시 봐야 한다. 길이, 위치, 한계, TCP가 바뀌는 건 상관없다.

## 동작 방식

1. URDF를 읽고 끝점까지의 체인 외 관절을 q=0으로 고정해 축소 모델 생성
2. seed(기본 q=0)에서 먼저 풀고, 실패하면 관절 범위 안의 무작위 초기값으로 최대 30회 재시도
3. 매 반복마다 관절값을 URDF 한계 안으로 클램프해서 범위 밖의 해는 나오지 않음
4. 해가 여러 개면 seed와 가장 가까운 해 선택 (불필요한 큰 동작 방지)
5. 결과를 FK로 다시 계산해서 출력 (허용 오차: 위치 0.1 mm, 자세 약 0.06°)

## 한계

- 충돌 검사 없음 (URDF에 collision 형상이 없음). MoveIt 단계에서 처리
- 속도·가속도 한계는 궤적 생성 단계에서 처리
- 관절 0점은 CAD 저장 자세 기준. 실제 모터 0점과 다르면 OpenCR 쪽에서 오프셋 보정 필요
- 집는 점이 비틀기 축에서 28 mm 아래에 있어, roll을 돌리면 집는 점이 반지름 28 mm 원을 그림
  (노브를 돌릴 때는 노브 축이 집는 점이 아니라 비틀기 축과 맞아야 함)
