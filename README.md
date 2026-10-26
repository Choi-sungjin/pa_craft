# pa_craft

CRAFT 로봇의 ROS 2 워크스페이스입니다. 센서 입력부터 인지, 미션, 계획, 제어까지 기능별 패키지로 나눈 기본 구조를 담고 있습니다. 각 기능의 세부 구현은 별도 브랜치에서 작업한 뒤 통합할 예정입니다.

## 폴더 구조

```text
pa_craft/
├── .gitignore
├── README.md
├── requirements.txt
├── firmware/
│   ├── wheel_clock/            # wheel x2 + clock OpenCR
│   ├── lm_arm/                 # LM + arm x4 + gripper OpenCR
│   └── libraries/CraftMotorFirmware/  # 공통 통신·모터 제어
└── src/
    ├── craft_bringup/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── launch/
    │       └── craft.launch.py
    ├── craft_control/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── control.yaml
    │   ├── include/
    │   │   └── craft_control/
    │   │       ├── arm_controller.hpp
    │   │       ├── clock_controller.hpp
    │   │       ├── lateral_controller.hpp
    │   │       └── longitudinal_controller.hpp
    │   └── src/
    │       ├── arm_controller.cpp
    │       ├── clock_controller.cpp
    │       ├── control_master.cpp
    │       ├── lateral_controller.cpp
    │       └── longitudinal_controller.cpp
    ├── craft_perception/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   ├── perception.yaml   # 모든 perception 설정 (master configure()가 읽음)
    │   │   └── slam.yaml
    │   ├── include/craft_perception/ # 일반 C++ 모듈 인터페이스
    │   ├── msg/                  # 검출·손잡이·puck 관측 메시지
    │   ├── models/               # knob_sign ncnn 모델 (puck는 AprilTag ID로 구분)
    │   └── src/
    │       ├── perception_master.cpp # 연결·configure·타이머 (control_master 형식)
    │       ├── d435_perception.cpp   # D435 클래스: yolo → knob_depth/object/obstacle, apriltag, puck_axis
    │       ├── c310_perception.cpp   # C310 클래스: knob_lever (자체 YOLO)
    │       ├── lidar_perception.cpp  # LiDAR 클래스: 전·후방 스캔 병합 → 장애물
    │       ├── yolo_detector.cpp     # ncnn YOLO
    │       ├── apriltag_reader.cpp
    │       ├── knob_depth.cpp
    │       ├── knob_lever.cpp
    │       ├── object_detector.cpp   # YOLO 박스 + depth → 3D 위치
    │       ├── obstacle_detector.cpp # YOLO 박스 + depth → 가까운 장애물 면
    │       ├── puck_axis.cpp         # puck 축 (puck_axis_live.py 이식)
    │       ├── shared_frame.cpp
    │       ├── lidar_obstacle_detector.cpp
    │       └── state_estimator.cpp
    ├── craft_planning/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── planning.yaml
    │   ├── scripts/
    │   │   └── planning_master.py
    │   ├── src/
    │   │   ├── __init__.py
    │   │   ├── kant_astar_arc.py
    │   │   └── path_planner.py
    │   ├── maps/maze.stl          # 기본 경로 계획 맵 (Gazebo 월드 좌표)
    │   ├── maps/kant/             # 원본 예제 맵
    │   ├── test/
    │   └── third_party/           # 원본 저장소 및 커밋 기록
    ├── lidar_processor/          # 전·후방 LDS-03 드라이버 launch, TF, 360도 병합
    └── craft_sensors/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            ├── c310_processor.cpp
            └── realsense_processor.cpp
```

## 주요 디렉터리

| 경로 | 역할 |
| --- | --- |
| `src/craft_bringup` | ROS 2 노드를 함께 실행하는 launch 파일 |
| `src/craft_sensors` | D435·C310 카메라 입력 처리 (IMU·엔코더는 control 쪽에서 발행) |
| `src/lidar_processor` | 전·후방 LiDAR 구동·TF·병합 → `/lidar/merged` ([실행 안내](src/lidar_processor/README.md)) |
| `src/craft_perception` | C++ PerceptionMaster + D435/C310 클래스 (ncnn·AprilTag·깊이·레버·puck 인지) ([실행·주기 설정](src/craft_perception/README.md)) |
| `src/craft_planning` | 주행 경로 계획 |
| `src/craft_control` | 로봇 팔·시계 조작 계획, 주행 및 조작 목표를 제어 입력으로 연결 |
| `firmware/wheel_clock`, `firmware/lm_arm` | OpenCR 두 대의 모터 펌웨어 ([설정·빌드 안내](firmware/README.md)) |

## 실행

```bash
source install/setup.bash
ros2 launch craft_bringup craft.launch.py                  # 전체를 한 터미널에서
ros2 run craft_bringup craft_dashboard.sh                  # terminator 4분할: 인지 / 판단 / 제어 / 인프라
ros2 run craft_bringup craft_dashboard.sh start_lidar:=false   # launch 인자는 모든 창에 그대로 전달
```

대시보드는 `craft.launch.py`를 `group:=perception|planning|control|infra`로 나눠 창마다 하나씩 띄웁니다. 한 창에서 Ctrl+C를 누르면 그 그룹만 종료됩니다.

`requirements.txt`는 Python 의존성 목록입니다. `build/`, `install/`, `log/`, `venv-craft/` 같은 로컬 생성물은 `.gitignore`로 제외합니다.
