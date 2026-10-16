# pa_craft

CRAFT 로봇의 ROS 2 워크스페이스입니다. 센서 입력부터 인지, 미션, 계획, 제어까지 기능별 패키지로 나눈 기본 구조를 담고 있습니다. 각 기능의 세부 구현은 별도 브랜치에서 작업한 뒤 통합할 예정입니다.

## 폴더 구조

```text
pa_craft/
├── .gitignore
├── README.md
├── requirements.txt
├── firmware/
│   └── motor_controller/
│       └── motor_controller.ino
└── src/
    ├── craft_bringup/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── launch/
    │       └── craft.launch.py
    ├── craft_control/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── include/
    │   │   └── craft_control/
    │   │       ├── lateral_controller.hpp
    │   │       └── longitudinal_controller.hpp
    │   └── src/
    │       ├── control_master.cpp
    │       ├── lateral_controller.cpp
    │       └── longitudinal_controller.cpp
    ├── craft_mission/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       └── mission_fsm.cpp
    ├── craft_perception/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── config/
    │   │   └── slam.yaml
    │   ├── scripts/
    │   │   ├── aruco_reader.py
    │   │   ├── depth_heatmap.py
    │   │   ├── knob_depth.py
    │   │   └── knob_lever.py
    │   └── src/
    │       ├── object_detector.cpp
    │       ├── object_pose_estimator.cpp
    │       ├── obstacle_detector.cpp
    │       └── state_estimator.cpp
    ├── craft_planning/
    │   ├── package.xml
    │   ├── setup.cfg
    │   ├── setup.py
    │   ├── craft_planning/
    │   │   ├── __init__.py
    │   │   ├── arm_planner.py
    │   │   ├── clock_planner.py
    │   │   ├── path_planner.py
    │   │   └── planning_master.py
    │   ├── maps/
    │   │   └── .gitkeep
    │   └── resource/
    │       └── craft_planning
    └── craft_sensors/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            ├── arm_encoder_processor.cpp
            ├── c310_processor.cpp
            ├── imu_processor.cpp
            ├── lidar_processor.cpp
            ├── realsense_processor.cpp
            └── wheel_encoder_processor.cpp
```

## 주요 디렉터리

| 경로 | 역할 |
| --- | --- |
| `src/craft_bringup` | ROS 2 노드를 함께 실행하는 launch 파일 |
| `src/craft_sensors` | LiDAR, 카메라, IMU, 엔코더 입력 처리 |
| `src/craft_perception` | 객체·장애물 인지와 로봇 상태 추정 |
| `src/craft_mission` | 미션 상태와 진행 순서 관리 |
| `src/craft_planning` | 경로, 로봇 팔, 시계 조작 목표 계획 |
| `src/craft_control` | 주행 및 조작 목표를 제어 입력으로 연결 |
| `firmware/motor_controller` | 모터 컨트롤러 펌웨어 |

`requirements.txt`는 Python 의존성 목록입니다. `build/`, `install/`, `log/`, `venv-craft/` 같은 로컬 생성물은 `.gitignore`로 제외합니다.
