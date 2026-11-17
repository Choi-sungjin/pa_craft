# CRAFT 로봇 ROS 2 워크스페이스

본 저장소는 CRAFT 로봇의 센서, 인지, 미션, 계획, 제어 및 시각화 코드를 노드의 역할에 따라 구성한 소스 배포본입니다. 현재 운영 환경은 ROS 2 Lyrical 기반 Jetson입니다. 로봇별 티칭값과 보정값을 포함하며, 전체 미션의 실물 검증 완료를 의미하지는 않습니다.

## 패키지 및 노드 구성

| 패키지 | 주요 실행파일 및 역할 |
| --- | --- |
| `craft_interfaces` | 계획·제어 간 작업 명령, 결과 및 상태 메시지를 제공합니다. |
| `craft_sensors` | `realsense_processor`: 그리퍼 카메라 입력, `uppercam_bridge`: 어퍼캠·LiDAR 박스 관측을 제공합니다. |
| `lidar_processor` | `lidar_processor`: 전후방 스캔을 처리합니다. `coin_d4_driver`는 외부 소스 의존성입니다. |
| `craft_perception` | `perception_master`: 물체·퍽·신호등·바코드 및 LiDAR 인지를 담당합니다. |
| `craft_planning` | `planning_master`: 주행·팔 작업 계획, `mission_master`: 미션 진행, `puck_picker`: 집기 요청 연결, `puck_execute`: 보정된 집기 단계 실행을 담당합니다. `dice_demo`도 포함합니다. |
| `craft_control` | `control_master`: 주행·팔·그리퍼·클락 제어 및 펌웨어 통신을 담당합니다. |
| `craft_description` | 로봇 URDF 및 메시를 제공합니다. |
| `craft_bringup` | 통합 launch, 대시보드, `global_localize`, `odom_trail`, `box_marker`, `puck_marker`를 제공합니다. |

`src/craft_planning/v75_ws`는 팔 모델과 참조 구현을 보관하는 디렉터리입니다. 별도 운영 워크스페이스로 빌드하지 않습니다.

## 함께 제공되는 자료

프로젝트 2의 문제·설계·시험·개인별 기여와 보고서 이미지는 [프로젝트 2 제출 자료](project2/README.md)에 정리했습니다. 공식 회차 등 미확인 증빙은 별도로 표시했습니다.

- `firmware/`: OpenCR 펌웨어와 공통 모터 라이브러리입니다.
- `demo/puck_pick/`: 티칭·집기 설정, Python 참조 구현 및 인수인계 자료입니다.
- `tools/realsense_check/`: `grip_yaw`, `upper_puck` 카메라 실행파일과 보정 파일입니다. 카메라 도구는 `craft_sensors`에서 함께 빌드·설치하며, 독립 CMake 빌드는 소스 실험용으로 유지합니다.
- `src/craft_perception/models/`: ncnn·ONNX 모델 및 현재 장치용 TensorRT 엔진입니다. 다른 GPU 또는 TensorRT 버전에서는 ONNX로 엔진을 다시 생성해야 합니다.
- `demo/driving_sim/`, `demo/mission_sim/`, `test/`: 시뮬레이션 및 검증 자료입니다.

빌드 산출물, 촬영 데이터, 실행 기록, 로컬 환경 파일 및 백업 사본은 배포 대상에서 제외합니다. 티칭 및 보정 원본은 포함합니다.

## 설치 및 전체 빌드

통합 실행본은 설치된 패키지에서 노드·모델·티칭·보정값을 찾습니다. `demo/`·`tools/` 소스 경로는 실행 시 필요하지 않습니다. 설치본 아카이브 생성과 사용자 데이터 관리는 [배포 안내](docs/DEPLOYMENT.md)를 참조해 주십시오.

```bash
bash tools/build_runtime.sh
```

배포 산출물은 `dist/craft-runtime.tar.gz`이며, 원본 저장소와 별도로 실행에 필요한 설치 패키지를 묶습니다. 미션의 ObservePucks·PlacePuck·VerifyLocation은 아직 미구현이고 `drive_only: true`를 유지합니다.

새 환경에서는 ROS 2 Lyrical, colcon, rosdep, vcstool 및 C++ 개발 환경을 준비해 주십시오. 주요 네이티브 의존성은 OpenCV, librealsense2, ncnn, Pinocchio, coal, urdfdom, yaml-cpp, PNG, ZXing입니다. TensorRT·CUDA는 현재 Jetson 설정에 사용됩니다. `requirements.txt`는 Python 실험·모델 도구용이며 ROS 패키지 의존성을 대신하지 않습니다.

```bash
git clone --branch arm-teached https://github.com/Choi-sungjin/pa_craft.git ~/craft
cd ~/craft
source /opt/ros/lyrical/setup.bash
vcs import src < craft.repos
rosdep install --from-paths src --ignore-src -r -y
bash tools/build_workspace.sh
```

`craft.repos`는 LiDAR 드라이버의 확인된 커밋을 고정합니다. rosdep에서 해결되지 않는 네이티브 라이브러리는 설치 후 CMake 검색 경로를 지정해 주십시오. 예를 들어 ncnn을 별도 설치한 경우 다음과 같이 실행합니다.

```bash
cd ~/craft
bash tools/build_workspace.sh -Dncnn_DIR="$HOME/.local/lib/cmake/ncnn"
```

전체 빌드 스크립트는 ROS 패키지와 집기에 필요한 두 카메라 도구를 빌드하며, 로봇 노드를 실행하지 않습니다. ZXing이 없으면 바코드 인식이 비활성화되므로 실제 미션 환경에는 설치가 필요합니다.

## 실행

현재 로봇 Jetson의 `.bashrc`에 등록된 명령을 사용합니다. 터미널 네 개를 열고 아래 순서대로 실행해 주십시오.

**터미널 1: 미션 실행 환경**

```bash
mrun
```

**터미널 2: 카메라**

```bash
camera
```

**터미널 3: TUI**

```bash
tui
```

**터미널 4: 미션 시작**

터미널 1~3의 준비가 끝난 뒤 실행해 주십시오.

```bash
start
```
| `craft_voice` | 상태·인지 결과를 바탕으로 음성 안내와 질문 응답을 제공합니다. [실행 안내](src/craft_voice/README.md) |

`mrun`은 미션 노드를 포함한 실행 환경을 준비하고, `start`는 미션 시작 명령을 전달합니다. 실행 전 [미션 설정](src/craft_planning/config/mission.yaml)의 `drive_only` 등을 확인해 주십시오.

자세한 설정과 실행 절차는 [계획 패키지](src/craft_planning/README.md), [인지 패키지](src/craft_perception/README.md), [제어 패키지](src/craft_control/README.md), [LiDAR 패키지](src/lidar_processor/README.md), [펌웨어](firmware/README.md)를 참조해 주십시오. 집기의 실물 확인 범위와 미완료 항목은 [인수인계](demo/puck_pick/HANDOFF_20261115.md)에 기록되어 있습니다.

## home 복귀 명령

Jetson의 `.bashrc`에서 사용하는 `home` 등록 파일과 설정 방법은 [home 안내](docs/HOME.md)를 참조해 주십시오. 기존 `tools/go_home.py`를 호출하는 소스 작업용 명령이며, 등록 파일을 source하는 것만으로는 로봇이 움직이지 않습니다.
