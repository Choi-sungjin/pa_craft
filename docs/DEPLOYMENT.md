# CRAFT 통합 실행 패키지

본 배포본은 기존에 구현된 CRAFT 노드와 실행 자원을 함께 설치한 패키지입니다. 개발 디렉터리의 Python 데모를 직접 실행하지 않고, 설치된 C++ 노드와 카메라 도구를 사용합니다. 기존 로봇의 티칭값·보정값·모터 한계는 변경하지 않았습니다.

## 구현 및 검증 범위

| 항목 | 상태 |
| --- | --- |
| 센서·인지·계획·제어·시각화 | 기존 구현을 통합 launch에서 실행합니다. |
| 어퍼캠 박스 관측 | `uppercam_bridge`가 통합 launch에 포함됩니다. 별도 실행 스크립트가 필요하지 않습니다. |
| 로봇 모델 TF | `robot_state_publisher`가 설치된 URDF와 `/control/arm_joints`를 사용합니다. |
| 퍽 집기 | C++ `puck_picker` → `puck_execute` → planning 경로를 사용합니다. 설치본의 집기 경로는 `--collect`, `--find`, `--lie --cell 2\|5 --pick`입니다. |
| 미션 ObservePucks·PlacePuck·VerifyLocation | **미구현입니다.** 기존 코드가 시간 경과 후 성공 처리하는 상태이며, 이번 작업에서 구현하거나 완료로 간주하지 않습니다. |
| 기본 미션 설정 | `drive_only: true`를 유지합니다. 이를 false로 바꾸어도 전체 미션이 완성되지는 않습니다. |
| 실물 검증 | 이번 패키징 작업에서는 로봇을 구동하지 않습니다. 빌드·설치 경로·저장 데이터 확인과 실물 확인은 별개입니다. |

누운 퍽의 자동 집기는 기존 2/5번 티칭 주변으로 제한됩니다. 실린더 집기와 누운 퍽의 전체 실물 흐름 등 미확인 항목은 기존 상태를 유지합니다. 펌웨어 업로드도 사용자가 별도로 수행해야 합니다.

## 소스에서 빌드 및 패키징

ROS 2 Lyrical, colcon, vcstool, rosdep 및 네이티브 의존성이 설치된 환경에서 실행해 주십시오. ncnn, librealsense2, Pinocchio, coal, OpenCV, ZXing, TensorRT/CUDA 등은 대상 장치에 제공되어야 합니다. TensorRT 엔진은 현재 Jetson 환경용이며, 다른 GPU나 TensorRT 버전에서는 ONNX로 재생성해야 합니다.

```bash
cd ~/craft
source /opt/ros/lyrical/setup.bash
vcs import src < craft.repos
rosdep install --from-paths src --ignore-src -r -y
bash tools/build_runtime.sh
```

결과는 `dist/craft-runtime.tar.gz`입니다. 아카이브에는 9개 설치 패키지와 배포 안내·manifest만 들어갑니다. 소스 작업 폴더, 촬영 기록, 임시 코드, 빌드 트리는 포함하지 않습니다. `--symlink-install`의 링크는 실제 파일로 복사합니다. 생성된 manifest의 `source_revision`과 `source_dirty`를 통해 원본 상태를 확인할 수 있습니다.

이 아카이브는 시스템 의존성까지 포함한 컨테이너가 아닙니다. 동일한 아키텍처와 ROS·네이티브 라이브러리 환경에서 사용해 주십시오. 원본 저장소는 개발과 재빌드를 위해 보존합니다.

## 설치본 실행

아카이브를 원하는 위치에 풀고 해당 디렉터리에서 실행해 주십시오. 실제 장치를 사용하는 명령은 사용자가 직접 실행해야 합니다.

```bash
tar -xzf craft-runtime.tar.gz
cd craft-runtime
source install/setup.bash
ros2 launch craft_bringup craft.launch.py mission:=true
```

미션 노드는 시작 후 `/mission/command`의 START를 기다립니다. 기본 설정은 주행 전용입니다. `mission:=true`는 전체 대회 미션의 완성을 뜻하지 않습니다.

기존에 별도 실행하던 어퍼캠·robot_state_publisher와 동시에 실행하지 않도록 해 주십시오. 외부 프로세스를 유지해야 하는 경우 `start_uppercam:=false` 또는 `start_robot_state_publisher:=false`를 지정할 수 있습니다. `start_cameras:=false`는 기본적으로 어퍼캠도 끕니다. 대시보드에서는 미션 노드가 planning 그룹에서만 한 번 실행됩니다.

## 자원과 사용자 데이터

| 위치 | 내용 |
| --- | --- |
| `craft_planning/share/craft_planning/puck_pick` | 티칭 및 계획 기본 파일 |
| `craft_planning/share/craft_planning/urdf`, `meshes` | 팔 모델과 동작 검사에 필요한 메시 |
| `craft_sensors/share/craft_sensors/puck_camera` | 카메라 보정 기본 파일 |
| `craft_sensors/lib/craft_sensors` | 설치된 `grip_yaw`, `upper_puck` |
| `craft_perception/share/craft_perception/models` | 인지 모델 |
| `~/.local/share/craft/puck_pick`, `puck_camera` | 실행 중 사용하는 티칭·보정 사본, 생성 계획, 촬영·집기 기록 |

집기 실행부는 처음 사용할 때 설치된 기본 파일을 사용자 데이터 위치에 복사합니다. 이미 존재하는 파일은 업데이트 시에도 덮어쓰지 않습니다. 새로운 티칭·보정값을 적용할 때는 사용자 데이터 사본을 먼저 백업하고 명시적으로 갱신해 주십시오. 원본 `demo/puck_pick/teach.yaml`과 `tools/realsense_check/stand_cal.json`은 변경하지 않습니다.

사용자 데이터 루트는 `CRAFT_STATE_DIR` 또는 `XDG_DATA_HOME`으로 지정할 수 있습니다. `CRAFT_CAMERA_DATA_DIR`은 카메라 보정 디렉터리를, `CRAFT_CONTROL_CONFIG`는 집기 계산에 사용하는 제어 설정 파일을 지정합니다. 별도 제어 설정을 사용하는 경우 control 노드와 집기 계산에 같은 값을 적용해 주십시오.

`CRAFT_REPO`·`PUCK_PICK_DIR`은 기존 소스 기반 실험을 위한 명시적 호환 옵션입니다. 일반 설치본 실행에서는 설정하지 않습니다. `grip_locate.py`를 사용하는 기존 bigbro 등 Python 카메라 실험 옵션은 통합 집기 배포 경로에 포함하지 않으며, 해당 옵션은 소스 기반 작업에서 사용해 주십시오.

## 이번 패키징의 확인 결과 (2026-11-17)

- 변경 패키지 3개 빌드와 별도 배포용 9개 패키지 빌드가 완료되었습니다. 컴파일러 경고는 남아 있으며 무경고 빌드는 아닙니다. 기존 제어 코드 등의 경고 분석은 이번 패키징 범위에 포함하지 않았습니다.
- 생성된 아카이브를 별도 경로에 풀고, 원본 저장소의 환경 경로를 제거한 상태에서 확인했습니다. launch의 고정 실행파일 17개, 팔 URDF의 메시, 설치 자원과 동적 라이브러리 경로가 확인되었습니다.
- 누운 퍽 5번의 dry-run과 저장된 어퍼캠 프레임의 ONNX 재생을 확인했습니다.
- 저장된 그리퍼 프레임 한 건을 설치된 카메라 도구 → 실제 `camera_run` → 실제 `find_step`으로 전달했습니다. 해당 과거 프레임은 기존 소스 도구와 설치본 모두 동일하게 미검출로 판정했고, `find_step`의 거절을 확인했습니다. 이는 성공 집기나 실물 동작의 검증을 뜻하지 않습니다.
- 로봇 노드 구동, 실제 카메라 촬영 및 펌웨어 업로드는 수행하지 않았습니다.
