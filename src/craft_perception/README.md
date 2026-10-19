# C++ PerceptionMaster

`perception_master` 하나가 ROS 입출력, 설정, 타이머를 소유하고 일반 C++ 모듈을
호출합니다. 기본 bringup은 이전의 개별 perception 노드와 Python ArUco 노드를
실행하지 않습니다. `scripts/*.py`는 비교·개발용으로 보존했으며 설치하지 않습니다.
모델 export와 테스트에 사용하는 Python은 런타임 의존성이 아닙니다.

control과 같은 모듈별 헤더/구현 구조입니다. 각 모듈은 일반 클래스이며
ROS Node, 타이머, YAML 로딩은 master에만 있습니다.

공통 함수 규칙도 control의 설정·검증·계산 구분에 맞춥니다.

| 역할 | 공통 함수 | 호출 위치 |
| --- | --- | --- |
| 설정 검증·저장 | `configure(...) -> bool` | master 시작 시 한 번. false면 노드 시작 실패 |
| 입력 형식·값 검증 | `is_valid_input(...) -> bool` | master가 입력마다 검사. 계산·상태 변경 없음 |
| 인식·추정·변환 결과 계산 | `compute_result(...)` | 검증된 입력을 받아 master에 결과 반환 |

설정값이 없는 `KnobLever`, `StateEstimator`는 `configure()`가 필요하지 않습니다.
모듈별로 `configured` 플래그를 두지 않으며 master가 설정 성공 후 타이머를 시작합니다.
입력·결과 보관은 master가 소유하므로 모듈에 `set_input()`/`clear_input()`을 두지 않습니다.
검증 통과 후 검출 대상이나 유효 깊이를 못 찾은 경우는 빈 결과/`nullopt`이며,
입력 형식 오류와 구분합니다. 추론·자원 접근 실패는 예외로 master의 진단에 기록됩니다.

```text
craft_perception/
├── config/perception.yaml
├── include/craft_perception/
│   ├── perception_master.hpp
│   ├── object_detector.hpp
│   ├── aruco_reader.hpp
│   ├── knob_depth.hpp
│   ├── knob_lever.hpp
│   ├── depth_heatmap.hpp
│   ├── state_estimator.hpp
│   ├── obstacle_detector.hpp
│   ├── shared_frame.hpp
│   └── perception_types.hpp
└── src/
    ├── perception_master.cpp
    ├── object_detector.cpp
    ├── aruco_reader.cpp
    ├── knob_depth.cpp
    ├── knob_lever.cpp
    ├── depth_heatmap.cpp
    ├── state_estimator.cpp
    ├── obstacle_detector.cpp
    ├── shared_frame.cpp
    └── geometry_utils.hpp
```

| 모듈 | 기본 타이머 | 입력 / 역할 |
| --- | --- | --- |
| `object_detector` | 10 Hz | D435 BGR → ncnn YOLO 검출 |
| `c310_detector` | 10 Hz | C310 BGR → ncnn YOLO 검출 |
| `aruco` | 15 Hz | D435 BGR → 왼쪽부터 정렬한 ArUco ID |
| `knob_depth` | 10 Hz | D435 검출 결과 + 동일 프레임의 aligned depth → 위치·패널 법선·레버 방향 |
| `knob_lever` | 15 Hz | C310 검출 결과 + 동일 프레임 → 영상 평면 레버 방향 |
| `depth_heatmap` | 5 Hz | D435 aligned depth → BGR 히트맵 |
| `state_estimator` | 100 Hz | 지정한 odometry 입력 검증·전달 |
| `obstacle_detector` | 20 Hz | LaserScan의 유효 점을 원래 센서 좌표계로 투영 |

**타이머 Hz는 처리 목표/상한입니다.** 새 입력이 없으면 계산·발행하지 않습니다.
따라서 10 Hz 검출의 결과를 쓰는 15 Hz 레버 모듈도 새 결과는 최대 10 Hz입니다.
느린 계산이 설정 주기를 넘으면 자동으로 빨라지지 않습니다. 결과를 중복 발행해
겉보기 Hz를 맞추지 않고 진단으로 실제 처리율을 표시합니다. 모든 설정은
`config/perception.yaml`에 있으며 `<모듈>.enabled`, `<모듈>.hz`로 조정합니다.
모든 모듈의 `hz`는 YAML에서 필수로 읽으며 C++ 안에 주기 기본값을 중복하지 않습니다.
누락·NaN·Inf·0 이하·타이머로 표현할 수 없는 값은 시작 시 거부합니다.
현재 파라미터는 시작 시 적용됩니다. 실행 중 변경하려면 노드를 재시작하세요.
낮은 Hz로 조정하면 후속 모듈이 이전 검출 결과를 읽는 시점도 늦어집니다.
`max_input_age_s`는 검출·후속 모듈의 주기와 연산 지연을 고려해 함께 조정하세요.

## 실행과 데이터 공유

- 모듈마다 별도의 `MutuallyExclusive` callback group과 타이머가 있습니다.
  `MultiThreadedExecutor`가 실행하며 동일 모듈의 작업은 겹치지 않습니다.
- 기본 executor 스레드는 6개, 각 카메라의 ncnn 추론 스레드는 1개입니다.
  OpenCV 내부 스레드도 1개로 제한합니다. 실제 온보드 CPU에서 처리율을 측정해 조정하세요.
- 카메라마다 모델 인스턴스를 하나씩 둡니다. 두 카메라는 다른 영상이므로 각각
  추론하지만, 같은 카메라의 깊이·레버 계산은 검출 결과를 공유하여 재추론하지 않습니다.
- 카메라 입력 콜백은 최신 **메타데이터만** 저장합니다. 입력이 30Hz여도 처리하지 않을
  프레임을 복사하지 않습니다. 각 타이머가 프레임을 요청할 때 master가 스냅샷을 준비합니다.
- `SharedFrameReader`는 read-only `mmap`을 재사용합니다. 같은 프레임은 **BGR+Z16 전체를
  한 번만** 연속 복사하고 `shared_ptr<const SharedFrame>`으로 여러 모듈에 전달합니다.
  이후 모듈 전달·검출 결과 보관은 같은 소유 버퍼를 참조하며 이미지 깊은 복사를 하지 않습니다.
- Z16 깊이를 전체 float 영상으로 변환하지 않습니다. 깊이 모듈은 bbox/패널 ROI에서,
  히트맵은 해당 타이머 실행 시 픽셀 단위로 m 환산합니다. C310 캡처 버퍼도 재사용합니다.
- 추론 동안 생산자 버퍼를 잠그면 카메라 프레임을 잃으므로 스냅샷을 한 번 복사한 뒤
  즉시 잠금을 풉니다. 모델 전처리·히트맵 출력에 필요한 별도 버퍼는 남습니다.
- 검출 결과는 **추론에 사용한 원본 BGR·깊이·캘리브레이션**과 함께 저장합니다.
  새 이미지에 이전 bbox를 적용하거나 서로 다른 카메라의 bbox를 섞지 않습니다.
- 입력 시각을 출력 header에 보존하고 `max_input_age_s`보다 오래된 데이터/추론 결과를
  발행하지 않습니다. 진단 타이머·Hz 측정은 steady clock을 사용합니다.
- 타이머와 스레드 풀은 hard real-time 주기를 보장하지 않습니다.
  CPU가 포화되면 높은 Hz 모듈도 지연될 수 있습니다.

## ncnn 모델

`models/knob_sign/`에는 기존 `custom_yolo26/weights/v3.pt`를 실제 변환한 모델을
포함했습니다. 입력은 RGB float32 `[1,3,640,640]`, 출력은 `[9,8400]`이며
블롭 이름은 `in0`, `out0`입니다. 클래스는 다음 순서입니다.

```text
0 knob, 1 sign_red, 2 sign_yellow, 3 sign_green, 4 sign_off
```

현재 모델에는 puck 클래스가 없습니다. puck을 학습한 모델로 교체할 때는
`class_names`와 `knob_class_id`도 함께 수정해야 합니다.
가중치 SHA256과 export 도구 버전은 `provenance.json`에 기록했습니다.

Ultralytics 8.4.160의 ncnn export는 end-to-end 분기를 비활성화합니다.
이 구현은 디코딩된 **cx,cy,w,h + 클래스별 확률** 출력에 맞춰 정방형 letterbox,
BGR→RGB, 1/255 정규화, 클래스별 NMS, 원본 좌표 복원을 수행합니다.
출력 shape이 다르면 오류로 보고하며 임의로 형식을 추측하지 않습니다.
기존 `custom_yolo26/cpp/ncnn_infer` 스텁을 런타임에서 사용하지 않습니다.

다시 export할 때는 학습 환경에서 실행합니다. 배포 장치에서 Python 추론을
실행할 필요는 없습니다. 변환 도구 설치는 학습 환경에서 별도로 준비합니다.

```bash
YOLO_CONFIG_DIR=/tmp/craft-yolo-config custom_yolo26/.venv/bin/python \
  src/craft_perception/tools/export_model.py custom_yolo26/weights/v3.pt
```

## ROS 출력

| 토픽 | 메시지 | 내용 |
| --- | --- | --- |
| `/vision/aruco` | `std_msgs/Int32MultiArray` | 기존 호환: ID를 영상 왼쪽부터 정렬 |
| `/vision/detections` | `craft_perception/DetectionArray` | D435의 클래스·확률·픽셀 bbox |
| `/vision/c310_detections` | 같은 메시지 | C310 검출 |
| `/vision/knob_observations` | `craft_perception/KnobObservationArray` | D435 손잡이 위치·법선·레버 방향 및 유효성 |
| `/vision/c310_knob_observations` | 같은 메시지 | C310 손잡이 레버 방향; depth_valid=false |
| `/vision/object_detection` | `geometry_msgs/PoseArray` | 깊이를 얻은 D435 물체의 3D 위치, optical frame |
| `/vision/depth_heatmap` | `sensor_msgs/Image` | BGR8 히트맵, SensorDataQoS |
| `/lidar/obstacle` | `geometry_msgs/PoseArray` | LaserScan 원래 frame의 유효 점 |
| `/state/odom` | `nav_msgs/Odometry` | 지정한 입력 odometry 전달 |
| `/perception/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 실제 처리 Hz, 실행 시간, skip/error/overrun, 카메라별 복사 횟수·바이트 |

knob 위치는 카메라 optical frame에서 m 단위입니다. 레버 각도는 **영상 평면의
rad, 위쪽=0, 시계방향 양수**이며 로봇 관절각이 아닙니다. 관측 배열의 유효성
플래그를 확인해야 하며, 0으로 초기화된 무효 필드를 실제 관측으로 쓰면 안 됩니다.
빈 배열은 새 프레임에서 해당 물체를 찾지 못한 결과입니다. 입력이 끊기면 새 배열도
나오지 않으므로 소비자는 header 시각/수신 timeout을 확인해야 합니다.

기존 Python과 같은 알고리즘으로 중심 bbox의 유효 깊이 중앙값을 사용하고,
주변 사각 고리에서 깊이 차이가 작은 점을 모아 평면을 fitting합니다. 법선은
카메라를 향합니다. 레버는 빨강 HSV·밝기 Otsu·가장 큰 성분의 PCA로 추정합니다.
평면 fitting은 RANSAC이 아니며, deprojection은 pinhole 근사로 렌즈 왜곡을 무시합니다.

`/vision/object_detection`의 quaternion은 위치 목록 호환용 identity일 뿐 물체 자세가
아닙니다. 완전한 tool 자세를 만들어 `/vision/object_pose`로 발행하지 않습니다.
planning에서 optical frame을 `base_link`/`map`으로 변환하는 작업도 별도로 필요합니다.

**상태 융합은 기존부터 TODO였고 이번에도 구현하지 않았습니다.** `StateEstimator`는
`odometry_source_topic` 하나를 검증해 전달합니다. wheel·IMU·lidar EKF가 있다고 가정하면
안 됩니다. 장애물 검출 역시 단순 스캔 투영이며 클러스터링·좌표 변환은 하지 않습니다.

## 공유 메모리 v2

수정된 `craft_sensors`와 함께 재빌드·재시작해야 합니다. 구버전 3개 필드 입력은
거부합니다. D435는 `/craft_vision`, C310은 `/craft_c310`을 사용합니다.

```text
D435 /vision/image_offset:      [offset,w,h,sequence,ROS stamp ns,depth unit nm]
C310 /vision/c310_image_offset: [offset,w,h,sequence,ROS stamp ns]
slot: BGR8 + (D435 only Z16 depth) + uint64 sequence trailer
```

생산자는 exclusive flock, 소비자는 nonblocking shared flock을 사용합니다. 소비자는
슬롯 sequence가 알림과 일치하는지 확인한 뒤 복사하고 즉시 잠금을 풉니다.
이미 덮어쓴 슬롯·쓰기 중인 버퍼는 건너뜁니다. depth scale은 실제 센서값을 전달해
소비자가 필요한 픽셀을 m로 환산합니다. DDS로는 영상 대신 메타데이터만 보냅니다.
생산자 시작 시 크기 변경도 같은 잠금으로 보호하며, reader는 파일 크기·inode 변화를
확인해 resize와 프로세스 재시작 시 다시 매핑합니다.

`perception/shared_memory/realsense`, `perception/shared_memory/c310` 진단의
`input_frames`, `snapshots`, `bytes_copied`로 수신 프레임과 실제 복사량을 비교할 수 있습니다.

## 빌드와 검증

ROS 2, OpenCV 개발 패키지(aruco 포함), ncnn C++ 설치가 필요합니다.
센서까지 빌드하려면 기존처럼 librealsense2 SDK가 필요합니다.

```bash
colcon build --packages-up-to craft_bringup \
  --cmake-args -DBUILD_TESTING=ON -Dncnn_DIR="$HOME/ncnn-install/lib/cmake/ncnn"
source install/setup.bash
ros2 run craft_perception perception_master --ros-args --params-file \
  "$(ros2 pkg prefix craft_perception)/share/craft_perception/config/perception.yaml"
```

기존 `craft.launch.py`도 master를 실행하며 `perception_model_dir:=/absolute/model_dir`로
모델 경로를 바꿀 수 있습니다. 모델이 없거나 로드가 실패하면 시작 시 오류로 종료합니다.
모델 없이 ArUco만 점검하려면 두 detector의 `enabled`를 false로 설정하세요.

```bash
colcon test --packages-select craft_perception --event-handlers console_direct+
colcon test-result --verbose
./build/craft_perception/ncnn_smoke src/craft_perception/models/knob_sign
python3 src/craft_perception/test/test_master_integration.py \
  --executable ./build/craft_perception/perception_master \
  --model src/craft_perception/models/knob_sign
# 30Hz 카메라 + 2Hz 인식: 사용하지 않는 프레임을 복사하지 않는지 추가 검증
python3 src/craft_perception/test/test_master_integration.py \
  --executable ./build/craft_perception/perception_master \
  --model src/craft_perception/models/knob_sign --low-copy
```

통합 테스트는 독립 ROS domain과 고유 shm 이름으로 합성 D435/C310 입력을 보내며
실제 ncnn 추론 중 ArUco 처리와 입력 단절 시 발행 중단을 검사합니다. 실제 장치의
성능·캘리브레이션·검출 정확도 검증을 대신하지 않습니다.
