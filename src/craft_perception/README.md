# C++ PerceptionMaster

`perception_master` 하나가 ROS 입출력, 설정, 타이머를 소유하고 카메라별 클래스
`D435Perception`, `C310Perception`의 멤버 함수를 호출합니다 (`control_master`와 같은 형식).
이전의 개별 실행 Python 스크립트(`aruco_reader.py`, `knob_depth.py`, `knob_lever.py`,
`depth_heatmap.py`, `puck_axis_live.py`)와 분리 전 C++는 `~/craft_cam_backup/`에 백업했습니다.
모델 export와 테스트에 사용하는 Python은 런타임 의존성이 아닙니다.

| 계층 | 담당 |
| --- | --- |
| `PerceptionMaster` | `configure()`: `perception.yaml`을 읽어 `D435Config`/`C310Config`를 채우고 각 클래스의 `configure()` 호출. `connect()`: shm reader, pub/sub, 작업별 타이머 |
| `D435Perception` / `C310Perception` | 카메라 단위 알고리즘 묶음. ROS 없음. 결과 `nullopt` = 입력 무효(발행 안 함), 빈 vector = 유효 프레임에서 찾지 못함 |
| 모듈 (`YoloDetector`, `KnobDepth` …) | `configure(...) -> bool`, `is_valid_input(...)`, `compute_result(...)` 규칙 그대로 |

모듈별로 `configured` 플래그를 두지 않으며 master가 설정 성공 후 타이머를 시작합니다.
입력·결과 보관은 master가 소유합니다. 추론·자원 접근 실패는 예외로 master의 진단에 기록됩니다.

```text
craft_perception/
├── config/perception.yaml          # 모든 값 필수, C++ 기본값 없음
├── include/craft_perception/
│   ├── perception_master.hpp
│   ├── d435_perception.hpp         # D435Config, D435Perception
│   ├── c310_perception.hpp         # C310Config, C310Perception
│   ├── yolo_detector.hpp           # ncnn YOLO (구 ObjectDetector)
│   ├── aruco_reader.hpp
│   ├── knob_depth.hpp
│   ├── knob_lever.hpp
│   ├── object_detector.hpp         # YOLO 박스 + depth → 3D 위치
│   ├── obstacle_detector.hpp       # YOLO 박스 + depth → 가까운 장애물 면
│   ├── puck_axis.hpp               # puck_axis_live.py 이식
│   ├── lidar_obstacle_detector.hpp # 구 ObstacleDetector (LaserScan)
│   ├── state_estimator.hpp         # 대기 (odometry 전달만)
│   ├── shared_frame.hpp
│   └── perception_types.hpp
├── models/knob_sign/, models/puck_blue/
└── src/ (같은 이름의 .cpp, geometry_utils.hpp)
```

| 카메라 | 작업 (`schedule.<이름>`) | 기본 Hz | 입력 / 역할 |
| --- | --- | --- | --- |
| D435 | `d435_yolo` | 10 | BGR → knob_sign YOLO 1회. 결과를 프레임과 묶어 아래 3개가 공유 |
| D435 | `knob_depth` | 10 | YOLO knob 박스 + aligned depth → 위치·패널 법선 |
| D435 | `object_detector` | 10 | YOLO 전체 박스 + depth → 박스 중앙부 중앙값 깊이의 3D 위치 |
| D435 | `obstacle_detector` | 10 | YOLO 장애물 클래스 박스 + depth → 박스 내 가까운 면(하위 분위수), 거리 제한 |
| D435 | `aruco` | 15 | BGR → 왼쪽부터 정렬한 ArUco ID (YOLO와 별개) |
| D435 | `puck_axis` | 10 | 자체 puck YOLO(`puck_blue`) + depth → 선반 평면 RANSAC, standing/lying, 축 |
| C310 | `knob_lever` | 10 | 자체 knob_sign YOLO + 영상 평면 레버 방향 |
| - | `lidar_obstacle` | 20 | LaserScan 유효 점 투영 (기존 그대로) |
| - | `state_estimator` | 100 | 지정 odometry 검증·전달 (대기, 기존 그대로) |

`knob_depth`/`object_detector`/`obstacle_detector`는 `d435_yolo`가 꺼져 있으면 결과가 없습니다
(시작 시 경고). `puck_axis`는 depth가 필요해 C310이 아닌 D435에서 실행합니다.

**타이머 Hz는 처리 목표/상한입니다.** 새 입력이 없으면 계산·발행하지 않습니다.
따라서 10 Hz 검출의 결과를 쓰는 15 Hz 레버 모듈도 새 결과는 최대 10 Hz입니다.
느린 계산이 설정 주기를 넘으면 자동으로 빨라지지 않습니다. 결과를 중복 발행해
겉보기 Hz를 맞추지 않고 진단으로 실제 처리율을 표시합니다. 모든 설정은
`config/perception.yaml`에 있으며 `schedule.<작업>.enabled`, `schedule.<작업>.hz`로 조정합니다.
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
- 모델 인스턴스: D435 knob_sign, D435 puck, C310 knob_sign 각 1개. 각 인스턴스는 한 작업만
  사용하므로 서로 다른 작업은 병렬로 돌아도 같은 `ncnn::Net`을 동시에 쓰지 않습니다.
  D435의 깊이·물체·장애물 계산은 `d435_yolo` 결과를 공유하여 재추론하지 않습니다.
- 카메라 입력 콜백은 최신 **메타데이터만** 저장합니다. 입력이 30Hz여도 처리하지 않을
  프레임을 복사하지 않습니다. 각 타이머가 프레임을 요청할 때 master가 스냅샷을 준비합니다.
- `SharedFrameReader`는 read-only `mmap`을 재사용합니다. 같은 프레임은 **BGR+Z16 전체를
  한 번만** 연속 복사하고 `shared_ptr<const Frame>`으로 D435의 5개 작업에 전달합니다.
  클래스 함수는 `const SharedFrame &`를 받으며 검출 결과도 `shared_ptr<const DetectedFrame>`로
  공유합니다. 이미지 깊은 복사를 하지 않습니다. ROS 출력은 `unique_ptr`로 publish합니다.
- ROS 2 자체 zero-copy를 쓰지 않는 이유: intra-process 통신은 센서 노드와 같은 프로세스
  (component container)여야 하고, loaned message는 고정 크기 메시지에만 동작해
  `sensor_msgs/Image`에 쓸 수 없습니다. 프로세스 간은 현재 shm(DDS로는 메타데이터만)을 유지합니다.
- Z16 깊이를 전체 float 영상으로 변환하지 않습니다. 깊이 모듈은 bbox/패널 ROI에서,
  필요한 픽셀만 m로 환산합니다. C310 캡처 버퍼도 재사용합니다.
- 추론 동안 생산자 버퍼를 잠그면 카메라 프레임을 잃으므로 스냅샷을 한 번 복사한 뒤
  즉시 잠금을 풉니다. 모델 전처리에 필요한 별도 버퍼는 남습니다.
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

모델을 교체할 때는 `d435.yolo`/`c310.yolo`의 `class_names`와 `knob_class_id`도 함께 수정합니다.
`models/puck_blue/`는 `puck_axis_live.py`가 쓰던 `target_blue_v3` 640 ncnn export를 그대로 복사했습니다
(클래스 1개 `target_blue`, 출력 `[5,8400]`). 가중치 SHA256과 export 도구 버전은 각 `provenance.json`에 있습니다.

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

## TensorRT 백엔드 (Jetson)

`YoloDetector`는 추론만 백엔드(`ncnn` | `tensorrt`)로 분리합니다. letterbox, 디코딩, NMS,
좌표 복원은 공통이라 두 백엔드의 결과 형식이 같습니다. 각 `*.yolo` 블록의 `backend`로 고릅니다.

1. 학습 환경에서 ONNX를 만듭니다. `nms=None`으로 ncnn과 같은 `[1,4+classes,8400]` 출력을 유지합니다.
   ```bash
   custom_yolo26/.venv/bin/python src/craft_perception/tools/export_model.py custom_yolo26/weights/v3.pt \
     --format onnx --output src/craft_perception/models/knob_sign
   ```
2. Jetson에서 엔진을 만듭니다. 엔진은 그 장치·TensorRT 버전 전용이라 복사해 쓰지 않습니다.
   ```bash
   /usr/src/tensorrt/bin/trtexec --onnx=model.onnx --saveEngine=model.engine --fp16
   ```
3. `-DCRAFT_TENSORRT=on`으로 빌드하고 YAML의 `backend: tensorrt`로 바꿉니다.
   엔진 경로는 `engine`, 블롭 이름은 `tensorrt_blobs`(`images`, `output0`)입니다.

`CRAFT_TENSORRT=auto`(기본)는 TensorRT·CUDA가 있을 때만 백엔드를 빌드합니다. 없는 빌드에서
`backend: tensorrt`를 요청하면 시작 시 오류로 거부합니다. 전처리(BGR→RGB, /255, CHW)는 CPU,
추론은 GPU 스트림에서 실행하며 검출기마다 실행 컨텍스트가 하나입니다.

## ROS 출력

| 토픽 | 메시지 | 내용 |
| --- | --- | --- |
| `/perception/vision/d435/aruco` | `std_msgs/Int32MultiArray` | 기존 호환: ID를 영상 왼쪽부터 정렬 |
| `/perception/vision/d435/detections` | `craft_perception/DetectionArray` | D435 `d435_yolo`의 클래스·확률·픽셀 bbox |
| `/perception/vision/c310/detections` | 같은 메시지 | C310 `knob_lever`의 YOLO 검출 |
| `/perception/vision/d435/knob_observations` | `craft_perception/KnobObservationArray` | D435 손잡이 위치·법선 및 유효성; lever_valid=false |
| `/perception/vision/c310/knob_observations` | 같은 메시지 | C310 손잡이 레버 방향; depth_valid=false |
| `/perception/vision/d435/object_detection` | `geometry_msgs/PoseArray` | 깊이를 얻은 D435 YOLO 물체의 3D 위치, optical frame |
| `/perception/vision/d435/obstacle` | `geometry_msgs/PoseArray` | 장애물 클래스의 가장 가까운 면 위치, optical frame (신규) |
| `/perception/vision/d435/puck_observations` | `craft_perception/PuckObservationArray` | puck 중심·축·선반 법선·standing, optical frame (신규) |
| `/perception/lidar/obstacle` | `geometry_msgs/PoseArray` | LaserScan 원래 frame의 유효 점 |
| `/perception/state/odom` | `nav_msgs/Odometry` | 지정한 입력 odometry 전달 |
| `/perception/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 실제 처리 Hz, 실행 시간, skip/error/overrun, 카메라별 복사 횟수·바이트 |

knob 위치는 카메라 optical frame에서 m 단위입니다. 레버 각도는 **영상 평면의
rad, 위쪽=0, 시계방향 양수**이며 로봇 관절각이 아닙니다. 관측 배열의 유효성
플래그를 확인해야 하며, 0으로 초기화된 무효 필드를 실제 관측으로 쓰면 안 됩니다.
빈 배열은 새 프레임에서 해당 물체를 찾지 못한 결과입니다. 입력이 끊기면 새 배열도
나오지 않으므로 소비자는 header 시각/수신 timeout을 확인해야 합니다.

중심 bbox의 유효 깊이 중앙값을 사용하고,
주변 사각 고리에서 깊이 차이가 작은 점을 모아 평면을 fitting합니다. 법선은
카메라를 향합니다. 레버는 빨강 HSV·밝기 Otsu·가장 큰 성분의 PCA로 추정합니다.
평면 fitting은 RANSAC이 아니며, deprojection은 pinhole 근사로 렌즈 왜곡을 무시합니다.

`/perception/vision/d435/object_detection`의 quaternion은 위치 목록 호환용 identity일 뿐 물체 자세가
아닙니다. 완전한 tool 자세를 만들어 `/perception/vision/object_pose`로 발행하지 않습니다.
planning에서 optical frame을 `base_link`/`map`으로 변환하는 작업도 별도로 필요합니다.

puck 축은 shelf 평면(링 영역 RANSAC) 위 8~80 mm 점으로 계산합니다. p95 높이 > 45 mm면
standing(축 = 선반 법선), 아니면 평면에 투영한 2D PCA 장축이며 180° 대칭이라 영상 오른쪽을 향하게
맞춥니다. 원본 스크립트의 EMA 평활화는 여러 puck에 대응이 모호해 포함하지 않았습니다.
`obstacle_detector.class_ids` 기본값은 신호등 4종(1~4)입니다. 실제 장애물 클래스에 맞게 조정하세요.

**상태 융합은 기존부터 TODO였고 이번에도 구현하지 않았습니다(대기).** `StateEstimator`는
`odometry_source_topic` 하나를 검증해 전달합니다. wheel·IMU·lidar EKF가 있다고 가정하면
안 됩니다. 장애물 검출 역시 단순 스캔 투영이며 클러스터링·좌표 변환은 하지 않습니다.

## 공유 메모리 v2

수정된 `craft_sensors`와 함께 재빌드·재시작해야 합니다. 구버전 3개 필드 입력은
거부합니다. D435는 `/craft_vision`, C310은 `/craft_c310`을 사용합니다.

```text
D435 /vision/d435/frame_meta: [offset,w,h,sequence,ROS stamp ns,depth unit nm]
C310 /vision/c310/frame_meta: [offset,w,h,sequence,ROS stamp ns]
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
D435·C310의 knob_sign 모델 경로를 바꿀 수 있습니다. 모델이 없거나 로드가 실패하면 시작 시 오류로 종료합니다.
모델 없이 ArUco만 점검하려면 `schedule.d435_yolo`, `schedule.puck_axis`, `schedule.knob_lever`의
`enabled`를 false로 설정하세요(꺼진 작업의 모델은 로드하지 않습니다).

```bash
colcon test --packages-select craft_perception --event-handlers console_direct+
colcon test-result --verbose
./build/craft_perception/ncnn_smoke src/craft_perception/models/knob_sign
./build/craft_perception/ncnn_smoke src/craft_perception/models/puck_blue 1
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
