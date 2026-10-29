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

### 소유 관계

```text
PerceptionMaster  (rclcpp::Node, perception_master.cpp)
│   ROS 입출력 · configure() · 작업 타이머 · 진단
│
├── D435Perception d435_                     (d435_perception.cpp)
│   ├── YoloDetector    yolo_                → d435_yolo 작업 (knob_sign)
│   │   └── InferenceBackend  (NcnnBackend | TensorRtBackend)
│   ├── AprilTagReader  apriltag_            → 공용 태그 ID 인식
│   ├── KnobDepth       knob_depth_          → knob_depth 작업       ┐
│   ├── ObjectDetector  object_detector_     → object_detector 작업  ├ d435_yolo 박스 공유
│   ├── ObstacleDetector obstacle_detector_  → obstacle_detector 작업 ┘
│   └── PuckAxis        puck_axis_           → puck 태그 박스 + depth로 축 계산
│
├── C310Perception c310_                     (c310_perception.cpp)
│   ├── YoloDetector    yolo_                → knob_lever 작업 (knob_sign, 또는 seg 모델)
│   │   └── InferenceBackend
│   └── KnobLever       lever_               → hsv | yolo_seg 마스크 → 각도
│
├── LidarPerception lidar_                   → lidar_obstacle 작업 (병합 + 장애물)
├── StateEstimator        state_estimator_   → state_estimator 작업 (대기, odom 전달만)
│
└── CameraInput cameras_[2]   [kD435], [kC310]
    └── SharedFrameReader reader   → /dev/shm/craft_vision, /dev/shm/craft_c310 읽기
```

### 파일

```text
craft_perception/
├── config/perception.yaml          # 모든 값 필수, C++ 기본값 없음
├── config/tag_registry.yaml        # tag ID → mission 또는 puck 번호 공용 해시 테이블
├── config/ekf.yaml, slam.yaml, slam_ekf.yaml  # 위치추정 (craft_bringup이 실행)
├── include/craft_perception/
│   ├── perception_master.hpp
│   ├── d435_perception.hpp         # D435Config, D435Perception
│   ├── c310_perception.hpp         # C310Config, C310Perception
│   ├── yolo_detector.hpp           # YOLO (구 ObjectDetector), ncnn/TensorRT 백엔드, seg 마스크
│   ├── apriltag_reader.hpp
│   ├── knob_depth.hpp
│   ├── knob_lever.hpp
│   ├── object_detector.hpp         # YOLO 박스 + depth → 3D 위치
│   ├── obstacle_detector.hpp       # YOLO 박스 + depth → 가까운 장애물 면
│   ├── puck_axis.hpp               # puck_axis_live.py 이식
│   ├── lidar_obstacle_detector.hpp # 구 ObstacleDetector (LaserScan)
│   ├── state_estimator.hpp         # 대기 (odometry 전달만)
│   ├── imu_processor.hpp           # OpenCR IMU 2개 → 차체 yaw rate
│   ├── shared_frame.hpp
│   └── perception_types.hpp
├── models/knob_sign/
└── src/ (같은 이름의 .cpp, tensorrt_backend.cpp, geometry_utils.hpp)
```

| 카메라 | 작업 (`schedule.<이름>`) | 기본 Hz | 입력 / 역할 |
| --- | --- | --- | --- |
| D435 | `d435_yolo` | 10 | BGR → knob_sign YOLO 1회. 결과를 프레임과 묶어 아래 3개가 공유 |
| D435 | `knob_depth` | 10 | YOLO knob 박스 + aligned depth → 위치·패널 법선 |
| D435 | `object_detector` | 10 | YOLO 전체 박스 + depth → 박스 중앙부 중앙값 깊이의 3D 위치 |
| D435 | `obstacle_detector` | 10 | YOLO 장애물 클래스 박스 + depth → 박스 내 가까운 면(하위 분위수), 거리 제한 |
| D435 | `apriltag` | 15 | BGR → 왼쪽부터 정렬한 AprilTag ID. Planning은 바닥·벽면 태그를 미션으로 매핑 |
| D435 | `puck_axis` | 10 | 설정된 puck 밑면 AprilTag ID/ROI + depth → puck ID, 선반 평면, standing/lying, 축 |
| C310 | `knob_lever` | 10 | 자체 YOLO + 영상 평면 레버 방향. 레버 픽셀은 `c310.knob_lever.lever_source`로 선택 (`hsv` / `yolo_seg`) |
| - | `lidar_obstacle` | 20 | 전·후방 스캔 병합 + 장애물 점 투영 |
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
- 모델 인스턴스: D435 knob_sign, C310 knob_sign 각 1개. 각 인스턴스는 한 작업만
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
Puck은 색상 YOLO를 사용하지 않습니다. `config/tag_registry.yaml`을
`tag ID → {mission: 이름}` 또는 `tag ID → {puck: 번호}` 해시 테이블로 읽어
바닥·벽면 미션 태그와 puck 밑면 태그를 구분합니다. 현재 레지스트리는
발제의 공식 ID 배정을 기다리며 비어 있습니다.

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
| `/perception/vision/d435/apriltag` | `std_msgs/Int32MultiArray` | AprilTag ID를 영상 왼쪽부터 정렬. PlanningMaster가 설정된 미션 태그만 사용 |
| `/perception/vision/d435/detections` | `craft_perception/DetectionArray` | D435 `d435_yolo`의 클래스·확률·픽셀 bbox |
| `/perception/vision/c310/detections` | 같은 메시지 | C310 `knob_lever`의 YOLO 검출 |
| `/perception/vision/d435/knob_observations` | `craft_perception/KnobObservationArray` | D435 손잡이 위치·법선 및 유효성; lever_valid=false |
| `/perception/vision/c310/knob_observations` | 같은 메시지 | C310 손잡이 레버 방향; depth_valid=false |
| `/perception/vision/d435/object_detection` | `geometry_msgs/PoseArray` | 깊이를 얻은 D435 YOLO 물체의 3D 위치, optical frame |
| `/perception/vision/d435/obstacle` | `geometry_msgs/PoseArray` | 장애물 클래스의 가장 가까운 면 위치, optical frame (신규) |
| `/perception/vision/d435/puck_observations` | `craft_perception/PuckObservationArray` | 태그로 확정한 `puck_id`, 중심·축·선반 법선·standing, optical frame |
| `/perception/lidar/obstacle` | `geometry_msgs/PoseArray` | LaserScan 원래 frame의 유효 점 |
| `/perception/state/odom` | `nav_msgs/Odometry` | 지정한 입력 odometry 전달 |
| `/perception/imu/data` | `sensor_msgs/Imu` | 주(예비) OpenCR IMU의 차체 yaw rate, `base_link` (robot_localization 입력) |
| `/perception/localization/state` | `std_msgs/String` | `ok` / `dead_reckoning` / `relocalize` / `lost` (localization:=ekf). planning은 `lost`면 Fault |
| `/perception/localization/relocalized` | `geometry_msgs/PoseWithCovarianceStamped` | 넓은 탐색 재위치 결과 (ekf_global `pose1`) |
| `/perception/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 실제 처리 Hz, 실행 시간, skip/error/overrun, 카메라별 복사 횟수·바이트 |

### IMU 처리 (`imu_processor`, `imu.*`)

control_master는 두 OpenCR(wheel_clock, lm_arm, 모두 차체 고정)의 IMU를 원본 그대로
`/control/imu/wheel_clock`, `/control/imu/lm_arm`으로 보냅니다(보드 좌표계, 칩 영점 오프셋 포함).
`imu_processor`는 샘플마다 바로 처리합니다(타이머 작업이 아님).

- **보드별 보정**: 두 보드 모두 똑바로(보드 z = 차체 위쪽) 달려 있어 차체 yaw rate = gz − 오프셋입니다.
  바퀴가 멈춘 동안(`/control/wheel/odom`) 자이로 평균으로 칩 오프셋을 잽니다. 처음 `calibration_samples`개를
  끊김 없이 모으고, 그 뒤에도 멈출 때마다 `bias_tracking_gain`으로 따라갑니다(켠 뒤 온도 변화). 가속도는
  정지 판정(중력만 보이는지)에만 씁니다. 수평 방향(약 180° 돌아 있음)은 yaw rate에 영향이 없습니다.
  로그에 보드별 오프셋이 한 번 찍힙니다.
- **주/예비**: `primary` 보드(기본 wheel_clock)를 쓸 수 있으면 항상 그 값만 씁니다. 예비 보드는 주 보드가
  안 들어오거나(끊김, `imu_ok=0`) 아직 보정되지 않았을 때만 씁니다. 두 값을 섞지 않습니다. 센서가 두 개면
  다르다는 건 알아도 어느 쪽이 틀렸는지는 알 수 없어서입니다.
- **굳음 판정**: 원본 자이로 3축이 `frozen_samples`(10)번 연속 완전히 같으면 그 보드는 굳은 것으로 보고 쓰지
  않습니다(예비로 전환, 바이어스 추적도 멈춤). 펌웨어가 노이즈 있는 4개 평균을 소수로 보내므로 살아 있는 센서는
  같은 값을 반복하지 않습니다. 같은 수만큼 값이 바뀌면 돌아옵니다.
- **불일치 감시 (경고만)**: 두 보드는 최대 20 ms 어긋나게 샘플링하므로 각자의 직전 기울기로 같은 시각 값을
  맞춘 뒤 비교합니다. `max_disagreement + max_disagreement_ratio × |yaw rate|`보다 다른 출력이
  `disagreement_samples`번 이어지면 경고합니다. 쓰는 보드는 바꾸지 않습니다.
- **튀는 값 거르기 (보드별)**: |yaw rate|가 `max_yaw_rate`를 넘거나, 직전에 받아들인 값에서
  `max_yaw_accel × 시간간격 + jump_noise`보다 크게 바뀐 샘플은 버립니다. 다음 샘플이 같은 값이면 실제 변화로
  보고 받아들입니다(빠른 실제 변화는 한 샘플 늦게, 한 샘플짜리 튐은 끝까지 안 들어옴). 기준은 TurtleBot3
  Waffle Pi 값(최대 회전 1.82 rad/s, 회전 가속 3.2 rad/s²)입니다. 보정 중의 튐은 건너뛰고, 3개 이상 이어지면
  실제 움직임으로 보고 다시 시작합니다.
- **정지 중 0**: 바퀴가 멈춰 있고 yaw rate가 `still_rate`보다 작으면 0으로 냅니다(바이어스가 쌓이지 않음).
- 출력은 주 보드 샘플마다, 주 보드가 끊기면 예비 보드 샘플마다 냅니다. 보드가 모두 보정되기 전에는
  내지 않으니 켠 뒤 로봇을 2초 이상 멈춰 두세요.

### 위치추정 감시 (`localization.*`, localization:=ekf에서만)

- **SLAM 수락 판정**: SLAM 위치를 같은 시각의 ekf_global 위치와 Mahalanobis로 비교합니다(`rejection_sigma`,
  ekf.yaml `pose0_rejection_threshold`와 같은 값). 거부되면 IMU+바퀴만으로 가는 추측 항법(`dead_reckoning`)입니다.
- **스캔-지도 일치**: SLAM과 EKF가 함께 틀릴 수 있어서(밀렸을 때 slam_toolbox도 odom 예측 근처만 찾음), 1초마다
  최신 스캔을 EKF 위치에서 SLAM 지도에 대 점수를 냅니다. `misfit_checks`번 연속 `relocalize.min_score` 미만이면
  SLAM이 맞다고 해도 추측 항법으로 봅니다. 점수는 10초마다 로그에 찍히니 실물에서 이 값으로 기준을 조정하세요.
- **넓은 탐색 재위치**: 추측 항법이 `dead_reckoning_m`(0.3 m)를 넘으면 SLAM 지도에서 EKF 위치 ±`window_m`,
  ±`window_deg` 안을 다 찾습니다(sim 약 40 ms). 찾으면 `/perception/localization/relocalized`로 ekf_global에 넣고
  추측 항법을 다시 셉니다. 못 찾으면 `lost` → planning Fault. 외부 기준점 없이 SLAM 지도 자체가 기준입니다.
- **IMU 되먹임**: SLAM이 `imu_learning.stable_s` 이상 계속 받아들여지면 map→odom yaw(SLAM이 IMU 위에 더한 회전)를
  IMU의 남은 오차로 보고, 직진 구간의 변화율로 바이어스를, 회전 한 번의 비율로 배율을 쓰고 있는 보드에 천천히
  보정합니다(상한 `bias_limit`, `scale_limit`). sim에서 넣은 바이어스 +0.003 rad/s·배율 1.03을 −0.0029·0.971로 배웠습니다.

### LiDAR 연결

`perception_master`가 카메라 클래스와 같은 방식으로 `LidarPerception lidar_`를 소유합니다.
ROS 구독·TF 조회·타이머·발행은 master가 담당하고, `lidar_.merge(...)`로 병합한 뒤
`lidar_.detect_obstacles(...)`를 호출합니다. 병합 알고리즘은 `lidar_processor` 패키지의
`ScanMerger` 라이브러리로 분리해 standalone 노드와 공유합니다.

```text
/lidar/front/scan ─┐
                  ├─ master → lidar_.merge() ─┬─ /lidar/merged (SLAM/RViz)
/lidar/rear/scan ──┘                          └─ lidar_.detect_obstacles()
                                                   → /perception/lidar/obstacle
```

기본 `lidar.input_mode: raw`에서는 원본 스캔을 SensorDataQoS로 받습니다. `lidar_obstacle`
작업이 새 전방 스캔마다 최대 20 Hz로 실행되며, 전·후방 시각 차이가 `lidar.max_age`(0.2 s)를
넘거나 후방 TF/스캔이 없으면 전방만 처리합니다. `max_input_age_s`(0.5 s)가 지난 입력은
처리하지 않습니다. 전방 TF를 기다리는 동안에는 같은 스캔을 다시 시도합니다.
`perception.yaml`의 `lidar` 항목에서 bins·FOV·몸체 필터·병합 최대 거리를 설정합니다.

병합 출력은 `LaserScan`, `base_link`, Reliable이며 전방 측정 시각을 유지합니다.
장애물 출력은 기본 3 m 이내 점의 `PoseArray`입니다. NaN/Inf는 제외하고 z=0,
orientation은 단위 쿼터니언입니다. 개체별 군집이 아닌 스캔 점 집합이며 빈 결과도 발행합니다.

전체 실행 `ros2 launch craft_bringup craft.launch.py`는 드라이버·static TF만 별도로 띄웁니다.
별도 병합 노드는 실행하지 않습니다. Pi에서 센서를 실행하려면
`ros2 launch lidar_processor lidar_processor.launch.py start_processor:=false`를 사용하고,
PC에서는 `ros2 launch craft_bringup craft.launch.py start_lidar:=false`로 실행합니다.
Pi/PC의 ROS domain과 시각을 맞춰야 합니다.

Pi의 기존 병합 노드를 계속 쓰려면 전체 실행에
`start_lidar:=false lidar_input_mode:=merged`를 지정합니다. 이때 master는 `/lidar/merged`를
구독해 장애물만 계산하며 병합 토픽을 재발행하지 않습니다.

두 패키지를 빌드하고 워크스페이스를 source한 후
`python3 src/craft_perception/test/test_lidar_integration.py`로 master 내부 모듈을 검증합니다.
`--input-mode merged`를 추가하면 standalone 병합 노드 경로도 검증합니다.
별도 ROS domain에서 전·후방 가상 스캔을 발행해 좌표·header·거리 필터·빈 출력을 검사합니다.

knob 위치는 카메라 optical frame에서 m 단위입니다. 레버 각도는 **영상 평면의
rad, 위쪽=0, 시계방향 양수**이며 로봇 관절각이 아닙니다. 관측 배열의 유효성
플래그를 확인해야 하며, 0으로 초기화된 무효 필드를 실제 관측으로 쓰면 안 됩니다.
빈 배열은 새 프레임에서 해당 물체를 찾지 못한 결과입니다. 입력이 끊기면 새 배열도
나오지 않으므로 소비자는 header 시각/수신 timeout을 확인해야 합니다.

### knob 레버 마스크 (`c310.knob_lever.lever_source`)

| 값 | 레버 픽셀을 얻는 법 | 필요한 모델 |
| --- | --- | --- |
| `hsv` (기본) | knob 박스 안 빨강(HSV) 중 Otsu로 밝은 부분 → 노이즈 제거 → 가장 큰 덩어리 | 지금 knob_sign detect 모델 |
| `yolo_seg` | knob 박스 안에 중심이 있는 lever 클래스 중 최고 점수의 seg 마스크 | lever 클래스를 포함한 YOLO-seg 모델 |

두 방식 모두 그 뒤는 같습니다: PCA 축 → knob 박스 중심(피벗)에서 더 멀리 뻗은 쪽 → 시계 각도.
`yolo_seg`를 쓰려면 `c310.yolo`를 seg 모델로 바꾸고 blob을 3개(`ncnn_blobs: [in0, out0, out1]`,
`tensorrt_blobs: [images, output0, output1]`)로 적어 prototype 출력을 받고, `class_names`에 lever를 넣은 뒤
`lever_class_id`를 그 인덱스로 맞춥니다. blob이 3개면 `YoloDetector`가 seg 모델로 동작해 박스마다
`Detection::mask`를 채웁니다(계수·prototype 곱 → letterbox 역변환 → sigmoid>0.5). seg export의 blob 이름은
Ultralytics 기본값을 가정했으니 실제 export 결과로 확인하세요.

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

ROS 2, OpenCV 개발 패키지(aruco 모듈 포함, AprilTag 사전이 여기 있음), ncnn C++ 설치가 필요합니다.
센서까지 빌드하려면 기존처럼 librealsense2 SDK가 필요합니다.

```bash
colcon build --packages-up-to craft_bringup \
  --cmake-args -DBUILD_TESTING=ON -Dncnn_DIR="$HOME/ncnn-install/lib/cmake/ncnn"
source install/setup.bash
ros2 run craft_perception perception_master --ros-args --params-file \
  "$(ros2 pkg prefix craft_perception)/share/craft_perception/config/perception.yaml"
```

기존 `craft.launch.py`도 master를 실행하며 `d435_model_dir:=...`, `c310_model_dir:=...`로
카메라별 YOLO 모델 경로를 따로 바꿀 수 있습니다(launch 값이 yaml의 `*.yolo.model_dir`보다 우선). 모델이 없거나 로드가 실패하면 시작 시 오류로 종료합니다.
모델 없이 AprilTag와 puck depth를 점검하려면 `schedule.d435_yolo`와
`schedule.knob_lever`를 false로 설정하세요. `puck_axis`는 AprilTag를 사용하므로 켜 둘 수 있습니다.

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
실제 ncnn 추론 중 AprilTag 처리와 입력 단절 시 발행 중단을 검사합니다. 실제 장치의
성능·캘리브레이션·검출 정확도 검증을 대신하지 않습니다.

