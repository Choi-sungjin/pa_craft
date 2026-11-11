# 주사위 기억 로봇: 강화학습 적용 검토

검토일: 2026-11-02. 기준: `/home/craft/Downloads/mission.pdf` v5, 11쪽과 현재 working tree. 커밋되지 않은 구현도 포함했다. 소스·설정·기존 실험 기록을 검토한 설계 제안이며, 이번 검토에서 실물 시험이나 학습을 수행하지 않았다. 패키지의 알고리즘 경로를 검토했으며 외부 라이브러리 내부 전체나 모델 가중치를 검증한 것은 아니다.

결론: 첫 RL 실험은 **작업대 앞 마지막 접근·정렬**, 다음은 **집기·놓기의 작은 오차 보정**이 적합하다. A*, IK, 신호 기록, 할당, 판정 전체를 교체할 근거는 부족하다. 경기 완성이 목표라면 기본 미션과 일반적인 visual servo 기준선을 먼저 만든다.

## 1. 발제문이 요구하는 성능

- p.1, p.7: 시간 제한·시간 가산점 없음. 원점수 100점 중 선택 18점, 눈 배치 30점, 재검증 8점. 단순 속도보다 성공률이 중요하다.
- p.3: 1초 점등/1초 소등, 12회 기록, 마지막 3초 소등 확인. 불명 색·누락을 임의로 추정하면 안 된다. 동률 정렬은 R→Y→G 유지.
- p.4: 격자 간격 60 mm, 중심 오차 10 mm 이내, 2초 안정, 바닥 Z=70 mm. 빈 격자의 추가 퍽도 실패. 눈 방향은 지도 +Y 기준으로 고정.
- p.5: 12개 ID는 색상 고정 코드가 아니다. R/G는 색상, Y는 모양 통일. 전체 12개를 중복·누락 없이 동시에 배치한다.
- p.6–7: 새 센서 관측으로 장소별 ID·위치를 검증해야 한다. 목표 로그 복사는 불인정. 복귀는 차체 전체가 출발 구역 안에서 3초 정지.
- p.9–11: 선택 도전에도 별도 가산점은 없다. 경로 개선은 기본 완주 후 비교 과제다.

## 2. 현재 알고리즘과 RL 적합성

| 영역 | 현재 구현 및 근거 | 판단 |
|---|---|---|
| 카메라 입력 | `craft_sensors`: D435 RGB/depth 정렬·공유 메모리, C310 영상 | 유지. RL 대상보다 관측 품질의 기반 |
| 물체 인식 | `yolo_detector.cpp`: 모델 추론·후처리, ncnn/TensorRT backend | 필요 시 지도학습 개선. 인식만으로 RL이라 부르지 않음 |
| 퍽 인식 | `d435_perception.cpp`: 등록 AprilTag에서 ROI 생성. `puck_axis.cpp`: 주변 깊이 RANSAC 평면, 최소제곱 재적합, 높이 95 percentile, 투영 중심, 누운 퍽 PCA 축 | 유지·보강. 가려진 면 때문에 보이는 점의 중심과 실제 기하 중심이 다를 수 있어 실측 필요 |
| 퍽 속성 | `PuckObservation.msg`: ID, 중심, 축, standing 등. 색상·원기둥/직육면체 속성 필드 없음 | 이번 미션용 등록/관측 기능을 추가. standing은 모양 분류가 아님 |
| 기존 손잡이 인식 | `knob_depth.cpp`: 중앙 median depth·주변 평면. `knob_lever.cpp`: HSV/Otsu 또는 segmentation mask·연결 성분·PCA | 이번 미션의 주 병목으로 보이지 않음 |
| LiDAR 처리 | `scan_merger.cpp`: TF·FOV·floor ROI·각도 bin 병합. `lidar_calibrator.cpp`: 표적 기반 강체 정합 | 유지. 하드웨어/외부 파라미터 보정을 우선 |
| 장애물 필터 | `lidar_obstacle_filter.cpp`: 맵 벽 거리 제거, 다중 프레임 지속성, 연결 군집 크기 | 유지. RL의 성공 보상으로 장애물 제거 기준을 바꾸지 않음 |
| 위치 추정 | launch의 `robot_localization` EKF와 `slam_toolbox`; `state_estimator.cpp`는 검증·TF 변환. scan odometry/relocalizer는 likelihood field의 coarse-to-fine 탐색 | 유지. StateEstimator 자체가 EKF를 구현한 것은 아님 |
| 위치 신뢰도 | `localization_monitor.cpp`: Mahalanobis gating, dead reckoning 거리, 재위치 추정, IMU bias/scale 적응 | 유지. 이름의 learning은 reward 기반 RL이 아님 |
| 전역 경로 | `path_planner.cpp`: ESDF inflation, (x,y,4방향 heading) A*, 회전·벽 근접 비용, fillet 반경 이분 탐색, PASS/STOP_TURN/ALIGN, 속도 제한 | 유지. 고정 지도에서 먼저 RL로 바꿀 이점이 작음 |
| 경로 추종 | `lateral_controller.cpp`: Pure Pursuit·heading 오차에 따른 제자리 회전. `longitudinal_controller.cpp`: 제한 속도를 wheel rad/s로 변환 | 보조 RL 후보. 다만 마지막 접근을 별도 모드로 분리하는 편이 명확 |
| 주행 보정 | `wheel_odometry.cpp`: 실측 회전 다항식·제자리 선속도 억제. `slip_monitor.cpp`: wheel/IMU yaw 차이·covariance 증가 | 먼저 실측 보정과 적응제어 비교. RL 우선순위는 낮음 |
| 팔 계획 | `arm_ik_core.hpp`: Pinocchio DLS CLIK·관절 제한. `arm_planner.cpp`: multi-start, elbow load proxy, seed 거리 | IK 유지. 잔차 목표 보정은 후보 |
| 집기 접근점 | `grasp_region.cpp`: 정렬·실측 영역·연속 확인, 박스 거리 장벽 비용의 1 mm 탐색+golden section | 기하 계산 유지. 1차원 계산 대체보다 최종 접근 실행과 불확실성 대응을 학습 |
| 팔·LM 실행 | 모터 영점/방향/기어 변환, 범위 제한, 실제 관절 위치 기반 완료 | 유지. 저수준 PID·profile은 DYNAMIXEL/펌웨어 측 |
| 파지 | `gripper_controller.cpp`: 전류 threshold/debounce, 접촉각+squeeze 유지, 각도 또는 접촉 판정으로 완료 | 조건부 RL 후보. 닫힘 완료와 실제 퍽 파지 성공을 분리해야 함 |
| 적재 clock | `clock_controller.cpp`: 12개 slot을 최단 회전으로 정렬 | 유지. slot별 퍽 inventory는 미션 계층에서 관리할 필요 |
| 집기 데모 | `demo/puck_pick/plan.py`: 해석적 평면 IK·최소 tool tilt 탐색. `execute.py`: 미리 계산한 단계 실행 | 부분 대체 후보. 반복 visual feedback을 사용하는 잔차 제어로 확장 |
| 실행 관리 | `state_machine.cpp`, planning/control master: readiness, task ID, timeout, freshness, hold, stop/fault | 유지. learned policy도 이 계층을 통해 실행 |
| 전역 초기 위치 | `demo/corner/map_tools.py`: 지도 전체 위치·yaw 탐색, 상위 후보 분리, 국소 정밀 탐색 | 유지. 고정 맵의 관측 동작 선택만 추후 후보 |
| 기타 도구 | 주행·IMU·slip·LiDAR 측정, 회전 계수 fitting, 지도/posegraph 도구 | 학습 환경과 평가 데이터에 활용. 도구 자체 대체 필요성 낮음 |

C++ planning이 실행 경로이고 Python A*는 reference다. Python 파일만 바꾸면 현재 실행기에 반영되지 않는다.

## 3. 아직 없는 미션 기능과 선행 조건

검색한 현재 `src/`, `demo/`, `firmware/`에서 이번 과제용 신호 12회 기록, Code 128 해독·정렬, 전체 속성 할당, 주사위 패턴 생성, 세 장소 최종 재검증의 완성된 파이프라인은 찾지 못했다. `missions.yaml`과 `tag_registry.yaml`은 비어 있다. 이것은 기존 주행/집기 기능이 전혀 없다는 뜻이 아니라, 범용 작업 실행기가 새 과제 전체를 구현한 상태는 아니라는 뜻이다.

특히 먼저 해결해야 할 사항:

1. `grasp_region`의 lying/standing 범위가 모두 0이다. 앞면 위치·폭·good_plane도 0이어서 현재 설정으로 graspable/stop plan을 유효하게 만들 수 없다. 실측 없이 RL로 우회하지 않는다.
2. 일반 주행 목표 오차는 planning/control 모두 0.10 m, heading tolerance는 control 0.5 rad다. 이것만으로 10 mm 배치가 보장되지 않는다. 차체를 10 mm로 제어해야 한다는 뜻도 아니다. 작업대 상대 관측과 팔 제어를 합친 최종 퍽 위치 오차를 평가해야 한다.
3. IK tolerance 0.1 mm는 모델 계산의 수렴 오차다. 실물의 퍽 중심 오차 0.1 mm를 뜻하지 않는다. 카메라 TF, 깊이 중심 편향, 관절 영점, 백래시, 퍽과 집게의 상대 위치를 별도 측정한다.
4. 현재 팔 URDF에는 `inertial`/`collision` 요소가 없다. RViz 모델만으로 접촉 학습용 simulator가 준비된 것은 아니다. 질량·관성·마찰·구동 응답과 충돌 형상을 추가·검증해야 한다. IK의 elbow load는 실제 질량 없는 proxy다.
5. 데모의 로봇 앞=-Y, LM=X 설명과 주행/접근의 전방 +X 규약을 확인한다. 동일한 `base_link` 이름의 의미가 일치하는지 검증하고 실제 transform으로 연결한다. PDF의 mm/+Y 위 좌표와 SLAM map 좌표도 원점·회전·축을 보정한다.
6. 현재 퍽 인식은 등록 tag가 보여야 시작된다. README의 퍽 밑면 tag가 실물에서도 밑면에 있다면 선반 위에서는 보이지 않을 수 있다. 인식·재검증 시 ID 관측 방법을 확보한다. 이번 발제문에 없는 tag 조건은 실물에서 확인한다.
7. 고정 지도는 차체 원형 footprint로 검사한다. 팔·운반 퍽의 돌출과 작업대 접촉은 별도 검사 대상이다.

## 4. 1순위: 마지막 접근·정렬 정책

대체 범위는 A*나 `plan_stop()` 전체가 아니라, 작업대 앞 staging pose에서 실제 graspable 상태에 들어가는 **짧은 feedback 제어 구간**이다. 현재의 전방 1차원 stop 최적화는 y/heading 오차를 고치지 못한다. 그 부분을 baseline visual servo로 먼저 구현하고 RL과 비교한다.

- 관측: 작업대/퍽 상대 위치, box yaw, 퍽 축, 거리, 관측 신뢰도·age, base 속도, IMU yaw rate, wheel feedback, 최근 행동. 필요하면 짧은 관측 history로 지연·slip을 반영한다.
- 행동: baseline의 `(v, omega)`에 작은 `(delta_v, delta_omega)` 보정. 4WD skid steer에 직접 횡이동 명령을 주지 않는다. 속도·보정 범위는 실측 후 결정한다.
- 종료: 영역 내부뿐 아니라 정렬·실제 정지·신선한 연속 관측까지 확인한다. 최종 집기 성공과의 상관을 평가한다.
- 보상: 상대 오차 감소, 영역 안에서 안정 정지에 보상. 박스 접촉, 과도한 진동, 관측 상실에는 불이익. 가까이 가기만 하면 높은 점수를 받는 보상은 피한다.
- 실행: 별도 APPROACH 작업/phase와 feedback이 필요하다. 현재 TaskCommand에는 navigate/manipulator/gripper/clock/wait만 있으므로 정책 노드에서 바퀴로 직접 우회 발행하는 설계는 피한다. task ID·watchdog·hold·실측 stop 규약을 유지한다.
- 제약: 접근용 충돌 모델·정지거리 검사는 독립적으로 수행한다. 일반 주행의 큰 원형 inflation 때문에 도달 불가인 목표를 단순 허용하거나, RL 출력에 속도 clamp만 하는 것으로 충돌 안전을 주장하지 않는다.

RL 기대 효과는 조립·마찰·측정 지연의 변화 아래 성공률 향상이다. 기준선 visual servo로 충분히 되면 이 구간에도 RL을 넣을 이유는 약하다.

## 5. 2순위: 집기·놓기 잔차 보정

기존 IK로 구한 목표를 사용하고, 작업대 상대 카메라 관측과 모터 feedback으로 작은 목표 보정만 학습한다. 12개 운반 전체를 하나의 episode로 시작하지 않는다.

- 관측: 퍽·목표 격자의 상대 위치, 가능한 TCP 관측, 관절 위치·목표 오차, 그리퍼 각도·전류, 작업 단계, 주변 놓인 퍽/격자 점유.
- 행동: 도달 가능한 TCP 위치의 작은 보정과 제한된 자세/진입 설정. 5자유도 팔에 임의의 6D pose를 강제하지 않고 IK feasible set 안에서 제한한다. motor torque 직접 출력은 첫 실험에서 제외한다.
- 목표: 집기 후 실제로 퍽이 들렸는지, 놓기 후 중심 오차 10 mm 이내·2초 안정인지 관측으로 판정한다. 이미 배치한 이웃 퍽을 건드리면 실패로 반영한다.
- 기준선: 재보정한 IK + 고정 waypoint, 이어서 일반 visual servo, 마지막으로 residual policy를 비교한다. 일정한 offset은 재보정이나 회귀 모델로 해결되는지도 먼저 본다.
- 주의: 전류 상승은 물체 파지 외에도 집게/박스 걸림에서 나타날 수 있다. `is_target_reached()`의 닫힘 완료를 물체 보유 성공 label로 쓰지 않는다. 운반 중 낙하와 놓은 뒤 위치도 별도 확인한다.

놓기 작업의 10 mm 요구 때문에 기대 효과는 크지만, 접촉·가림·gripper 내 퍽 pose 추정까지 필요해 1순위 접근보다 구현 난도가 높다.

## 6. 후순위 후보

- slip에 따른 Pure Pursuit 보정: lookahead/속도 또는 bounded yaw 보정. 기존 실측 skid factor·회전 계수에 대해 부하·바닥 변화에서 오차가 남는지 먼저 확인한다. 단순 계수 예측이면 회귀/시스템 식별이 더 자연스럽다.
- 능동 관측: tag/퍽이 가려졌을 때 어느 관측 자세로 이동할지 선택. 위치 후보를 몇 개 두고 정보량/가림 기반 규칙과 비교한다. 보이지 않는 신호 색을 추측하는 용도로 사용하지 않는다.
- 파지 squeeze/진입 조정: 전류·각도 기반 고정 규칙보다 개선 여지가 실측될 때만. 전류 제한과 stop 판정은 학습에서 분리한다.
- 수집·배달 순서: 연구용으로는 가능하지만 현 과제에서는 우선순위가 낮다. 방문할 박스는 6개, 목적지는 3개이고 12-slot 저장장치가 있다. 실제 전량 적재/안전 운반 가능 여부에 따라 묶음 수거·배달 또는 용량 제약 경로 최적화를 결정한다. 직선거리 대신 A* 접근점 간 비용, 회전, 적재 상태와 충돌 위험을 사용한다.

## 7. RL로 대체하지 않을 부분

신호 기록은 색 검출+점등/소등 상태 전이+timestamp 검증, 바코드는 decoder+허용 문자열 whitelist, 정렬은 stable sort, 눈 좌표는 고정 template, 재검증은 새 관측과 목표의 matching으로 구현한다. 규칙이 명확하고 오류를 설명할 수 있어야 한다.

전체 퍽 할당도 작은 exact search가 적합하다. 전체 red 수를 NR, green 수를 NG, 신호 횟수를 r,y,g라 하면 Y에 필요한 색상 수는 다음과 같이 이미 결정된다.

```
Y_red   = NR - r
Y_green = NG - g
Y_red + Y_green = y
```

원기둥/직육면체 각각에 대해 해당 모양의 red가 Y_red개, green이 Y_green개 이상인지 검사하면 기본 가능 여부가 나온다. ID 중복·누락·개수·속성도 함께 검증한다. Y에 들어갈 ID 후보를 골라 남은 red/green을 R/G에 배정하면 된다. 최대 Y 부분집합 수도 C(12,6)=924개여서 이 규모의 전체 탐색을 RL로 바꿀 이유가 작다. 여러 유효 할당의 운반 비용이 다르면 그다음에 exact/heuristic 경로 비용을 비교한다.

## 8. 학습·검증 순서

1. 신호/바코드/속성/전체 할당/눈 template/재검증을 deterministic 미션 계층으로 완성한다. 임의 색상 보정이나 정답 로그 복사를 허용하지 않는다.
2. 한 작업대에서 양 모양·양 색상·다양한 상대 위치로 관측 및 graspable 실측을 한다. 오차 원인을 센서, 좌표계, 기구, 제어로 분리한다.
3. baseline visual servo 접근과 IK pick/place를 완성하고 실패 사례를 기록한다. 실험마다 실제 ID·속성·시작 상태를 남긴다.
4. 첫 RL은 짧은 접근 episode의 저차원 state/action으로 구성한다. 연속 action용 PPO 또는 SAC를 후보로 두되 현 단계에서는 성능 우열을 단정하지 않는다. simulator/연산 예산/데이터 재사용 여부에 맞춰 하나를 선택한다.
5. domain randomization 범위는 실측으로 정한다. 접근은 마찰·slip·지연·노이즈·적재량, 조작은 추가로 질량·백래시·gripper 접촉 특성을 포함한다. 실제 policy 입력에는 배포 시 관측할 수 있는 값만 넣는다.
6. 같은 초기 조건에서 baseline, visual servo, residual RL을 비교한다. 학습 seed를 여러 개 사용하고 평가 조건은 학습에서 분리한다. simulator 결과와 실물 결과는 분리한다.
7. 접근 성공률·접촉 수·관측 상실·최종 상대 오차, 집기 성공률·낙하율, 배치 최대/분위수 오차·10 mm/2초 합격률, 세 장소 동시 정답·전체 완주율을 기록한다. 평균 reward만으로 판단하지 않는다. 시행 수와 성공 수, 가능하면 신뢰구간을 함께 제시한다.

12개가 모두 성공해야 하는 특성상 개별 작업 신뢰도가 중요하다. 독립·동일 성공률 p라는 단순 가정에서 12회 모두 성공할 확률은 p^12이며 p=0.95면 약 54%, p=0.99면 약 89%다. 실제 오류는 상관될 수 있고 집기·놓기·운반 단계도 있어, 이 수치를 완주율 예측치로 사용하면 안 된다.

## 9. 참고 근거와 적용 한계

- Johannink et al., *Residual Reinforcement Learning for Robot Control*: 기존 feedback control과 RL residual의 결합 및 실제 접촉 조립 실험. 기존 IK/제어에 보정을 추가하자는 방법적 근거다. CRAFT에서 개선된다는 실험 결과는 아니다. https://arxiv.org/abs/1812.03201
- Peng et al., *Sim-to-Real Transfer of Robotic Control with Dynamics Randomization*: dynamics randomization을 이용한 robot pushing의 sim-to-real 실험. 마찰·구동 불확실성 randomization의 근거이며 CRAFT pick/place 전이 성공을 보장하지 않는다. https://xbpeng.github.io/projects/SimToReal/
