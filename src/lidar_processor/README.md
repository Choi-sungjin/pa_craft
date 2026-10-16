# lidar_processor

CRAFT 로봇의 전방·후방 LDS-03 LiDAR 두 대를 하나의 **로봇 중앙 가상 360° LiDAR**로 합치는 패키지입니다. 다이어그램의 `lidar_processor` 에 해당하며, **라즈베리파이에서 실행**합니다.

```text
LDS-03 전방 ─▶ /lidar/front/scan ─┐
                                   ├─▶ lidar_processor ─▶ /lidar/merged ─▶ obstacle_detector, 2D SLAM
LDS-03 후방 ─▶ /lidar/rear/scan  ─┘     (FOV 필터 → 점 일체화 → 로봇 중앙 기준 거리)
```

## lidar_processor 가 하는 일

1. **FOV 필터**: 각 LiDAR 의 로봇 바깥 방향 기준 ±(FOV/2) 밖이면서 로봇 중앙에서 `fov_near_m` 안쪽인 점을 버립니다. FOV 만으로 자르면 옆쪽이 통째로 안 보여서, 가까운 점(기둥)만 지우도록 했습니다. 로봇 기둥·몸체가 걸리는 각도를 잘라냅니다. FOV 중심은 TF 로 자동 계산하므로 LiDAR 장착 방향과 무관합니다.
2. **점 일체화**: 두 LiDAR 의 점을 TF 로 `base_link`(로봇 중앙) 좌표로 옮겨 하나의 점 집합으로 합칩니다.
3. **가상 360° LiDAR**: 로봇 중앙에서 본 각도를 0.5° 칸(720칸)으로 나누고, 칸마다 가장 가까운 점만 남깁니다. 따라서 `/lidar/merged` 의 `ranges[k]` 는 **로봇 중앙에서 장애물까지의 거리**입니다.

## 토픽

| 토픽 | 타입 | frame | QoS | 비고 |
| --- | --- | --- | --- | --- |
| **`/lidar/merged`** | LaserScan (−180°~180°, 0.5°) | `base_link` | RELIABLE | **팀 노드가 구독하는 유일한 출력** |
| `/lidar/front/scan`, `/lidar/rear/scan` | LaserScan | `laser_front`, `laser_rear` | BEST_EFFORT | 드라이버 원본. 드라이버가 별도 프로세스라 노드 간 전달용으로 남음 |
| `/tf_static` | TF | | | `base_link → laser_front / laser_rear` |

`/lidar/merged` 에서 각도 `θ = angle_min + k × angle_increment`, 거리 `ranges[k]` (NaN = 그 방향에 점 없음).

## 처음 한 번만: Pi 설정

```bash
# 1) LDS-03 드라이버 (소스 빌드)
cd ~/robot_ws/src && git clone -b main https://github.com/ROBOTIS-GIT/coin_d4_driver.git

# 2) LiDAR 포트 고정 (꽂힌 USB 구멍 기준)
bash lidar_processor/scripts/find_lidar_ports.sh          # KERNELS 값 확인
#    다르면 udev/99-craft-lidar.rules 의 KERNELS 를 고친 뒤
sudo cp lidar_processor/udev/99-craft-lidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/lidar_front /dev/lidar_rear               # 두 링크가 보여야 함

# 3) PC 와 같은 도메인
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc && source ~/.bashrc

# 4) 빌드
cd ~/robot_ws && colcon build --packages-select coin_d4_driver lidar_processor
```

## 실행 (Pi)

```bash
source ~/robot_ws/install/setup.bash
ros2 launch lidar_processor lidar_processor.launch.py
```

정상 로그:

```text
TF laser_front: x=0.163 y=0.000 yaw=180.0deg
TF laser_rear: x=-0.163 y=0.000 yaw=0.0deg
```

| launch 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `front_fov_deg` / `rear_fov_deg` | `200.0` | 각 LiDAR 의 로봇 바깥 방향 기준 전체 FOV [deg] |
| `fov_near_m` | `0.30` | FOV 밖 점 중 로봇 중앙에서 이 거리 안쪽만 버림 [m]. 기둥은 지우고 옆쪽 먼 장애물은 남김 |
| `body_radius_m` | `0.25` | 로봇 중앙에서 이 거리 안쪽 점은 FOV 와 상관없이 버림 [m]. FOV 안쪽에 걸리는 기둥 제거용 |
| `front_x`, `front_y` / `rear_x`, `rear_y` | `0.1437`, `-0.0113` / `-0.1437`, `0.0113` | 로봇 중앙 기준 LiDAR 위치 [m] (퍽 보정 결과) |
| `front_yaw_deg` / `rear_yaw_deg` | `180.0` / `0.97` | LiDAR 방향 (두 대 모두 180° 돌려 장착, 후방은 퍽 보정 +0.97°) |
| `front_port` / `rear_port` | `/dev/lidar_front` / `/dev/lidar_rear` | udev 링크 |

### FOV 조정 (실행 중)

```bash
ros2 param set /lidar_processor front_fov_deg 190.0
ros2 param set /lidar_processor rear_fov_deg 190.0
```

RViz 에서 `/lidar/merged` 를 보며 기둥 점이 사라질 때까지 줄이고, 정한 값을 launch 기본값에 반영합니다. 값을 줄일수록 로봇 옆쪽 가까운 구역이 안 보이게 됩니다.

## 확인 (PC)

```bash
ros2 topic hz /lidar/merged          # 약 10Hz
rviz2 -d <lidar_processor 경로>/rviz/lidar_processor.rviz
```

`rviz/lidar_processor.rviz`: Fixed Frame `base_link`, LaserScan 은 Best Effort 구독, Publish Point 도구 포함.

방향 확인 (`scripts/check_dir.py`, PC 에서 실행): 각 LiDAR 에서 가장 가까운 점의 방향을 로봇 기준 각도로 출력합니다. 정면 물체는 두 LiDAR 모두 약 0°, 왼쪽 물체는 약 +90° 가 나와야 합니다.

```bash
python3 scripts/check_dir.py
```

## 설정값과 근거

| 항목 | 값 | 근거 |
| --- | --- | --- |
| LiDAR 위치 | 전방 (0.1437, −0.0113), 후방 (−0.1437, 0.0113) | 퍽 보정 결과 (LiDAR 간 0.288 m, 자로 잰 28 cm 와 8 mm 차이). CAD 값(±0.163)에서 교체 |
| LiDAR 방향 | 전방 180°, 후방 0.97° | 두 LiDAR 모두 180° 돌려 장착됨(원위치가 어려워 그대로 사용). 같은 물체를 두 LiDAR 가 같은 좌표로 보고, 로봇 오른쪽 사람이 −y 에 찍히는 것 확인 |
| 로봇 중앙 | 두 LiDAR 의 가운데 = `base_link` | CAD 몸체(330 mm) 중앙과 일치 |
| 스캔 높이 | z=0.122 | 미실측. 2D 결과에는 영향 없음 |
| LiDAR 포트 | `1-1.2.1`=front, `1-1.2.3`=rear | PA18 Pi 실측. `udev/99-craft-lidar.rules` |

참고: 두 LiDAR 중심 사이를 자로 재면 28 cm, CAD 에서는 약 32.6 cm 입니다. 퍽 보정 결과(0.288 m)가 실측과 맞아 CAD 대신 보정 값을 씁니다.

## 두 LiDAR 좌표 일치화 (기준 물체 보정)

`lidar_processor` 는 TF(각 LiDAR 의 위치·방향)를 믿고 점을 `base_link` 로 옮깁니다. TF 가 실제 장착과 다르면 같은 물체가 두 LiDAR 에서 다른 위치로 찍혀 어긋난 채로 합쳐집니다. 그래서 두 LiDAR 가 **함께 보는 물체를 데이터로 맞춰** TF 를 보정합니다. 로봇을 세워 둔 상태에서 한 번만 계산해 launch 의 TF 값으로 고정하므로 실행 중 연산은 늘지 않습니다.

### 검토한 방법

| 방법 | 방식 | 판단 |
| --- | --- | --- |
| ① 수동 맞춤 | RViz 에서 두 스캔을 겹쳐 보며 x, yaw 를 손으로 조정 | 쉽지만 사람 눈에 의존, 근거를 숫자로 남기기 어려움 |
| **② 기준 물체(퍽)** | 두 LiDAR 가 함께 보는 곳에 퍽을 세우고 중심을 맞춤 | **최종 채택** |
| ③ 직선(벽) 특징 | 벽을 직선으로 뽑아 각도·거리 차이로 계산 | 평평한 벽만으로는 앞뒤 위치가 안 정해짐 (④와 같은 문제) |
| ④ ICP | 두 점 묶음이 가장 잘 겹치도록 회전·이동을 반복 계산 | 처음 채택 → 실기에서 실패 (아래) |
| ⑤ 주행 기반 | 움직이며 각 LiDAR 궤적·SLAM 지도를 비교 | 오도메트리 필요, 가장 복잡 |

### ICP 를 먼저 시도했다가 바꾼 이유

처음에는 ICP 를 골랐습니다(원본 360° 스캔 전체를 자동으로 맞추고, 보정 전후 오차를 숫자로 남길 수 있어서). 실기 결과는 다음과 같았습니다.

| 시도 | 조건 | 5cm 이내 대응 | 보정 후 RMS | LiDAR 간 거리 |
| --- | --- | --- | --- | --- |
| 1 | 몸체 점 포함 (range_min 0.12) | 38→40% | 20.8 mm | 0.295 m |
| 2 | 몸체 점 제외 (range_min 0.35) | 12% | 18.2 mm | 0.325 m (거의 안 움직임) |
| 3, 4 | 몸체 제외 + 양옆에 로봇보다 긴 판 | 12%, 7% | 19.1, 23.2 mm | **0.293 m, 0.318 m** (반복마다 2.5 cm 차이) |

- 1의 대응은 대부분 **로봇 몸체 점**이었습니다(몸체를 빼자 12% 로 떨어짐).
- 앞뒤 끝에 달린 두 LiDAR 가 함께 보는 곳은 **로봇 양옆뿐**이고, 거기 있는 것은 평평한 벽입니다. 평평한 벽은 벽을 따라 앞뒤로 밀어도 모양이 같아서 **앞뒤 위치(x, 곧 LiDAR 간 거리)를 정할 단서가 없습니다.** 그래서 같은 자리에서 반복해도 결과가 흔들렸습니다.
- 퍽처럼 작은 물체는 점처럼 위치가 하나로 정해지므로 이 문제가 없습니다.

### 보정 방법 (퍽)

1. 로봇을 세우고, 로봇 중앙에서 **0.8 m 안에는 퍽만** 둡니다 (다른 물건·벽은 0.8 m 밖으로).
2. 퍽 **3개**를 로봇 양옆 약 0.5 m 거리에, 서로 다른 앞뒤 위치로 세웁니다 (예: 왼쪽 앞, 왼쪽 뒤, 오른쪽 가운데). 두 LiDAR 가 모두 볼 수 있어야 합니다.
3. 퍽 반지름을 재서 넣고 실행합니다 (Pi, `lidar_processor.launch.py` 실행 중).

   ```bash
   ros2 run lidar_processor lidar_calibrator --ros-args -p puck_radius:=0.035
   ```

4. 출력 확인 후 `launch/lidar_processor.launch.py` 기본값에 반영하고 다시 빌드합니다.

   ```text
   두 LiDAR 가 함께 찾은 퍽: 3개
     퍽 1 잔차: 2.9 mm ...
   보정 후: 같은 퍽 위치 차이 RMS  2.9 mm
   LiDAR 중심 간 거리: 0.282 m (기존 TF 0.326 m)
   front_x:=0.1408 front_y:=0.0034 front_yaw_deg:=180.00 rear_x:=-0.1408 rear_y:=-0.0034 rear_yaw_deg:=2.26
   ```

   (위 숫자는 합성 데이터 시험 예시. 실제 거리 0.280 m 를 0.282 m, 각도 오차 0.25° 로 찾음)

판단 기준: 퍽 3개 이상, 보정 후 잔차 각 10 mm 이하, 2~3회 반복 시 LiDAR 간 거리 차이 1 cm 이내.

잘못 짝지어지거나 일부가 가려진 퍽은 자동으로 뺍니다: 가장 큰 잔차가 `outlier_mm`(15) 보다 크면, 퍽을 하나씩 빼 보며 남은 퍽이 가장 잘 맞는 경우를 골라 그 퍽을 제외하고 다시 계산합니다(최소 3쌍). 그래서 **퍽은 4개 이상** 두는 것이 좋습니다.

실기 1회차(퍽 4개, 제외 기능 추가 전): 잔차 47 / 44 / 4 / 14 mm 로 한 퍽(로봇 왼쪽 뒤, 전방 LiDAR 에서 몸체 옆을 스치듯 보이는 위치)이 크게 어긋났습니다. 그 퍽을 빼고 계산하면 잔차 6 / 9 / 4 mm, LiDAR 간 거리 0.290 m 로, 자로 잰 28 cm 에 가까워집니다.

동작: 원본 스캔을 `samples`(30)개씩 모아 1 cm 격자 평균 → 현재 TF 로 로봇 중앙 기준 `roi_min`(0.25)~`roi_max`(0.8) m 고리 안의 점만 남김 → 각도 순으로 3 cm 이내 이웃끼리 덩어리 → 퍽 크기 이하만 퍽 후보 → LiDAR 는 퍽 앞면 반원만 보므로 점 평균을 LiDAR 반대 방향으로 (2/π)·반지름 만큼 옮겨 중심으로 사용 → 두 LiDAR 의 같은 퍽끼리 짝지어 최소제곱으로 후방 위치·각도 계산 → 두 LiDAR 가운데를 `base_link` 로 다시 잡아 출력.

### 보정 결과 (2026-10-16, PA18)

| 실행 | 사용한 퍽 | 퍽별 잔차 | LiDAR 간 거리 | 후방 yaw | 판정 |
| --- | --- | --- | --- | --- | --- |
| 채택 | 4개 (1개 자동 제외) | 3.7 / 4.0 / 8.2 / 2.6 mm (RMS 5.1) | 0.288 m | 0.97° | 통과 |
| 그 외 | 2~3개 | 10~38 mm | 0.27~0.315 m | 0~3.5° | 잔차 초과 |

반영 후 확인: 같은 퍽을 두 LiDAR 로 본 위치 차이 RMS **15.3 mm** (CAD 값일 때 47~82 mm). 다시 계산해도 14.0 mm 로 거의 안 줄어 현재 값이 최적에 가깝습니다(재계산 0.295 m, 1.13°).

남은 약 15 mm 는 센서 한계 수준입니다. ROBOTIS 공식 LDS-03 사양의 거리 정확도는 ±10 mm(0.05~1 m), ±20 mm(1 m 이상), 각도 분해능 0.9° 입니다. 두 LiDAR 오차가 겹치면 약 √2 × 10 ≈ 14 mm 입니다. ([LDS-03 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_03/))

결과 영향: 각도 오차 약 1° 기준, 두 LiDAR 가 함께 보는 로봇 양옆에서 0.5 m 거리 약 0.9 cm, 1 m 약 1.7 cm 어긋날 수 있습니다.

주의: 퍽 받침대가 검은 기둥이라 스캔 높이에서 기둥이 잡혀 퍽이 실행마다 조각나거나 붙어 잡혔을 가능성이 있습니다. 더 정밀하게 하려면 밝은색의 넓은 원통을 기준 물체로 쓰세요.

## 동작 특성과 한계

- **발행 주기 약 10Hz.** 전방 스캔이 들어올 때마다 가장 최근 후방 스캔과 합쳐 발행합니다. LDS-03 회전 속도(약 10Hz)를 그대로 따라갑니다.
- **전/후방 허용 시간차 0.2 s** (`max_age`). 이보다 오래된 후방 스캔은 버리고 전방만 발행합니다(경고 로그). 로봇이 빠르게 움직이면 앞뒤 점이 조금 어긋날 수 있습니다.
- **가까운 거리 인식 한계** (이전 박스 필터 방식에서 측정, 그대로 두기로 함)
  - 좌우: 몸체 측면 바깥 약 2 cm 이내
  - 전후: LiDAR 하우징 바깥 약 6 cm 이내 (드라이버 최소 측정거리: LiDAR 중심에서 0.10 m)
  - FOV 방식으로 바뀌면서 좌우 한계는 FOV 값에 따라 달라지므로 다시 측정해야 합니다.
- 기체가 완성되면 LiDAR 위치·FOV 를 다시 맞춰야 합니다.

## 문제 해결

| 증상 | 원인 | 해결 |
| --- | --- | --- |
| Ctrl+C 로 launch 가 안 꺼짐 | 드라이버가 시리얼 대기 중 종료 신호를 못 받음(추정) | 새 터미널에서 `pkill -f multi_coin_d4_node; pkill -f lidar_processor; pkill -f static_transform_publisher` |
| RViz 에 원본 스캔이 안 보임, Pi 로그에 `incompatible QoS` | RViz 가 Reliable 로 구독 | Reliability Policy 를 Best Effort 로 (또는 `rviz/lidar_processor.rviz` 사용) |
| PC 에서 토픽이 안 보임 | `ROS_DOMAIN_ID` 불일치 또는 오래된 데몬 정보 | 양쪽 `echo $ROS_DOMAIN_ID` 확인, PC 에서 `ros2 daemon stop` |
| 점이 튀고 `scan_time` 이 2 s 처럼 이상함 | 드라이버가 두 개 실행되어 같은 포트를 나눠 읽음 | `pkill -f multi_coin_d4_node` 후 launch 하나만 실행 |
| `/dev/lidar_*` 가 없음 | udev KERNELS 값이 실제와 다름 | `find_lidar_ports.sh` 로 확인 후 규칙 수정 |
| 같은 물체가 두 LiDAR 에서 다른 위치에 찍힘 | yaw 또는 위치값이 실제 장착과 다름 | `check_dir.py` 로 방향 확인 후 launch 인자 수정 |

## 팀 노드와의 관계

팀 패키지 `craft_sensors` 의 `lidar_processor` 와 이름·토픽이 같습니다. 둘 중 하나만 실행해야 합니다.
