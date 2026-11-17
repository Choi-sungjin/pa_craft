# lidar_floor

LiDAR 바닥 제거(`scan_merger` `floor_roi`) 값을 bag에서 다시 뽑는 도구. 2026-11-01 밤 작업(HANDOFF "LiDAR 바닥 보정과
비대칭 ROI", `docs/worklog/2026-11-01.md`)의 분석 스크립트를 복구해서 정리했다. 필터 코드는
`src/lidar_processor/src/scan_merger.cpp`, 값은 `src/craft_perception/config/perception.yaml` `lidar.floor_roi`.

## 필터가 하는 일

LiDAR별(앞/뒤)·좌우별(base_link 왼/오른쪽)로, LiDAR의 바깥쪽 축에서 잰 각도로 판단한다.

- `*_clear_deg` 안: 전부 남김.
- `*_clear_deg` ~ 90°: `step_deg`(5°) 구간마다 LiDAR에서 `*_max_range_m[구간]` 안의 점만 남김(그보다 멀면 빔이 바닥에 닿은 것).
- `/lidar/merged` 자체에 걸리므로 SLAM·위치 추정·장애물 전부 영향. 먼 옆벽도 같이 잘린다(구석 위치 찾기 단서 감소, 11-03).

## RViz로 실시간 보기 (`roi_view.py`)

raw 스캔에 scan_merger와 같은 컷(화각, 몸체, perception.yaml `floor_roi`)을 걸어 남는 점 / 바닥이라 지워진 점을 따로 쏜다.
`lidar_processor.launch.py`의 `/lidar/merged`에는 ROI가 없으므로 이걸로 본다. perception.yaml을 고치면 노드를 재시작.

```bash
# Jetson: LiDAR launch 켠 상태에서
_craft_peer; cd ~/craft && source install/setup.bash && /usr/bin/python3 tools/lidar_floor/roi_view.py
# 노트북
scp craft@<jetson>:~/craft/tools/lidar_floor/roi_view.rviz ~/roi_view.rviz
rviz2 -d ~/roi_view.rviz
```

- 초록 `/lidar/roi/kept`: merger가 남기는 점, 빨강 `/lidar/roi/floor_removed`: ROI가 지운 점(1초 잔상), 노란 선 `/lidar/roi/region`: 남기는 영역(3 m까지).

## 정지 방식 (넓은 빈 곳에 가만히 세워 두기, 11-01 `floor_cal_0034` 방식)

주변에 아무것도 없으면 LiDAR에 찍히는 점은 전부 바닥이다. 그걸 그대로 각도별로 잘라 ROI를 만든다. 맵·위치 추정 필요 없음.

```bash
# Jetson 터미널 1: LiDAR만
cd ~/craft && source install/setup.bash && ros2 launch lidar_processor lidar_processor.launch.py
# 터미널 2: 20초 기록 (-d 는 20초마다 파일 쪼개기라 안 멈춤 → timeout 사용) (사람은 기록 시작 전에 2 m 밖으로)
cd ~/craft && source install/setup.bash
timeout -s INT 20 ros2 bag record -o ~/bags/floor_static_$(date +%H%M) --topics /lidar/front/scan /lidar/rear/scan /tf_static
# 로봇을 다른 자리·방향(90°씩)으로 옮겨 가며 몇 번 더 기록하면 좋다

cd ~/craft/tools/lidar_floor
/usr/bin/python3 floor_static.py floor_static_1501 floor_static_1503 floor_static_1505
```

- LiDAR에서 `--max-floor-m`(2.0 m) 안에 벽·박스·사람·케이블이 없어야 한다. 그보다 먼 점은 방 벽으로 보고 버린다.
  `floor_cal_0034`는 3~4 m에 방 벽이 있어서 4 m로 돌리면 벽을 바닥으로 센다.
- 출력: LiDAR·좌우별 5° 구간 바닥 점 수와 거리, **지금 perception.yaml ROI가 그중 몇 점을 통과시키는지**, 새 표(yaml 블록).
- 규칙: 바닥 점 `--min-pts`(20)개 이상인 첫 구간 − 5° = `clear_deg`, 구간 최대 거리 = 1st percentile − 0.15 m
  (점이 몇 개뿐이면 가장 가까운 점 − 0.15), 바닥 없는 구간 8.0, 바닥이 한 점도 없는 쪽은 clear 90 / [8.0](아무것도 안 자름).
- `floor_cal_0034`(평평한 바닥) 결과: 바깥쪽 절반에서 바닥은 **뒤 LiDAR 왼쪽 80~90°, 0.95~1.2 m에만** 찍힌다
  (스캔당 5.3점). 지금 ROI는 이걸 전부 지운다. 11-01에 경기장 bag으로 넘어간 이유는 평평한 바닥에서는 바깥쪽에 바닥이
  거의 안 찍히고, 경기장 바닥 요철·주행 중 차체 기울기에서 찍히는 바닥이 문제였기 때문이다. 정지 방식은 **실제로 달릴 바닥에서** 찍어야 의미가 있다.

## 순서 (주행 bag 방식)

```bash
source ~/craft/install/setup.bash
cd ~/craft/tools/lidar_floor

# 0) bag 기록: 반드시 raw 스캔 포함 (/lidar/merged는 이미 ROI로 잘려 있음). 위치 추정이 되는 주행이어야 함.
ros2 bag record -o ~/bags/floor_$(date +%H%M) --topics /lidar/front/scan /lidar/rear/scan /lidar/merged /tf /tf_static

# 1) (선택) 평평한 바닥에서 빔 모델 확인: 원뿔 + 장착 기울기
/usr/bin/python3 cone_fit.py ~/bags/<평평한바닥bag> out/cone_fit.png

# 2) 바닥이 어느 각도부터 찍히는지 보기 (경기장 bag)
/usr/bin/python3 floor_by_angle.py <bag> --map ~/maps/<그 경기장 맵>.yaml            # raw = ROI 전
/usr/bin/python3 floor_by_angle.py <bag> --map ~/maps/<맵>.yaml --source merged     # 지금 ROI 후 남은 바닥

# 3) ROI 표 뽑기 + 교차 검증 (그룹끼리 서로 안 본 bag으로 검증). 마지막에 perception.yaml 블록을 찍는다.
/usr/bin/python3 derive_roi.py --map ~/maps/<맵>.yaml --bags <bagA>,<bagB> <bagC>
```

- `--map`은 **bag을 찍은 경기장의 맵**이어야 한다. 맵 벽에서 10 cm 넘게 떨어진 점을 바닥으로 세고, 4 cm 안을 벽으로 센다.
  맵에 없는 박스·사람도 바닥으로 세므로 비어 있는 경기장에서 찍는 게 좋다.
- 추출 결과는 `out/rows_<bag>_<source>.npy`에 캐시된다. 맵을 바꾸면 지운다.
- 규칙(옵션 기본값): 45°부터 5° 구간 중 바닥 비율이 0~45° 평균의 3배 이상인 첫 구간 = 바닥 시작각.
  `clear_deg` = 시작각 − 5°(`--clear-back`), 구간 최대 거리 = 그 구간 바닥 점 거리 1st percentile − 0.15 m(`--margin`),
  바닥 점 20개 미만 구간은 8.0(= range_max, 전부 남김). 11-01 원본은 0(전부 버림)이었는데 그때 표에서는 안 걸렸다.

## 2026-11-01 결과 (재현 확인 2026-11-09)

맵 `arena3_clean`, bag `pocket_2248`, `pocket_2309`(이 둘은 `/lidar/merged`만 있음 → `--source merged`), `fov180_2359`.

```bash
/usr/bin/python3 derive_roi.py --map ~/maps/arena3_clean.yaml --bags pocket_2248,pocket_2309 fov180_2359 --source merged
```

출력 표가 지금 perception.yaml `floor_roi`와 한 자리까지 같다.

| | clear | 구간별 최대 거리 m |
|---|---|---|
| 앞 왼 | 70° | 0.48 0.83 0.88 0.62 |
| 앞 오 | 60° | 1.32 1.32 1.35 0.76 0.68 0.48 |
| 뒤 왼 | 65° | 0.70 0.71 0.56 0.48 0.45 |
| 뒤 오 | 50° | 0.73 0.60 0.55 0.58 0.57 0.50 0.52 0.46 |

세 bag 전체에 적용: 바닥 점 1000개당 27.5 → 5.0, 55~90°에서 64.0 → 3.1, 맵 벽 점 92.2 % 유지(55~90° 79.2 %).
교차 검증(안 본 bag): pocket 표 → fov180 바닥 48.3 → 14.9, 벽 76.9 %; fov180 표 → pocket 바닥 14.1 → 2.9, 벽 96.4 %.

평평한 바닥 `floor_cal_0034`(빔 높이 앞 1.2 cm, 뒤 1.1 cm 실측): 평면(pitch + roll) 모델 실패(rms 33~113 cm),
원뿔 0.6° + 바깥쪽 들림 1.65° + roll 0.1~0.2° 모델 rms 0.9~1.4 cm. 다른 각도로 외삽은 부정확해서 ROI는 경기장 bag 실측으로 정했다.

## 파일

- `floor_common.py`: bag 읽기, TF, LiDAR 장착(앞 x 0.155, 뒤 x −0.1295, 둘 다 뒤집힘), 맵 벽 거리.
- `floor_static.py`: 정지 bag(주변에 아무것도 없음) → ROI 표.
- `derive_roi.py`: 주행 bag + 맵 → ROI 표 + 교차 검증 + yaml 블록.
- `floor_by_angle.py`: 각도 구간별 바닥(벽 밖) 점 비율·거리.
- `cone_fit.py`: 평평한 바닥 bag → 원뿔 + 기울기 모델, 그림.
- `legacy/`: 11-01 원본 그대로(경로·맵 하드코딩, `$S` 캐시 디렉터리 인자). `floorfit.py`·`floor2.py`·`floor3.py`(평면 모델 시도),
  `floorplot.py`(시간 구간별 스캔 그림), `fovcheck.py`(몸체 중심 vs LiDAR 중심 화각 컷), `floorcheck.py`(경로 근처 점이
  장애물 필터에서 어떻게 처리됐는지), `arena_floor.py`·`cv.py`(위 두 도구의 원본).
