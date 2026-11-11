# Corner demo: 어디에 놓아도 오른쪽 아래 구석으로

저장한 SLAM 맵(`~/maps/arena3_clean`) 안 아무 곳에 로봇을 놓으면, 스스로 위치를 찾고 맵의 오른쪽 아래
구석까지 경로를 짜서 주행한다.

| 단계 | 하는 일 | 코드 |
|---|---|---|
| localization | slam_toolbox localization mode가 `arena3_clean.posegraph`를 불러옴 | `craft.launch.py slam_map:=` |
| global localization | 정지 상태 `/lidar/merged` 1초를 맵 전체(벽에서 0.18 m 이상, 6 cm × 3°)에 맞춤 → refine → `/initialpose` | `map_tools.ScanMatcher` |
| 목표 | 시작점에서 갈 수 있는(벽 0.24 m) 칸 중 오른쪽 아래 끝, 벽에서 0.30 m | `map_tools.corner_goal` |
| 주행 | planning `START` → `/planning/target_base_pose` → Pure Pursuit | planning_master, control_master |

## 실행 (Jetson)

```bash
ros2 launch craft_bringup corner_demo.launch.py           # 터미널 1 (map:= 로 다른 맵)
/usr/bin/python3 ~/craft/demo/corner/run_corner.py --dry-run   # 터미널 2: 위치 찾기 + 목표 출력만
/usr/bin/python3 ~/craft/demo/corner/run_corner.py             # 실제 주행 (Ctrl+C = STOP)
```

- 로봇은 정지 상태로 둔다(위치 찾기 약 6초). 두 후보가 비슷하면(`AMBIGUOUS`) 조금 돌리거나 옮겨서 다시 실행.
- planning은 control(두 보드), odom, LiDAR 장애물, **팔 관절 상태**가 모두 최신이어야 Ready가 된다.
- `--corner tr|bl|tl` 로 다른 구석, `--skip-localize` 는 slam_toolbox 위치를 그대로 쓴다.

## 오프라인 확인 (하드웨어 없음)

`test_map_tools.py`: 맵에 가상 병합 scan(앞뒤 LiDAR 각 ±90°, 거리 잡음 1 cm)을 만들어 global localization 검증.
2026-11-01 arena3_clean: 6/6 성공, 오차 ≤ 0.5 cm / 0.2°, 1등 0.97~0.98 vs 2등 0.55~0.79.
`planned_paths.png`: planning 알고리즘(Python 구현, planning.yaml 파라미터)으로 시작점 여러 곳 → 구석 경로.
