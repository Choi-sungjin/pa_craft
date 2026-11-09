# mapmaker

SLAM으로 찍은 맵(slam_toolbox save)을 경기장 좌표(발제·maze-dice: 왼쪽 아래 외벽 기둥 중심 원점, +X 오른쪽, +Y 위쪽, m)로
옮기고 A*/위치추정용 맵을 만드는 도구 모음. 초레지(`~/maps/arena5_legend`)를 다시 찍으면 `pipeline` 하나로 다시 만든다.

| 폴더 | 기능 |
| --- | --- |
| `pipeline/make_arena_map.py` | 전체 순서 한 번에: 정합 → pose graph 변환 → 렌더 → 박스 완성 → 검증·그림 |
| `register/register_maze.py` | 맵 ↔ maze-dice 벽 그림 정합 (x, y, θ), 경기장 좌표 맵의 잔차 확인(`--check`) |
| `posegraph/` | slam_toolbox pose graph 도구(C++, Karto): `info`, `trim`, `render`, `transform` |
| `figure/compare_figure.py` | maze-dice 그림과 맵을 같은 mm 축으로 나란히 |
| `reference/` | `maze-dice.png`(발제 그림, 정합 기준), `maze-dice-L2box.png`(L2 받침대를 205 mm 박스로 바꾼 그림) |

박스 검출·완성은 A* 쪽 모듈 `src/craft_planning/src/box_detector.py`를 그대로 부른다.

```bash
cd ~/craft/tools/mapmaker/posegraph && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4  # 처음 한 번
/usr/bin/python3 ~/craft/tools/mapmaker/pipeline/make_arena_map.py ~/maps/arena6      # -> ~/maps/arena6_arena.*
ros2 launch craft_bringup corner_demo.launch.py map:=$HOME/maps/arena6_arena
```

결과(`<base>_arena`):

| 파일 | 내용 |
| --- | --- |
| `.posegraph/.data` | 경기장 좌표로 옮긴 pose graph (slam_toolbox 위치추정) |
| `_raw.pgm/.yaml` | 0.02 m 렌더 (A* 격자 = 맵 해상도, max_expansions 500k 기준) |
| `.pgm/.yaml` | `_raw` + 박스 9개를 205 mm 정사각형 테두리로 완성 (A*·전역 위치추정이 쓰는 맵) |
| `_1cm.pgm/.yaml` | 0.01 m 렌더 (보기·검증용) |
| `_boxes.png`, `_vs_maze-dice.png` | 박스 검출 결과, maze-dice와 나란히 |

- 시스템 python(`/usr/bin/python3`)으로 돌린다: 저장소 venv의 numpy 2와 시스템 scipy가 안 맞는다.
- 어떤 출력도 덮어쓰지 않는다(있으면 거부). 원본 `<base>.*`는 건드리지 않는다.
- 정합 비용 = 맵 벽 픽셀에서 그림 벽까지 평균 거리(150 mm에서 자름). 초레지: 37 mm(실제 미로와 도면 차이 포함),
  최적점 근처가 평평해서 정합 정확도는 약 1 cm / 0.1°.
- 정합이 이상하면 `--transform X Y THETA_DEG`로 직접 준다(초레지: 2.10668 3.18076 175.0289).
