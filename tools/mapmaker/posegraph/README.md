# posegraph_tool (mapmaker/posegraph)

slam_toolbox pose graph(`<base>.posegraph` + `<base>.data`)를 slam_toolbox 자신의 Karto 라이브러리로 읽고 고치는 도구.
삭제 순서는 `lifelong_slam_toolbox::removeFromSlamGraph`와 같다(graph → sensor manager → dataset).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4   # 약 20초
./build/posegraph_tool info   ~/maps/arena3 > nodes.csv          # scan별 id, time, pose, 거리 통계
./build/posegraph_tool trim   ~/maps/arena3 ~/maps/arena3_clean 1176   # id >= 1176 삭제, 새 이름으로 저장
./build/posegraph_tool render ~/maps/arena3_clean ~/maps/arena3_clean  # 남은 scan -> pgm/yaml
./build/posegraph_tool transform ~/maps/arena5_legend ~/maps/arena5_legend_arena 2.10668 3.18076 175.0289
                                   # 맵 전체 강체 이동 p' = R(theta) p + (x, y), theta [deg]
./build/posegraph_tool optimize  ~/maps/arena5_legend ~/maps/arena5_legend_opt   # slam_toolbox Ceres로 한 번 최적화
```

- `optimize`: 저장된 pose graph가 제약의 최적점이 아닐 수 있다. slam_toolbox는 위치추정 모드에서 첫 loop closure 때 그래프 전체를
  다시 최적화하는데, arena5_legend는 그때 첫 노드 근처를 중심으로 2.26° 돌아서(2026-11-08) A*용 pgm과 최대 20 cm 어긋났다
  (run_corner "slam_toolbox did not settle", 박스 충돌). **새 SLAM 저장본은 `optimize` → `render <out> <out> 0.01` →
  `make_arena_map.py <out>` 순서로 만든다.** 결과를 다시 optimize해서 0 m / 0°면 된 것. slam_toolbox와 같은 플러그인·기본값
  (slam.yaml에 ceres 설정 없음)이라, 실행 중 맵과 비교해 96 % 일치, 남은 회전 0.23°.
- arena5_legend_opt_arena1cm(2026-11-08 22:23, corner_demo 기본): 아래 맵의 1 cm 판. `_1cm` 렌더에 box_detector(min_coverage 0.30)로
  박스 9개 완성, posegraph/data는 arena5_legend_opt_arena 것을 심링크. A* 5.8 m 42 ms(2 cm 19 ms), 접근 계획 234 ms, 막힘 확인·local path
  20 ms(2 cm 5 ms)로 여유 있음.
- arena5_legend_opt_arena(2026-11-08): 위 순서로 만든 맵. 정합 (2.16168, 3.19290, 172.9636°), 박스 완성은
  min_coverage 0.30(0.20이면 S1이 실제 박스 서쪽 145 mm의 0.26짜리 흔적을 잡아 박스가 커짐).
- 경기장 좌표 맵 전체 과정은 `../pipeline/make_arena_map.py`가 이 도구를 불러서 한다(`../README.md`).
- 원본은 덮어쓰지 않는다(출력 파일이 있으면 거부).
- 매핑 끝에 로봇을 들어 올린 구간 찾기: `info`의 p90_range가 2 m 안팎에서 6~7 m로 뛰는 곳, 그 직전 손이 닿아
  거리가 0.6 m로 떨어지는 곳(arena3: 1176)부터 끝까지 `trim`.
- `render`는 slam_toolbox와 같은 방식(min_pass_through 2, occupancy_threshold 0.1)이다. arena3 원본에서 slam_toolbox가
  저장한 pgm과 픽셀 99.99% 일치.
- Karto의 Mapper/Dataset 소멸자가 이 데이터에서 예외를 던져서, 결과를 다 쓴 뒤 `std::_Exit`로 끝낸다.
- `transform`은 scan의 odometric/corrected pose와 link의 절대 pose를 같이 옮긴다. link의 pose 차이·공분산은 첫 pose
  기준 좌표라 그대로 둔다. 위치추정(slam_toolbox)과 A*가 같은 경기장 좌표(maze-dice: 왼쪽 아래 외벽 기둥 중심 원점,
  +X 오른쪽, +Y 위쪽)를 쓰게 하려고 만들었다.
- arena5_legend → arena5_legend_arena: maze-dice 벽 그림에 chamfer 정합(평균 벽 거리 37 mm, 실제 미로와 도면 차이 포함)
  해서 얻은 (2.10668 m, 3.18076 m, 175.0289°). 변환 후 다시 정합하면 0.01°, 0.1 mm 이내.
  A*/전역 위치추정용 pgm은 0.02 m(`render ... 0.02`, max_expansions 기준 해상도), 1 cm 렌더는 `_1cm`으로 따로 둔다.
