# posegraph_tool

slam_toolbox pose graph(`<base>.posegraph` + `<base>.data`)를 slam_toolbox 자신의 Karto 라이브러리로 읽고 고치는 도구.
삭제 순서는 `lifelong_slam_toolbox::removeFromSlamGraph`와 같다(graph → sensor manager → dataset).

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4   # 약 20초
./build/posegraph_tool info   ~/maps/arena3 > nodes.csv          # scan별 id, time, pose, 거리 통계
./build/posegraph_tool trim   ~/maps/arena3 ~/maps/arena3_clean 1176   # id >= 1176 삭제, 새 이름으로 저장
./build/posegraph_tool render ~/maps/arena3_clean ~/maps/arena3_clean  # 남은 scan -> pgm/yaml
```

- 원본은 덮어쓰지 않는다(출력 파일이 있으면 거부).
- 매핑 끝에 로봇을 들어 올린 구간 찾기: `info`의 p90_range가 2 m 안팎에서 6~7 m로 뛰는 곳, 그 직전 손이 닿아
  거리가 0.6 m로 떨어지는 곳(arena3: 1176)부터 끝까지 `trim`.
- `render`는 slam_toolbox와 같은 방식(min_pass_through 2, occupancy_threshold 0.1)이다. arena3 원본에서 slam_toolbox가
  저장한 pgm과 픽셀 99.99% 일치.
- Karto의 Mapper/Dataset 소멸자가 이 데이터에서 예외를 던져서, 결과를 다 쓴 뒤 `std::_Exit`로 끝낸다.
