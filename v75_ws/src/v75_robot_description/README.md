# v75_robot_description

V75 로봇의 URDF, 메시, RViz 표시용 launch가 들어 있는 ROS2 패키지.

## 실행

```bash
cp -r v75_robot_description ~/ros2_ws/src/
cd ~/ros2_ws && colcon build --packages-select v75_robot_description
source install/setup.bash
ros2 launch v75_robot_description display.launch.py          # 관절 슬라이더 창 포함
ros2 launch v75_robot_description display.launch.py gui:=false
```

필요한 패키지가 없으면 설치:

```bash
sudo apt install ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-xacro
```

## 원본에서 바꾼 점

- 패키지 이름을 URDF의 `package://v75_robot_description/...` 경로에 맞춤 (받은 launch/rviz는 v78 기준이었음)
- `meshes/`는 원본(344 MB, 720만 면)을 형상 유지 수준으로 줄인 것(약 30만 면, 내장 그래픽에서도 가볍게 돌도록). 원본으로 바꾸려면 같은 이름으로 덮어쓰기
- `slide_link_gray.stl`이 meshes에는 있었지만 URDF에 없어서 `slide_link`에 회색 visual로 추가
- `tool_tcp` 링크 추가: 두 집게 패드 사이 중심. `tool_roll_link` 기준 (0, 102.4, 0) mm (비틀기 축 위)
- `shoulder_link_*.stl` 3개를 어깨 축(x) 기준 +15° 회전: 메시(v78 추정)와 URDF(v75)의 어깨 저장 자세 차이 보정. 회전 후 팔꿈치 축이 어깨 링크 끝 원형 허브 중심과 일치(링크 길이 188 mm는 URDF와 같음). 원본 메시로 바꿀 때는 이 3개도 같은 회전 필요
- 그리퍼 메시(`tool_roll_link_*` 4개, `jaw_*_link_*` 4개)를 tool_roll 원점 기준 x축 +15° 회전: 회전 전에는 두 집게가 URDF 집게 관절(±10 mm 대칭)과 맞지 않고 잡는 점이 축에서 28 mm 벗어나 있었음. +15°에서 두 집게와 그리퍼 몸체가 비틀기 축 기준 대칭이 되고 잡는 점이 축 위에 옴 (어깨와 같은 15° 차이)
- `wrist_pitch_link_*.stl` 2개를 손목 축(x) 기준 +15° 회전: 손목 안의 비틀기 서보(어깨 서보와 같은 모델)를 ICP로 정합해 보니 서보 축이 URDF 비틀기 축보다 15° 기울어 있었음. 회전 후 서보 축이 URDF 비틀기 축과 정확히 일치하고 손목-그리퍼 사이 꺾임이 없어짐
- 색상: RViz는 한 링크의 여러 visual에 첫 번째 색만 칠하므로 visual마다 고정 링크(`<링크>_vis_<색>`)로 분리. 밝은 팔레트로 변경, 그리퍼 몸체 보라 / 위 집게 분홍 / 아래 집게 하늘색 / 패드 흰색 / 노브 빨강 (비틀기 회전이 보이도록)
- **노브 받침대 분리**: 원래 base_link 메시 앞쪽에 붙어 있던 받침대(기둥·판·다리 3덩어리)와 `selector_link`(노브)를 로봇에서 떼어 `urdf/knob_station.urdf`(world에 고정)로 옮김. 메시: `knob_station_black.stl`, `knob_link_red.stl`. 로봇 URDF에는 이제 노브가 없음
- `camera_optical_frame` 추가: 그리퍼 옆 노란 상자(카메라) 렌즈 중심, `tool_roll_link` 기준 (−65, 55.8, 0) mm, z축 = 보는 방향(그리퍼와 같은 방향)
- RViz: TF 표시를 켜면(Displays → TF 체크) `tool_tcp` 등 좌표계 이름이 보임

## 확인된 문제

- 모든 관절의 `effort`, `velocity`가 0, `collision` 형상 없음 (표시용으로는 문제없음, MoveIt 전에 수정 필요)
