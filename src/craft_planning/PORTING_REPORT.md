미션 Python 노드 → C++ 포팅 결과 (2026-11-16)

기본 미션은 odom_trail, box_marker, puck_marker, global_localize, puck_picker 및 자식 puck_execute를 C++로 실행한다. 원본 Python 파일과 설치 항목은 남겼다. 커밋·stash·reset·checkout·clean·펌웨어 업로드·로봇 구동·시리얼 접근은 하지 않았다. mission.yaml의 기존 drive_only: false를 확인했으며 설정을 바꾸지 않았다.

변경한 기존 파일과 작업 시작 전 백업:

| 파일 | 백업 |
| --- | --- |
| `src/craft_bringup/CMakeLists.txt` | `/home/craft/port_backup/src/craft_bringup/CMakeLists.txt` |
| `src/craft_bringup/package.xml` | `/home/craft/port_backup/src/craft_bringup/package.xml` |
| `src/craft_bringup/launch/craft.launch.py` | `/home/craft/port_backup/src/craft_bringup/launch/craft.launch.py` |
| `src/craft_planning/CMakeLists.txt` | `/home/craft/port_backup/src/craft_planning/CMakeLists.txt` |
| `src/craft_planning/package.xml` | `/home/craft/port_backup/src/craft_planning/package.xml` |

추가 파일 전체 목록. 이 중 포팅 중간본을 다시 수정한 파일에는 같은 상대 경로의 중간본 백업이 있다. 신규 파일 백업은 작업 전 원본이 아니라 포팅 중간 스냅샷이다.

| 추가 파일 | 중간본 백업 |
| --- | --- |
| `src/craft_bringup/src/odom_trail.cpp` | 신규 — 없음 |
| `src/craft_bringup/src/box_marker.cpp` | 신규 — 없음 |
| `src/craft_bringup/src/puck_marker.cpp` | 신규 — 없음 |
| `src/craft_bringup/src/global_localize.cpp` | 신규 — 없음 |
| `src/craft_bringup/src/map_tools.hpp` | 신규 — 없음 |
| `src/craft_bringup/test/map_tools_cli.cpp` | 신규 — 없음 |
| `src/craft_bringup/test/map_tools_parity.py` | 신규 — 없음 |
| `src/craft_planning/src/puck_picker.cpp` | `/home/craft/port_backup/src/craft_planning/src/puck_picker.cpp` |
| `src/craft_planning/src/puck_execute/camera.cpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/camera.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/constants.hpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/constants.hpp` |
| `src/craft_planning/src/puck_execute/executor.cpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/executor.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/geometry.cpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/geometry.cpp` |
| `src/craft_planning/src/puck_execute/geometry.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/legacy.cpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/legacy.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/main.cpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/pick_policy.cpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/pick_policy.cpp` |
| `src/craft_planning/src/puck_execute/pick_policy.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/plan.cpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/plan.cpp` |
| `src/craft_planning/src/puck_execute/plan.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/plan_generator.cpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/plan_generator.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/puck_observations.cpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/puck_observations.hpp` | 신규 — 없음 |
| `src/craft_planning/src/puck_execute/runtime.cpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/runtime.cpp` |
| `src/craft_planning/src/puck_execute/runtime.hpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/runtime.hpp` |
| `src/craft_planning/src/puck_execute/teach.cpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/teach.cpp` |
| `src/craft_planning/src/puck_execute/teach.hpp` | `/home/craft/port_backup/src/craft_planning/src/puck_execute/teach.hpp` |
| `src/craft_planning/test/puck_execute_math_cli.cpp` | `/home/craft/port_backup/src/craft_planning/test/puck_execute_math_cli.cpp` |
| `src/craft_planning/test/puck_execute_parity.py` | `/home/craft/port_backup/src/craft_planning/test/puck_execute_parity.py` |
| `src/craft_planning/test/puck_legacy_cli.cpp` | 신규 — 없음 |
| `src/craft_planning/test/puck_legacy_parity.py` | `/home/craft/port_backup/src/craft_planning/test/puck_legacy_parity.py` |
| `src/craft_planning/test/puck_camera_cli.cpp` | 신규 — 없음 |
| `src/craft_planning/test/puck_camera_parity.py` | 신규 — 없음 |
| `src/craft_planning/test/puck_cli_checks.py` | 신규 — 없음 |
| `src/craft_planning/third_party/nlohmann/LICENSE.MIT` | 신규 — 없음 |
| `src/craft_planning/third_party/nlohmann/README.port.md` | 신규 — 없음 |
| `src/craft_planning/third_party/nlohmann/json.hpp` | 신규 — 없음 |

`PORTING_REPORT.md`도 이번 작업에서 새로 추가했다. 임시 gripcal 검증에서 생성한 계획 백업은 `/home/craft/port_backup/test_generated/` 아래에 따로 보관했다. 실제 demo/puck_pick 계획 파일은 덮어쓰지 않았다.

실행·의존성:

- puck_picker 기본 executor_path는 ament 패키지 prefix/lib/craft_planning/puck_execute다. 명시적으로 .py 경로를 주면 기존 python3 -u 호환 실행을 유지한다.
- puck_execute는 runtime YAML/JSON을 읽는다. 기본 파일 위치는 원본과 같다. 오프라인 격리용 PUCK_PICK_DIR 및 CRAFT_REPO 경로 재정의를 지원하며, 보정·티칭 데이터를 실행 파일에 굽지 않는다.
- Eigen/yaml-cpp와 기존 Pinocchio의 URDFDOM 파서를 재사용했다. 별도 시스템 패키지는 설치하지 않았다. JSON 처리는 로봇의 기존 librealsense에 있는 nlohmann/json 3.12.0 헤더와 MIT 라이선스를 함께 넣었다.
- 기존 bigbro/gripcal/gripzero/held 검사에서 호출하던 grip_locate.py는 비ROS 카메라 유틸리티로 유지한다. 기본 미션 collect/find/lie는 이 유틸리티를 호출하지 않는다.
- ARM Release FMA가 지도 후보의 0.3 m NMS 경계를 바꾸는 것을 재현했다. global_localize와 map_tools_cli에 -ffp-contract=off를 적용해 Python 계산 순서와 맞췄다.

최종 검증:

- 전체 빌드: 11개 패키지 성공. 새 타깃의 -Wall -Wextra -Wpedantic 컴파일 경고 0개. GCC의 기존 ABI 안내 note와 posegraph_tool/realsense_check의 install target 없음 colcon 경고는 별개로 남는다.
- 최종 설치/빌드 산출물 CTest: 4/4 통과, 총 64.72초. 수학·파싱 198입력, 시나리오·기하 34건 + 재계획 YAML 전체 3건 + gripcal 1건, 카메라 실제 핸들러 19건, 잘못된 CLI 거절 6건 + 정상 dry-run 5건.
- 카메라 검증에는 실제 저장된 서 있는 퍽 및 누운 퍽 depth 스택을 grip_yaw --stack으로 재생해 Python/C++ 실제 보정 함수를 비교한 사례가 포함된다. 카메라 프로세스 종료 7, timeout, 실행 파일 없음도 확인했다.
- brief_arena1cm의 빌드된 map_tools_cli parity 통과. global score/second 0.9990/0.7769, near 0.9990/0.3494가 양쪽에서 일치했다. 테스트는 원래 pose 1e-6, score 1e-4 조건을 유지했다.
- ros2 launch craft_bringup craft.launch.py --show-args 파싱 성공. launch에 .py Node executable이 없음을 확인했다. 실제 launch는 실행하지 않았다.
- 독립 sol6.1 검토에서 SIGINT, 닫힌 stdin, 숫자 접미사, 누운 퍽 선택·LM 보정, 반올림, URDF 고정 관절 보존 등을 확인하고 발견 사항을 수정했다. 마지막 소스 검토에 미해결 항목은 없었다.

남은 TODO(port)와 동작 차이 전체:

| 위치 | 차이 |
| --- | --- |
| `src/craft_planning/src/puck_execute/camera.cpp:195` | 일부 카메라 진단 문자열이 원본보다 짧다. 선택·계산된 자세·중단 한계와 구조화된 per-pick 데이터는 유지한다. 콘솔 문구의 문자 단위 동일성은 보장하지 않는다. |
| `src/craft_planning/src/puck_execute/plan_generator.cpp:65` | 재계획 실패 시 plan.py stdout/stderr의 상세 진단을 그대로 재현하지 않는다. 실패 결과와 계획 계산은 이식했다. |

검증 한계: 실물 모터·카메라 대여·재도킹은 구동하지 않았다. 추가 C++ picker 모의 ROS 테스트는 샌드박스 UDP 소켓 거절(Operation not permitted)로 discovery부터 실패했으므로 통합 테스트 통과로 세지 않았다. 구형 --lying 경로의 현재 plan_lying.yaml은 Python과 C++ 모두 10 cm 위 자세에서 IK 실패하여 성공 경로 비교가 불가능했다. 미션용 --lie --cell 2|5 --pick과는 별도 경로다.

다시 검증하는 명령(하드웨어 구동 없음):

```bash
cd ~/craft && source /opt/ros/lyrical/setup.bash && source install/setup.bash
ctest --test-dir build/craft_planning --output-on-failure -R '^(puck_execute_parity|puck_legacy_parity|puck_camera_parity|puck_cli_checks)$'
python3 src/craft_bringup/test/map_tools_parity.py build/craft_bringup/map_tools_cli src/craft_planning/maps/brief_arena1cm.yaml
ROS_LOG_DIR=/tmp/craft-port-launch-logs ros2 launch craft_bringup craft.launch.py --show-args
```

사용자가 빌드하고 미션 노드를 시작하는 명령:

```bash
cd ~/craft &&
source /opt/ros/lyrical/setup.bash &&
export MAKEFLAGS='-j2 -l2' &&
colcon build --symlink-install --packages-skip approach_sim driving_sim &&
source install/setup.bash &&
ros2 launch craft_bringup craft.launch.py mission:=true arm:=true localization:=amcl \
  map_file:=/home/craft/maps/brief_arena1cm.yaml \
  slam_map:=/home/craft/maps/noboxes_legend_opt_arena
```

다른 터미널에서 미션 START:

```bash
cd ~/craft && source install/setup.bash &&
ros2 topic pub -t 6 -r 2 /mission/command std_msgs/msg/String "{data: START}"
```
