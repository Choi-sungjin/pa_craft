# S1 집기 실패 후 수정 — 2026-11-16

1번 파지 보존 수정은 구현·빌드·오프라인 검증 완료. 2번 관절 시간 변경은 사용자의 명시 중지 조건에 따라 **미적용**했다. 현재 공통 프로파일 시간에서 손목이 속도 상한을 결정하는 이동은, 어깨 시간을 유지하면서 손목을 확실히 먼저 끝낼 수 없다.

## 변경과 백업

기존 파일 9개는 전부 수정 전에 `/home/craft/port_backup/<아래 상대 경로>`에 원본을 보관했다. 해당 경로에 기존 백업이 없었으므로 port_backup2는 사용하지 않았다.

| 변경 파일 | 내용 |
|---|---|
| `src/craft_control/src/control_master.cpp` | SUCCEEDED + fresh heartbeat + 무고장 + 비STOP 조건의 hold에서 확인된 파지만 유지. hold 중 fault/heartbeat 만료로 바뀌어도 clear. 송신 후 판명되는 TX fault도 마지막 status.fault 분기에서 clear. tf2 deprecated `.h`를 `.hpp`로 교체 |
| `src/craft_control/src/gripper_controller.cpp` | `clear_target(true)`는 확인된 파지 목표와 latch를 보존. 기본 clear는 기존대로 해제 |
| `src/craft_control/include/craft_control/gripper_controller.hpp` | 보존 인자와 안전 조건 문서화 |
| `src/craft_control/include/craft_control/planning_watchdog.hpp` | 요청 hold와 heartbeat 만료를 구별할 `fresh()` 공개 |
| `src/craft_control/include/craft_control/task_tracker.hpp` | STOPPED의 5개 controller 반복문을 명시적 bounds + `.at()`로 변경. GCC 13의 inlining overflow 경고 제거, 동작 동일 |
| `src/craft_control/test/test_control_controllers.cpp` | 성공 hold의 squeeze/latch 보존→STOP clear, 미확인 close/open의 비보존, heartbeat fresh 경계 검사 |
| `src/craft_planning/src/puck_execute/executor.cpp` | close 성공 후 bool 확인을 최대 500ms 기다림. 결과/파지 토픽 도착 순서 차이 흡수, 확인 없으면 기존대로 중단 |
| `demo/puck_pick/execute.py` | 같은 최대 500ms 확인 대기 |
| `tools/realsense_check/grip_locate.py` | 예전 bigbro의 closed-jaw HELD 검사도 모든 모양에 prism reference·좌우/깊이/score 기준 적용 |

새 파일(이전 원본 없음): 이 보고서, `src/craft_planning/test/puck_grasp_checks.py`, `src/craft_planning/test/puck_joint_timing.py`, `demo/puck_pick/runs/s1_fix_1116_offline/` 검증 기록.

## 파지 기준 조사

- control의 전류 기준은 원래부터 모양 공통: **절대 전류 0.1 A 이상이 연속 0.1초**, 접촉각에서 닫힘 방향 **0.0349 rad ≈ 2°** squeeze. current hold 옵션은 0(꺼짐). 수치는 변경하지 않았다.
- C++/Python executor의 전류 파지 확인은 모양 분기 없이 같은 `/control/gripper_grasped`를 사용한다. 확인을 제거하지 않았다.
- `pick_policy`는 후보 우선순위와 자세를 고른다. 파지 bool을 판정하지 않는다.
- standing/collect의 `stand_cal.json`은 2/5/8번 칸의 공통 `grip_ref`를 쓴다. 모양별 파지 기준표가 없다. 실린더의 yaw 생략은 방향 결정이며 전류 파지 판정과 별개다.
- 예전 bigbro의 `grip_locate.py --held`에는 차이가 있었다: prism `(cx=853, bottom=313)` / cylinder `(846,317)` 기준과 cylinder의 좌우 검사 생략. 이제 둘 다 **prism 기준**, 좌우 ±10px, 아래쪽 경계 +10px/−25px, score ≥0.85를 쓴다. 미검출은 BAD, prism 기준이 없으면 기존과 같이 UNKNOWN이다.
- `grip_ref.json`, `grip_zero.json`의 역사적인 모양별 항목은 **닫기 전 위치 보정용**이며 파지 bool/닫은 뒤 HELD 판정에 쓰는 기준이 아니다. 위치 오차 가설로 보정값을 바꾸지 않았다. 실린더 전용 보정은 생성하지 않았다.

## 팔 시간: 미적용 이유와 수치

Python `step_seconds()` / C++ `step_seconds()` → TaskCommand.duration_s → control `TIME,<ms>` → firmware `moveMs()` / `writeTargets()` 경로다. via 이동은 최대 관절 각도 차이/40 deg/s, 최소 3초의 시간도 적용한다. control.yaml의 host arm/LM speed·accel caps 4개는 현재 모두 0(꺼짐)이다. firmware 속도 상한은 그대로 작동한다: arm 50×0.229 rpm = 68.7 deg/s, LM 180×0.229 rpm ≈27.546 mm/s. 관절 제한·전류·속도·가속도 설정은 변경하지 않았다.

펌웨어 `CraftMotorFirmware.h:447`의 공통 시간:

`T = ceil_ms(max(TIME, Δmotor_i / (0.75 × vmax_i × scale_i)))`

각 변경 축의 프로파일 시간은 `max(100ms, floor_ms(T × scale_i))`. ramp는 앞뒤 각각 25%. 현재 LM/어깨/팔꿈치/손목/roll의 scale은 모두 1, gripper만 0.7이다. 이미 목표에 있는 축은 새 프로파일을 쓰지 않는다. 실제 settling 시간은 이 프로파일 계산과 다를 수 있다.

대표 조건: 중앙 LM, yaw 0°, 위치 보정 0, 퍽 앞거리 112.5mm. `standfind 2`는 비아→관측 자세, `stand load 2`는 같은 관측 자세 높이의 들기 자세→carry via. **런타임 시간은 변경하지 않았으므로 전/후 동일**하다.

| 이동 | LM | 어깨 | 팔꿈치 | 손목 | tool_roll |
|---|---:|---:|---:|---:|---:|
| standfind 2 전=후 | 3.171s | 3.171s | 3.171s | 3.171s | 3.171s |
| stand load 2 전=후 | 5.166s | 5.166s | 5.166s | 5.166s | 이미 목표, 이동 없음 |

standfind 2의 각 축 cap 최소시간은 LM 0.17225 / 어깨 0.13702 / 팔꿈치 0.74313 / 손목 **3.17031** / roll 1.75449초다. 손목 163.35° 이동이 공통 시간을 결정하므로 어깨 3.171초보다 손목을 확실히 먼저 끝내려면 상한을 올리거나 어깨를 늦춰야 한다. stand load 2의 손목 206.65° 최소시간은 4.01067초로, 이 이동만 보면 기존 5.166초 어깨보다 빨리 끝낼 여유가 있다.

**적용하지 않은** scale=0.5(어깨는 1) 단순 변경은 standfind 2의 어깨를 6.341초, load 2의 어깨를 8.022초로 늘린다. 157mm 예제도 기존 공통 3.166/5.160초 → 어깨 6.332/8.012초다. shoulder timing 보존 조건을 위반한다.

선택지:

1. **어깨 시간·안전 상한 보존, 상한에 걸린 축의 동시 완료 허용.** 펌웨어를 축별 독립 시간으로 바꾸면 여유 있는 축은 빨라질 수 있다. standfind 2의 손목은 어깨와 거의 함께 끝난다. 별도 펌웨어 구현·검증·사용자 업로드 필요.
2. **다른 모든 축 선완료를 우선하고 어깨 지연 허용.** 어깨의 이동 시간 변경을 허용해야 한다.
3. **어깨를 고정한 준비 이동 후 어깨 이동.** 별도 단계로 분리하므로 “같은 move” 조건과 전체 순서가 달라진다. 티칭 자세/충돌 검토가 먼저 필요하다.

기존 상한을 올리는 변경은 하지 않았다. standfind/stand pick/stand load/lie 전체에 시간 변경을 부분 적용하지 않았다.

수치 재현(ROS 노드/카메라/시리얼 생성 없음):

```bash
cd ~/craft
source /opt/ros/lyrical/setup.bash
source install/setup.bash
python3 src/craft_planning/test/puck_joint_timing.py
```

## 빌드·검증

```bash
cd ~/craft
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select craft_control craft_planning
```

- 두 패키지 최종 빌드 성공, warning 0 / error 0 / stderr 0. 첫 빌드의 planning에는 GCC aarch64의 기존 C++17 ABI **note** 1개가 있었다. 첫/최종 로그는 `runs/s1_fix_1116_offline/{craft_control,craft_planning}_{stderr,final_stderr}.log`.
- 새 파지 보존 테스트는 수정 전 실제 실패(hold 후 false·squeeze 목표 소실), 수정 후 통과. XML `s1_grasp_before.xml` / `s1_control_after.xml`.
- 영향받는 GripperController/PlanningWatchdog/TaskTracker **13/13 통과**. 늦은 TX fault 보완 후 최종 빌드에서도 13/13 재확인(`s1_control_final.xml`).
- 전체 control gtest **74/75 통과**. 기존 `LateralController.InPlaceTurnKeepsItsDirectionAndSlowsNearTheTarget` 하나 실패: 양수여야 할 w[1] = −4.291318. 백업의 원본 테스트를 별도 빌드해 동일 실패 확인(`s1_original_lateral.xml`), lateral 구현은 수정하지 않았다.
- `puck_grasp_checks.py`: 실제 Python `run_steps`를 fake transport/time으로 호출, 즉시 true·지연 true·미확인·실패 4조건. 실제 HELD 함수에 prism/cylinder 각각 기준·경계·거절 7조건(총14조건). 모두 통과. 실제 카메라/ROS transport는 생성하지 않았다.
- 요청한 `puck_execute_parity`, `puck_legacy_parity`, `puck_camera_parity`, `puck_cli_checks` **4/4 통과**, 총65.00초. 기대값 변경 없음. camera parity에는 저장 스택 재생과 실제 `find_step`/`lie_yaw_step` 호출이 포함된다.
- Python 컴파일 검사와 `git diff --check` 통과.
- control_master 노드를 시작하는 integration test는 사용자 금지에 따라 실행하지 않았다. 실제 control_loop 전체 및 C++ DDS 지연은 실물/노드 통합 검증 범위로 남는다.

## sol6.1 독립 검토

별도 gpt-6.1-sol(high)이 원본 대비 control_master/gripper/watchdog/TaskTracker 및 executor diff를 검토했다. 완료 hold 후 RX는 정상인데 TX만 실패하면 status.fault=true여도 squeeze가 남는 누락을 지적했고, 메인이 해당 분기를 직접 확인해 fault 계산 직후 clear를 추가했다. 검토자가 최종 적용본을 다시 읽고 **남은 blocker 없음**으로 확인했다. 성공 hold 유지 조건, hold 중 fault/stale 재clear, STOP 기본 clear, explicit open reset, 최대500ms 대기도 확인했다. 메인도 해당 분기와 테스트 출력을 직접 확인했다. 늦은 TX fault 조합의 control_master 노드 통합 검증은 실행하지 않았다.

## 첫 실물 실행에서 볼 것

- 새 빌드가 사용되도록 사용자 측에서 관련 프로세스를 재시작한 뒤, close SUCCEEDED → planning hold 전후 `/control/gripper_grasped=true`와 조의 압착 유지 확인.
- 실제 STOP/fault/명시 open에서 bool false 및 이전 squeeze 목표가 해제되는지 확인. STOP clear는 조를 강제로 여는 명령이 아니라 현재 위치 유지로 돌아가는 기존 동작이다.
- 팔 시간은 그대로이므로 벽으로 뻗는 현상은 아직 해결되지 않았다. 해당 이동을 개선된 것으로 간주하면 안 된다.
- 하드웨어 구동·카메라/시리얼 접근·mrun/launch/control_master 실행·펌웨어 업로드·git commit/stash/reset/checkout/clean을 수행하지 않았다.
