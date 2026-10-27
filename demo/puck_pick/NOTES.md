# CRAFT 퍽 집기 데모: 실행 방법과 작업 기록 (2026-10-24)

## 1. 실행 명령어

### RViz 미리보기 (하드웨어 없이)
```bash
~/craft/demo/puck_pick/show.sh      # plan.py로 다시 계산한 뒤 RViz에서 반복 재생, Ctrl+C로 종료
```
- 이 컴퓨터 안에서만 통신합니다(`ROS_DOMAIN_ID=77`, LOCALHOST).
- 높이 보정(2.5 cm) 때문에 RViz에서는 집게가 퍽보다 2.5 cm 위에서 멈춘 것처럼 보입니다. 정상입니다.

### 실물 실행
```bash
# 0) 전원 켜기 전: 팔을 받치고 LM을 모터 쪽 끝에 붙인다 (회전 수 기준)
# 1) 보드 상태 확인 (LM 808~852°, '!!' 경고 없음)
checkarm
# 2) 팔이 너무 접혀 있으면 손으로 반쯤 펴기 (팔 받친 상태에서)
relaxarm      # 토크 OFF
holdarm       # 토크 ON, 그 자리 유지
# 3) 터미널 A: control
cd ~/craft && source install/setup.bash
ros2 launch craft_bringup craft.launch.py group:=control
# 4) 터미널 B: 퍽 집기
cd ~/craft && source install/setup.bash
python3 demo/puck_pick/execute.py            # --dry-run: 단계만 출력
```
- 단축 명령: `sis1` = 집어서 clock 12번 슬롯에 적재, `sis2` = 12번 슬롯에서 꺼내 앞(집던 자리)에 내려놓기 (`execute.py --unload`). `sis`는 `sis1`과 같음.
- `sis2` 단계: 들어 올린 자세 → LM 적재 라인 → clock 12 → 그리퍼 열기 → wrist를 접은 채 잡을 위치 6 cm 위 → 수직으로 내려가 잡기(적재 자세 3 cm 아래) → 그리퍼 닫기 → **집게 각도를 유지한 채 수직으로 3 cm 올려 적재 자세로** → 앞으로 넘기기(sis1에서 퍽을 들고 들어온 경로의 역순) → LM 퍽 라인 → 내려놓기 → 그리퍼 열기 → 위로 → LM·팔 원위치
- 빼는 경로 (2026-10-27 수정): 예전엔 잡은 뒤 6 cm 위(wrist 고정)로 바로 올라가 시작부터 집게가 기울어(1 cm에 6.7°, 2 cm에 13°, 휨 5 mm) 퍽이 슬롯을 긁었다. 잡는 자세와 적재 자세는 집게 각도(세 pitch 관절 합)가 같아 그 사이 관절 보간은 각도가 일정하고 휨 1.3 mm. 적재 자세보다 위로는 이 각도를 wrist 제한(−125°) 때문에 유지할 수 없다
- 실행 중 키 입력
  - **Enter**: 다음 단계 실행
  - **`bro`**: 남은 단계를 한 번에 실행
  - **`q`**: 중단(복귀할지 물어봄)
  - **Ctrl+C**: 비상 정지
- 단계 순서: arm above the puck → open gripper → LM to the puck line → descend → close gripper → lift the puck → clock to slot 12 (적재 guide) → arm to the loading pose → open gripper (clock에 넣기) → back above the puck line → LM back → arm back
- `execute.py`를 Ctrl+C로 끊었다면 control도 재시작합니다(STOPPING 상태가 남음).

### 보드 관련 명령 (`~/.bashrc`)
| 명령 | 동작 |
| --- | --- |
| `resetarm` / `resetwheel` / `resetall` | 펌웨어 컴파일 + 업로드(= 보드 재부팅). 끝나면 피드백 확인까지 자동. 로그: `~/.cache/craft_fw/log/` |
| `checkarm` / `checkwheel` / `checkall` | 피드백 1초 읽고 상태·경고 출력 (control 꺼진 상태에서). 정상 CSV가 10줄 미만이면 실패 |
| `relaxarm` / `holdarm` | 팔 보드 토크 OFF / ON (control 꺼진 상태에서, 팔 받치고) |
| `wizard [arm\|wheel\|all]` | usb_to_dxl 업로드 + DYNAMIXEL Wizard 2.0 실행. 창을 닫은 뒤 `reset*` |
| `usbfix` | Jetson USB 허브 `1-2` 재연결. 업로드 뒤 포트가 안 돌아올 때(3.2). control 끈 상태에서 |
| `bro` | 터미널 A: 보드 확인 후 control 실행 |
| `sis1` / `sis2` | 터미널 B: 집어서 clock 12에 적재 / clock 12에서 꺼내 앞에 내려놓기 |

- **보드 잠금**: 위 명령(`sis*` 제외)은 보드별로 하나씩만 실행됩니다. 사용 중이면 `[lock] arm board busy (...)`로 바로 거부합니다(기다리지 않음). `bro`는 control이 끝날 때까지, `wizard`는 Wizard 창을 닫을 때까지 잠금을 유지합니다. 이유는 3.1.

### 데모 위치
- 2026-10-27부터 git에 포함(`demo/puck_pick/`). 지우려면 `git rm -r demo`.

---

## 2. 접근 방식

1. **URDF를 실물에 맞춤**
   - 실측한 링크 길이, LM 판·레일 길이, 그리퍼 길이를 반영하고 메시도 그에 맞게 변형했습니다.
   - 조인트 방향은 그대로 두어, 기존에 맞춘 모터 오프셋이 그대로 유효합니다.
2. **IK**
   - LM(slide)이 좌우(X)를 맡고, shoulder·elbow·wrist가 앞뒤·위아래 평면을 해석적으로 풉니다.
   - 집게는 기본적으로 수직 아래를 향하고, 손목 제한에 걸리면 **필요한 만큼만** 집게 끝을 앞쪽으로 기울입니다. 1° 단위, 최대 30°, 모든 관절이 제한에서 3° 이상 떨어지게 합니다.
3. **잡는 높이**
   - 손가락이 퍽 위쪽 `grasp_coverage`(현재 50%)만 감싸게 합니다.
   - 진입 높이는 손가락 끝이 퍽 윗면 1 cm 위에 오는 높이(4 cm)입니다.
4. **실행**
   - `/planning/task_command`로 control_master에 단계별 작업(MANIPULATOR / GRIPPER)을 보내고, `/control/task_result`를 기다립니다.
   - 끝나면 시작 자세로 되돌아가서 바로 다시 실행할 수 있습니다.
5. **잡는 비율 비교** (13 cm, 높이 보정 2.5 cm 기준)

   | 잡는 비율 | 잡을 때 기울기 | 퍽 앞뒤 허용 범위 |
   | --- | --- | --- |
   | 30% | 19° | 11.4~19.4 cm |
   | 40% | 16° | 11.4~19.4 cm |
   | **50% (채택)** | **14°** | 11.4~19.4 cm |
   | 60% | 12° | 11.4~19.4 cm |

   - 허용 범위는 진입 자세가 결정하기 때문에 잡는 비율과 상관없이 같습니다. 비율이 바꾸는 건 기울기뿐입니다.
   - 토크 차이는 무시할 수준입니다. 질량은 가정한 값입니다.
   - 높이 보정이 없으면(오프셋 재보정 후) 50%에서 기울기 4°, 허용 범위 6~22 cm 이상입니다.
   - 적재를 고려해 잡는 비율은 50% 이하로 유지합니다.

### 실측 반영값
| 항목 | CAD | 실측 / 적용 |
| --- | --- | --- |
| 와플 너비 | 267.1 mm | 26.7 cm (일치) |
| LM 지지판 | 259.6 mm | 25.7 cm, 와플 왼쪽(LM 모터 쪽) 끝보다 0.7 cm 바깥에서 시작 |
| LM 가이드 레일 | 200 mm | 16.8 cm, 지지판 중앙 |
| 바닥 → shoulder 축 | 122.25 mm | **114 mm** (2026-10-27. 예전 값은 모든 TCP를 실제보다 8.25 mm 높게 계산) |
| shoulder → elbow | 188.0 mm | 190 mm |
| elbow → wrist pitch | 95.0 mm | 92 mm |
| wrist pitch → tool roll 혼 | 56 mm | 55 mm |
| tool roll 혼 → 손가락 끝 | 125.3 mm | 133 mm (TCP 0.1024 → 0.1101) |
| LM 모터 끝에서 와플 왼쪽 끝 → 캐리지 블록 | 모델 67.4 mm | 36 mm → slide 원점 −0.01135, 실물 1 cm 초과 후 **−0.00135** |

### 잡기 좋은 영역 (2026-10-27, shoulder 114 mm 반영 URDF)

선반 7 cm 위 퍽(높이 60 mm), 잡는 TCP 높이 127 mm, 접근 4 cm 위, 모든 관절 3° 여유, 기울기는 필요한 만큼만(plan.py 기준). 접근 → 잡기 수직 하강 구간 전체가 URDF·모터 제한 안이어야 "잡을 수 있음". 팔은 앞뒤·위아래만, 좌우는 LM 혼자 정하므로 영역은 **직사각형**이다.

**추천 영역 (퍽 중심, 약 96 × 80 mm)**

| 방향 | 범위 | URDF `base_link` |
| --- | --- | --- |
| 앞뒤 | **LiDAR 앞면에서 60~140 mm 앞** | y = −26 ~ −106 mm (로봇 앞 = −y) |
| 좌우 | **LiDAR 중심에서 −58 ~ +38 mm** (+ = LM 모터에서 먼 쪽) | x = 70 ~ 166 mm |
| 중심 | LiDAR 중심 −10 mm, LiDAR 앞면 앞 100 mm | x 118, y −66 mm |

**앞뒤 (팔)**

| LiDAR 앞면에서 | 잡을 때 기울기 | 접근할 때 기울기 | 판단 |
| --- | --- | --- | --- |
| 10~50 mm | 0° | 30~14° | 몸체에 너무 가깝고 접근이 많이 기울어짐 |
| **60~140 mm** | **0° (수직)** | **10~13°** | **가장 좋음** |
| 150~200 mm | 0~9° | 14~21° | 잡을 때도 기울기 시작 |
| 210~250 mm | 11~20° | 22~30° | 한계 (30° 제한) |
| 250 mm 초과 | – | – | 불가 |

- 접근 높이에선 wrist 하한이 거의 항상 걸려 관절 여유는 어디서나 약 3°로 같다. 그래서 판단 기준은 **기울기**.
- 10~50 mm는 계산상 닿지만 퍽 반폭 15 mm + 벌린 집게를 생각하면 LiDAR에 너무 붙는다.

**좌우 (LM)**
- slide −0.0438 ~ 0.063 → TCP x 171.4 ~ 64.6 mm. 백래시 보정 2 mm와 양쪽 약 5 mm 여유를 빼서 70 ~ 166 mm. 앞뒤 위치와 무관.

**현재 데모 위치**: LiDAR 중심 +40 mm, 앞면 앞 130 mm. 앞뒤는 영역 안이지만 **좌우는 LM 끝에서 1.5 mm**(slide −0.0403 + 2 mm, 한계 −0.0438)로 경계 밖. `puck_x_offset`을 영역 중심(−0.01)으로 옮기는 것을 권장(아직 안 바꿈).

**한계**: 기구학만 봄(선반 모서리·몸체 충돌, 옆 물체와 집게 폭 미확인). URDF 치수·관절 오프셋이 맞다는 전제. 경계(60 mm, 140 mm)는 실물로 한 번씩 확인 필요.

---

## 3. Troubleshooting

| 증상 | 원인 | 해결 |
| --- | --- | --- |
| RViz에 `m38_arm ... Could not load mesh` | 같은 LAN의 다른 PC가 도메인 0에서 `/robot_description` 발행. 또 `ros_environment` 훅이 `source` 때마다 탐색 범위를 SUBNET으로 덮어씀 | `show.sh`에서 모든 `source` 뒤에 `ROS_DOMAIN_ID=77`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 설정 |
| LM이 반대로 가서 충돌할 뻔함 | Extended Position 모드는 전원을 켤 때 회전 수가 사라짐(0~360°) | 전원을 켤 때 항상 LM을 모터 끝에 붙임 + 펌웨어 `zero_tick=-8192`. `checkarm`이 범위 밖이면 경고 |
| 명령을 보내도 팔이 안 움직임 | 호스트가 `%.4f`로 반올림한 값이 펌웨어 제한 밖이 되어 프레임 전체가 버려짐 | YAML·펌웨어 제한값을 소수 4자리에서 안쪽으로 반올림 |
| 실물이 계획보다 약 2.5 cm 낮게 내려감 | 팔 관절 오프셋 오차(RViz에서 눈으로 맞춤). 그리퍼 길이 8 mm 차이는 별도로 수정함 | 임시: `plan.py`의 `hw_z_correction=0.025`. **근본 해결: 오프셋 재보정** |
| LM이 퍽보다 1 cm 더 감 | slide 원점 오차 | URDF slide 원점 +10 mm |
| 그리퍼를 닫을 때 팔이 처짐, 들어 올릴 때 쥔 힘이 풀림 | 새 작업을 시작할 때 control이 팔·LM·그리퍼 목표를 지움 → 처진 현재 위치를 다시 붙잡음 | 새 작업에서는 팔·LM·그리퍼 목표 유지. STOP·hold·fault 때만 지움 |
| 잡다가 fault | 잡은 뒤에도 목표가 229°(완전히 닫힘) → 모터가 막힌 채 최대 전류 → 과부하 오류로 추정 | 잡았다고 판정한 순간의 각도 **+2°**에서 고정해 쥠(`gripper_grasp_squeeze_angle`) |
| `resetarm` 두 번 → 보드가 USB에서 사라짐 (`error -71`) | 재부팅 중에 업로드가 겹쳐 보드가 멈춤(아래 3.1). 12 V로 계속 켜져 있어서 USB만 다시 꽂아서는 안 풀림 | 먼저 `usbfix`(허브 재연결, 3.2). 안 되면 RESET, 그래도 안 되면 전원 차단. 재발 방지: 보드 잠금 + 업로드 후 피드백 확인 + 업로드 로그(3.1) |
| `>> open gripper?`에서 진행 안 됨 (1) | 이전 실행을 Ctrl+C로 끊어 control이 STOPPING에 머묾 | control 재시작 |
| `>> ...?`에서 진행 안 됨 (2), 엘보 발열 | 시작 자세가 제한 밖(엘보 320° > 318°)이거나, 무게 때문에 2° 넘게 처져서 팔 보드가 계속 "이동 중"(상태 1) → 어떤 작업도 완료 안 됨 | 팔 이동을 첫 단계로 변경. 실패 시 관절별 목표·실제·오차 출력(`<-- off`). 팔을 제한 안에 두고 시작 |
| `sis2` 내려놓을 때 그리퍼가 열리면 팔이 아래로 툭 떨어짐 (2026-10-27) | clock에서 꺼낸 퍽은 앞에서 잡은 퍽보다 집게 안 깊숙이 잡힘 → 앞 잡기 높이(127 mm)까지 내리면 퍽이 선반(7 cm)을 누름 → I 게인이 계속 밀다가 그리퍼가 열리는 순간 받침이 사라짐 | 잡기 높이 **1 cm 위**(`PUT_DOWN_ABOVE`, TCP 137 mm)에서 놓음. 퍽은 최대 1 cm 떨어짐 |
| 실행 중 팔 보드 USB 끊김 (20:00:57) | 미확인 (12 V 출렁임 또는 케이블·허브 접촉 의심) | 재발하면 그 순간 LM이 움직였는지 확인 → LM 속도를 낮춰서 비교 |
| 선반 위에서 수직으로 못 들어 올림 | 손목 하한 119° | 필요한 만큼만 기울이기 + 관절 여유 3° |
| LM이 너무 느림 | profile velocity 30 (약 4.7 mm/s) | 180 (약 28 mm/s), 가속 30 |
| `cat`으로 OpenCR 출력이 안 보임 | USB CDC는 DTR이 켜져야 송신 | pyserial로 `dtr=True` (`checkarm`) |
| colcon 빌드 중 Jetson 메모리 부족 | 병렬 컴파일 | `MAKEFLAGS=-j1 --parallel-workers 1` |

### 3.1 업로드가 겹치면 보드가 멈추는 이유 (2026-10-27 분석)

어제 결론의 "인터럽트"는 보드가 스스로 만드는 하드웨어 인터럽트(IRQ)가 아니라, **진행 중인 업로드·재부팅에 호스트가 두 번째 업로드(또는 포트 접근)로 끼어든 것**이다. 보드의 USB 인터럽트는 그 요청을 처리하는 통로일 뿐이다.

`opencr_ld`: ROBOTIS가 만든 OpenCR 업로드 프로그램(`~/tools/OpenCR/arduino/opencr_develop/opencr_ld/`, C 소스 포함). `_craft_fw`(`resetarm` 등)가 컴파일된 `.bin`을 이것으로 보드에 굽는다. 순서:
1. 포트를 1200 baud로 열고 `"OpenCR 5555AAAA"` 전송 → 보드를 부트로더로 보냄
2. **상태 확인 없이 1.5초 고정 대기**
3. 같은 포트를 다시 열어 보드 이름 확인 → 플래시 지우기 → 쓰기 → CRC 확인
4. 앱으로 점프 명령

보드 쪽 동작 (OpenCR 코어·부트로더 소스에서 확인):
- **앱 → 부트로더**: 앱의 USB ISR이 1200 baud 설정을 보고 플래그를 세우고, 다음 수신 패킷에서 10 ms 워치독(IWDG)을 켜서 MCU를 리셋한다. 이 플래그는 앱에서 지워지지 않아, 그 뒤엔 아무 바이트나 받아도 리셋된다.
- **부트로더**: 리셋 원인이 워치독이면 부트로더에 남고, 아니면 바로 앱으로 점프한다.
- **부트로더에는 1200 baud·매직 처리 코드가 없다.** 부트로더 상태에서 두 번째 `resetarm`이 오면 매직은 무시되고, 1.5초 뒤 두 업로드의 지우기·쓰기 명령이 섞여 들어간다. → 플래시가 깨지거나 USB가 응답 없는 상태(`-71`)로 남을 수 있음. **가장 유력.**
- **부트로더 → 앱**: 리셋이 아니라 점프다(USB·HAL만 끄고 `__disable_irq()`는 주석 처리). 점프 직후 앱 초기화 중에 호스트가 포트를 건드리면 덜 준비된 USB 인터럽트가 돌 수 있음. **의심 단계.**
- 당시 커널 로그가 남아 있지 않아 위 둘 중 무엇인지는 **확정하지 못함**. 코드 구조와 증상(두 번 실행 → `-71`)으로 추론한 것.

대책 (호스트 쪽에서 겹치지 않게 막기). **1·3·4 구현 (2026-10-27, `~/.bashrc`), 2·5 미구현**:
1. **보드 잠금(`flock`, `_craft_locked`)**: `reset*`·`check*`·`relax/hold*`·`wizard`·`bro`가 보드별 잠금(`~/.cache/craft_fw/{arm,wheel}.lock`)을 잡고 실행. 사용 중이면 기다리지 않고 거부하며 누가 잡고 있는지 출력. `bro`는 control이 끝날 때까지 두 보드 잠금 유지. `wizard arm|wheel`은 Wizard 2.0 창을 닫을 때까지 그 보드 잠금 유지(백그라운드 감시, 실행 파일 이름으로 판별). `sis*`는 ROS로만 통신해서 해당 없음. 업로드 전에 포트가 생긴 지 3초 미만이면 3초까지 대기
2. (미구현) **`opencr_ld` 수정 빌드**: 고정 1.5초 대신 포트가 사라졌다 부트로더로 다시 잡히는 것을 확인하고 진행. 이미 부트로더면(보드 이름 응답이 바로 오면) 매직을 건너뜀
3. **업로드 후 확인**: 포트가 돌아오면 1초 대기 → `_craft_check`로 필드 수가 맞는 CSV(팔 8개, 바퀴 7개)가 1초에 10줄 이상 오는지 확인(앱은 50 Hz로 계속 보내고 부트로더는 조용함) → 1초 더 대기 후 잠금 해제. 응답이 없으면 "5초 뒤 check, 그래도 조용하면 reset 한 번만" 안내. `check*`도 같은 기준(예전엔 한 줄만 파싱돼도 통과)
4. **업로드 로그 저장**: 매번 전체 출력과 그 시간대 커널 로그(`journalctl -k`)를 `~/.cache/craft_fw/log/<시각>_<sketch>_<port>.log`에 남김. 다음에 걸리면 원인 확정
5. (보험) Jetson GPIO → OpenCR RESET 핀 연결로 전원을 다 뽑지 않고 리셋
- 부트로더 자체 수정은 비추천: 부트로더를 다시 굽다가 실패하면 보드를 복구하기 어려움.

### 3.2 겹침이 아닌 두 번째 원인: 호스트 USB 꼬임, 허브 재연결로 복구 (2026-10-27)

**사례**: 보드 잠금이 있는 상태에서 `resetwheel` **한 번만** 실행(겹침 없음). 로그 `~/.cache/craft_fw/log/20261027-101809_wheel_clock_opencr_wheel_clock.log`.

| 시각 | 사건 |
| --- | --- |
| 10:18:10~11 | 앱 → 부트로더 정상, 부트로더 ttyACM1로 잡힘 |
| ~10:18:14 | 지우기 1.2 s, 쓰기 1.2 s, CRC OK, `jump_to_fw` → USB 끊김(정상) |
| 10:18:19 | 5초 뒤 장치 재등장, descriptor 요청 무응답 (`-110`) |
| 10:19:22~29 | 허브가 포트 전원 재시도 → `not responding to setup address`, `-71`, `unable to enumerate` |
| 10:22~ | 보드 RESET, 이어서 SW2+RESET(부트로더 모드)까지 했지만 **같은 증상**. 매번 `WARN: invalid context state for evaluate context command` |
| 10:30 | **보드·케이블·전원은 그대로 두고** Jetson 쪽 허브 `1-2`만 unbind/bind → 같은 포트(1-2.1)에서 즉시 복구. `checkwheel` 정상(idle, `imu_ok 1`), 새 펌웨어도 정상 |

**결론**
- 보드나 펌웨어가 죽은 것이 아니라 **호스트(Jetson USB 컨트롤러·허브)가 그 포트 상태를 꼬인 채로 유지**한 것. 근거: 우리 펌웨어와 무관한 부트로더도 같은 포트에서 실패, xHCI 경고, 허브 재연결만으로 복구.
- 매번 나는 것은 아님: 같은 날 09:14, 09:16에는 같은 포트에서 `resetwheel` 정상. 업로드 끝 `jump_to_fw`에서 보드가 USB를 갑자기 끊었다 다시 붙는 순간 **가끔** 꼬이는 것으로 추정(확정 아님).
- 추정 메커니즘: OpenCR은 Full-speed라 고속 허브의 TT(Transaction Translator)를 거친다. 바퀴 보드는 Jetson의 Realtek 허브(1-2)에 직결되어 그 TT를 쓰고, 팔 보드는 QinHeng 허브(1-2.2) 밑이라 다른 TT를 쓴다. 갑작스러운 끊김에 전송이 TT에 걸린 채 남으면 그 포트에 어떤 장치도 주소를 못 받는다.
- 지금까지 "전원을 다 뽑아야 풀렸다"는 것도 실제로는 그 과정에서 허브가 리셋된 덕이었을 가능성이 크다.
- 같은 시간대에 QinHeng 허브(팔 보드·LiDAR)도 여러 번 끊겼다 붙음(`hub_ext_port_status failed (-71)`). 케이블을 옮기던 시간과 겹치는지 미확인. 재발하면 이 허브도 따로 볼 것.

**대응 (구현, `~/.bashrc`)**
- `usbfix`: `sudo`로 허브 `1-2` unbind → 2초 → bind. 그 허브 밑 장치(OpenCR 둘, LiDAR)가 몇 초간 끊기므로 OpenCR 포트를 누가 쓰고 있으면 거부. 죽은 포트가 있으면 커널 재시도가 끝날 때까지 1~2분 걸리고 그동안 Ctrl+C가 안 먹는다(`D` 상태, 정상).
- `reset*` 업로드 후 포트를 최대 40초 기다리면서 커널 로그의 `-110`/`-71`/`unable to enumerate`를 감시. 보이면 해당 줄과 함께 "허브가 그 포트를 꼬이게 함 → control 끄고 `usbfix`"를 바로 안내(전원 차단·RESET 불필요). 에러 없이 안 돌아오면 케이블 확인 후 RESET 한 번 안내.
- 복구 순서: `usbfix` → 안 되면 그때 RESET → 그래도 안 되면 Jetson 재부팅 또는 전원 차단.

---

## 4. 구현 사항 (변경 파일)

### URDF / 메시: `src/craft_planning/v75_ws/src/v75_robot_description/`
- `scripts/fit_measured.py` (신규): 원본 CAD 메시(git `f909f20`)에 실측값을 반영해 메시를 다시 생성합니다. 숫자만 고쳐 다시 실행하면 됩니다.
  - 팔 링크: 링크 중간 구간만 늘이거나 줄입니다.
  - LM 묶음: X 방향으로 맞추고, 레일은 168 mm로 줄입니다.
  - 손가락: 중간 부분만 늘립니다.
- `urdf/v75_robot.urdf` (+ `v75_arm_ik` 사본 동일)
  - 조인트 원점: elbow, wrist, tool_roll, tool_tcp, slide, idler
  - 이전 값은 주석으로 남김
- `src/craft_planning/test/test_arm_planner.cpp`: q=0 TCP 기대값 갱신. 2026-10-27: y 0.412477 → **0.415477** (URDF TCP 0.115 → 0.118로 바꾼 뒤 테스트를 안 맞춰서 실패하던 것; 관절 0에서 집게가 +y를 향해 y만 3 mm 차이)
- `src/craft_planning/src/arm_planner.cpp` (2026-10-27): IK가 시드 해에서 멈추지 않고 시드 + 랜덤 재시작 해를 모두 모은 뒤 **elbow 하중이 가장 작은 해**를 고름(같으면 시드에 가까운 해). URDF에 질량이 없어 wrist pitch·tool roll·TCP 원점을 같은 질량으로 본 elbow 축 중력 모멘트로 근사. 테스트 `PrefersTheBranchWithTheLeastElbowLoad`

### control: `src/craft_control/`
- `control_master.cpp`: `clear_targets_for_new_task()` 추가. 새 작업에서 팔·LM·그리퍼 목표를 유지합니다.
- `gripper_controller.{hpp,cpp}`: 잡음 판정을 유지하고, 판정 순간 각도 + 조임 각도로 고정합니다.
- `config/control.yaml`: `gripper_grasp_squeeze_angle: 0.0349` (2°)
- `config/control.yaml`: `arm_angle_tolerance`·`lm_angle_tolerance` 1°·2° → **0.5°** (2026-10-27, 펌웨어 `lm_arm/config.h` 팔 관절 4개·LM과 같은 값. 둘은 항상 같이 바꿀 것)
- `test/test_control_controllers.cpp`: 잡음 판정 유지와 조임 테스트 추가

### 펌웨어: `firmware/`
- `CraftProtocol.h`, `CraftMotorFirmware.h`: `RELAX` / `HOLD` 명령
  - RELAX: 토크 OFF, 피드백 유지, CSV 명령 무시, 상태 2
  - HOLD: 그 자리에서 토크 ON
- `lm_arm/config.h`: LM `zero_tick=-8192`, profile velocity 180, 가속 30
- `test/test_firmware.cpp`: RELAX/HOLD 테스트
- `README.md`: 프로토콜 설명

### 호스트 셸: `~/.bashrc`
- 백업: `~/.bashrc.bak-*`
- `_craft_locked`: 보드별 `flock` 잠금(`~/.cache/craft_fw/{arm,wheel}.lock`). 공개 명령은 전부 이걸로 감쌈
- `resetarm`/`resetwheel`/`resetall`: 포트 3초 안정 대기 → 업로드 → 실패 감지 → 포트 복귀 대기 → 피드백 확인. 매번 로그 저장
- `checkarm`/`checkwheel`/`checkall`: 필드 수가 맞는 CSV 10줄 이상이어야 통과
- `relaxarm`/`holdarm`
- `wizard`: Wizard 창이 닫힐 때까지 잠금 유지
- `usbfix`, 업로드 후 USB 꼬임 감지·안내 (3.2)
- `bro`, `sis1`/`sis2`(`sis` = `sis1`)

### 데모 (삭제 가능): `demo/puck_pick/`
- `plan.py`: URDF 파싱, IK, 궤적 생성, 제한 검사 → `plan.yaml`
  - 기울이기는 필요한 만큼만, 관절 여유 3°
  - 선반 높이 7 cm, 잡는 비율, 높이 보정 반영
- `show.py`, `show.sh`, `demo.rviz`: RViz 미리보기(퍽, 선반, 라이다 마커)
- `execute.py`: 실물 실행
  - 단계 실행, `bro`로 연속 실행
  - 실패하면 관절별 오차 출력 후 복귀
  - 시작 자세로 복귀

---

## 5. 남은 일 / 주의

- [ ] shoulder 높이 114 mm 반영 후 `sis1`/`sis2` 실물 재검증 (2026-10-27: 잡기 높이가 실제로 8 mm 올라가고 집게가 수직이 됨)
- [ ] **팔 관절 오프셋 재보정**: 2.5 cm 높이 보정을 없애야 합니다. 기울기와 허용 범위가 가장 크게 좋아집니다.
- [ ] **기울기 허용치 실험**: 집게가 앞뒤로 닫히는 최악의 방향으로 0~30°를 5° 단위로 각 3회 시도해, "잡을 수 있는 자리"의 근거를 만듭니다.
- [ ] **퍽 상태별 잡는 방법**: 서 있음, 누움, 누운 채 돌아감, 원통. tool_roll로 회전을 맞추고, 상태별로 잡을 수 있는 영역을 계산합니다.
- [ ] **RealSense로 퍽 위치·자세 인식**: 현재 USB2에 연결되어 있어 USB3 포트로 옮겨야 합니다.
- [ ] 뒤로 넘겨서 적재하는 동작 계획
- [ ] (보류) 팔 실험은 2026-10-27 기준 중단, 다음은 주행 실험. 남은 팔 항목: 데모 퍽 위치를 잡기 영역 중심으로, planning 통합(미션·step 모드·HIL)
- [ ] `test_planning_watchdog_integration` 실패: 새 작업 시작 처리 변경을 되돌려도 실패하므로 그 변경과는 무관하고, 원인은 미확인입니다.
- [ ] 실행 중 팔 보드 USB 끊김 원인 확인
- [x] 업로드 겹침 방지 (3.1의 대책 1·3·4). 잠금·업로드 후 확인·로그는 10:17~10:18 실제 업로드에서 동작 확인
- [x] 호스트 USB 꼬임 복구 `usbfix` + 업로드 후 감지·안내 (3.2)
- [ ] USB 꼬임 재발 여부 관찰: 바퀴 보드를 QinHeng 허브 쪽으로 옮기면 안 나는지(TT 가설 확인)
- [ ] (선택) 3.1 대책 2: `opencr_ld` 수정 빌드
- [ ] 같은 LAN에 다른 팀이 도메인 0을 사용 중: 실제 로봇 스택도 `ROS_DOMAIN_ID` 분리를 권장합니다.
- [x] 커밋·푸시: 원격 `control-test` 브랜치 (2026-10-27, 데모 포함)
