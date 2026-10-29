# 모터 축 확인 시나리오

## 단일 모터 확인

모터 하나를 직접 보거나 움직일 때는 DYNAMIXEL Wizard 2.0을 씁니다. control을 끄고
`wizard`(팔/LM), `wizard wheel`, `wizard all`(둘 다)로 해당 OpenCR에 USB↔DXL 통과 펌웨어
(`firmware/usb_to_dxl`)를 올린 뒤 Wizard에서 그 포트에 Protocol 2.0, 1 Mbps로 연결합니다.
업로드 때 모터 전원이 한 번 끊기므로 팔과 LM을 받칩니다. 끝나면 Wizard 창을 닫고
`resetarm` / `resetwheel`로 원래 펌웨어를 올립니다(LM은 먼저 모터 쪽 끝에 붙입니다).

Wizard 창이 열려 있는 동안은 그 보드의 `reset*` / `check*` / `relax*` / `hold*` / `bro`가
`[lock] ... board busy`로 거부됩니다. 업로드가 겹치면 OpenCR이 부트로더에서 멈춰(`error -71`)
전원을 모두 뽑아야 하므로, 이 명령들은 보드별 잠금(`~/.bashrc`의 `_craft_locked`)을 잡고 하나씩만 실행됩니다.

## 바퀴 슬립 실험

사다리꼴 가속 기울기별로 1 m 직진 슬립을 잰다. 방법과 조건은 [slip/README.md](slip/README.md),
실행은 `slip plan` / `slip run --speed high|low --accel <m/s²>` / `slip summary`(`~/.bashrc`).

## 위치추정·IMU 실물 확인 (TODO)

두 IMU 처리, 튀는 값 필터, SLAM 감시와 재위치, IMU 되먹임을 실물에서 확인하는 순서와 기준은
[localization/README.md](localization/README.md)에 있다. 2026-10-29에 sim으로만 검증했다.

## 자이로 배율 실험 (TODO)

두 OpenCR(MPU-9250)의 자이로 배율 오차를 제자리 회전으로 보드별로 잰다. 데이터시트 허용 범위는 ±3%다.
주행 시나리오를 마친 뒤에 할 예정이다. 사전 작업과 절차는 [gyro_scale/README.md](gyro_scale/README.md)에 있다.

## 하드웨어 없는 자동 테스트

저장소 루트에서 실행합니다. C++11 컴파일러만 필요합니다.

```bash
g++ -std=c++11 -Wall -Wextra -Werror \
  -Ifirmware/test/fakes -Ifirmware/libraries/CraftMotorFirmware/src \
  test/test_motor_axes.cpp -o /tmp/craft-test-motor-axes
/tmp/craft-test-motor-axes

# CSV 오류, 범위 초과, watchdog 복구, 모터 오류 래치 회귀 검사
g++ -std=c++11 -Wall -Wextra -Werror \
  -Ifirmware/test/fakes -Ifirmware/libraries/CraftMotorFirmware/src \
  firmware/test/test_firmware.cpp -o /tmp/craft-test-firmware
/tmp/craft-test-firmware
```

`test_motor_axes.cpp`는 두 보드의 실제 `config.h`를 읽고 9개 축의 ID와 모드,
부팅 시 바퀴 정지/위치 유지, 축별 양·음 방향 목표 전달, 다른 축 목표 유지,
피드백 채널 순서와 tick 양자화 오차를 검사합니다. LM은 범위 중간에서 양방향으로
검사합니다. 가짜 버스에서만 확인 플래그를 활성화하며 실제 설정은 변경하지 않습니다.
물리 방향, 실제 영점, 모터 통신 또는 기계적 이동은 검증하지 않습니다.

## 실기기 축별 점검

모터 모델·단위·실측 제한·영점을 먼저 확인하고 [펌웨어 안내](../firmware/README.md)에
따라 보드를 준비합니다. 바퀴를 지면에서 띄우고 팔과 LM의 하중을 지지합니다.
비상 정지 수단을 준비하고 제어 노드 및 다른 시리얼 송신기를 종료한 뒤 보드 하나씩 점검합니다.
전원 투입 후 IMU 영점 보정을 위해 5초간 정지합니다.

| 보드 | CSV 인덱스 (0부터) | 축 | 모터 ID | 명령 단위 |
| --- | --- | --- | --- | --- |
| wheel_clock | 0 | wheel_l | 1 | rad/s |
| wheel_clock | 1 | wheel_r | 2 | rad/s |
| wheel_clock | 2 | clock | 3 | 모터 rad |
| lm_arm | 0 | LM | 1 | 모터 rad (m 아님) |
| lm_arm | 1 | shoulder | 2 | 모터 rad |
| lm_arm | 2 | elbow | 3 | 모터 rad |
| lm_arm | 3 | wrist_pitch | 4 | 모터 rad |
| lm_arm | 4 | tool_roll | 5 | 모터 rad |
| lm_arm | 5 | gripper | 6 | 모터 rad |

각 축에 다음 순서를 반복하고 결과를 기록합니다. USB CDC는 115200, 줄 끝은 `\n`이며
항상 보드 전체 채널을 송신합니다. 정상 명령은 20 ms마다 갱신합니다.
위치축의 다른 채널에는 현재 측정한 유지 위치를, 바퀴에는 0을 넣습니다.
위치축 전체에 0을 보내는 방식으로 초기화하지 않습니다.

1. **부팅:** 명령 없이 위치축은 현재 위치를 유지하고 바퀴는 정지하는지 확인합니다.
   두 보드의 모든 위치/속도 피드백과 state를 기록합니다. 정상 정지 state는 0입니다.
2. **단일 축 양방향:** 실측 제한 안에서 작은 양의 변화량을 보내고 원래 위치로 돌아온 뒤
   작은 음의 변화량을 보냅니다. 경계 근처에서는 먼저 충분한 여유를 확보합니다.
   예상한 축 하나만 움직이고, 물리 방향과 피드백 부호가 설계 좌표계와 일치해야 합니다.
   바퀴는 저속 양·음 속도 후 0으로 복귀합니다. 위치축은 이동 중 state=1,
   도착 후 오차가 설정 tolerance 이내이고 state=0이어야 합니다.
3. **유지:** 원래 측정 위치로 복귀하여 다른 축에 의도치 않은 이동이 없는지 확인합니다.
4. **통신 중단:** 낮은 속도/작은 이동 조건에서 명령을 250 ms 이상 중단합니다.
   다음 제어 주기에 바퀴 목표는 0, 위치축 목표는 중단 시점의 측정 위치로 유지되고
   state=2여야 합니다. 실제 정지 시간과 이동량을 별도로 기록합니다.
   새 유효 유지 명령으로 timeout이 복구되는지 확인합니다.
5. **재시작:** 하중을 지지하고 정지한 상태에서 재부팅한 뒤 영점과 초기 유지를 다시 확인합니다.
   다회전 위치는 전원 차단 후 보존되지 않으므로 clock과 팔/LM의 영점을 재확인합니다.

범위 초과/잘못된 CSV/버스 오류 주입은 위 자동 테스트로 먼저 확인합니다.
실기기에서 이동 중 케이블을 뽑아 오류를 만들지 않습니다. 팔의 토크 해제는 하중 낙하를 유발할 수 있습니다.
물리 방향·영점·제한을 확정한 뒤 호스트의 `control.yaml` 매핑과도 대조합니다.

| 축 | 보드/포트 | 모델 | 영점 tick | +방향 실제 동작 | -방향 실제 동작 | 최대 오차 | timeout 정지 | 결과/비고 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| wheel_l | | | | | | | | |
| wheel_r | | | | | | | | |
| clock | | | | | | | | |
| LM | | | | | | | | |
| shoulder | | | | | | | | |
| elbow | | | | | | | | |
| wrist_pitch | | | | | | | | |
| tool_roll | | | | | | | | |
| gripper | | | | | | | | |

## 기록된 물리 방향과 적용 설정

하드웨어 Reverse OFF에서 관찰한 raw 방향입니다. 모터 모델은 바퀴 XM430-W210,
나머지 XM430-W350입니다.

| 축 | 관찰한 raw 방향 | 적용 위치와 부호 |
| --- | --- | --- |
| wheel_l ID 1 | + → 전진 | 펌웨어 +1 |
| wheel_r ID 2 | + → 후진 | 펌웨어 -1 |
| clock ID 3 | + → 위에서 보면 CCW | 펌웨어 +1, YAML +1 |
| LM ID 1 | + → LM 모터 쪽 (URDF slide 축 -X, 같은 방향) | 펌웨어 +1, YAML +1 |
| shoulder ID 2 | URDF +와 같은 방향 | 펌웨어 +1, YAML +1 |
| elbow ID 3 | URDF +와 같은 방향 | 펌웨어 +1, YAML +1 |
| wrist_pitch ID 4 | URDF +와 반대 방향 | 펌웨어 +1, YAML -1 |
| tool_roll ID 5 | URDF +와 같은 방향 | 펌웨어 +1, YAML +1 |
| gripper ID 6 | + → 234도 쪽으로 닫힘·파지 | 펌웨어 +1, 열림 94도 / 닫힘 목표 229도 (끝단 234도에서 5도 여유) |

팔 부호는 URDF 기준 자세(+Y 앞, +Z 위)에서 비교한 값입니다. 오프셋 보정 후
실기 검증이 남아 있습니다. wrist pitch 제한은 70.1–340도이며 확인 플래그는 false입니다.
LM 영점은 -11.6도(가이드 밀린 뒤 재측정 -391.8–527.5도, 이전 249.4도), clock 슬롯 12 영점은 210.0도(적재 guide 위치, 이전 314.6도)이며 모두 호스트에서 적용합니다.
취소된 LM 317.2도 영점은 사용하지 않습니다. 보드 업로드·실제 이동은 하지 않았습니다.
