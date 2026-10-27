# CRAFT OpenCR 펌웨어

OpenCR 두 대에 각각 아래 스케치를 올립니다. 기존 `motor_controller.ino`의
TODO를 실제 구현으로 교체했습니다. 호스트는 `src/craft_control/src/control_master.cpp`입니다.

| 보드 / 스케치 | 명령 CSV | 피드백 CSV |
| --- | --- | --- |
| `wheel_clock/wheel_clock.ino` | `wheel_l,wheel_r,clock` | `wheel_l,wheel_r,clock,wheel_state,clock_state,imu_ok,gz` |
| `lm_arm/lm_arm.ino` | `lm,shoulder,elbow,wrist_pitch,tool_roll,gripper` | 같은 6개 위치 + `arm_lm_state,gripper_current_a` |

USB CDC `Serial`, 115200 설정, ASCII CSV, 줄 끝 `\n`입니다. `\r\n`도 받습니다.
피드백은 명령을 받기 전부터 자체 송신하며 정상 버스에서는 약 50 Hz입니다.
wheel 값은 **rad/s**, 나머지 축은 **모터 rad**, 그리퍼 전류는 **A**입니다.
LM의 m→rad 변환과 IK 관절 매핑은 호스트가 담당합니다.

`wheel_clock` 보드는 OpenCR 내장 IMU(2018년 보드: MPU9250, ±2000 dps)의 yaw 각속도 `gz`만 같은 줄 뒤에 붙입니다.
EKF가 쓰는 값이 이것뿐이라서입니다. 단위는 **rad/s**, 값은 보드 좌표계 그대로이고 장착 방향(현재 뒤집힘)은
호스트의 `base_link→imu_link` TF가 처리합니다. 전원 투입 후 최대 5초간 자이로 영점을 잡으므로
**그동안 로봇을 움직이지 마세요.** IMU가 응답하지 않으면 `imu_ok=0`, `gz=0`을 보냅니다.

## 하드웨어 설정

각 스케치의 `config.h`를 실제 하드웨어에 맞춰 수정하세요. 레포에는 모터 모델,
실제 영점 정보가 없으므로 `kHardwareConfirmed=false`가 기본입니다. 이 상태에서는
모터 버스에 접근하지 않고 `state=2` 피드백만 보냅니다. 설정을 확인한 뒤 `true`로
바꿔 빌드하면 초기 위치를 유지하며 토크를 켭니다.

- 기본 통신은 OpenCR `Serial3`, DIR 84, `BDPIN_DXL_PWR_EN`, Protocol 2.0, 1 Mbps입니다.
- 확정 ID는 wheel/clock 보드 **wheel_l=1, wheel_r=2, clock=3**입니다.
  CSV 채널 순서는 기존 `wheel_l,wheel_r,clock`을 유지하고 각 채널의 모터 ID만 매핑합니다.
- LM/arm 보드는 **LM=1, shoulder=2, elbow=3, wrist_pitch=4, tool_roll=5, gripper=6**입니다.
  보드마다 별도 버스이므로 ID가 같아도 됩니다.
  이 펌웨어는 모터 ID나 통신 속도를 재설정하지 않습니다.
- 기본 환산값은 4096 ticks/rev, 속도 0.229 RPM/tick을 사용하는 X-series 기준입니다.
  LM/arm 보드의 그리퍼는 `PRESENT_CURRENT`를 지원해야 합니다.
  확인된 모델은 바퀴 XM430-W210, 나머지 XM430-W350이며 전류 환산은 0.00269 A/tick입니다.
- 위치축은 **Extended Position Control Mode(4)**, wheel은 **Velocity Control Mode(1)**입니다.
  필요한 모드/레지스터를 지원하지 않거나 버스 읽기/쓰기가 실패하면 오류로 정지합니다.
- `direction`은 물리 모터 회전 방향, `zero_tick`은 호스트 모터각 0의 raw 위치입니다.
  `angle_rad = direction * (raw - zero_tick) * units_per_tick`입니다.
  호스트의 IK→모터 방향·오프셋과 중복 적용하지 않도록 좌표계를 맞추세요.
- 모든 모터의 하드웨어 Reverse는 OFF 기준입니다. wheel 펌웨어 방향은 왼쪽 +1,
  오른쪽 -1, clock +1입니다. LM/arm/gripper 펌웨어 방향은 모두 +1로 raw 부호를 유지합니다.
  URDF 방향 보정은 호스트에서만 shoulder +1, elbow +1, wrist_pitch -1, tool_roll +1,
  LM +1로 적용합니다 (URDF slide 축을 -X로 바꿔 raw +와 일치). 팔 방향은 기준 자세에서의 실측 설명에 따른 판단이며 영점 보정 후
  실기 검증이 남아 있습니다. 두 계층에서 같은 방향 보정을 중복 적용하지 않습니다.
- clock 영점은 슬롯 12에서 314.6도로 YAML에만 적용하며 펌웨어 zero_tick은 0입니다.
- wheel은 `max_wheel=7.8`을 따르고, LM·arm·gripper 제한은 현재 `control.yaml`의
  실측값과 일치합니다.
- 도착 허용오차(`tolerance`, 모터 각도)는 `lm_arm`의 LM과 팔 4축(shoulder, elbow,
  wrist_pitch, tool_roll)이 **0.5도**(약 6 tick, LM 캐리지 약 0.06 mm), gripper는 2도이며
  `control.yaml`의 `lm_angle_tolerance`·`arm_angle_tolerance`·`gripper_angle_tolerance`와
  **일치시켰습니다**(2026-10-27, 이전 LM 2도·팔 1도). 보드는 각 축이 이 범위 안에 들어올
  때까지 `state=1`(moving)을 보내므로, 한쪽만 바꾸면 호스트가 더 좁을 때 작업이 끝나지
  않습니다. 바꿀 때는 두 곳을 같이 바꾸고 `resetarm`으로 다시 올립니다.
- `PROFILE_ACCELERATION` 예시 raw 값은 10, 위치축 `PROFILE_VELOCITY`는 30입니다.
  실제 모델의 단위와 하중에 맞춰 조정하세요. 가속 프로파일과 PID는 DYNAMIXEL이 담당합니다.
  `DRIVE_MODE=0`으로 정방향·속도 기반 프로파일을 설정하고 물리 방향은 소프트웨어로 환산합니다.

clock은 각도를 0~2π로 접지 않습니다. 음수와 2π 초과 목표를 그대로 raw 위치로
변환하므로 호스트가 계산한 최단 회전 목표를 추종합니다. Extended Position의
범위는 모델에 맞춰 제한해야 합니다. 다회전 위치는 전원 차단 후 영구 보존되지
않으므로 재시작 시 clock 슬롯과 LM/팔의 실제 영점을 확인해야 합니다.
자동 원점 복귀나 리미트 스위치 입력은 구현하지 않았습니다. 레포에 해당 배선 정보가 없습니다.

호스트의 `arm_ik_mapping_confirmed`, `lm_mapping_confirmed`,
`gripper_mapping_confirmed`는 별도 설정이며 기존 기본값 `false`를 유지합니다.

## 정지 및 오류 처리

- 부팅 시 바퀴 목표는 0, 위치축 목표는 실제 측정 위치입니다. 피드백은 계속 나갑니다.
- 명령 전체의 채널 수·숫자·범위를 검사하고, 유효한 프레임만 한 번에 반영합니다.
  NaN/Inf, 초과 길이, 잘못된 CSV, 범위 초과는 버리고 watchdog도 갱신하지 않습니다.
- 유효 명령이 250 ms 끊기면 wheel은 0, 위치축은 그 순간의 위치를 유지합니다.
  이때 `state=2`이며 새 유효 명령으로 복구됩니다. 최초 명령 전에는 정상 idle입니다.
- 모터 통신 실패, 하드웨어 오류, 예상하지 않은 토크 해제는 **리셋까지 유지되는 오류**입니다.
  가능한 축은 속도 0/새로 읽은 위치 유지로 정지합니다. 위치를 읽거나 유지 목표를
  쓰지 못한 축에는 토크 해제를 시도합니다. 통신이 완전히 끊긴 모터의 정지는 보장할 수 없습니다.
  팔 축의 토크 해제 시 하중이 내려갈 수 있으므로 지지한 상태에서 점검해야 합니다.
- 상태는 `0=idle`, `1=using`, `2=fault`입니다. 위치축은 목표 오차와 모터 `MOVING`을
  함께 검사하며, wheel은 목표 속도 또는 실제 속도가 0 근처가 아니면 using입니다.
- 오류 피드백의 위치·전류는 마지막 완전한 샘플입니다. 최초 샘플 전 오류는 0입니다.
  호스트는 `state=2`일 때 도착/파지 완료를 false로 보고합니다. timeout 복구를 위해
  명령 송신은 계속하며, 펌웨어의 영구 오류에서는 수신 목표를 무시합니다.
- 한 줄 전체가 `RELAX`이면 보드의 모든 축 토크를 끕니다(손으로 옮기기용, 팔은 떨어집니다).
  피드백은 계속 나가고 전원이 유지되는 한 다회전 위치도 유지됩니다. 해제 중에는 CSV 명령을
  무시하고 `state=2`입니다. `HOLD`는 그 자리의 위치를 목표로 잡고 토크를 다시 켭니다.
  최초 명령 이후의 의도한 해제이므로 위의 "예상하지 않은 토크 해제" 오류가 아닙니다.
  호스트 단축 명령: `relaxarm` / `holdarm` (`~/.bashrc`, control_master 종료 후).

## 빌드 및 업로드

Arduino CLI, OpenCR 보드 패키지, Dynamixel2Arduino 라이브러리가 필요합니다.
저장소 루트에서 실행합니다. 공통 코드는 로컬 Arduino 라이브러리로 전달합니다.

```bash
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR \
  --library firmware/libraries/CraftMotorFirmware firmware/wheel_clock
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR \
  --library firmware/libraries/CraftMotorFirmware firmware/lm_arm

# control_master를 종료하고 USB 물리 포트별 보드를 확인한 뒤 실행
arduino-cli upload --fqbn OpenCR:OpenCR:OpenCR -p /dev/ttyACM1 firmware/wheel_clock
arduino-cli upload --fqbn OpenCR:OpenCR:OpenCR -p /dev/ttyACM0 firmware/lm_arm
```

`ttyACM0/1` 번호는 바뀔 수 있으므로 호스트 `control.yaml`의 포트는
`/dev/serial/by-path/`의 실제 경로를 사용하는 편이 좋습니다.

Arduino IDE에서는 `firmware/libraries/CraftMotorFirmware`를 스케치북의
`libraries/`에 설치하고 각각의 `.ino`를 열어 OpenCR 보드로 빌드하세요.

## 호스트에서의 펌웨어 로직 테스트

하드웨어 없이 CSV 경계값, 잘못된 프레임, millis 오버플로, raw 단위·방향 변환,
6채널 순서, signed 전류, timeout 정지·유지·복구, 모터 오류 래치를 검사합니다.
테스트의 모터 버스는 가짜이므로 실기기의 통신·기계 동작 검증은 별도로 필요합니다.

```bash
g++ -std=c++11 -Wall -Wextra -Werror \
  -Ifirmware/test/fakes -Ifirmware/libraries/CraftMotorFirmware/src \
  firmware/test/test_firmware.cpp -o /tmp/craft-test-firmware
/tmp/craft-test-firmware
```

OpenCR 통신 핀은 [ROBOTIS 공식 라이브러리 문서](https://github.com/ROBOTIS-GIT/Dynamixel2Arduino#supported-boards),
모터별 단위·모드·제한은 [ROBOTIS X-series 문서](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)를 참고하세요.
