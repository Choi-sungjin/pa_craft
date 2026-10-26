# OpenCR 구분 (Jetson)

실측 결과 두 OpenCR의 VID:PID는 `0483:5740`, USB 시리얼은 모두
`FFFFFFFEFFFF`입니다. `/dev/serial/by-id`는 충돌하므로 구분에 사용하지 않습니다.

| 현재 장치 (변경 가능) | ID_PATH | udev 별칭 |
| --- | --- | --- |
| `/dev/ttyACM0` | `platform-3610000.usb-usb-0:2.2.2:1.0` | `/dev/opencr_wheel_clock` |
| `/dev/ttyACM1` | `platform-3610000.usb-usb-0:2.1:1.0` | `/dev/opencr_arm_lm` |

별칭은 보드가 아닌 물리 USB 포트를 식별합니다. 허브/케이블 위치를 고정하고 라벨을 붙입니다.
2026-10-26 하드웨어 변경: arm/LM은 `2.1`(ttyACM1), wheel/clock은 `2.2.2`(ttyACM0)입니다.
`ttyACM0/1` 번호만으로 역할을 판단하지 않습니다.

저장소 루트에서 설치합니다. 제어 노드와 업로더를 먼저 종료합니다.

```bash
sudo install -m 644 config/udev/99-craft-opencr.rules /etc/udev/rules.d/99-craft-opencr.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match='ttyACM*'
udevadm settle
ls -l /dev/opencr_wheel_clock /dev/opencr_arm_lm
```

`src/craft_control/config/control.yaml`의 `arm_lm_port`와
`clock_wheel_port`에 해당 별칭을 반영했습니다. 두 케이블을 재연결하거나 재부팅한 뒤
각 별칭이 같은 물리 보드를 가리키는지 확인합니다. 부트로더/DFU 모드에는 적용되지 않습니다.
USB 포트를 바꾸면 `udevadm info -q property -n /dev/ttyACM0` 등으로 새 ID_PATH를
확인하고 규칙을 수정합니다. 권한은 기존 시스템의 dialout 정책을 따릅니다.

udev 규칙 없이도 `/dev/serial/by-path/`의 해당 경로를 YAML에 직접 지정할 수 있습니다.

# LiDAR 전·후방 구분 (Jetson)

LDS-03 두 대는 VID:PID(`10c4:ea60`)와 `/dev/serial/by-id`가 같아서 물리 USB 포트로 구분합니다.
전방 LiDAR를 손으로 가려 확인했습니다 (2026-10-24).

| 현재 장치 (변경 가능) | KERNELS | udev 별칭 |
| --- | --- | --- |
| `/dev/ttyUSB1` | `1-2.4` | `/dev/lidar_front` |
| `/dev/ttyUSB0` | `1-2.1` | `/dev/lidar_rear` |

`src/lidar_processor/udev/99-craft-lidar.rules`는 Pi 기준이므로 Jetson에서는 이 파일을 설치합니다.

```bash
sudo install -m 644 config/udev/99-craft-lidar.rules /etc/udev/rules.d/99-craft-lidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match='ttyUSB*'
udevadm settle
ls -l /dev/lidar_front /dev/lidar_rear
```

포트를 바꾸면 `bash src/lidar_processor/scripts/find_lidar_ports.sh`로 KERNELS를 다시 확인합니다.
