#!/usr/bin/env bash
# Pi 에서 실행: 연결된 USB 시리얼 장치의 물리 포트 경로(KERNELS)를 출력
# LiDAR 를 하나씩 꽂아 보면서 어느 USB 구멍이 어느 KERNELS 인지 확인하세요.
#   bash find_lidar_ports.sh  (또는 ros2 run lidar_processor find_lidar_ports.sh)
shopt -s nullglob
devs=(/dev/ttyUSB* /dev/ttyACM*)
[ ${#devs[@]} -eq 0 ] && { echo "USB 시리얼 장치 없음"; exit 1; }
for d in "${devs[@]}"; do
  vid=$(udevadm info -q property -n "$d" | sed -n 's/^ID_VENDOR_ID=//p')
  pid=$(udevadm info -q property -n "$d" | sed -n 's/^ID_MODEL_ID=//p')
  model=$(udevadm info -q property -n "$d" | sed -n 's/^ID_MODEL=//p')
  # 장치 바로 위 USB 인터페이스 부모의 KERNELS (예: 1-1.3:1.0) 에서 ':1.0' 앞부분이 포트 경로
  kern=$(udevadm info -a -n "$d" | grep -m1 -oP 'KERNELS=="\K[0-9]+-[0-9.]+(?=:)')
  printf "%-14s VID:PID=%s:%s  KERNELS=%-8s %s\n" "$d" "$vid" "$pid" "$kern" "$model"
done
echo
echo "* 10c4:ea60 (CP2102) = LDS-03 LiDAR,  0483:5740 = OpenCR"
ls -l /dev/lidar_front /dev/lidar_rear 2>/dev/null || echo "* /dev/lidar_* 심볼릭 링크 아직 없음"
