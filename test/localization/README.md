# 위치추정·IMU 실물 확인 (TODO)

**상태: 미실시.** 2026-10-29에 sim으로만 검증한 기능들을 실물에서 확인한다.
sim 수치는 `src/craft_perception/README.md`(IMU 처리, 위치추정 감시)와 각 설정 주석에 있다.
위에서부터 순서대로 한다. 뒤 실험이 앞 결과(특히 2, 5)에 기대기 때문이다.

## 0. 준비
- [ ] 로봇 PC의 `~/.bashrc`를 새 버전으로 바꾼다. 수정본은 개발 PC의 `~/Downloads/craft_bashrc.imu`에 있다.
  `checkwheel`/`checkarm`가 새 패킷(12/15칸)과 IMU 상태를 읽는다.
- [ ] `resetall`로 두 보드 펌웨어를 올린다. 두 보드 모두 IMU를 보낸다. 업로드 직후 5초는 로봇을 움직이지 않는다.
- [ ] `checkwheel`, `checkarm`: 두 보드 모두 `imu_ok 1`, `|a|` 약 1 g, `upright`인지 확인한다.

## 1. IMU 보정 로그
- [ ] control과 perception을 켜고 2초 이상 정지한다.
- [ ] 로그에 `wheel_clock IMU calibrated`와 `lm_arm IMU calibrated`가 뜨는지 본다.
- [ ] 기울기와 칩 오프셋을 기록한다. 기울기는 수 도 이내, 오프셋은 데이터시트상 ±0.09 rad/s 이내여야 한다.

## 2. IMU 정지 drift (IMU가 정말 정확한가)
- [ ] 정지 상태로 60초 두고 `/odometry/local` yaw 변화를 본다. 정지 중 0 처리 때문에 거의 0이어야 한다.
- [ ] 바닥 테이프 표시에 맞춘 뒤 제자리 5바퀴(1800°)를 돌고 표시와의 각도 차를 잰다.
  자이로 배율이 맞는지 보는 것이다. 자세한 절차는 [../gyro_scale/README.md](../gyro_scale/README.md).
- [ ] 같은 실험을 SLAM을 켜고 반복한다. 로그 `IMU correction from SLAM yaw`의 배율이 위 측정값 쪽으로
  가는지 본다. 바이어스는 ±0.005, 배율은 ±5% 상한 안에서 안정돼야 한다.

## 3. 회전 프로파일과 튀는 값 필터
- [ ] 실제 주행 시나리오대로 회전시킨다. 로그 `IMU samples dropped by the spike filter`가 계속 늘면,
  실제 회전 가속이 Waffle Pi 기준(3.2 rad/s²)보다 크다는 뜻이다. 그때는 `perception.yaml`의
  `imu.max_yaw_accel`을 실제 값에 맞게 올린다(control 가속 설정을 확정한 뒤).
- [ ] 정상 주행 중 `OpenCR IMUs disagree` 경고가 뜨는지 본다. 뜨면 두 칩의 배율 차이가 8%보다 큰 것이다.
  2번 결과로 `imu.max_disagreement_ratio`를 정한다.

## 4. 예비 IMU 전환
- [ ] 주행 중에 wheel_clock IMU가 끊기는 상황을 만든다. 로그 `primary IMU wheel_clock unavailable: using
  backup lm_arm`이 뜨고 위치가 유지되는지 본다. 방법은 아직 없다(펌웨어에서 imu_ok=0을 강제하는 디버그
  옵션이 필요하다).

## 5. 스캔-지도 점수 기준 (가장 중요)
- [ ] 경기장을 정상적으로 몇 바퀴 돈다. 10초마다 찍히는 `scan-to-map score at the EKF pose`를 모아서
  최저값과 하위 5%를 낸다. sim은 최저 0.55, 하위 5% 0.73이었다.
- [ ] `perception.yaml`의 `localization.relocalize.min_score`를 정상 최저값보다 충분히 낮게 맞춘다.
  지금 값은 0.5다. 너무 높으면 정상 주행 중 오경보로 재위치가 돌고, 최악에는 lost(planning Fault)가 된다.

## 6. 밀림 → 재위치 / lost
- [ ] 주행 중 로봇을 옆으로 약 20 cm 민다. 로그 순서가 `ok -> dead_reckoning` → `relocalized after dead
  reckoning` → `ok`인지 본다. sim은 재위치에 16.5 cm, 3° 보정, 38 ms였다.
- [ ] 약 60 cm 민다. `relocalization failed ... localization lost`가 뜨고 planning이 Fault로 가는지 본다.
  RESET 전에는 다시 움직이지 않아야 한다.
- [ ] 재위치 계산 시간(로그의 ms)을 로봇 PC에서 기록한다.

## 7. EKF 모드 대 SLAM 모드
- [ ] 같은 경로를 `localization:=ekf`(기본)와 `localization:=slam`으로 몇 바퀴씩 돈다.
- [ ] 정해진 지점(테이프 표시)에서 실제 위치와 `/perception/state/odom`의 차이를 잰다.
- [ ] 쓰레기 스캔이 생기는 상황(LiDAR 앞을 사람이 지나감)을 넣어 비교한다.

## 8. 로봇 PC 부하와 주기
- [ ] 다음 토픽 주기를 잰다. sim에서는 각각 50, 50, 50, 50, 3.6, 4.6 Hz였다.

  ```bash
  ros2 topic hz /perception/imu/data /odometry/local /odometry/global /perception/state/odom /perception/lidar/pose /perception/localization/state
  ```
- [ ] `top`으로 perception_master, slam_toolbox, ekf_node의 CPU를 본다. sim 노트북에서 perception_master는 약 10%였다.
