# 주행·라이다 RViz 시뮬레이션

```bash
bash ~/craft/demo/driving_sim/run.sh
```

별도 ROS domain **216**에서 실행한다. 실제 `planning_master`와 실제
`LateralController`/`LongitudinalController`를 사용하고, 하드웨어 제어 노드·드라이버·카메라는
시작하지 않는다. RViz를 닫으면 이 시뮬 프로세스들도 종료한다.

화면에서 다음을 비교한다.

| 표시 | 의미 |
|---|---|
| 회색/검정 지도 | 6×4 m 테스트 공간과 알려진 장애물 |
| 지도 주변 청록 영역 | 실제 planner가 계산한 차체 여유 공간 |
| 회색 직선 | 요청한 시작→목표 직선 참고선 (A* 경로가 아님) |
| 보라색 경로 | 해당 작업의 최초 계획 |
| 청록색 경로 | planner가 현재 발행한 주행 경로/로컬 구간 |
| 노란색 선 | 모사 로봇이 실제로 이동한 궤적 |
| 빨간/파란 점 | 전방/후방 라이다의 거리 측정 끝점 |
| 초록 점 | 지도에 없으며 3프레임 유지된 라이다 측정점 → planner 장애물 입력 |
| 주황 박스 | 주행 중 나타나는, 사전 지도에는 없는 장애물 |

네 목표를 순회하고 반복한다. 0.8m 이동 후 현재 경로 앞에 미지 장애물이 나타나서,
스캔→새 장애물 입력→정지/재계획→우회를 관찰할 수 있다. 한 순회가 끝나면 동적 장애물을
초기화해서 다음 순회에서도 다시 감지하게 한다. RViz의 **2D Goal Pose**는 planner가
요청을 기다리는 동안 쓸 수 있다. 현재 발행 경로를 표시하며, 내부 A* 탐색 트리나
전체 전역 경로를 별도로 재구성해 표시하는 기능은 넣지 않았다.

```bash
# 센서 수학 검증 (창/ROS 노드 없이)
bash ~/craft/demo/driving_sim/run.sh --verify-only

# 창 없이 한 바퀴: 별도 터미널에서 실행 결과를 검증할 수 있다.
bash ~/craft/demo/driving_sim/run.sh --no-rviz --once
python3 ~/craft/demo/driving_sim/verify_run.py --timeout 180
```

`--once`는 자동 반복을 끄고 도착한 상태를 유지한다. 실행 중 같은 domain216의
`/drive_sim/command` (`std_msgs/String`)에 `pause`, `play`, `obstacle`을 보낼 수 있다.
초기화는 RViz를 닫고 런처를 다시 실행한다. `DRIVE_SIM_DOMAIN_ID`로 다른 격리 도메인을
지정할 수 있다. 실제 로봇 도메인과 섞지 않도록 기본216을 사용한다.

## URDF와 모사 범위

- 모델은 `src/craft_description/urdf/v307_full_robot.urdf`. 팔은 고정하고 바퀴 관절은 회전한다.
- URDF의 `front_laser`: `(+0.23867,+0.00002,+0.0567)`, yaw0;
  `rear_laser`: `(-0.06767,0,+0.0507)`, yawπ. 바닥은 base_link 아래0.034346m이다.
  실물 드라이버의 `laser_front`/`laser_rear` 보정 TF와 다른 **CAD 프레임**이다.
- 각 라이다는 바깥쪽180°, 361개 광선, 10Hz 목표, 최소0.1m/최대8m로 모사한다.
  높이가 센서보다 낮은 동적 물체는 보이지 않고, 앞 물체가 뒤 물체를 가린다.
  거리 노이즈3mm/누락1%를 넣고, 원점을 옮겨 720개 각도 bin으로 병합한다.
- 바퀴 운동학은 현재 `control.yaml`의 반지름0.033m/track0.289m/유효 간격 약0.333m,
  속도·가속도 제한을 사용한다. CAD 반지름0.034346m/track0.288m/축간격0.196m와는
  별도이며, 차체 질량·슬립·바닥 접촉·센서 반사/자가차폐/스캔 중 움직임 왜곡은 모사하지 않는다.
- 위치 추정은 **정답 위치**이며 SLAM 검증이 아니다. IMU는 yaw rate/중력과 작은 잡음만 낸다.
- 차체 충돌은 STL의 XY 경계 직사각형과 지도 셀/장애물의 겹침으로 검사한다.
  CAD 앞 코너가 약0.326m이므로 시뮬에서만 planner 반경을 **0.34m**로 설정했다.
  실제 프로젝트의 설정·URDF·보정 파일은 바꾸지 않는다.
- `world.yaml`의 정적 물체로 `generate_map.py`가 지도를 생성한다. 동적 물체는 planner 지도에
  넣지 않고 오직 모사 거리 측정으로 전달한다. 기존 실물 경기장 맵이 아닌 테스트 공간이다.

## 검증 기록

`runs/verification.json`: 실제 C++ planner/controller로 네 목표 도착, 충돌0,
미지 장애물 때문에 추가 전역 계획1회, 경로 갱신5회. 실제 계획 계산은 첫 검증에서
3.8~16.7ms였고, 다른 실행에서도 비슷한 수준이었다.

`test_sensors.py` 6개: 벽 거리/앞 물체 가림, 후방 센서 좌표 회전, 낮은 물체 제외,
노이즈/누락/NaN, 최소 거리, 센서 원점을 반영한 병합, 회전된 지도와 차체 충돌을 확인한다.

- `runs/geometry.json`: URDF 및 CAD/제어 치수 비교
- `runs/lidar_validation.json`: 실제 ROS 스캔 수신·프레임·광선 수 확인
- `runs/validated_run.json`: 검증 시점의 전체 실행 상태
- `runs/planning.log`: A* 전개 수/계획 시간/로컬 경로 갱신 로그
- `runs/rviz_initial_plan.png`, `runs/rviz_replanning.png`: 실제 RViz 화면
