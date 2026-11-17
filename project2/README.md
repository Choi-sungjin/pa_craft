# 프로젝트 2 — 주사위 기억 로봇 제출 자료

기존 구현, 문제 예제, 실물 개발 시험의 근거를 제출 항목별로 정리한 자료임. 전체 경기 완주 또는 최종 제출 완료를 의미하지 않음. 검토 기준일은 2026-11-17임.

## 자료를 읽는 순서

| 자료 | 의미 | 현재 내용과 상태 |
| --- | --- | --- |
| [문제 패키지](problem/problem.md) | 무엇을 풀어야 하는지 정함 | 공개 Q1의 입력·12개 배치·정답·소프트웨어 검증을 수록함. 팀 문제 확정과 다른 팀 검토는 미확인임 |
| [시스템 설계](docs/architecture.md) | 어떤 노드가 어떤 정보를 주고받는지 설명함 | 현재 코드의 동작과 미구현 단계를 구분함 |
| [시험 결과](results/test-results.csv) | 실제로 무엇을 확인했는지 기록함 | 문제 계산, 기존 빌드 기록, 실물 개발 시험을 구분함 |
| [개발 시험 보고서](docs/report.md) | 인식·집기·실패 과정을 설명함 | 가로 3장 × 세로 2장 보고서 이미지와 원본 로그를 포함함 |
| [공식 1회차](results/round-01/README.md) / [2회차](results/round-02/README.md) | 회차별 원점수와 완주 여부를 판정함 | 공식 경기로 확인된 자료가 없어 미확인 상태임 |
| [재현 시험](results/reproduction.md) | 다른 팀원이 같은 환경에서 실행했는지 확인함 | 실행자·환경·결과 증빙 미확인임 |
| [개인 기여](docs/contributions.md) | 구현·시험·리뷰·설명을 개인별로 연결함 | 노션 개인 일지 87개 항목을 검토하여 4명별 기여와 미확인 근거를 구분함 |
| [오류 목록](docs/issues.md) | 실패 원인과 남은 작업을 추적함 | 실제 GitHub Issue와 문서상 미등록 항목을 구분함 |

선택·수행한 도전 과제의 증빙이 확인되지 않아 `challenge/`는 생성하지 않음. 선택 과제는 필수 평가 100점에 가산되지 않음.

## 문제 입력과 실행 결과의 구분

`problem/signals.json`은 **운영자가 제시할 Q1 정답 입력**, `problem/answer.json`은 **그 입력에 대한 정답**임. `results/.../signal.json`은 **로봇이 관측한 결과**, `allocation.json`은 **계산한 계획**, `verification.json`은 **배치 후 새 관측의 근거**를 담는 위치임. 계획을 복사하여 재검증 결과로 사용하지 않음.

Q1은 R4·Y2·G6 / P2-01임. 개발 시험 `integ21_1116_1429`의 기록은 R4·Y4·G4 / P2-02임. 서로 다른 자료이므로 Q1 정답으로 해당 시험의 정확도나 점수를 판정하지 않음.

## 실행 코드와 환경

실행 코드는 저장소의 [src](../src), [firmware](../firmware), [tools](../tools)에 유지함. 문서 폴더에 코드 사본을 중복 생성하지 않음. 노드별 구성과 의존성은 [저장소 README](../README.md), 설치본 패키징은 [배포 안내](../docs/DEPLOYMENT.md)를 따름.

미션·집기 코드 검토 기준은 `1724eec09a53c790bb9cced066fea784eadd22c9`이며, 게시 직전 원격 변경을 반영한 소스 기준은 `a732c8d7429cb39b011176753dcbc428c64c9dfb`임. 두 버전 사이 미션·집기 핵심 코드는 동일함. 기존 설치본의 소스 기준은 `5794d640ec75854fde1848a3d7663505ad2c0a90`임. 11월 16일 개발 시험을 실행한 정확한 커밋은 확인되지 않았으며 두 커밋 중 하나로 소급 지정하지 않음.

확인한 개발 호스트는 Ubuntu 24.04.4 LTS / aarch64 / Python 3.12.3 / ROS 2 Lyrical 환경임. 실물 구성은 Jetson, 어퍼캠, RealSense D435 그리퍼캠, LiDAR, OpenCR, 팔·LM 레일·그리퍼·클락 적재부임. 장치 일련번호와 모든 네이티브 라이브러리·펌웨어 버전은 별도 재현 기록에 보완해야 함. 원격 변경 반영으로 현재 소스에는 `craft_voice`도 포함되지만, 기존 9패키지 설치본에는 포함되지 않음. 음성 기능의 별도 설정·실행은 [음성 패키지 안내](../src/craft_voice/README.md)를 따름. 다른 GPU에서는 TensorRT 엔진 호환성을 확인해야 함.

새 환경에서 저장소 루트 기준으로 의존성을 준비한 뒤 빌드함.

```bash
source /opt/ros/lyrical/setup.bash
vcs import src < craft.repos
rosdep install --from-paths src --ignore-src -r -y
bash tools/build_workspace.sh
```

사용자가 장비 초기화와 주변 확인을 마친 후 실행하는 명령임. **현재 기본값 `drive_only: true`는 주행 확인용이며 전체 경기 실행 완료 설정이 아님.** `false`로 변경하는 것만으로 미구현 단계가 완성되지 않음.

```bash
source install/setup.bash
ros2 launch craft_bringup craft.launch.py mission:=true
```

별도 터미널에서 미션 시작 시 사용하는 명령임.

```bash
source install/setup.bash
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: START}'
```

정지할 때 미션과 계획 계층에 STOP을 전달함.

```bash
source install/setup.bash
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: STOP}'
ros2 topic pub --once /planning/command std_msgs/msg/String '{data: STOP}'
```

정지와 원인 제거 후 새 시험을 준비할 때 계획·미션 상태를 각각 초기화함. 공식 경기에서 수동 수정 후 같은 회차를 이어가는 용도로 사용하지 않음.

```bash
source install/setup.bash
ros2 topic pub --once /planning/command std_msgs/msg/String '{data: RESET}'
ros2 topic pub --once /mission/command std_msgs/msg/String '{data: RESET}'
```

물리 초기화는 해당 문제의 퍽 ID·색상·모양·시작 좌표, 빈 L1~L3, 출발 자세, 바코드, 신호 시퀀스를 대조하는 작업임. 소프트웨어 RESET만으로 물리 배치가 초기화되지는 않음. 현재 지도 설정과 문제지 좌표의 차이는 [출처 검토](docs/source-review.md)에 기록했으며 설정값을 임의로 바꾸지 않았음.

## 확인된 결과와 남은 제출 조건

개발 시험은 로그상 집기 5회 완료 후 S3 두 번째 퍽을 집는 단계에서 LM 레일 범위 초과로 중단됨. 목적지 배치·새 관측 검증·복귀 성공 근거는 없음. `ObservePucks`, `PlacePuck`, `VerifyLocation`은 현재 시간 경과로 성공을 반환하는 미구현 단계임.

공식 1·2회차 증빙, 다른 팀 문제 검토, 다른 팀원의 재현 시험, 개인별 PR·리뷰·시험·설명 근거가 추가되어야 최종 제출 여부를 판정할 수 있음. 미확인 점수는 0점으로 확정하지 않고 비워 둠.

보고서 이미지와 작은 원본 로그는 저장소에서 열 수 있음. 전체 영상과 배그는 로컬에만 있으며 [영상 목록](results/development/integ21_1116_1429/video-manifest.json)에 크기·해시·원본 경로를 기록함. **외부 접근 가능한 영상 링크는 아직 없음.** 대용량 원본을 Git에 직접 넣지 않았음.

## 버전과 제출 메시지

현재 정리 대상 브랜치는 `arm-teached`임. 원격에서 먼저 이루어진 `main`→`arm-teached` 병합(`a732c8d`)을 반영했음. 이번 제출 자료를 `main`으로 병합하는 단계와 최종 제출 태그는 아직 완료하지 않았음. 문제 확정·검토 후 `project2-problem-submit`, 최종 자료 확인·`main` 병합 후 `project2-submit`으로 버전을 고정해야 함.

제출 메시지에 사용할 현재 정보는 다음과 같음.

- 팀명: A팀 크래프트 (최성진·한지훈·심규진·박준명)임.
- 저장소: https://github.com/Choi-sungjin/pa_craft
- 버전: 이 문서를 포함한 `arm-teached` 커밋. 최종 제출 태그는 미생성임.
- 문제·배치: `project2/problem/`의 Q1 검증 예제임. 팀 문제 확정 전임.
- 실행 안내: 이 README 및 루트 배포 안내임.
- 영상·로그: `project2/docs/report.md`, `project2/results/development/integ21_1116_1429/`임. 영상 공개 링크 보완 필요함.
- 개인 근거: `project2/docs/contributions.md`임. 개인별 연결은 확인된 출처에 한해 반영함.
