# 4WD 제자리 회전 실험

**상태: 미실시.** 바퀴 4개 모두 구동하도록 바꾼 뒤(2026-10-30) 네 바퀴가 제대로 돌고 제자리 회전이
되는지 확인한다. 앞바퀴 ID 1(왼쪽)·2(오른쪽), 뒷바퀴 ID 3(왼쪽)·4(오른쪽), clock은 ID 5다(예전 3).

## 원리

- 펌웨어(`firmware/wheel_clock/config.h`의 `kFollowers`)가 뒷바퀴를 같은 쪽 앞바퀴 채널의 follower로
  돌린다. 호스트 명령은 그대로 `wheel_l,wheel_r,clock` 3채널이다. 피드백 `wheel_l`/`wheel_r`는 그쪽
  **앞·뒤 모터 평균**이다.
- 그래서 피드백만으로는 뒷바퀴 방향 부호가 틀린 것을 **알 수 없다**. 부호가 틀리면 모터는 반대로 돌고
  피드백도 같은 부호로 뒤집혀 정상처럼 보인다. 뒷바퀴 방향(ID 3 `+1`, ID 4 `-1`)은 아직 추정값이라,
  바퀴를 띄우고 눈으로 확인하는 단계(1단계)를 먼저 한다.
- 바닥 회전은 `rotate.py`가 바퀴 OpenCR에 50 Hz로 직접 명령한다(`slip_test.py`와 같은 연결).
  바퀴 속도 = ∓ω·sep/2/r이고, 바퀴 오도메트리 yaw가 목표 각도에 닿으면 멈춘다. IMU(gz − 정지 오프셋)를
  적분한 값이 실제 회전이다.
- **wheel/IMU**: 4WD 스키드 스티어는 제자리 회전에서 바퀴가 옆으로 긁히므로 1보다 크다. 정상이면 1.2~2
  정도이고, 앞뒤 축간거리 L이 좌우 간격 W에 비해 길수록 커진다. 이 비율 × 0.289 m(`wheel_track`)가 **유효 바퀴 간격**이다.
- **tracking**: 가속 램프(약 0.7 s)가 끝난 뒤 각 쪽 피드백 평균 ÷ 명령이다. 0.9보다 낮으면 바닥 마찰을
  이기지 못하는 것이다(토크 부족/스톨). 그 상태로 오래 두면 모터가 과부하 fault로 간다.

## 준비

1. **ID**: `wizard wheel`로 Wizard에서 버스를 스캔한다. ID 1~5가 모두 보이고 모델(바퀴 XM430-W210,
   clock XM430-W350)이 맞는지 확인한다. clock을 3에서 5로 바꿨는지도 본다. Wizard를 닫는다.
2. **펌웨어**: 로봇을 가만히 두고 `resetwheel` 한 번. 부팅 때 ID 3·4 ping이 실패하면 fault가 난다.
3. `checkwheel`: 상태 0, `imu_ok 1`.
4. control_master는 **끈다**(포트가 사용 중이면 스크립트가 거부한다). 아래 명령은 보드 잠금을 잡고 실행한다.
5. 통제 조건: 팔은 접은 자세, 퍽 없음, 같은 바닥. 시작·끝 배터리 전압을 기록한다.
   축간거리 L(앞·뒤 바퀴 축 중심 사이)과 좌우 간격 W를 한 번 줄자로 잰다.

## 1단계: 바퀴 띄워서 방향 확인 (필수, 먼저)

로봇 밑에 받침을 넣어 **네 바퀴가 모두 바닥에서 뜨게** 한다. 바퀴마다 테이프로 표시를 붙이면 잘 보인다.

```bash
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/rotate.py --lifted
```

다섯 단계를 2초씩 돈다: 왼쪽 앞으로, 왼쪽 뒤로, 오른쪽 앞으로, 오른쪽 뒤로, 왼쪽 회전(왼쪽 뒤로, 오른쪽 앞으로).
단계마다 스크립트가 피드백 평균을 판정(`ok`/`CHECK`)하고, 그쪽 **앞·뒤 바퀴가 둘 다** 맞는 방향으로
돌았는지 `y`/`n`으로 묻는다. 명령하지 않은 쪽은 멈춰 있어야 한다.

| 증상 | 원인 | 조치 |
| --- | --- | --- |
| 뒷바퀴만 반대로 돈다 | follower 방향 부호 | `config.h` `kFollowers`의 그 바퀴 `direction` 부호를 바꾸고 `resetwheel` |
| 엉뚱한 쪽 바퀴가 돈다 | ID가 바뀜(3↔4 등) | Wizard에서 ID 재설정 |
| 뒷바퀴가 안 돈다, 피드백 약 절반 | follower 토크/배선 | `checkwheel`, 배선, Wizard에서 상태 확인 |
| 앞바퀴부터 반대 | 기존 설정 문제 | `test/README.md`의 방향 표부터 다시 확인 |

`lifted check PASSED`가 나올 때까지 고친다. 결과는 `logs/<시각>_lifted_<ω>.csv`에 남는다.
방향이 확정되면 `test/README.md` 방향 표의 "(확인 필요)"를 지운다.

## 2단계: 바닥 첫 회전 (저속, 안전 확인)

바닥에 내려놓고 주변 0.5 m를 비운다. 이상하면 바로 Ctrl+C(0.5 s 정지 명령 뒤 종료).

```bash
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/rotate.py --omega 0.2 --turns 1 --note "first"
```

`tracking`이 양쪽 0.9 이상이고 fault·timeout 없이 끝나면 3단계로 간다.

## 3단계: 본 실험

로봇 중심(좌우 바퀴 축 사이 가운데)을 바닥에 테이프 십자로 표시하고, 차체 앞 방향을 테이프 선으로 표시한다.
한 회차 = 90°씩 4번(바퀴 오도메트리 기준, 2초 IMU 오프셋 측정 포함). 회차마다 로봇을 십자에 다시 맞춘다.

| ω [rad/s] | 바퀴 속도 [rad/s] | 방향 | 횟수 |
| --- | --- | --- | --- |
| 0.3 | 1.31 | CW, CCW | 각 3 |
| **0.6 (control 값)** | 2.62 | CW, CCW | 각 3 |
| 1.0 | 4.36 | CW, CCW | 각 3 |

```bash
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/rotate.py --omega 0.6 --turns 4 --note "tile 12.1V"
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/rotate.py --omega 0.6 --turns 4 --ccw --note "tile 12.1V"
```

`--note`에는 바닥 종류와 배터리 전압을 적는다. 회차마다 `logs/<시각>_rotate_<ω>.csv`(50 Hz 원본)가 남고,
회전마다 한 줄씩 `logs/rotate_results.csv`에 쌓인다(wheel/IMU, 유효 간격, tracking, 소요 시간).

회차가 끝나면 손으로 잰다:
- **중심 이동**: 십자에서 로봇 중심이 얼마나 벗어났는지(mm). 제자리 회전이면 작아야 한다.

## 앞바퀴만 구동 (2WD 비교, 선택)

기본 설정은 4WD(`kRearWheelsDrive = true`)다.

`firmware/wheel_clock/config.h`의 `kRearWheelsDrive`를 `false`로 바꾸고 `resetwheel`. 앞바퀴(ID 1, 2)만
구동하고 뒷바퀴(ID 3, 4)는 토크를 꺼서 끌려간다. 피드백은 앞바퀴 값만이다. 같은 명령으로 3단계 조건 일부(ω 0.6, CW·CCW
각 3회)를 돌려 4WD와 wheel/IMU, tracking을 비교한다. `--note`에 `2wd`를 적는다.

```bash
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/rotate.py --omega 0.6 --turns 4 --note "2wd tile 12.1V"
```

4WD로 돌아갈 때는 `true`로 바꾸고 `resetwheel`.

## 합격 기준

- [ ] 1단계 `lifted check PASSED`(네 바퀴 방향과 피드백).
- [ ] 바닥 전 회차에서 fault·timeout 없음.
- [ ] tracking 양쪽 0.9 이상(ω 0.6). 1.0에서 떨어지면 그 값이 실사용 상한이다.
- [ ] 같은 조건 wheel/IMU 편차 5% 이내(반복성), CW와 CCW 차이 10% 이내(좌우 대칭).
- [ ] 4회 회전 뒤 중심 이동 30 mm 이내.

## 해석과 반영

- 2WD 때(2026-10-29~30)는 바퀴가 376° 돌 때 IMU가 1~85°밖에 안 돌았다(그 로그는 하드웨어 변경으로
  2026-10-31에 삭제). 4WD로 이 값이 안정된 1.2~2 근처로 오면 개선된 것이다.
- wheel/IMU가 크고 일정하면 정상적인 스키드 스티어다. ω 0.6 회차들의 wheel/IMU 중앙값을 `control.yaml`의
  `skid_steer_factor`에 넣는다(지금은 0 = 기하 추정 1 + (0.230/0.289)² ≈ 1.63). control의 Pure Pursuit,
  제자리 정렬 회전, 바퀴 오도메트리 yaw가 모두 이 유효 간격을 쓴다. 직진에는 영향이 없다.
- 반복마다 wheel/IMU가 크게 흔들리면 바닥 접지가 고르지 않은 것이다(4바퀴 중 하나가 뜸). 차체 수평과
  바퀴 높이를 확인한다.
- CW/CCW 비대칭이면 한쪽 follower 방향·게인이나 무게중심 치우침을 의심한다.
- tracking이 ω에 따라 떨어지면 제자리 회전 속도 상한을 그 아래로 잡는다(`wheel_rotate_angular_speed`).

## 제자리 회전 맵: 바퀴 속도차 → 실제 각속도 (`turn_map.py`)

**상태: 미실시.** 하드웨어 변경 전 측정(2026-10-30, 로그는 10-31 삭제)에서 wheel/IMU가 같은 ω에서도
1.2~7.2로 크게 흔들렸다. 그래서 유효 간격 하나(`skid_steer_factor`)로 계산하지 않고, 명령하는 바퀴 속도차 D = wr − wl [rad/s]와
IMU로 잰 실제 각속도의 관계를 직접 표로 만든다. 이 표를 거꾸로 보간하면 원하는 각속도에 해당하는 D가 나온다.
바닥이 고르지 않으므로 같은 실험을 3번(실험마다 다른 바닥 지점에서) 한다. **같은 D에서 지점마다 다른 각속도가 나오면 이 방식은
폐기한다**(아래 판정).

### 원리

- 한 시행은 wl = −s·D/2, wr = +s·D/2(s = +1 CCW, −1 CW)인 제자리 회전이다. 펌웨어 가속 램프(D/2 ÷ 3.75 rad/s²)와
  안정화 0.5 s가 지나면 **3 s 동안 IMU yaw rate(gz − 오프셋)의 평균**을 낸다. 오프셋은 시행 직전에 정지 상태에서 1 s 동안 잰다.
  바퀴 반지름이나 간격은 쓰지 않는다. 비교용 `geo/imu`(= r·D_fb/track ÷ IMU)만 함께 기록한다.
- 시행 순서는 실험·회차마다 무작위로 섞는다. 배터리 전압 하강이나 모터 발열이 큰 D에만 몰려 결과를 기울이지 않게 하려는 것이다.
- 표준편차 `±`가 크면 스틱슬립이다(돌다 서다를 반복). 데드밴드 근처에서 그렇게 나온다.

### 준비

1. 1단계(`rotate.py --lifted`)를 통과한 상태, `resetwheel`, `checkwheel`에서 상태 0과 `imu_ok 1` 확인.
2. 실험할 바닥 지점 3곳을 테이프 십자로 표시한다. 지점마다 반경 0.5 m를 비운다.
3. 팔은 접은 자세, 퍽 없음, control_master 끔. 지점마다 시작과 끝 배터리 전압을 `--note`에 적는다.

### 시나리오

| 단계 | 명령 | 내용 | 소요(추정) |
| --- | --- | --- | --- |
| 1. 실험 1 | `turn_map.py run --spot 1 --note "타일 12.3V"` | 12단계 × CW/CCW × 3회 = **72회**. 첫 실행 때 시행 순서 계획(`logs/turn_map_plan.csv`)을 만든다 | 약 9분 |
| 2. 실험 2 | `run --spot 2 ...` | 같은 실험(순서만 다름) | 약 9분 |
| 3. 실험 3 | `run --spot 3 ...` | 같은 실험(순서만 다름) | 약 9분 |
| 4. 분석 | `turn_map.py fit` | 지점 의존 판정(KEEP/BORDERLINE/DISCARD), 단조 보간표 `logs/turn_map_table.yaml` | — |

실험 1회의 시행 조건(순서는 무작위):

| D = wr − wl [rad/s] | 바퀴 속도 ±D/2 [rad/s] | 기하 계산 ω = r·D/track [rad/s] | 목적 |
| --- | --- | --- | --- |
| 0.5, 1.0, 1.5, 2.0 | 0.25 ~ 1.0 | 0.06 ~ 0.23 | 데드밴드(정지 마찰), 스틱슬립 구간 |
| 3.0, 4.0, 5.0, 6.0 | 1.5 ~ 3.0 | 0.34 ~ 0.69 | 정렬 회전 사용 구간(`wheel_rotate_angular_speed` 0.6 근처) |
| 8.0, 10.0, 12.0, 14.0 | 4.0 ~ 7.0 | 0.91 ~ 1.60 | 고속 회전, 상한 확인(`max_wheel` 7.8) |
| 각 D × CW, CCW × 3회 | | | 좌우 대칭, 반복성(지점 판정의 기준 잡음) |

기하 계산 ω는 슬립이 없을 때의 값이다. 실제로는 스키드 때문에 이보다 작게 나온다.

```bash
cd ~/craft/test/drive
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/turn_map.py run --spot 1 --note "tile 12.3V"
/usr/bin/python3 turn_map.py fit
```

- `run`은 시작할 때 로봇을 지점 십자 위에 놓으라고 묻는다. 회차(pass)가 바뀔 때도 다시 묻는다(회전하면서 조금씩 벗어나므로).
- Ctrl+C나 fault로 멈춰도 끝난 시행은 `logs/turn_map_results.csv`에 남는다. 같은 명령을 다시 실행하면 남은 시행만 이어서 한다.
- 50 Hz 원본은 `logs/<시각>_turn_map_spot<k>.csv`에 저장된다.
- 처음부터 다시 하려면 `plan --force`로 계획을 새로 만든다. 결과는 seed로 구분되므로 이전 결과와 섞이지 않는다.

### 판정: 지점 의존이면 폐기

`fit`은 (D, 방향)마다 실험 3번(지점 3곳)의 중앙값을 비교한다. 각 열의 의미는 다음과 같다.
- `spread`: 지점 중앙값의 최대 − 최소 (전체 중앙값 대비 %)
- `repeat`: 같은 지점에서 3회 측정한 값의 최대 − 최소 중 가장 큰 것

`spread`가 **15%를 넘고 동시에 `repeat`의 2배를 넘으면** 그 단계는 `DEPENDENT`다.
반복 잡음으로는 설명되지 않는 지점 차이라는 뜻이다. 판정 대상은 실제로 도는 단계(|ω| > 0.05 rad/s)만이고,
기준값은 스크립트 상단의 `SPREAD_MAX`, `NOISE_X`, `DISCARD_SHARE`다.

| 결과 | 조건 | 조치 |
| --- | --- | --- |
| `KEEP` | `DEPENDENT` 0개 | 표를 control에 반영 |
| `BORDERLINE` | 25% 미만 | 표시된 단계만 같은 지점에서 다시 재고 판단 |
| `DISCARD` | 25% 이상 | **이 방식 폐기**. D → ω 표 하나가 이 바닥에서 성립하지 않는다 |

데드밴드 근처(스틱슬립)는 `repeat`가 커서 `DEPENDENT`가 잘 나오지 않는다. 그 구간은 판정보다 "어디서부터 도는가"만 본다.

그 밖의 확인 항목:
- [ ] 전 시행에서 fault 없음, D ≤ 12에서 tracking 0.9 이상.
- [ ] `repeat`가 사용 구간(D 3~10)에서 5% 안팎. 크면 회차 사이 배터리, 발열, 지점 이탈을 의심한다(판정도 둔해진다).
- [ ] CW/CCW 차이 10% 이내. 넘으면 표를 방향별로 쓴다(`fit`은 원래 방향별로 출력한다).

### 해석과 반영

- `fit`은 지점 3곳의 중앙값을 단조 증가로 맞추고(PAVA) D ↔ |ω| 표를 쓴다. 원하는 |ω|에 해당하는 D는 이 표를 선형 보간해서 얻는다.
  이 표가 control의 제자리 회전, teleop 각속도, 바퀴 odom yaw의 `effective_separation` 계산을 대신하게 된다(별도 작업).
- 첫 "turns from D" 아래 구간에서는 로봇이 돌지 않는다. 그보다 작은 |ω|는 명령하지 않도록 최소 회전 속도를 정한다.
- `DISCARD`면 표 대신 IMU yaw rate 폐루프(목표 ω − IMU ω로 D를 조정)로 회전을 제어하는 쪽으로 간다.
- IMU가 기준값이므로 자이로 배율 오차(데이터시트 기준 ±3%, `test/gyro_scale` 미실시)가 표에 그대로 들어간다.

## 4WD 직진 주행 실험

`straight.py`: 휠 오도메트리로 목표 거리를 사다리꼴 속도로 가고, IMU yaw를 P 제어로 0에 유지한다
(ω = −k·yaw). 주행 전 2초 정지해서 자이로 오프셋을 잰다. 끝나면 줄자 거리와 옆 이탈을 입력받아
`logs/straight_results.csv`에 쌓는다(오도메트리, 슬립 mm/m, 최종 yaw, tracking). 주행마다 50 Hz 원본은
`logs/<시각>_straight_imu.csv`에 남는다.

### 준비

1. `config.h`에서 `kRearWheelsDrive = true`(4WD)인지 확인하고 로봇을 가만히 둔 채 `resetwheel`, `checkwheel`.
2. **1단계 바퀴 띄워서 방향 확인(`--lifted`)을 반드시 먼저 통과**한다. 뒷바퀴 방향이 틀리면 앞뒤가 서로
   싸우면서 직진이 안 된다.
3. 바닥에 3 m 직선 테이프와 출발선을 붙인다. 로봇 옆면에 **바퀴 축 중심 위치**를 표시해 기준점으로 쓴다.
   차체 중심을 테이프 선에 맞추고, 앞 방향을 선과 평행하게 둔다. 앞쪽 2.5 m를 비운다.
4. 팔은 접은 자세, 퍽 없음. 시작·끝 배터리 전압 기록. control_master는 끈다.

### 순서

| 단계 | 명령 옵션 | 횟수 | 목적 |
| --- | --- | --- | --- |
| A. 첫 주행 | `--target 0.5 --speed 0.05` | 1 | 안전, 방향 확인 |
| B. 기준 | `--target 2.0 --speed 0.1` | 3 | 4WD 기본 성능 |
| C. 고속 | `--target 2.0 --speed 0.2` | 3 | 속도 영향 |
| D. 헤딩 비교 | `--target 2.0 --speed 0.1 --yaw wheel` | 3 | IMU 보정 없을 때 휘는 정도 |
| E. 2WD 비교(선택) | B와 같음, `kRearWheelsDrive = false` + `resetwheel` | 3 | 4WD 효과 |

```bash
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/straight.py --target 2.0 --speed 0.1 --note "4wd tile 12.1V"
```

Enter로 출발한다. 정지 후 측정값을 입력한다(Enter만 치면 건너뜀).
- 출발선 → 기준점 거리 [mm]: 테이프 방향 거리.
- 옆 이탈 [mm]: 테이프 선에서 차체 중심까지 거리, 왼쪽이 +.

회차마다 로봇을 출발선에 다시 맞춘다. Ctrl+C를 누르면 0.5 s 정지 명령을 보낸 뒤 끝난다.

### 합격 기준 (B 기준, 초안)

- [ ] fault·timeout 없음, tracking 양쪽 0.95 이상.
- [ ] 슬립 |mm/m| 20 이하, 3회 편차 5 mm/m 이하.
- [ ] 옆 이탈 2 m에서 30 mm 이하, 최종 IMU yaw ±1° 이내.
- [ ] D(휠 yaw)보다 B(IMU yaw)의 옆 이탈이 작다. 아니면 IMU 오프셋이나 부호를 의심한다.

### 해석

- **슬립이 +로 크다**: 바퀴가 헛돈다. 가속을 낮추거나(`--accel`) 접지(바퀴 높이, 무게 배분)를 본다.
- **슬립이 −로 일정하다**: 실제 바퀴 반지름이 0.033 m보다 크다. `control.yaml`의 `wheel_radius` 보정 후보다.
- **tracking이 한쪽만 낮다**: 그쪽 follower가 반대 방향이거나 끌린다. `--lifted` 점검을 다시 한다.
- **IMU yaw는 0인데 옆으로 밀린다**: 차체가 방향은 유지하면서 옆으로 미끄러진다(크랩). 출발 자세나 바닥 경사를 확인한다.
- 2WD(E)와 비교해서 슬립과 옆 이탈이 줄면 4WD 효과가 있는 것이다.

## 기록표

| 시각 | ω | 방향 | wheel/IMU (평균) | tracking L/R | 중심 이동 [mm] | 배터리 [V] | 비고 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| | | | | | | | |

## 키보드 주행 (WASD)

시나리오 사이에 로봇을 옮기거나 손으로 몰아볼 때 쓴다. 다른 스크립트와 같이 바퀴 OpenCR에 50 Hz로 직접
명령하므로 control_master는 끈다. 시작할 때 2초 가만히 두면 자이로 오프셋을 잰다.

```bash
_craft_locked wheel /usr/bin/python3 ~/craft/test/drive/teleop.py
```

`w`/`s`는 목표 속도를 0.02 m/s씩, `a`/`d`는 목표 각속도를 0.1 rad/s씩 바꾼다(누를 때마다 한 단계).
명령은 목표까지 0.1 m/s², 1.0 rad/s²로 램프한다. `space` 정지, `x` 회전만 0, `r` 오도메트리·yaw 리셋,
`q`/Ctrl+C 종료(0.5 s 정지 명령 후). 상한은 0.2 m/s, 1.0 rad/s(`--max-speed`, `--max-omega`).
화면에 피드백, 오도메트리, 바퀴/IMU yaw가 계속 나오고 `logs/<시각>_teleop.csv`에 남는다.

## LiDAR odometry 검증 (`lidar_odom:=true`, 2026-10-31)

바퀴 슬립 때문에 x, y를 LiDAR odometry(perception `lidar_odometry`, 병합 스캔을 키프레임에 매칭)로 받는
구성이다. EKF 입력은 `craft_perception/config/ekf_lidar.yaml`에 있다: LiDAR vx, vy(주), IMU yaw rate,
바퀴 vx, vyaw(공분산 크게, 슬립 중 100배). 아래 SLAM 매핑 명령에 `lidar_odom:=true`를 infra와 perception
양쪽에 붙인다(`slaminfra lidar_odom:=true`, `slamper lidar_odom:=true`).

1. 바닥에 출발 표시(테이프 십자 + 정면 방향)를 한다. 특징이 많은 곳(코너, 문, 기둥)을 지나는 루프를 정한다.
2. 기록(궤적 + SLAM 그래프):
   `ros2 bag record -o loop_<번호> --topics /control/wheel/odom /perception/lidar/odom /odometry/local /perception/lidar/pose /perception/lidar/map /perception/lidar/slam_toolbox/pose_graph /perception/lidar/slam_toolbox/loop_closure_event /perception/lidar/slam_toolbox/new_node_event`
3. 천천히 한 바퀴 돌아 출발 표시에 같은 방향으로 맞춘 뒤 기록을 끈다.
4. `/usr/bin/python3 ~/craft/test/drive/odom_trajectories.py <bag>`: 휠, LiDAR, EKF, SLAM 궤적을 겹친 그림과
   소스별 끝-출발 거리(=루프 드리프트)와 방향 오차. EKF가 휠보다 출발점에 가깝게 끝나야 한다.
5. 루프 클로저(RViz 없이, ssh): `/usr/bin/python3 ~/craft/test/drive/slam_graph.py <bag>`. 맵 위에 포즈 그래프를
   그리고, 노드 번호가 10 넘게 떨어진 연결선(=루프 클로저)을 굵게 표시해 PNG로 남긴다. 루프 클로저 이벤트 시각도 출력한다.
   주행 중 실시간으로는 `ros2 topic echo /perception/lidar/slam_toolbox/loop_closure_event`에 메시지가 뜨면 닫힌 것이다.

로그 확인: perception의 `LiDAR odometry score`(0.4 미만이면 매칭 실패로 다음 스캔부터 다시 시작),
control의 `wheel slip detected / over`.

## 키보드로 몰면서 LiDAR SLAM 매핑

**한 번에 띄우기 (2026-10-31):** control + LiDAR·EKF·SLAM + perception + bag 기록을 launch 하나로.

```bash
ros2 launch craft_bringup slam.launch.py           # lidar_odom:=true, bag 은 logs/loop_<다음 번호>
/usr/bin/python3 ~/craft/test/drive/teleop_ros.py  # 다른 터미널(키 입력)
ros2 run craft_bringup save_map.sh arena2          # SLAM 끄기 전에, Jetson 에서: ~/maps/arena2.{pgm,yaml,posegraph}
```

`bro` 와 달리 보드 점검을 먼저 하지 않는다. 바퀴가 안 움직이면 끄고 `checkwheel`(필요하면 `resetwheel`).
아래는 터미널을 그룹별로 나눠 띄우는 기존 방법이다.

### RViz는 호스트(노트북)에서

SLAM은 Jetson(ssh)에서 돌리고 RViz만 같은 와이파이의 노트북에서 띄운다. 표준 메시지만 보므로 노트북에 이
저장소는 필요 없다. 캠퍼스 와이파이는 멀티캐스트를 막을 수 있어서 Jetson IP를 직접 지정한다(Fast DDS 유니캐스트).
Jetson은 아무것도 바꾸지 않는다(도메인 0 기본값, 와이파이 `10.2.17.163`, 2026-10-31 기준 DHCP라 바뀔 수 있음).

```bash
# 노트북 (Ubuntu 24.04 + ROS 2 Lyrical 권장; 다른 배포판은 표준 메시지라 대개 되지만 보장되지 않음)
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS=10.2.17.163
scp craft@10.2.17.163:~/craft/src/craft_bringup/rviz/slam.rviz ~/craft_slam.rviz
ros2 topic list | grep perception/lidar     # Jetson 토픽이 보이면 연결됨
rviz2 -d ~/craft_slam.rviz
```

- 토픽이 안 보이면: 노트북 방화벽(`sudo ufw status`; 켜져 있으면 `sudo ufw allow from 10.2.17.163`),
  같은 와이파이인지, Jetson IP(`ip -4 -br addr`)를 확인한다. 그래도 안 되면 Jetson에서 노드를 띄우는 모든 터미널에
  `export ROS_STATIC_PEERS=<노트북 IP>`를 넣고 다시 띄운다.
- TF 시간 오류(extrapolation)가 나면 두 기기 시계를 맞춘다(`timedatectl`, Jetson은 NTP 동기화됨).
- `slam.rviz`: Fixed Frame `map`, SLAM 맵, 병합 스캔, 포즈 그래프(루프 클로저 선), EKF odom 화살표(LiDAR·휠 odom은
  꺼 둠, 체크하면 겹쳐 보임), TF. 와이파이가 버거우면 merged scan을 끈다.

`teleop.py`는 바퀴 보드에 직접 붙어서 그동안은 오도메트리·IMU가 안 나온다. SLAM을 같이 돌릴 때는
control_master를 띄우고 `teleop_ros.py`로 `/control/teleop/cmd_vel`(Twist)을 보낸다. control_master는
실행 중인 task가 없고 바퀴/clock 보드가 정상이면 이 명령으로 바퀴를 돌린다(스키드 유효 간격, `max_v`·`max_wheel` 적용).
planning hold는 적용하지 않으므로 planning 없이 된다. 입력이 0.3 s(`teleop_timeout_s`) 끊기면 멈춘다.

터미널 4개(카메라 없이):

```bash
ros2 launch craft_bringup craft.launch.py group:=infra localization:=slam start_cameras:=false  # LiDAR, EKF, SLAM
ros2 launch craft_bringup craft.launch.py group:=perception localization:=slam                   # 스캔 병합, IMU 처리
ros2 launch craft_bringup craft.launch.py group:=control                                         # 바퀴 odom, IMU
/usr/bin/python3 ~/craft/test/drive/teleop_ros.py
```

control을 띄운 뒤 2초쯤 로봇을 가만히 둔다(IMU 영점). 키는 `teleop.py`와 같고 기본 상한은 0.15 m/s,
0.6 rad/s다. 화면에 EKF(`/odometry/local`)와 SLAM(`/perception/lidar/pose`) 위치가 나온다.
RViz에서 Fixed Frame `map`, `/perception/lidar/map`, `/lidar/merged`를 본다. 천천히, 회전은 특히 천천히 돈다.

맵 저장(slam_toolbox):

```bash
# .pgm + .yaml. slam_toolbox의 save_map 서비스는 /map을 기다리다 실패한다(맵이 /perception/lidar/map으로 remap됨).
# SLAM(infra)이 돌고 있을 때 저장한다. 토픽은 -t로 준다(-r 리맵은 map_saver_cli가 무시: 'map'을 기다리다 실패).
ros2 run nav2_map_server map_saver_cli -t /perception/lidar/map -f $HOME/maps/arena --ros-args \
  -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: $HOME/maps/arena}"  # 이어서 매핑용
```
