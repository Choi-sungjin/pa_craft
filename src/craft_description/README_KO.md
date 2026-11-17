# V307 전체 로봇 URDF + USD

> **URDF:** `urdf/v307_full_robot.urdf` (메시는 `meshes/visual`, `meshes/collision`, URDF 기준 상대 경로 `../meshes/...`)
> **USD:** `usd/v307_full_robot.usda` (UsdPhysics: 관절 묶음 루트, 강체, 질량, 관절·한계·구동, 충돌 메시 포함, 메시 내장)
> 전부 한 번에 옮길 때는 `V307_URDF_USD.zip`을 쓴다.

## 무엇을 기준으로 만들었나
- **형상:** V303 통합본(V298 회전 서랍·받침대, 실제 2020 프로파일, 1·2층 기둥과 M3, M5×14·M5×8 볼트)이다. 상단 D435 거치대는 실물대로 통째로 앞뒤 뒤집었고, 카메라 숙임은 **사용자 실측 43.6°**다. 받침 볼트는 길이를 몰라 넣지 않았다.
- 보이는 BRep 1,167개(V305 장면)와 Fusion 메시 바디 28개(바퀴·타이어, 뒤 모터, Jetson, 그리퍼 렌더 메시)를 모두 넣었다.
- **좌표:** REP-103(x 앞, y 왼쪽, z 위)이다. base_link 원점은 CAD (129, 228, -4.25) mm로, 네 바퀴 축 중심이며 V186과 같다. 단위는 m, kg, s이고, URDF 각도는 rad, USD 각도는 deg다.
- **링크·관절(12개):** lm_slide(직선, -0.021~0.108 m), shoulder, elbow, wrist_pitch, tool_roll, jaw_upper/jaw_lower(직선 -0.010~0.0127 m), carousel(연속), 바퀴 4개(연속)다. 한계·힘·속도는 V186 joints_final 값이다.
  - 손목 pitch와 tool_roll 중심은 현재 실측값(V251)을 썼다. V186 값은 손목 마운트를 바꾸기 전 위치라 쓰지 않았다.
  - 모든 링크 원점은 관절 중심이고, 영점 자세(CAD 접힘 자세)에서 base_link와 방향이 같다.
- **센서 프레임:** 
  - `camera_top_color/depth_link` + `_optical_frame`: 상단 D435, 43.6° 숙임
  - `camera_wrist_color/depth_*`: 손목 D435(V249)
  - `front_laser`, `rear_laser`, `imu_link`, `base_footprint`: V186 값
  - `tcp`: 집게 끝에서 15 mm 안쪽 두 손바닥 중간, x = 접근 방향
  - 광학 프레임은 z 시선, x 오른쪽, y 아래다.
- **질량(합계 6.92 kg):**
  - V186 바디 질량표 966개를 썼다. 이름이 "본체1"처럼 일반적인 것은 잘못 매칭되므로 표를 쓰지 않았다.
  - 새 부품은 재질로 추정했다. PLA는 1.24 g/cm³ × 0.85(채움 70%)로 72개, 강철 볼트·샤프트·자석은 7.85로 146개, 알루미늄 프로파일은 2.70으로 3개다.
  - PB721 배터리는 480 g이고, 참조 바디 5개는 질량 0이다.
  - 링크별 질량은 `LINKS.json`에 있다. base 5.00, carousel 0.46, wrist_pitch 0.58, tool_roll 0.20 kg 등이다.
- **메시:**
  - 표시용은 링크별 하나로 합쳤고, 4,000면이 넘는 바디는 0.5 mm 격자로 정점을 합쳤다.
  - 충돌용은 바디별 볼록 껍질을 합친 것이다. 참조 바디는 빼고, base_link에서는 아주 작은 바디도 뺐다. USD 충돌 메시는 `convexDecomposition` 근사다.

## 검증 (`VALIDATION.json`) — PASS
- URDF 트리: 루트 base_link 하나, 모든 관절의 부모·자식 링크가 있음, 메시 파일 모두 있음, 관성 텐서 정상
- 영점 자세 URDF 메시 조립 외곽과 CAD 장면 외곽 차이: 0.0 mm
- 상단 카메라 광학 z축 숙임: 43.60°(실측과 같음)
- 손목 카메라 프레임: CAD (191.5, 219.7, 277) mm, D435 렌즈 위치와 일치
- USDA: 강체 13개, 관절 12개, 괄호 균형, 같은 prim 중복 정의 없음

## 확인 못 한 것
- USD를 실제 USD 도구(pxr·Isaac Sim)로 열어 보지 못했다. 이 PC에 pxr이 없어 텍스트 규격대로 직접 썼다. 열 때 오류가 나면 알려 주면 바로 고친다.
- 질량은 표·추정값이고 실측이 아니다(새 PLA 부품 72개는 채움 추정).
- 상단 카메라 받침 볼트 길이와 실제 접촉, 카메라 보정값(내부·외부 파라미터)
- 바퀴 마찰·접촉 재질, 구동기 게인은 V186 값을 그대로 썼다.
- 뒤 바퀴는 6절 확정대로 구동 대상이 아니지만, URDF에서는 다른 바퀴처럼 연속 관절이다. 시뮬레이션에서는 구동을 끄면 된다.

## 다시 만들기
- `python make_urdf_usd.py` (Python 3.14, `cad/review_dependencies` 사용)
- 입력: `delivery/V306_.../WORLD_SCENE_V306_FULLFLIP_MEASURED_43p6.json`(BRep), `MESH_BODIES.json`(Fusion 메시 바디)
- 검증: `python validate.py`

## RViz 확인 도구

```bash
ros2 launch craft_description arm_sliders.launch.py   # LM 가이드 + 팔 관절만 슬라이더
ros2 launch craft_description box_reach.launch.py     # 전방 박스(205×205×70 mm) 커버 검사
```

### `arm_sliders` — 고른 관절만 슬라이더로

- 기본은 `lm_slide`(mm), `shoulder`/`elbow`/`wrist_pitch`/`tool_roll`(deg)이다. 나머지 움직이는 관절(집게, 캐러셀, 바퀴)은 0으로 고정해 발행한다.
- 다른 URDF: `urdf:=/경로/로봇.urdf joints:="joint_a joint_b"`. `joints:="''"`이면 움직이는 관절 전부에 슬라이더를 띄운다.
- `.urdf`는 파일을 그대로 읽고, `.xacro`일 때만 xacro를 쓴다.

### `box_reach` — 박스 윗면을 TCP로 다 덮는지 검사

- **조절**
  - 관절: TCP 위치를 창에 mm로 표시하고, RViz에는 파란 점으로 보인다.
  - 관절 리밋 체크박스: 관절마다 한계 ON/OFF. OFF면 그 관절 한계를 무시하고 판정한다(회전 ±180°, 직동 ±300 mm).
  - 숫자 칸: 모든 슬라이더 옆. 값을 넣고 Enter. 관절은 한계 안으로 자르고, 나머지는 슬라이더 범위를 넓힌다.
  - LM 스트로크: 현재 범위의 중앙을 기준으로 양쪽 대칭으로 늘이거나 줄인다. URDF 한계도 같이 바꾼다.
  - 링크 길이 4개: L1 어깨–팔꿈치, L2 팔꿈치–손목, L3 손목–비틀기, L4 비틀기–TCP
  - 박스: 로봇 앞면과의 거리, 좌우 중심, 검사 높이
  - TCP 각도 허용: 0이면 수직 아래만 보고, 180이면 방향과 상관없이 본다.
  - 높이 여유(기본 30 mm): 박스 윗면 점 바로 위 0~여유 높이 중 어디서든 TCP가 그 각도로 오면 그 점은 초록이다.
- **링크 길이 변경:** URDF 관절 원점을 같은 방향으로 늘이거나 줄여 robot_state_publisher에 다시 넣는다. 메시도 링크 방향으로 같이 늘인다. 양 끝 관절 허브(링크 길이의 30%, 최대 35 mm)는 모양을 그대로 두고 그 사이만 늘이거나 줄이며, 끝 허브 너머는 늘어난 만큼 평행 이동한다. L4는 그리퍼 몸체와 두 집게를 함께 늘인다. 늘인 메시는 임시 폴더(`/tmp/box_reach_meshes_*`)에 STL로 쓰고, 종료할 때 지운다. 충돌 메시는 바꾸지 않는다.
- **박스 위치:** 박스는 +x(전방), 바닥 `base_footprint`(z −34.3 mm)에 놓는다. 거리는 박스 높이대(바닥~박스 높이)와 박스 좌우 폭 안에 있는 몸체 메시의 가장 앞 점부터 잰다(V307 기본 배치: x 242.8 mm). 그래서 거리 0이면 박스가 몸체에 닿는다. 팔과 LM 가이드는 빼고 잰다. 좌우 기본값은 LM 가이드 중앙이다.
- **판정**
  - 박스 윗면을 5 mm 격자로 나눠, 닿으면 초록, 못 닿으면 빨강으로 칠한다.
  - 좌우는 LM 가이드 범위로, 전후와 높이는 팔(x‑z 평면 3링크 IK)과 관절 한계로 본다.
  - 충돌은 보지 않는다.
- **launch 인자:** `stroke:=210`(시작 LM 스트로크 mm), `limits:="shoulder:0:45 elbow:-30:170"`(관절 한계 바꾸기, deg/mm, URDF 파일은 그대로)
- **무게 토크 패널(창 오른쪽):**
  - 현재 자세에서 무게(중력)만으로 shoulder, elbow, wrist_pitch에 걸리는 토크를 보여 준다. URDF 링크 질량과 무게중심, 그리고 집게 하중(g 입력)을 쓴다. 가속과 마찰은 뺀다.
  - 관절과 모터가 직결이라 관절 토크가 곧 모터 토크다.
  - 모터는 XM430‑W350 @12 V 기준이다. 스톨 4.1 N·m는 ROBOTIS e-manual 값이고, 정격 0.82 N·m는 스톨의 20%로 잡은 추정값이다. 각 값을 정격·스톨 대비 %로 보여 준다(초록 ≤ 정격, 주황 ≤ 스톨, 빨강 > 스톨).
  - 링크 길이를 바꾸면 무게중심은 메시와 같은 방식으로 옮기지만 질량은 그대로 둔다.
  - 모터 값은 파라미터 `motor_name`, `motor_stall_nm`, `motor_rated_nm`로 바꾼다.
- **다른 로봇:** `box_reach.py` 파라미터 `slide_joint`, `arm_joints`, `roll_joint`, `tcp_joint`, `front_axis`, `front_face`, `floor_z`, `box_size`를 바꾼다.

### 현재 베스트 설정 (2026-11-18)

`box_reach`로 찾은 값 중 지금까지 가장 좋은 설정이다. 전체 커버 **71.4%**(좌우 100%, 전후·높이 71.4%)다.

| 항목 | 값 | 원래 |
|---|---|---|
| LM 스트로크 | 210 mm (중앙 기준 대칭, −61.5~148.5 mm) | 129 mm |
| shoulder 한계 | 0~45° (뒤로 꺾임 없음) | −18~45° |
| L1 어깨–팔꿈치 | 110 mm | 188 mm |
| L2 팔꿈치–손목 | 110 mm | 95 mm |
| L3 손목–비틀기 | 45 mm | 45 mm |
| L4 비틀기–TCP | 152.6 mm | 152.6 mm |
| 로봇 앞면–박스 거리 | 15 mm | |
| 박스 좌우 중심 | 0 mm | |
| TCP 각도 허용 | 수직 아래 ±10° | |
| 높이 여유 | 윗면 위 30 mm까지 | |

다시 띄우려면 아래 명령을 쓴다. 링크 길이, 거리, 각도 허용은 창에서 위 값으로 맞춘다.

```bash
ros2 launch craft_description box_reach.launch.py stroke:=210 limits:="shoulder:0:45"
```
