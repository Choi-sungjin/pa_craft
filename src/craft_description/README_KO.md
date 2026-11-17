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
