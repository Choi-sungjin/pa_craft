# v186_pick demo

V186 URDF(CAD 2026-11-01, `~/V186_RL_THREE_SETS_*`의 C_full_robot)로 만든 퍽 집기 RViz 미리보기. 하드웨어 없음.
시나리오는 `demo/puck_pick`과 같다: 7 cm 선반 위에 선 퍽(Ø30 × 60 mm), 앞 LiDAR 앞면에서 13 cm, 4 cm 위 접근 → 수직 하강 → 닫기.

    ./get_model.sh          # 처음 한 번: 분할 zip 합치기(SHA-256 확인) -> model/ (38 MB, git 제외)
    ./show.sh               # plan.py가 plan.yaml보다 새것일 때만 다시 계획(약 1분), RViz 반복 재생
    ./show.sh --plan        # 강제로 다시 계획
    /usr/bin/python3 -I plan.py   # 계획만

- V186 좌표는 REP-103(x 앞, y 왼쪽, z 위), `base_link` = 바퀴 축 중심, 바닥은 34.3 mm 아래. v75 URDF(앞 = −Y)와 다르다.
- 관절: `lm_slide`(+y), `shoulder`·`elbow`·`wrist_pitch`(y축), `tool_roll`, 턱 `jaw_upper`/`jaw_lower`(+ = 열림, 0에서 간격 24 mm).
  한계는 CAD 간섭 기준이라 실물 모터 리밋·오프셋(control.yaml)과는 아직 연결되지 않았다.
- IK: slide가 y, 나머지 세 관절이 x·z와 접근 방향을 수치로 푼다. 기울기는 0°부터 양쪽으로 1°씩, 리밋 3° 안쪽 + 선반·퍽과 링크 메시 3 mm 이상(턱은 퍽 접촉 허용).
- 가정값은 `plan.py` 상단 `ASSUME`.
