# puck_compare

같은 D435 프레임에서 우리 PuckAxis(donut + RANSAC, C++ 그대로)와 CenterPose를 둘 다 돌려 비교한다.

    record.py   D435 color + aligned depth -> 세션 폴더 (ros2 bag엔 depth가 없음: shm에만 있음)
    compare.py  세션마다 puck_axis_offline(우리) + CenterPoseAdapter -> results/{ours,centerpose}.csv
    shelf_detect.py  depth만으로 선반 윗면 평면 + 사각형 + 앞 모서리 -> results/shelf.csv
    export_yolo.py  세션 -> YOLO 데이터셋 (0 target_blue, 1 shelf), shelf는 shelf_detect로 자동 프리라벨, 검수 후 학습
    analyze.py  세션별 표: hit, 축 떨림(jit), 중심 떨림, 선반 법선 대비 tilt 오차, standing 판정, 지연, 두 방법 차이

## 녹화

스택이 돌고 있을 때는 shm에서 읽고(`--source shm`, 기본값), 스택이 꺼져 있을 때는 D435를 직접 연다(`--source rs`).
세션 하나당 퍽 자세는 하나로 고정하고, 퍽은 움직이지 않게 둔다.

    source ~/craft/install/setup.bash && cd ~/craft/demo/puck_compare   # record.py는 venv 파이썬(pyrealsense2)
    ~/craft/venv-craft/bin/python3 record.py --name stand_35 --label standing --note "35 cm, 정면"
    ~/craft/venv-craft/bin/python3 record.py --name lie_0    --label lying --yaw-deg 0
    ~/craft/venv-craft/bin/python3 record.py --name lie_45   --label lying --yaw-deg 45
    ~/craft/venv-craft/bin/python3 record.py --name lie_90   --label lying --yaw-deg 90
    # 옵션: --frames 100 --hz 5 --out ~/bags/puck_compare

## 실행 / 분석

    ./compare.py ~/bags/puck_compare/* --tags 23            # perception.yaml의 tag_registry 퍽 태그 id
    ./compare.py ~/bags/puck_compare/* --only ours --box yolo   # 박스를 puck_blue YOLO로
    ./analyze.py ~/bags/puck_compare/* --csv summary.csv

CenterPose는 `centerpose_adapter.py`의 `_infer`만 채우면 된다(Objectron 9점 cuboid -> y축 = 퍽 축).

## 알아둔 것

- `tag_box_scale: 2.0`(현재 기본값)이면 박스가 누운 퍽(6 cm)을 잘라 먹어서 PCA 축이 박스 대각선 쪽으로 끌려간다.
  합성 장면(0.22 m, yaw 30°) 결과: scale 2 -> 40.9°, 3 -> 36.0°, 4 -> 30.0°. 실물 세션에서 `--tag-scale`을 바꿔 가며 확인할 것.
- `config/tag_registry.yaml`이 지금 `tags: {}`라서 라이브 `puck_axis`는 퍽 결과를 하나도 내지 않는다.
- shelf_detect.py 합성 장면(바닥 + 20.5x20.8x7 cm 선반) 결과: 중심 오차 2~13 mm, 앞 모서리 꼭짓점 오차 9~12 mm,
  크기는 1 cm 정도 크게 나옴(옆면 윗부분 점이 섞임). 프레임당 0.4 s(Python). 선반이 화면에서 잘리면 중심이 밀림.
- shm 모드 녹화는 `/lidar/front/scan`도 프레임마다 저장한다. 매칭하려면 camera_color_optical_frame -> base_link TF가 필요한데 아직 어디에도 없음.
