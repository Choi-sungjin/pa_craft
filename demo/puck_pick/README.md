# puck_pick demo

선반(7 cm) 위 퍽을 집어 clock 12번 슬롯에 적재(`sis1`)하고, 다시 꺼내 앞에 내려놓는(`sis2`) 실물 데모.
planning을 거치지 않고 `/planning/task_command`로 control_master에 직접 단계별 명령을 보낸다.

    ~/craft/demo/puck_pick/show.sh                          # RViz 미리보기 (하드웨어 없음)
    python3 demo/puck_pick/plan.py                          # IK 계획 -> plan.yaml
    bro   # 터미널 A: 보드 확인 + control
    sis1  # 터미널 B: 집어서 적재   (execute.py, --dry-run: 단계만 출력)
    sis2  # 터미널 B: 꺼내서 내려놓기 (execute.py --unload)

가정값은 `plan.py` 상단 `ASSUME`. 실행 방법, 잡기 좋은 영역, 트러블슈팅, 변경 기록은 [NOTES.md](NOTES.md).
