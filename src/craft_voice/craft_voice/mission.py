"""The 2026-11 competition mission ("주사위 기억 로봇", arena 4464 x 6364 mm), for narration and the demo."""

MISSION_CONTEXT = (
    '주사위 기억 미션(경기장 4.5 x 6.4 m, 벽으로 나뉜 미로): 출발 구역에서 시작해 바코드(P2-01~P2-04)를 읽어 오름차순/내림차순을 정하고, 신호등이 1초씩 12번'
    ' 깜빡이는 빨강·노랑·초록 순서를 기억한 뒤 색별 횟수를 센다. 시작 박스 6곳에서 퍽 12개를 12칸 캐러셀에 싣고, 횟수 순서대로 L1·L2·L3(AprilTag '
    '12·13·14)에 퍽을 주사위 눈 모양으로 내려놓는다. 놓은 자리를 다시 확인하고 출발 구역으로 돌아와 3초 멈추면 끝난다.'
)

# Spoken names of the proposed planning step ids ('*' copies that part of the id). They must match the
# step ids in planning's missions.yaml once the mission is written there.
STEP_NAMES = (
    'read_barcode:바코드 읽기,observe_signal:신호등 관측,count_sort:횟수 계산과 정렬,load_S*:시작 박스 * 퍽 싣기,allocate:장소별 퍽 '
    '배정,place_L*:L* 주사위 눈 놓기,verify_L*:L* 다시 확인,return_home:출발 구역으로 복귀'
)

# Simulated run in mission order (labels and step ids as planning_master would publish them).
DICE_SCENARIO = (
    'Idle:2, Ready:4, Run.PrepareTask@read_barcode:0.3, Run.Execute@read_barcode:5, '
    'Run.Complete@read_barcode:0.05, Run.PrepareTask@observe_signal:0.3, Run.Execute@observe_signal:26, '
    'Run.Complete@observe_signal:0.05, Run.PrepareTask@count_sort:0.3, Run.Execute@count_sort:3, '
    'Run.Complete@count_sort:0.05, Run.PrepareTask@load_S1:0.3, Run.Compute@load_S1:0.5, '
    'Run.Execute@load_S1:14, Run.Complete@load_S1:0.05, Run.PrepareTask@load_S2:0.3, '
    'Run.Compute@load_S2:0.5, Run.Execute@load_S2:14, Run.Complete@load_S2:0.05, '
    'Run.PrepareTask@allocate:0.3, Run.Execute@allocate:3, Run.Complete@allocate:0.05, '
    'Run.PrepareTask@place_L1:0.3, Run.Compute@place_L1:0.5, Run.Execute@place_L1:18, '
    'Run.Complete@place_L1:0.05, Run.PrepareTask@place_L2:0.3, Run.Compute@place_L2:0.5, '
    'Run.Execute@place_L2:18, Run.Complete@place_L2:0.05, Run.PrepareTask@verify_L1:0.3, '
    'Run.Execute@verify_L1:6, Run.Complete@verify_L1:0.05, Run.PrepareTask@return_home:0.3, '
    'Run.Compute@return_home:0.5, Run.Execute@return_home:12, Run.Complete@return_home:0.05, Ready:6'
)
