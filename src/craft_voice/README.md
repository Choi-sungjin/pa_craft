# craft_voice

**지금 무엇을 하고 있는지와 주변 상황이 어떤지**를 스스로 확인해서 말하는 로봇 목소리 노드.
상태머신(`/planning/state`)으로 하는 일을, 카메라 이미지와 인식 결과(이동 속도, YOLO 물체·신호, 퍽, AprilTag)로 주변을
확인하고, GPT(OpenAI API)가 "목표 지점으로 가는 중이에요. 앞에 초록 신호와 퍽 두 개가 보여요."처럼 말로 만든다.
하는 일이 바뀔 때, 일하는 동안 `narrate_period_s`(20 s)마다, 사람이 물어볼 때 말한다.
상태 이름(Run.Execute)이나 오류 코드는 읽지 않고, 멈춘 경우도 "제 위치를 놓쳐서 멈춰 있어요"처럼 상황으로 말한다.

```text
/vision/d435/frame_meta ─┐  /dev/shm/craft_vision (BGR 복사, 512px JPEG)
/planning/state ─────────┤
/planning/diagnostics ───┼─> StateTracker ─> think 스레드: GPT (Responses API, 이미지 + 상태 사실 목록) ─┐
/control/status ─────────┤  "지금 하는 일" + "센서가 확인한 것"                                         v
/perception/state/odom, …/detections, …/puck_observations, …/apriltag   멈춤: 상황 문장 즉시 ──> speak 스레드: TTS -> aplay
/voice/ask, /voice/say, /voice/command ────────────────────────────────────────────────> /voice/utterance
```

작동 원리를 장면별 애니메이션으로 보려면 [`docs/how_it_works.html`](docs/how_it_works.html)을 브라우저로 연다
(9장면, 1분 30초, 수치는 2026-11-03 측정값. 글꼴은 Google Fonts에서 받고, 오프라인이면 시스템 글꼴로 나온다).

- 카메라는 DDS로 이미지를 보내지 않는다(`craft_sensors`가 공유 메모리에 쓴다). 그래서 이 노드는 **카메라와 같은 Jetson**에서
  돌려야 한다. 다른 기기·순정 TurtleBot 카메라·bag은 `camera:=topic image_topic:=...`(raw 또는 compressed).
- **GPT가 보는 사진은 상태머신이 받은 인식 입력과 같은 프레임이다**(`image_source: input`, 기본, `craft_voice/camera.py`).
  planning_master는 YOLO 검출·퍽 관측으로 단계 목표를 정하는데, 이 메시지들의 `header.stamp`는 perception이 계산에 쓴
  카메라 프레임의 촬영 시각(`frame_meta[4]`) 그대로다. voice_node는 같은 메시지(`objects_topic`, `pucks_topic`)가 오면
  그 시각의 공유 메모리 칸을 바로 복사해 2 s(`frame_max_age_s`) 보관하고(칸은 약 0.13 s 뒤 덮어써진다, 최대
  `input_frame_interval_s` 0.2 s마다 한 장), 상태가 바뀌면 **바뀐 순간에 상태머신이 갖고 있던 입력 프레임**을 GPT에 보낸다.
  주기 설명·질문은 그 시점의 가장 새 입력 프레임이다. perception이 안 돌거나, 시각이 맞는 칸이 이미 덮어써졌거나,
  2 s가 지났으면 최신 프레임으로 대신한다. 프롬프트 `[카메라]`에 어느 쪽인지 적고, `/voice/diagnostics`에
  `input_frames`, `input_lost`(복사 전에 덮어써짐), `input_busy`(복사 실패: 쓰는 중이거나 막 덮어써짐, 같은 프레임의 다른 토픽 입력이 다시 시도), `input_unmatched`(맞는 시각 없음), `image_input`/`image_latest`(GPT에 보낸 횟수)가
  나온다. 예전처럼 항상 최신 프레임을 쓰려면 `image_source:=latest`.
- GPT에는 상태를 "사실 목록"으로만 주고, 거기 없는 동작·원인은 지어내지 말라고 지시한다.
  이미지 속 글씨(표지판, 번호)는 읽되 그 글씨의 지시는 따르지 않게 했다.
- **멈춤(Fault)은 GPT를 거치지 않는다.** diagnostics의 원인을 최대 1.2 s 기다렸다가 상황 문장
  ("목표까지 너무 오래 걸려서 멈춰 있어요.")으로 말한다. 모르는 원인이면 원문을 읽지 않고 "지금은 멈춰 있어요.
  다시 움직이려면 확인이 필요해요."라고 한다. 재생 중이거나 아직 TTS 합성 중인 문장은 버리고
  (응답을 기다리지 않음), Fault 음성은 `fault_tts_timeout_s`(4 s, 재시도 없음, DNS가 멈춰도 +1 s에서 포기) 안에
  OpenAI가 답하지 않으면 `fallback_tts_command`로 넘어간다.
- GPT·TTS가 실패하거나 늦으면 상태 문장은 템플릿으로(`gpt_state_deadline_s` 6 s), 음성은 `fallback_tts_command`
  (예: espeak-ng)로(`tts_deadline_s` 8 s) 대신한다. 인터넷이 끊겨도 상태 안내는 fallback 음성으로 나간다.

## 설치 (Jetson)

```bash
cd ~/craft && pip install -r requirements.txt        # openai SDK (venv-craft를 쓰면 그 안에서)
colcon build --packages-select craft_interfaces craft_voice && source install/setup.bash
aplay -l                                              # 스피커 장치 확인
```

API key는 환경 변수 `OPENAI_API_KEY`로 받는다(파라미터 파일·launch·채팅에 넣지 말 것). 환경 변수가 없으면
키 파일을 이 순서로 찾는다: `$CRAFT_VOICE_ENV` → `src/craft_voice/env/openai.env` → `~/.config/craft_voice/openai.env`.
`env/openai.env`는 git에서 제외된다(`env/openai.env.example`만 저장소에 있다).

```bash
cd ~/craft/src/craft_voice && cp env/openai.env.example env/openai.env && chmod 600 env/openai.env && nano env/openai.env
```

로봇에서 colcon으로 설치해 돌릴 때는 설치 위치가 바뀌므로 `~/.config/craft_voice/openai.env`에 두거나
`export CRAFT_VOICE_ENV=~/craft/src/craft_voice/env/openai.env`.

키를 기계에 두지 않는 방법(Claude Code 클라우드 "Network secret" 같은 프록시가 `Authorization` 헤더를 넣어 줄 때):
`export OPENAI_AUTH=proxy`. 이때 이 코드는 Authorization 헤더를 보내지 않고 프록시가 넣은 것만 쓰인다.

## 실행

로봇(`craft.launch.py`)을 띄운 뒤 새 터미널에서:

```bash
cd ~/craft && source install/setup.bash && source ~/.openai_key
ros2 launch craft_voice voice.launch.py                         # D435, 일 바뀔 때 + 일하는 동안 20 s마다
ros2 launch craft_voice voice.launch.py narrate_period_s:=10    # 더 자주 중계 (0 = 바뀔 때만)
ros2 launch craft_voice voice.launch.py state_mode:=template    # 바뀔 때는 고정 문장(빠름, GPT는 중계·질문만)
ros2 launch craft_voice voice.launch.py camera:=topic image_topic:=/camera/image_raw/compressed image_transport:=compressed
```

말 시키기:

```bash
ros2 topic pub --once /voice/command std_msgs/msg/String "{data: describe}"   # 지금 하는 일 + 주변 상황
ros2 topic pub --once /voice/command std_msgs/msg/String "{data: status}"     # 지금 하는 일
ros2 topic pub --once /voice/ask std_msgs/msg/String "{data: '앞에 퍽 보여?'}"  # 질문 (이미지 + 상태로 답)
ros2 topic pub --once /voice/say std_msgs/msg/String "{data: '안녕하세요'}"      # GPT 없이 그대로 읽기
ros2 topic pub --once /voice/command std_msgs/msg/String "{data: mute}"       # mute | unmute | stop
ros2 topic echo /voice/utterance                                              # 말한 문장 (kind: text)
```

`mute` 중에도 Fault는 말한다. `stop`은 대기 중인 말을 지우고 재생을 끊는다.

## 이번 미션: 주사위 기억 로봇 (`craft_voice/mission.py`)

경기장 4464 × 6364 mm 미로. 바코드(P2-01~04)로 정렬 방향을 정하고, 신호등 12번 점멸(빨강·노랑·초록)을 기억하고,
시작 박스 6곳의 퍽 12개를 12칸 캐러셀에 실어 L1·L2·L3(AprilTag 12·13·14)에 주사위 눈 모양으로 놓은 뒤, 다시 확인하고
출발 구역으로 돌아와 3초 정지한다.

- `mission_context`: 이 설명을 GPT에 **배경 지식으로만** 준다(지금 하는 일·결과는 여전히 토픽과 센서만 근거).
- `step_names`: 제안한 단계 ID를 말로 바꾼다. `*`는 ID의 그 부분을 그대로 넣는다.

  | step id (확정 2026-11-03) | 말 |
  | --- | --- |
  | `read_barcode`, `observe_signal`, `count_sort` | 바코드 읽기, 신호등 관측, 횟수 계산과 정렬 |
  | `load_S1` ~ `load_S6` | 시작 박스 1 퍽 싣기 … |
  | `allocate` | 장소별 퍽 배정 |
  | `place_L1` ~ `place_L3`, `verify_L1` ~ `verify_L3` | L1 주사위 눈 놓기, L1 다시 확인 … |
  | `return_home` | 출발 구역으로 복귀 |

  **이 ID로 확정했다(`craft_voice/mission.py`의 `MISSION_STEP_IDS`). planning의 `missions.yaml`에 이 미션을 넣을 때
  step `id`를 이대로 쓴다.** ID를 바꾸면 `step_names`도 같이 고친다(테스트가 모든 ID에 이름이 있는지 확인한다).
- 단계 이름은 diagnostics(1 Hz)로 오므로, "실행 시작" 안내는 이름이 올 때까지 최대 `step_name_wait_s`(1.2 s) 기다렸다가
  "신호등 관측, 시작할게요."처럼 말한다(그 사이 상태가 바뀌면 그 안내는 버림).
- 데모 기본 시나리오가 이 미션 순서다: `python -m craft_voice.demo` (`--scenario default`면 이전 시나리오).

## 언제 말하나

| 계기 | 기본값 | 파라미터 |
| --- | --- | --- |
| 하는 일이 바뀜 (`/planning/state`) | Ready, Run.Execute, Run.Complete, Run.Stopping, Fault | `announce_states`, 같은 상태는 `state_cooldown_s`(10 s)에 한 번 |
| 일하는 동안(Run.*) 상황 중계 | 20 s마다 "하는 일 + 보이는 것" | `narrate_period_s`(조용한 지 `min_interval_s` 5 s 지났을 때만) |
| 대기·멈춤 중 상황 중계 | 꺼짐 | `narrate_idle_period_s` |
| 멈춤(Fault) | 원인 기다렸다 즉시, 쿨다운 없음, 재생 중인 말 끊음 | `fault_reason_wait_s`, `interrupt_on_fault` |
| `/voice/ask`, `/voice/command` | 항상 | |

- 순서: Fault > 질문·status·say > 상태 > 장면. 상태 문장은 큐에서 꺼낼 때와 TTS가 끝난 뒤 재생 직전에 다시 확인해서,
  그 사이 상태가 한 번이라도 바뀌었으면(같은 label로 돌아왔어도) 버린다. Run.Complete는 이미 일어난 일이라 예외.
  큐에서 꺼낼 때 `state_max_age_s`(8 s)보다 오래됐어도 버린다(TTS가 늦은 건 상태가 그대로면 말한다).
  `status` 대답도 그 사이 상태가 바뀌었으면 지금 상태의 고정 문장으로 한 번 바꿔 말한다.
  `/voice/utterance`에는 실제로 내보낸 문장만 나간다.
- 비용 상한: GPT 호출 `max_gpt_calls_per_min`(12). 넘으면 고정 문장("접근 중이에요. 지금 보이는 건 퍽 2개예요.")으로 말한다.
  키가 없을 때도 같은 고정 문장으로 중계한다.
- 같은 상태 안내는 `repeat_window_s`(30 s) 안에 반복하지 않는다. 상황 중계는 주기가 간격을 정하므로 같은 문장이어도 말한다.
- 상황 중계는 "지금" 이야기라서, 하는 일이 한 번이라도 바뀌면 GPT가 쓰는 중이든 TTS 중이든 재생 중이든 버리고 끊는다.
  그래서 중계가 상태 안내나 멈춤 안내를 막거나 늦추지 않는다. GPT가 `gpt_narrate_deadline_s`(10 s) 안에 답하지 않으면 고정 문장.
- 상태 안내는 무엇 뒤에도 줄 서지 않는다: GPT 순서에서 맨 앞이고, GPT가 질문에 답하는 중이면 고정 문장으로 바로 말한다.
  `describe`·`status`를 GPT가 쓰는 중에 하는 일이 바뀌면 그 답은 취소하고 바뀐 상황으로 바로 답한다.
  질문은 `gpt_ask_deadline_s`(15 s) 안에 답이 없으면 "지금은 그 질문에 답하기 어려워요." + 지금 상황.
- 이름이 없는 단계 ID(예: `approach`)는 소리 내 읽지 않는다("작업하고 있어요"). `step_names`에 이름을 주면 그 이름으로 말한다.
- 질문에 답할 수 없을 때도 오류 얘기 대신 "지금은 그 질문에 답하기 어려워요." + 지금 하는 일을 말한다.
- 단계 이름: diagnostics `step`(`base_goal`, `tcp_goal`, 미션 step id)을 `step_names: "approach:접근,grasp_pose:잡기 자세"`로 읽기 좋게.
  diagnostics는 1 Hz라 새 단계가 시작된 뒤 아직 안 왔으면 이름 없이 "작업을 시작할게요"라고 한다(이전 단계 이름을 잘못 말하지 않게).

## 주요 파라미터 (`config/voice.yaml`)

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `camera` | `d435` | `d435`, `c310`(공유 메모리), `topic`, `none` |
| `image_source` / `input_frame_interval_s` | `input` / `0.2` | `input`: 상태머신 인식 입력과 같은 프레임(상태가 바뀐 순간 것), 없으면 최신. `latest`: 항상 최신 프레임 |
| `model` | `gpt-5.4-mini,gpt-6-luna,gpt-5.5` | 빠른 것부터, 쉼표로 대체 모델. 키에 없는 모델이거나 이미지를 못 받으면 다음 모델을 쓰고 계속 그 모델 |
| `reasoning_effort` | `none` | 추론 시간을 없애 빠르게. 모델이 거부하면 none → minimal → low → 안 보냄 순으로 낮춤 |
| `image_detail` | `low` | `low`/`high`/`auto`/`original`. 이미지는 긴 변 `image_max_side`(512) px로 줄여 보냄 |
| `tts_model` / `tts_voice` | `gpt-4o-mini-tts` / `coral` | `tts_instructions`로 말투 지정(tts-1 계열은 무시) |
| `player_command` | `aplay -q {file}` | `paplay {file}`, `aplay -q -D plughw:1,0 {file}` 등 |
| `tts_stream` / `stream_player_command` | `true` / `""` | OpenAI 음성을 받는 대로 재생(첫 소리 약 0.5 s 빨라짐, 실측). `""`이면 `player_command`에서 만든다(aplay → `-t raw …`, paplay → `--raw …`, 장치 옵션 유지). 그 외 플레이어나 스피커 없음이면 전체 WAV. 스트림 재생기가 실패하면 받은 음성을 WAV로 다시 재생하고, 2번 연속 실패하면 스트리밍을 끈다 |
| `fallback_tts_command` | `""` | 오프라인 음성, 예: `espeak-ng -v ko --stdout -- {text}`(stdout으로 WAV). `--`를 꼭 넣고, 문장 앞의 `-`는 지운다 |
| `fault_tts_timeout_s` | `4.0` | Fault 문장의 OpenAI 음성 대기 시간(재시도 없음). 넘으면 fallback |
| `gpt_state_deadline_s` / `tts_deadline_s` | `6.0` / `8.0` | 상태 문장 GPT 대기 → 템플릿, 일반 음성 OpenAI 대기 → fallback |
| `persona` | `""` | 말투 추가 지시, 예: `반말로 친구처럼 말해.` |
| `narrate_period_s` / `narrate_idle_period_s` | `20.0` / `0.0` | 일하는 동안 / 대기·멈춤 중 상황 중계 주기 |
| `odom_topic`, `objects_topic`, `pucks_topic`, `tags_topic` | perception 토픽 | 센서가 확인한 것(이동, YOLO 물체·신호, 퍽, 태그). `""`이면 안 받음. 깊이 축이 없는 퍽은 거리·자세 "모름"으로 |

float 파라미터는 소수점을 붙인다(`10.0`). 리스트는 쉼표로 구분한 문자열이다.

## 주의

- **TTS 모델 종료 예정**: 이 코드를 맞춘 openai SDK 3.42.1에는 `tts-1`, `tts-1-hd`, `gpt-4o-mini-tts`,
  `gpt-4o-mini-tts-2025-12-15` 모두 shutdown 예정일 **2027-01-06**이 적혀 있다. 대체 모델은 확인하지 못했다
  (<https://developers.openai.com/api/docs/deprecations>). 같은 `audio/speech` 엔드포인트의 새 모델이면 `tts_model`만 바꾸면 된다.
- 카메라 이미지가 OpenAI로 나간다(요청에 `store: false`). 사람이 찍히는 곳이면 `camera:=none` 또는 `use_image:=false`.
- Wi-Fi(인터넷)가 끊기면 GPT·OpenAI TTS가 안 된다. Fault 안내를 꼭 들어야 하면 `fallback_tts_command`를 설정한다.
- 지연 시간은 아직 실측하지 않았다(GPT 응답 + TTS 두 번의 왕복). 너무 느리면 `state_mode: template`, 더 작은 모델, `reasoning_effort: low`.
- 음성 TTS는 raw PCM(24 kHz, 16-bit mono)으로 받아 WAV 헤더를 직접 붙인다. 소리가 빠르거나 느리면 `tts_sample_rate` 확인.

## 노트북에서 시험 (ROS·로봇 없이, Python 가상환경만)

로봇이 쓰는 것과 같은 판단 코드(Narrator, StateTracker)에 **가상 상태머신**(`craft_voice/sim.py`, planning_master와
같은 label·diagnostics 형식)과 **노트북 웹캠**을 붙인다. `/planning/state`와 공유 메모리 대신 쓴다는 것만 다르다.

Linux / macOS:

```bash
python3 -m venv ~/voice-venv && . ~/voice-venv/bin/activate && pip install openai==3.42.1 opencv-python numpy
cd ~/craft/src/craft_voice && export OPENAI_API_KEY=sk-... && python -m craft_voice.demo
```

Windows (PowerShell, 스피커는 winsound 사용 — Windows에서는 아직 실행해 보지 않았다):

```powershell
py -m venv $HOME\voice-venv; & $HOME\voice-venv\Scripts\Activate.ps1; pip install openai==3.42.1 opencv-python numpy
cd craft\src\craft_voice; $env:OPENAI_API_KEY="sk-..."; python -m craft_voice.demo
```

기본 시나리오(반복): Idle 3 s → Ready 6 s → base_goal 실행 10 s → 완료 → approach 실행 8 s → Fault(실행 시간 초과) 12 s.
실행 중 입력: 그냥 문장 = 질문, `/describe`, `/status`, `/state Run.Execute@approach`, `/fault localization lost`,
`/resume`(시나리오로 복귀), `/say 안녕`, `/mute`, `/unmute`, `/stop`, `/quit`. GPT 응답 시간은 로그 `GPT state 1.8s ...`로 나온다.

| 옵션 | 기본값 | 설명 |
| --- | --- | --- |
| `--camera` | `0` | 웹캠 번호, 사진·동영상 경로, `none` |
| `--scenario` | 위 시나리오 | `Label[@step][!reason]:초, ...` (reason에 `:`가 있어도 됨) |
| `--template` | 끔 | 상태 안내는 고정 문장(GPT는 질문·장면만) |
| `--narrate-period` / `--narrate-idle-period` | `20` / `0` | 일하는 동안 / 대기 중 상황 중계 주기 (가상 이동 속도도 넣어 줌) |
| `--player` | `auto` | Linux paplay→aplay→ffplay, macOS afplay, Windows winsound. 또는 `{file}`이 든 명령 |
| `--fallback-tts` | `""` | 예: `espeak-ng -v ko --stdout -- {text}` (Linux) |
| `--model`, `--voice`, `--base-url` | `gpt-5.4-mini,gpt-6-luna,gpt-5.5`, `coral`, `""` | 로봇 설정과 같은 의미 |

키가 없으면 고정 문장만 말한다(`--fallback-tts`가 있으면 그 음성으로).

## 시간 재기 (`python -m craft_voice.bench`)

"GPT 응답을 받은 뒤 음성을 만들어 스피커가 시작할 때까지"를 로봇과 같은 코드 경로로 잰다.

```bash
cd ~/craft/src/craft_voice && python -m craft_voice.bench                       # OpenAI TTS (키 필요)
python -m craft_voice.bench --gpt --camera photo.jpg                             # GPT 응답 + 사건→소리 전체
python -m craft_voice.bench --fallback-tts "espeak-ng -v ko --stdout -- {text}"   # 오프라인 TTS
```

| 항목 | 뜻 |
| --- | --- |
| `tts` | 문장 → WAV (OpenAI TTS 또는 fallback) |
| `to_play` | 문장 넘김 → 플레이어 시작 (대기열 + TTS + WAV + 프로세스 실행) |
| `audio` | 말 길이 |
| `gpt`, `event->play` | `--gpt`일 때: GPT 응답 시간, 상태 변화 → 플레이어 시작 |

GPT 모델 고르기: 로봇의 실제 중계 프롬프트와 사진(없으면 그린 장면)으로 모델 × reasoning을 나란히 잰다.

```bash
python -m craft_voice.bench --compare "gpt-5.4-mini,gpt-6-luna,gpt-5.5,gpt-5.6-luna" --efforts none,low --runs 3 --camera test/images
```

`--camera`에 폴더를 주면 실행마다 사진을 돌려 쓴다. `test/images/`에는 OpenCV `samples/data`의 사진 3장
(실내 사무실 바닥, 바닥의 둥근 물체들, 상자, Apache-2.0)이 들어 있다.

이 저장소의 개발 컨테이너(Xeon 2.8 GHz, 사운드 카드 없이 `aplay -D null`)에서 잰 값, 10회 중앙값:
코드 자체 오버헤드(TTS 0 s로 둠) `to_play` 0.00 s, espeak-ng `tts` 0.02 s·`to_play` 0.01 s, 말 길이 2.5~5.0 s.
Jetson·경기장 Wi-Fi에서는 네트워크가 달라 다시 재야 한다.

실측 2026-11-03, Claude Code 클라우드 컨테이너에서 Network secret(프록시가 키를 넣음)으로 OpenAI 직접 호출,
로봇의 실제 중계 프롬프트 + `test/images` 사진 3장, 중앙값:

| 모델 (reasoning none) | GPT 응답 | 비고 |
| --- | --- | --- |
| **gpt-5.4-mini** | **1.02 s** (0.99~1.80, n=6) | 기본값. 문장 자연스러움, 사진 묘사는 짧은 편 |
| gpt-6-luna | 1.59 s (1.31~2.08, n=6) | 사진 묘사가 더 구체적(책상·의자 등) |
| gpt-5.6-luna | 1.60 s (1.36~1.94, n=6) | 반말·존댓말이 섞임 |
| gpt-5.5 | 2.47 s (n=3) | reasoning low가 1.93 s로 오히려 빠름 |

같은 조건에서 reasoning low는 none보다 0.3~0.5 s 느렸다. 기본값(gpt-5.4-mini, none)으로 상태 변화 → 플레이어 시작
(GPT 문장 + 사진 + OpenAI TTS) **2.89 s** (2.38~3.15, n=5), 그중 TTS(문장 → 플레이어 시작) 1.25 s.
같은 측정을 노트북(이전 기본값 gpt-5.5·low, 사진 없음)에서 했을 때는 3.67 s였다(네트워크가 달라 직접 비교는 참고만).

**TTS 스트리밍**(`tts_stream: true`, 기본): OpenAI 음성을 다 받기 전에, 첫 조각이 오는 대로 `stream_player_command`
(aplay raw stdin)로 흘려 보낸다. 같은 날 같은 환경 실측(중앙값, n=6):

| 구간 | 전체 WAV를 받고 재생 | 스트리밍 |
| --- | --- | --- |
| 문장 → 첫 소리(플레이어에 첫 오디오) | 1.38 s (0.72~2.14) | **0.89 s** (0.53~1.18) |
| 상태 변화 → 첫 소리 (GPT + 사진 + TTS) | 2.89 s (n=5) | **1.88 s** (1.54~2.37) |

음성은 말하는 속도보다 빨리 도착해서(약 5 s 분량이 1.4 s 안에) 재생 중 끊기지 않는다. 첫 조각이 `tts_deadline_s` 안에
안 오거나 에러면 fallback TTS로, 첫 조각이 왔을 때 이미 상태가 바뀌었으면 말하지 않는다. 멈춤(Fault) 문장은 지금처럼
전체 WAV(짧은 마감 + fallback)로 말한다. macOS/Windows처럼 stdin 재생기가 없으면 `tts_stream: false`(데모는 자동).

## 테스트

ROS 없이 돈다(rclpy 불필요, `test_gpt_http.py`만 openai SDK 필요 — 로컬 가짜 서버라 key·인터넷 불필요):

```bash
cd ~/craft/src/craft_voice && python3 -m pytest -q test
```

| 파일 | 확인하는 것 |
| --- | --- |
| `test_frames.py` | 공유 메모리 슬롯 복사(D435/C310), 시퀀스 불일치·락·잘못된 메타·파일 재생성, ROS Image 변환, JPEG 축소 |
| `test_camera.py` | 인식 입력과 같은 시각의 프레임 고르기(최신이 아닌 그 프레임), 덮어써진 칸·맞는 시각 없음·2 s 지남 → 최신, 상태가 바뀐 순간의 입력, 같은 프레임 한 번만, 스레드 동시 사용 |
| `test_robot_state.py` | 하는 일·보이는 것 문장(조사 포함), 멈춤을 상황으로 말하고 오류 원문은 읽지 않음, 상태 변화 감지, 쿨다운, 1틱 Run.Complete, Fault 원인 대기·이전 원인 재사용 안 함, 문맥 문장 |
| `test_narrator.py` | 일하는 동안만 주기 중계, 센서 인식이 GPT 프롬프트에 들어감, GPT 경로, 템플릿 폴백, Fault 우선·끊기(재생 중·TTS 합성 중·TTS가 멈춘 네트워크, 실제 스레드), Fault 음성 마감 후 fallback, 죽은 네트워크에서도 상태 안내(GPT·TTS 마감), 지난 상태·status 문장 버리기/바꾸기(TTS 뒤 재확인 포함), 종료 중 발행 안 함, 호출 상한, 주기 장면, mute/stop |
| `test_gpt_http.py` | 실제 openai SDK가 보내는 `/v1/responses`·`/v1/audio/speech` 요청 내용, reasoning 거부 시 재시도, WAV, 응답 없는 서버에서 Fault 음성이 빨리 포기 |
| `test_audio.py` | 플레이어 명령(`{file}`), stop으로 끊기, 시작 전 취소, 오류, shell 없이 `{text}` 전달, 문장이 옵션(`-w` 등)이 되지 않음 |
| `test_sim.py`, `test_demo.py` | 가상 상태머신 형식(planning_master와 같은 diagnostics), 데모를 실제 프로세스로 띄워 가짜 OpenAI 서버와 끝까지(질문·장면·상태·Fault·재생) |
