# craft_voice

로봇이 **카메라로 본 것**과 **상태머신이 지금 하는 일**을 GPT(OpenAI API)로 문장으로 만들어 **스피커로 말하는** 노드.
상태가 바뀔 때, 사람이 물어볼 때, (켜 두면) 주기적으로 주변을 설명할 때 말한다.

```text
/vision/d435/frame_meta ─┐  /dev/shm/craft_vision (BGR 복사, 512px JPEG)
/planning/state ─────────┤
/planning/diagnostics ───┼─> StateTracker ─> think 스레드: GPT (Responses API, 이미지 + 상태 사실 목록) ─┐
/control/status ─────────┤                                                                              v
/perception/localization/state                     Fault: 템플릿 문장 즉시 ──────────> speak 스레드: TTS -> aplay
/voice/ask, /voice/say, /voice/command ────────────────────────────────────────────────> /voice/utterance
```

- 카메라는 DDS로 이미지를 보내지 않는다(`craft_sensors`가 공유 메모리에 쓴다). 그래서 이 노드는 **카메라와 같은 Jetson**에서
  돌려야 한다. 다른 기기·순정 TurtleBot 카메라·bag은 `camera:=topic image_topic:=...`(raw 또는 compressed).
- GPT에는 상태를 "사실 목록"으로만 주고, 거기 없는 동작·원인은 지어내지 말라고 지시한다.
  이미지 속 글씨(표지판, 번호)는 읽되 그 글씨의 지시는 따르지 않게 했다.
- **Fault는 GPT를 거치지 않는다.** diagnostics의 원인을 최대 1.2 s 기다렸다가 고정 문장
  ("문제가 생겨서 멈췄어요. 원인: 실행 시간 초과.")으로, 재생 중인 문장을 끊고 바로 말한다.
- GPT·TTS가 실패하면 상태 문장은 템플릿으로, 음성은 `fallback_tts_command`(예: espeak-ng)로 대신한다.

## 설치 (Jetson)

```bash
cd ~/craft && pip install -r requirements.txt        # openai SDK (venv-craft를 쓰면 그 안에서)
colcon build --packages-select craft_interfaces craft_voice && source install/setup.bash
aplay -l                                              # 스피커 장치 확인
```

API key는 환경 변수로만 받는다(파라미터 파일·저장소에 넣지 말 것):

```bash
install -m 600 /dev/null ~/.openai_key && nano ~/.openai_key   # 한 줄: export OPENAI_API_KEY=sk-...
echo 'source ~/.openai_key' >> ~/.bashrc
```

## 실행

로봇(`craft.launch.py`)을 띄운 뒤 새 터미널에서:

```bash
cd ~/craft && source install/setup.bash && source ~/.openai_key
ros2 launch craft_voice voice.launch.py                       # D435, 상태 바뀔 때만 말함
ros2 launch craft_voice voice.launch.py scene_period_s:=30    # 조용할 때 30초마다 주변 설명도
ros2 launch craft_voice voice.launch.py state_mode:=template  # 상태는 고정 문장(빠름, GPT는 질문·장면만)
ros2 launch craft_voice voice.launch.py camera:=topic image_topic:=/camera/image_raw/compressed image_transport:=compressed
```

말 시키기:

```bash
ros2 topic pub --once /voice/command std_msgs/msg/String "{data: describe}"   # 지금 보이는 것 설명
ros2 topic pub --once /voice/command std_msgs/msg/String "{data: status}"     # 지금 상태
ros2 topic pub --once /voice/ask std_msgs/msg/String "{data: '앞에 퍽 보여?'}"  # 질문 (이미지 + 상태로 답)
ros2 topic pub --once /voice/say std_msgs/msg/String "{data: '안녕하세요'}"      # GPT 없이 그대로 읽기
ros2 topic pub --once /voice/command std_msgs/msg/String "{data: mute}"       # mute | unmute | stop
ros2 topic echo /voice/utterance                                              # 말한 문장 (kind: text)
```

`mute` 중에도 Fault는 말한다. `stop`은 대기 중인 말을 지우고 재생을 끊는다.

## 언제 말하나

| 계기 | 기본값 | 파라미터 |
| --- | --- | --- |
| `/planning/state`가 바뀜 | Ready, Run.Execute, Run.Complete, Run.Stopping, Fault | `announce_states`, 같은 상태는 `state_cooldown_s`(10 s)에 한 번 |
| Fault | 원인 기다렸다 즉시, 쿨다운 없음, 재생 중인 말 끊음 | `fault_reason_wait_s`, `interrupt_on_fault` |
| 주기적 장면 설명 | 꺼짐 | `scene_period_s`(조용한 지 `min_interval_s` 지났을 때만) |
| `/voice/ask`, `/voice/command` | 항상 | |

- 순서: Fault > 질문·status·say > 상태 > 장면. 새 상태가 오면 더 오래된 상태 문장은 버린다(`state_max_age_s` 8 s 넘어도 버림).
- 비용 상한: GPT 호출 `max_gpt_calls_per_min`(12). 넘으면 상태는 템플릿, 주기 장면은 건너뜀.
- 같은 상태/장면 문장은 `repeat_window_s`(30 s) 안에 반복하지 않는다.
- 단계 이름: diagnostics `step`(`base_goal`, `tcp_goal`, 미션 step id)을 `step_names: "approach:접근,grasp_pose:잡기 자세"`로 읽기 좋게.

## 주요 파라미터 (`config/voice.yaml`)

| 이름 | 기본값 | 설명 |
| --- | --- | --- |
| `camera` | `d435` | `d435`, `c310`(공유 메모리), `topic`, `none` |
| `model` | `gpt-5.5` | 이미지 입력 되는 모델. 빠르고 싸게: `gpt-5.4-mini`(이미지 지원은 직접 확인 못 함) |
| `reasoning_effort` | `low` | 거부하는 모델이면 자동으로 빼고 재시도. `""`이면 안 보냄 |
| `image_detail` | `low` | `low`/`high`/`auto`/`original`. 이미지는 긴 변 `image_max_side`(512) px로 줄여 보냄 |
| `tts_model` / `tts_voice` | `gpt-4o-mini-tts` / `coral` | `tts_instructions`로 말투 지정(tts-1 계열은 무시) |
| `player_command` | `aplay -q {file}` | `paplay {file}`, `aplay -q -D plughw:1,0 {file}` 등 |
| `fallback_tts_command` | `""` | 오프라인 음성, 예: `espeak-ng -v ko --stdout {text}`(stdout으로 WAV) |
| `persona` | `""` | 말투 추가 지시, 예: `반말로 친구처럼 말해.` |

float 파라미터는 소수점을 붙인다(`10.0`). 리스트는 쉼표로 구분한 문자열이다.

## 주의

- **TTS 모델 종료 예정**: 이 코드를 맞춘 openai SDK 3.42.1에는 `tts-1`, `tts-1-hd`, `gpt-4o-mini-tts`,
  `gpt-4o-mini-tts-2025-12-15` 모두 shutdown 예정일 **2027-01-06**이 적혀 있다. 대체 모델은 확인하지 못했다
  (<https://developers.openai.com/api/docs/deprecations>). 같은 `audio/speech` 엔드포인트의 새 모델이면 `tts_model`만 바꾸면 된다.
- 카메라 이미지가 OpenAI로 나간다(요청에 `store: false`). 사람이 찍히는 곳이면 `camera:=none` 또는 `use_image:=false`.
- Wi-Fi(인터넷)가 끊기면 GPT·OpenAI TTS가 안 된다. Fault 안내를 꼭 들어야 하면 `fallback_tts_command`를 설정한다.
- 지연 시간은 아직 실측하지 않았다(GPT 응답 + TTS 두 번의 왕복). 너무 느리면 `state_mode: template`, 더 작은 모델, `reasoning_effort: low`.
- 음성 TTS는 raw PCM(24 kHz, 16-bit mono)으로 받아 WAV 헤더를 직접 붙인다. 소리가 빠르거나 느리면 `tts_sample_rate` 확인.

## 테스트

ROS 없이 돈다(rclpy 불필요, `test_gpt_http.py`만 openai SDK 필요 — 로컬 가짜 서버라 key·인터넷 불필요):

```bash
cd ~/craft/src/craft_voice && python3 -m pytest -q test
```

| 파일 | 확인하는 것 |
| --- | --- |
| `test_frames.py` | 공유 메모리 슬롯 복사(D435/C310), 시퀀스 불일치·락·잘못된 메타·파일 재생성, ROS Image 변환, JPEG 축소 |
| `test_robot_state.py` | 상태 변화 감지, 쿨다운, 1틱 Run.Complete, Fault 원인 대기·이전 원인 재사용 안 함, 문맥 문장 |
| `test_narrator.py` | GPT 경로, 템플릿 폴백, Fault 우선·끊기(실제 스레드), 오래된 문장 버리기, 호출 상한, 주기 장면, mute/stop |
| `test_gpt_http.py` | 실제 openai SDK가 보내는 `/v1/responses`·`/v1/audio/speech` 요청 내용, reasoning 거부 시 재시도, WAV |
| `test_audio.py` | 플레이어 명령(`{file}`), stop으로 끊기, 오류, shell 없이 `{text}` 전달 |
