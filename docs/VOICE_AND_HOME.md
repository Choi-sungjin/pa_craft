# 음성 서비스와 home 복귀 명령

## 음성 서비스

`src/craft_voice`는 Jetson의 `~/craft-voice`에서 사용하는 음성 패키지입니다. 상태머신·인지·배터리 정보를 읽고, 정해진 안내·녹음·음성 합성으로 말합니다. 마이크 질문도 받으며 주행·팔 명령을 발행하지 않습니다. 자세한 토픽·설정·시험 방법은 [패키지 설명](../src/craft_voice/README.md)에 있습니다.

음성 합성과 질문 응답의 모델 이름·목소리·마이크·재생 명령은 `src/craft_voice/config/voice_robot.yaml`에서 설정합니다. 별도 모델 가중치를 저장소에 넣는 구조가 아니라 OpenAI API와 장치의 녹음·캐시를 사용합니다. API 키, 원본 녹음, 생성된 음성 캐시는 저장소에 포함하지 않습니다.

음성만 준비할 때는 기존 모델 도구용 `requirements.txt` 전체 대신 사용 중인 Python 환경에 `openai==3.42.1`을 설치합니다. ROS의 rclpy·메시지와 NumPy·OpenCV도 필요합니다. 키는 `~/.config/craft_voice/openai.env` 등 장치의 별도 파일에 두고 [예제](../src/craft_voice/env/openai.env.example)의 변수 이름만 참조합니다.

이미 `craft-voice.service`가 실행 중인 Jetson에서는 두 번째 음성 노드를 동시에 시작하지 않습니다. 아래 명령은 서비스가 없는 환경에서 사용자가 직접 실행하는 예시입니다.

```bash
cd ~/craft
bash src/craft_voice/scripts/run_voice.sh
```

이 경로는 colcon 설치 없이 소스에서 실행합니다. `scripts/craft-voice.service`는 기존 `~/craft-voice` 배치를 위한 예제이므로 `~/craft/src/craft_voice`에서 사용하려면 복사한 서비스의 `ExecStart`를 실제 `scripts/run_voice.sh` 경로로 맞춥니다. 기존 키·녹음·캐시와 서비스 설정은 별도로 보존합니다.

음성은 통합 launch에 자동 추가하지 않습니다. 기존 9개 패키지의 `craft-runtime.tar.gz`에도 음성 서비스와 `home` 설정을 자동 포함하지 않습니다. 소스 배포본에서 별도로 설정합니다.

## .bashrc의 home 명령

Jetson의 기존 `home()` 함수는 ROS 환경을 읽고 `tools/go_home.py`를 실행합니다. 같은 기능을 다른 터미널에서도 등록할 수 있도록 [home.bash](../tools/shell/home.bash)로 제공합니다. 파일을 읽는 것만으로는 로봇이 움직이지 않습니다.

아래 한 줄을 `~/.bashrc`의 **기존 home 함수보다 뒤**에 한 번 넣습니다. 소스 작업 경로가 `~/craft`가 아니면 해당 경로로 바꿉니다.

```bash
source "$HOME/craft/tools/shell/home.bash"
```

현재 터미널에도 등록하고 등록 여부만 확인하려면:

```bash
source "$HOME/craft/tools/shell/home.bash"
type home
```

`home`을 직접 실행하면 미션을 중단·초기화하고, 위치를 보정한 뒤 시작 위치로 주행하며 방향을 맞춥니다. 목표는 기존 `tools/go_home.py`가 `src/craft_planning/config/mission.yaml`의 `home`에서 읽습니다. 맵과 실행파일은 기존 도구가 사용하는 `~/maps/brief_arena1cm.yaml`, `~/craft/install/.../global_localize`에 있어야 합니다. 실제 주행은 사용자가 주변 공간과 제어 상태를 확인한 뒤 실행합니다.

등록 파일의 환경 변수는 다음과 같습니다. 기존 복귀 도구 안의 장치·맵 경로를 바꾸지는 않습니다.

| 변수 | 기본값 | 용도 |
| --- | --- | --- |
| `CRAFT_WS` | 등록 파일이 들어 있는 저장소 | setup.bash와 go_home.py를 찾는 경로 |
| `ROS_DISTRO` | `lyrical` | ROS 설치 환경 |
| `ROS_DOMAIN_ID` | `61` | 로봇이 사용하는 통신 영역 |
| `CRAFT_HOME_ROS_SETUP` | `/opt/ros/<ROS_DISTRO>/setup.bash` | 별도 ROS 설치 경로 |
| `CRAFT_HOME_PYTHON` | `/usr/bin/python3` | rclpy를 불러올 수 있는 Python 실행파일 |

## 로봇 구동 없는 검사

음성 테스트는 가짜 API·메시지·재생기를 사용합니다. 실제 마이크·스피커 출력 또는 주행 검증을 대신하지는 않습니다. `home` 등록 테스트는 임시 ROS 환경과 가짜 실행파일로 명령 전달·오류·환경 격리를 검사합니다.

```bash
cd ~/craft
source /opt/ros/lyrical/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/src/craft_voice:$PYTHONPATH" python3 -m pytest -q src/craft_voice/test tools/shell/test_home.py
```
