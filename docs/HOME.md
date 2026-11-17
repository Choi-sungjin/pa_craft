# home 복귀 명령 등록

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

등록 테스트는 임시 ROS 환경과 가짜 실행파일로 명령 전달·오류·종료 코드·환경 격리를 검사합니다. ROS 노드나 실제 복귀 코드는 실행하지 않습니다.

```bash
cd ~/craft
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tools/shell/test_home.py
bash -n tools/shell/home.bash
```
