#!/usr/bin/env bash
# craft 2D 플래닝 시뮬레이터
#   src/craft_sim/sim.sh build                         빌드 (ncnn 없으면 시뮬레이터 전용 작업공간 .sim_ws 에)
#   src/craft_sim/sim.sh run 02_box_L1 [--rviz]        시나리오 하나 (이름 또는 yaml 경로)
#   src/craft_sim/sim.sh all [--rviz]                  scenarios/*.yaml 전부 + 요약표
#   src/craft_sim/sim.sh rviz                          같은 ROS 환경으로 RViz 만
#   src/craft_sim/sim.sh test                          단위 시험 (pytest)
# 결과: $CRAFT_SIM_OUT (기본 ~/craft_sim_runs)/<시나리오>_<시각>/report.html
# 로봇·다른 PC와 섞이지 않도록 ROS_DOMAIN_ID=63(CRAFT_SIM_DOMAIN), 이 컴퓨터 안에서만 찾기(LOCALHOST)로 고정한다.
set -e
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"   # src/craft_sim
REPO="$(cd "$HERE/../.." && pwd)"
SIMWS="${CRAFT_SIM_WS:-$REPO/.sim_ws}"
DISTRO="${ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | head -n 1)}"

ros_base() {
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/$DISTRO/setup.bash"
}

ws_install() {  # 시뮬레이터 전용 작업공간이 있으면 그것, 없으면 저장소 install
  if [ -f "$SIMWS/install/setup.bash" ]; then echo "$SIMWS/install"; else echo "$REPO/install"; fi
}

ros_env() {
  ros_base
  local inst; inst="$(ws_install)"
  if [ ! -f "$inst/setup.bash" ]; then echo "[sim] 빌드가 없음: $0 build 먼저 실행" >&2; exit 1; fi
  # shellcheck disable=SC1091
  source "$inst/setup.bash"
  export ROS_DOMAIN_ID="${CRAFT_SIM_DOMAIN:-63}"
  export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  unset ROS_STATIC_PEERS CYCLONEDDS_URI
  export RMW_IMPLEMENTATION="${CRAFT_SIM_RMW:-rmw_fastrtps_cpp}"
  export CRAFT_SIM_SRC="${CRAFT_SIM_SRC:-$REPO/src}"
}

have_ncnn() {
  [ -n "$ncnn_DIR" ] && return 0
  find /usr /usr/local /opt -name 'ncnnConfig.cmake' -print -quit 2>/dev/null | grep -q .
}

build() {
  ros_base
  if [ ! -d "/opt/ros/$DISTRO/share/pinocchio" ]; then
    if [ -n "$CRAFT_SIM_PINOCCHIO" ]; then
      export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:$CRAFT_SIM_PINOCCHIO"
    else
      echo "[sim] ros-$DISTRO-pinocchio 가 없음: sudo apt install ros-$DISTRO-pinocchio (또는 CRAFT_SIM_PINOCCHIO=<pinocchio prefix>)" >&2
    fi
  fi
  local pkgs=(craft_planning craft_control craft_sim)
  if have_ncnn; then
    cd "$REPO"
    colcon build --symlink-install --packages-up-to "${pkgs[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  else
    # ncnn 이 없는 PC: craft_perception 대신 메시지 + tag_registry 만 만드는 대체 패키지(perception_stub)로 따로 빌드
    echo "[sim] ncnn 없음 → 시뮬레이터 전용 작업공간 $SIMWS (craft_perception 은 perception_stub 으로)"
    mkdir -p "$SIMWS/src/craft_perception"
    for p in craft_interfaces craft_control craft_planning lidar_processor craft_sim; do
      ln -sfn "$REPO/src/$p" "$SIMWS/src/$p"
    done
    ln -sfn "$HERE/perception_stub/CMakeLists.txt" "$SIMWS/src/craft_perception/CMakeLists.txt"
    ln -sfn "$HERE/perception_stub/package.xml" "$SIMWS/src/craft_perception/package.xml"
    ln -sfn "$REPO/src/craft_perception/msg" "$SIMWS/src/craft_perception/msg"
    cd "$SIMWS"
    CRAFT_SRC="$REPO/src" colcon build --symlink-install --packages-up-to "${pkgs[@]}" \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  fi
}

cmd="${1:-help}"; shift || true
case "$cmd" in
  build) build ;;
  run)   ros_env; exec ros2 run craft_sim run "$@" ;;
  all)
    ros_env
    status=0
    for f in "$(ros2 pkg prefix craft_sim)"/share/craft_sim/scenarios/*.yaml; do
      echo "=== $(basename "$f")"
      ros2 run craft_sim run "$f" "$@" || status=1
    done
    ros2 run craft_sim summary || true
    exit $status ;;
  rviz)  ros_env; exec rviz2 -d "$(ros2 pkg prefix craft_sim)/share/craft_sim/rviz/sim.rviz" ;;
  test)  ros_base; cd "$HERE" && CRAFT_SIM_SRC="$REPO/src" python3 -m pytest -q test ;;
  *)     sed -n 2,9p "$0" ;;
esac
