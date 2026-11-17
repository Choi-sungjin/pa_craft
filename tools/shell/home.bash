# ~/.bashrc: source "$HOME/craft/tools/shell/home.bash"
# Register the command only. Sourcing this file never starts a ROS node.
_CRAFT_HOME_DEFAULT_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

home() {
  (
    local ws="${CRAFT_WS:-$_CRAFT_HOME_DEFAULT_WS}"
    local ros_setup="${CRAFT_HOME_ROS_SETUP:-/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash}"
    if [[ ! -r "$ws/install/setup.bash" || ! -r "$ws/tools/go_home.py" ]]; then
      echo "[home] 작업 경로 또는 빌드 결과를 찾지 못했어요: $ws" >&2
      return 1
    fi
    source "$ros_setup" || return
    source "$ws/install/setup.bash" || return
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
    "${CRAFT_HOME_PYTHON:-/usr/bin/python3}" "$ws/tools/go_home.py" "$@" \
      2> >(grep -v 'RTPS_TRANSPORT_SHM Error' >&2)
  )
}
