#!/usr/bin/env bash
# 4-pane terminator dashboard: one craft.launch.py group per pane.
#   +-------------+-------------+
#   | PERCEPTION  |  PLANNING   |
#   +-------------+-------------+
#   |  CONTROL    |    INFRA    |
#   +-------------+-------------+
# Usage: craft_dashboard.sh [launch args...]   e.g. craft_dashboard.sh start_lidar:=false
# Ctrl+C in a pane stops only that group; the pane drops to a shell with the workspace sourced.
set -euo pipefail

ws="${CRAFT_WS:-$(cd "$(dirname "$(readlink -f "$0")")/../../.." && pwd)}"
setup="$ws/install/setup.bash"
[[ -f "$setup" ]] || { echo "not found: $setup (build first or set CRAFT_WS)" >&2; exit 1; }
command -v terminator >/dev/null || { echo "terminator is not installed" >&2; exit 1; }

args="$*"
pane() {  # pane <group>
  printf "bash -c 'source %s; ros2 launch craft_bringup craft.launch.py group:=%s %s; exec bash'" \
    "$setup" "$1" "$args"
}

config="$(mktemp --suffix=.terminator)"
trap 'rm -f "$config"' EXIT
cat > "$config" <<EOF
[global_config]
  title_hide_sizetext = True
[profiles]
  [[default]]
    scrollback_infinite = True
[layouts]
  [[craft]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1800, 1000
      title = CRAFT dashboard
    [[[rows]]]
      type = VPaned
      parent = window0
      order = 0
    [[[top]]]
      type = HPaned
      parent = rows
      order = 0
    [[[bottom]]]
      type = HPaned
      parent = rows
      order = 1
    [[[perception]]]
      type = Terminal
      parent = top
      order = 0
      profile = default
      title = PERCEPTION
      command = "$(pane perception)"
    [[[planning]]]
      type = Terminal
      parent = top
      order = 1
      profile = default
      title = PLANNING
      command = "$(pane planning)"
    [[[control]]]
      type = Terminal
      parent = bottom
      order = 0
      profile = default
      title = CONTROL
      command = "$(pane control)"
    [[[infra]]]
      type = Terminal
      parent = bottom
      order = 1
      profile = default
      title = INFRA (sensors / LiDAR / EKF / SLAM)
      command = "$(pane infra)"
[plugins]
EOF

terminator -u -g "$config" -l craft
