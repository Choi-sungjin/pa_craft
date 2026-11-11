#!/usr/bin/env bash
# Save the running SLAM map BEFORE stopping slam_toolbox (on the robot, same ROS domain):
#   <dir>/<name>.pgm + .yaml  occupancy grid (nav2 map_saver, from /perception/lidar/map)
#   <dir>/<name>.posegraph + .data  slam_toolbox pose graph, to continue mapping or localize later
# Usage: save_map.sh <name> [dir]      dir defaults to ~/maps; an existing <name> is never overwritten.
set -euo pipefail

name="${1:?usage: save_map.sh <name> [dir]}"
dir="${2:-$HOME/maps}"
out="$dir/$name"
mkdir -p "$dir"
for f in "$out.yaml" "$out.pgm" "$out.posegraph"; do
  [[ -e "$f" ]] && { echo "exists: $f (pick another name)" >&2; exit 1; }
done

ros2 run nav2_map_server map_saver_cli -t /perception/lidar/map -f "$out" \
  --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10.0
timeout 20 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$out'}" \
  || echo "pose graph not saved: is slam_toolbox still running?" >&2
ls -la "$out".*
