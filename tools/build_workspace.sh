#!/usr/bin/env bash
# Build the ROS packages and camera helpers without starting robot nodes.
set -eo pipefail
craft_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
craft_ros_distro="${ROS_DISTRO:-lyrical}"
source "/opt/ros/${craft_ros_distro}/setup.bash"
cd "$craft_repo_root"

if [[ ! -f src/coin_d4_driver/package.xml ]]; then
  echo 'Missing coin_d4_driver. Run: vcs import src < craft.repos' >&2
  exit 1
fi

colcon build --base-paths src --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release "$@"
