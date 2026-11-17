#!/usr/bin/env bash
# Dedicated deployment prefix; do not reuse the developer's incremental install tree.
set -eo pipefail
craft_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
craft_ros_distro="${ROS_DISTRO:-lyrical}"
source "/opt/ros/${craft_ros_distro}/setup.bash"
cd "$craft_repo_root"
if [[ ! -f src/coin_d4_driver/package.xml ]]; then
  echo 'Missing coin_d4_driver. Run: vcs import src < craft.repos' >&2
  exit 1
fi
colcon --log-base log_runtime build --base-paths src \
  --build-base build_runtime --install-base install_runtime \
  --packages-up-to craft_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
python3 tools/package_runtime.py
