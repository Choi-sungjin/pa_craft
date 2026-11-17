#!/usr/bin/env bash
# Decision visualization on its own ROS domain. Closing RViz stops this simulation.
set -eo pipefail
sim_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_workspace="$(cd -- "$sim_here/../.." && pwd)"
source "$sim_workspace/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${PICK_SIM_DOMAIN_ID:-217}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
sim_output="$sim_here/runs/policy_sim"
mkdir -p "$sim_output"
export ROS_LOG_DIR="$sim_output/ros_log"
if [[ "${1:-}" == '--verify-only' ]]; then
  exec python3 "$sim_here/pick_policy_sim.py" --verify-only --report "$sim_output/verification.json"
fi
sim_pid=''
rviz_pid=''
cleanup() {
  trap - EXIT INT TERM
  for sim_child in "$sim_pid" "$rviz_pid"; do
    if [[ -n "$sim_child" ]] && kill -0 "$sim_child" 2>/dev/null; then
      kill -INT "$sim_child" 2>/dev/null || true
    fi
  done
  wait "$sim_pid" "$rviz_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
python3 -u "$sim_here/pick_policy_sim.py" --report "$sim_output/verification.json" "$@" > "$sim_output/simulation.log" 2>&1 &
sim_pid=$!
rviz2 -d "$sim_here/pick_policy_sim.rviz" > "$sim_output/rviz.log" 2>&1 &
rviz_pid=$!
echo "Pick decision simulation: ROS domain $ROS_DOMAIN_ID; logs: $sim_output"
wait -n "$sim_pid" "$rviz_pid"
