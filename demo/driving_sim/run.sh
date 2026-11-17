#!/usr/bin/env bash
# Isolated simulation: production planner/controller algorithms, no hardware drivers or cameras.
set -eo pipefail
sim_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_root="$(cd -- "$sim_dir/../.." && pwd)"
source "$sim_root/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${DRIVE_SIM_DOMAIN_ID:-216}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
mkdir -p "$sim_dir/runs"
export ROS_LOG_DIR="$sim_dir/runs/ros_log"
sim_pids=()
sim_rviz=true
sim_repeat=true
for arg in "$@"; do
  case "$arg" in
    --no-rviz) sim_rviz=false ;;
    --once) sim_repeat=false ;;
    --verify-only) exec python3 "$sim_dir/sensors.py" --verify-only ;;
    *) echo "Usage: $0 [--no-rviz] [--once] [--verify-only]" >&2; exit 2 ;;
  esac
done
cleanup() {
  trap - EXIT INT TERM
  for child in "${sim_pids[@]}"; do kill -INT "$child" 2>/dev/null || true; done
  for child in "${sim_pids[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
if [[ ! -f "$sim_root/tools/driving_sim/build/CMakeCache.txt" ]]; then
  cmake -S "$sim_root/tools/driving_sim" -B "$sim_root/tools/driving_sim/build"
fi
cmake --build "$sim_root/tools/driving_sim/build" -j2 > "$sim_dir/runs/build.log" 2>&1
python3 "$sim_dir/generate_map.py" "$sim_dir/world.yaml" > "$sim_dir/runs/map_generation.log"
python3 - "$sim_dir" "$sim_root" <<'PY'
import sys,yaml,json,time
from pathlib import Path
sim_dir,root=map(Path,sys.argv[1:])
world=yaml.safe_load((sim_dir/'world.yaml').read_text())
(sim_dir/'runs/run_started.json').write_text(json.dumps({'started':time.time()}))
report=sim_dir/'runs/report.json'
if report.exists():
    (sim_dir/'runs/previous_report.json').write_bytes(report.read_bytes())
    report.unlink()
robot=(root/'src/craft_description/urdf/v307_full_robot.urdf').read_text()
(sim_dir/'runs/robot_description.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':robot,'publish_frequency':50.}}}))
(sim_dir/'runs/plant_params.yaml').write_text(yaml.safe_dump({'drive_sim_plant':{'ros__parameters':{'control_yaml':str(root/'src/craft_control/config/control.yaml'),'start':[float(v) for v in world['start']]}}}))
PY
"$sim_root/tools/driving_sim/build/drive_plant" --ros-args --params-file "$sim_dir/runs/plant_params.yaml" > "$sim_dir/runs/plant.log" 2>&1 &
sim_pids+=("$!")
sim_rsp_prefix="$(ros2 pkg prefix robot_state_publisher)"
"$sim_rsp_prefix/lib/robot_state_publisher/robot_state_publisher" --ros-args --params-file "$sim_dir/runs/robot_description.yaml" > "$sim_dir/runs/robot_state.log" 2>&1 &
sim_pids+=("$!")
python3 -u "$sim_dir/sensors.py" --ros-args -p "world_yaml:=$sim_dir/world.yaml" -p "report_file:=$sim_dir/runs/report.json" -p "repeat:=$sim_repeat" > "$sim_dir/runs/sensors.log" 2>&1 &
sim_pids+=("$!")
"$sim_root/install/craft_planning/lib/craft_planning/planning_master" --ros-args \
  --params-file "$sim_root/src/craft_planning/config/planning.yaml" \
  --params-file "$sim_root/src/craft_bringup/config/rates.yaml" \
  --params-file "$sim_dir/planning_override.yaml" \
  -p "map_file:=$sim_dir/map.yaml" > "$sim_dir/runs/planning.log" 2>&1 &
sim_pids+=("$!")
if [[ "$sim_rviz" == true ]]; then
  rviz2 -d "$sim_dir/drive.rviz" > "$sim_dir/runs/rviz.log" 2>&1 &
  sim_pids+=("$!")
fi
echo "Driving simulation on isolated ROS domain $ROS_DOMAIN_ID; logs $sim_dir/runs"
wait -n "${sim_pids[@]}"
