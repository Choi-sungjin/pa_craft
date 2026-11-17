#!/usr/bin/env bash
# Actual mrun mission stack; replace wheel OpenCR and raw LiDAR devices only. No cameras.
set -eo pipefail
mission_here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mission_root="$(cd -- "$mission_here/../.." && pwd)"
source "$mission_root/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${MISSION_SIM_DOMAIN_ID:-215}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=''
mission_out="$mission_here/runs"
mkdir -p "$mission_out"
export ROS_LOG_DIR="$mission_out/ros_log"
mission_pids=()
cleanup() {
  trap - EXIT INT TERM
  for child in "${mission_pids[@]}"; do kill -INT "$child" 2>/dev/null || true; done
  for child in "${mission_pids[@]}"; do wait "$child" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
if [[ "${1:-}" == '--verify-only' ]]; then exec python3 -m pytest -q "$mission_here/test_hardware.py"; fi
rm -f "$mission_out/ports.yaml"
python3 -u "$mission_here/hardware_sim.py" --ros-args -p "ports_file:=$mission_out/ports.yaml" > "$mission_out/hardware.log" 2>&1 &
mission_pids+=("$!")
for attempt in {1..100}; do
  [[ -f "$mission_out/ports.yaml" ]] && break
  kill -0 "${mission_pids[0]}" 2>/dev/null || { cat "$mission_out/hardware.log"; exit 1; }
  sleep .1
done
[[ -f "$mission_out/ports.yaml" ]] || { echo 'PTY setup timed out' >&2; exit 1; }
python3 - "$mission_here" "$mission_root" <<'PY'
import sys,yaml,json,time
from pathlib import Path
here,root=map(Path,sys.argv[1:]);out=here/'runs'
ports=yaml.safe_load((out/'ports.yaml').read_text())
assert ports['control_master']['ros__parameters']['clock_wheel_port'].startswith('/dev/pts/')
params=ports
params['perception_master']={'ros__parameters':{f'schedule.{name}.enabled':False for name in ('d435_yolo','apriltag','object_detector','obstacle_detector','puck_axis','traffic_light','puck_survey','uppercam_puck')}}
params['planning_master']={'ros__parameters':{'box_approach.camera_enabled':False,'box_approach.camera_only_longitudinal':False}}
params['mission_master']={'ros__parameters':{'log_dir':str(out/'mission_logs')}}
(out/'overlay.yaml').write_text(yaml.safe_dump(params))
robot=(root/'src/craft_description/urdf/v307_full_robot.urdf').read_text()
(out/'robot_description.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':robot}}}))
(out/'started.json').write_text(json.dumps({'started':time.time(),'scope':'actual mrun stack, wheel PTY and raw LiDAR devices; camera inputs are excluded fixtures'}))
PY
# Launch the same production composition and mrun maps. The overlay changes only device ports/camera exclusions.
ros2 launch craft_bringup craft.launch.py \
  mission:=true arm:=false localization:=slam \
  map_file:=/home/craft/maps/brief_arena1cm.yaml \
  slam_map:=/home/craft/maps/noboxes_legend_opt_arena \
  start_lidar:=false start_cameras:=false \
  "params_overlay:=$mission_out/overlay.yaml" > "$mission_out/stack.log" 2>&1 &
mission_pids+=("$!")
"$mission_root/install/craft_sensors/lib/craft_sensors/uppercam_bridge" --ros-args \
  -p "urdf:=$mission_root/src/craft_description/urdf/v307_full_robot.urdf" \
  -p "calibration:=$mission_root/demo/corner/uppercam_selected_calibration.json" \
  -p "policy:=$mission_root/demo/corner/uppercam_sensor_policy.json" \
  -p camera_enabled:=false -p enable_validated_square:=true > "$mission_out/lidar_box.log" 2>&1 &
mission_pids+=("$!")
"$(ros2 pkg prefix robot_state_publisher)/lib/robot_state_publisher/robot_state_publisher" \
  --ros-args --params-file "$mission_out/robot_description.yaml" > "$mission_out/robot_model.log" 2>&1 &
mission_pids+=("$!")
python3 -u "$mission_here/monitor.py" --ros-args -p "report_file:=$mission_out/report.json" > "$mission_out/monitor.log" 2>&1 &
mission_pids+=("$!")
if [[ "${1:-}" != '--no-rviz' ]]; then
  rviz2 -d "$mission_here/mission.rviz" > "$mission_out/rviz.log" 2>&1 &
  mission_pids+=("$!")
fi
echo "Existing arena/full mission simulation, isolated domain $ROS_DOMAIN_ID. Logs: $mission_out"
wait -n "${mission_pids[@]}"
