#!/usr/bin/env bash
# Upper-cam box sensor bridge: C++ node craft_sensors/uppercam_bridge (2026-11-12, port of uppercam_sensor_bridge.py,
# which stays here as the reference: same topics and decisions, geometry checked on boxgap_1108_1236).
# Usage: demo/corner/run_uppercam_bridge.sh [extra ROS args, e.g. -p preview_hz:=2.0 -p serial:=...]
set -e
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
[ -n "$SSH_CLIENT" ] && export ROS_STATIC_PEERS=${SSH_CLIENT%% *}
source "$REPO/install/setup.bash"
C="$REPO/demo/corner"
# nice 10: the bridge yields CPU to control/EKF when the Jetson is full (2026-11-12)
# preview_hz 15: traffic_light counts 1 s blinks on the preview
exec nice -n 10 "$REPO/install/craft_sensors/lib/craft_sensors/uppercam_bridge" --ros-args \
  -p urdf:="$REPO/src/craft_description/urdf/v307_full_robot.urdf" \
  -p calibration:="$C/uppercam_selected_calibration.json" \
  -p policy:="$C/uppercam_sensor_policy.json" \
  -p enable_validated_square:=true -p preview_hz:=15.0 "$@"
