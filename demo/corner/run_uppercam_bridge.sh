#!/usr/bin/env bash
# Upper-cam bridge with the Python environment it needs (2026-11-09):
#   pyrealsense2 lives only in venv-craft, whose numpy 2.x breaks the system scipy; the system python has numpy 1.x and
#   scipy but no pyrealsense2. So: system python + a dir holding only a link to the venv's pyrealsense2.
# Usage: demo/corner/run_uppercam_bridge.sh [extra bridge args]
set -e
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
RS_ONLY="$HOME/.cache/craft_rs_only"
if [ ! -e "$RS_ONLY/pyrealsense2" ]; then
  mkdir -p "$RS_ONLY"
  ln -sfn "$REPO/venv-craft/lib/python3.12/site-packages/pyrealsense2" "$RS_ONLY/pyrealsense2"
fi
[ -n "$SSH_CLIENT" ] && export ROS_STATIC_PEERS=${SSH_CLIENT%% *}
source "$REPO/install/setup.bash"
export PYTHONPATH="$RS_ONLY:$PYTHONPATH"
/usr/bin/python3 -c "import pyrealsense2, scipy" || { echo "[uppercam] pyrealsense2/scipy import failed"; exit 1; }
cd "$REPO"
exec /usr/bin/python3 demo/corner/uppercam_sensor_bridge.py \
  --urdf "$REPO/src/craft_description/urdf/v307_full_robot.urdf" \
  --calibration "$REPO/demo/corner/uppercam_selected_calibration.json" \
  --enable-validated-square "$@"
