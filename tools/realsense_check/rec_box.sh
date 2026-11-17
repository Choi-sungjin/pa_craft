#!/bin/bash
# 어퍼캠 RGB+뎁스 (RealSense db3) + 전면 LiDAR (/lidar/front/scan + /tf_static, rosbag2) 동시 녹화. Ctrl+C로 정지.
#   ./rec_box.sh [out_dir [rs_bag.py args]]      (default ~/bags/boxgap_MMDD_HHMM)
OUT=${1:-$HOME/bags/boxgap_$(date +%m%d_%H%M)}
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash; source ~/craft/install/setup.bash
trap true INT                                     # Ctrl+C stops rs_bag.py (foreground); the rest are stopped below
setsid ros2 launch lidar_processor lidar_processor.launch.py start_processor:=false > "$OUT/lidar_launch.log" 2>&1 &
LAUNCH=$!
timeout 20 ros2 topic echo --once /lidar/front/scan > /dev/null 2>&1 || echo "!! /lidar/front/scan 안 나옴 ($OUT/lidar_launch.log)"
ros2 bag record -o "$OUT/lidar" --topics /lidar/front/scan /tf_static > "$OUT/lidar_bag.log" 2>&1 &
BAG=$!
sleep 1
python3 "$(dirname "$0")/rs_bag.py" --out "$OUT" "${@:2}"   # extra args, e.g. --seconds 40
kill -TERM $BAG 2>/dev/null; wait $BAG
kill -TERM -- -$LAUNCH 2>/dev/null; wait $LAUNCH   # whole group: driver + static TF
ros2 bag info "$OUT/lidar" 2>/dev/null | grep -E "Duration|Messages|Topic:"
ls -la "$OUT"
# bundle for the analysis PC: instructions, code, URDF, baseline series
mkdir -p "$OUT/code"; H="$(dirname "$0")"
cp "$H/BOXGAP_INSTRUCTIONS.md" "$OUT/INSTRUCTIONS.md"
cp "$H/box_range.py" "$H/read_bag.py" "$H/rs_bag.py" "$H/rec_box.sh" "$H/out/series2.csv" "$H/../../src/craft_description/urdf/v307_full_robot.urdf" "$OUT/code/"
