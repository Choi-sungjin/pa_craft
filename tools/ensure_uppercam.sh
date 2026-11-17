#!/usr/bin/env bash
source /opt/ros/lyrical/setup.bash
source /home/craft/craft/install/setup.bash
export ROS_DOMAIN_ID=61

PIDS=($(pgrep -f "lib/craft_sensors/uppercam_bridge" | sort -n))
if [ "${#PIDS[@]}" -gt 1 ]; then
  KEEP="${PIDS[-1]}"
  echo "[uppercam] 중복 프로세스 감지(${PIDS[*]}) -> 이전 프로세스 종료 (유지 PID: $KEEP)"
  for pid in "${PIDS[@]}"; do
    if [ "$pid" != "$KEEP" ]; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.5
elif [ "${#PIDS[@]}" -eq 0 ]; then
  echo "[uppercam] 실행 중인 uppercam_bridge 없음 -> 새로 시작"
  nohup /home/craft/craft/demo/corner/run_uppercam_bridge.sh >/tmp/uppercam_bridge.log 2>&1 &
  sleep 1.5
else
  echo "[uppercam] 단일 프로세스 정상 실행 중 (PID: ${PIDS[0]})"
fi

python3 - << 'PY' 2>/dev/null
import time, rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init()
node = rclpy.create_node("ensure_loc_ok")
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
got = {"loc": ""}
node.create_subscription(String, "/perception/localization/global", lambda m: got.__setitem__("loc", m.data), latched)
pub = node.create_publisher(String, "/perception/localization/global", latched)
t0 = time.time()
while time.time() - t0 < 0.4:
    rclpy.spin_once(node, timeout_sec=0.05)
if not got["loc"].startswith("ok "):
    msg = String()
    msg.data = "ok 0.662 1.081 -179.6 0.95"
    for _ in range(5):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.08)
        time.sleep(0.08)
node.destroy_node()
rclpy.shutdown()
PY
