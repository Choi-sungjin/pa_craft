#!/usr/bin/env bash
# Ask the running voice node for a remark by name on /voice/event (std_msgs/String), e.g. from the firmware
# upload in ~/.bashrc (_craft_fw):  ~/craft-voice/scripts/voice_event.sh firmware_done
# Names: craft_voice/signals.py EVENT_REMARKS, with key=value numbers after the name in one argument, e.g.
#   voice_event.sh "gripcam_measured top=61 yaw=-22.9"   voice_event.sh "puck_loaded slot=7"
# Same ROS setup as run_voice.sh (ROS_DOMAIN_ID default 61).
# Returns at once when nobody listens (the voice service is off): it never holds up the caller for long.
set +u
NAME="${1:?usage: voice_event.sh <name>}"
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
exec timeout 15 python3 - "$NAME" <<'PYEOF'
import sys, time
import rclpy
from std_msgs.msg import String
rclpy.init()
node = rclpy.create_node('voice_event')
pub = node.create_publisher(String, '/voice/event', 10)
end = time.monotonic() + 3.0
while pub.get_subscription_count() == 0 and time.monotonic() < end:   # wait for discovery
    rclpy.spin_once(node, timeout_sec=0.1)
if pub.get_subscription_count():
    pub.publish(String(data=sys.argv[1]))
    rclpy.spin_once(node, timeout_sec=0.3)
    print(f'[voice] /voice/event {sys.argv[1]}')
else:
    print('[voice] no voice node listening on /voice/event')
node.destroy_node()
rclpy.shutdown()
PYEOF
