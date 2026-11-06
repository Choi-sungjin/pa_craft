#!/usr/bin/env bash
# Throwaway demo: plan the puck pick and play it in RViz (no hardware). Delete with: rm -rf ~/craft/demo
# show.sh --lying: puck on its side, gripped on its long axis (plan_lying.yaml)
# show.sh --sis1:  the whole sis1 run (pick, load into clock slot 12, back), from execute.py's steps
# show.sh --sis2:  the sis2 run (take the puck out of the clock with the jog-taught poses), execute.py's steps
set -e
cd "$(dirname "$(readlink -f "$0")")"
WS=$(cd ../.. && pwd)
source /opt/ros/lyrical/setup.bash
# show.sh --sis1: the whole sis1 step list (execute.py) as one loop, step name on top (sis1_sim.py)
if [ "$1" = --sis1 ]; then
  /usr/bin/python3 -I plan.py
  (source "$WS/install/setup.bash" && /usr/bin/python3 sis1_sim.py)
  PLAN=sis1_sim.yaml
# show.sh --sis2: the sis2 run (nominal -> via -> clock grasp with the jog-taught poses -> elbow forward), sis2_sim.py
elif [ "$1" = --sis2 ]; then
  (source "$WS/install/setup.bash" && /usr/bin/python3 sis2_sim.py)
  PLAN=sis2_sim.yaml
else
  /usr/bin/python3 -I plan.py "$@"
  PLAN=plan.yaml; [ "$1" = --lying ] && PLAN=plan_lying.yaml
fi
URDF=$WS/src/craft_planning/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf
# Preview only: drop the camera bracket on the tool (tool_roll_link_vis_orange/yellow/green, out to the side).
/usr/bin/python3 - "$URDF" > /tmp/puck_demo_rsp.yaml <<'PY'
import re, sys, yaml
u = open(sys.argv[1]).read()
for part in ('orange', 'yellow', 'green'):
    n = f'tool_roll_link_vis_{part}'
    u = re.sub(rf'\s*<link name="{n}">.*?</link>', '', u, flags=re.S)
    u = re.sub(rf'\s*<joint name="{n}_joint"[^>]*>.*?</joint>', '', u, flags=re.S)
print(yaml.safe_dump({'/**': {'ros__parameters': {'robot_description': u}}}))
PY
source "$WS/src/craft_planning/v75_ws/install/setup.bash"   # package:// meshes for RViz
# Isolate the preview: another PC on the LAN (domain 0) publishes its own /robot_description.
# Set after every 'source' (ros_environment's hook resets the discovery range to SUBNET).
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
trap 'kill 0' EXIT
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/puck_demo_rsp.yaml &
/usr/bin/python3 show.py "$PLAN" &
rviz2 -d demo.rviz
