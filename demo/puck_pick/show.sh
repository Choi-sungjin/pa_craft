#!/usr/bin/env bash
# Throwaway demo: plan the puck pick and play it in RViz (no hardware). Delete with: rm -rf ~/craft/demo
# show.sh --lying: puck on its side, gripped on its long axis (plan_lying.yaml)
set -e
cd "$(dirname "$(readlink -f "$0")")"
WS=$(cd ../.. && pwd)
source /opt/ros/lyrical/setup.bash
/usr/bin/python3 -I plan.py "$@"
PLAN=plan.yaml; [ "$1" = --lying ] && PLAN=plan_lying.yaml
URDF=$WS/src/craft_planning/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf
/usr/bin/python3 -c "import yaml,sys;print(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':open(sys.argv[1]).read()}}}))" "$URDF" > /tmp/puck_demo_rsp.yaml
source "$WS/src/craft_planning/v75_ws/install/setup.bash"   # package:// meshes for RViz
# Isolate the preview: another PC on the LAN (domain 0) publishes its own /robot_description.
# Set after every 'source' (ros_environment's hook resets the discovery range to SUBNET).
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
trap 'kill 0' EXIT
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file /tmp/puck_demo_rsp.yaml &
/usr/bin/python3 show.py "$PLAN" &
rviz2 -d demo.rviz
