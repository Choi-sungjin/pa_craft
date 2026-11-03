#!/usr/bin/env bash
# V186 URDF puck pick preview in RViz (no hardware). Replans only when plan.py is newer than plan.yaml
# (or with --plan); planning takes about a minute on the Jetson.
set -e
cd "$(dirname "$(readlink -f "$0")")"
HERE=$(pwd)
[ -d model/meshes/visual ] || { echo "model/ missing: run ./get_model.sh first"; exit 1; }
source /opt/ros/lyrical/setup.bash
if [ "$1" = --plan ] || [ ! plan.yaml -nt plan.py ]; then /usr/bin/python3 -I plan.py; fi
# Mesh paths in the CAD export are relative ('../meshes/...'); RViz needs absolute file:// URIs.
# Collision geometry is dropped (model/ keeps only the visual meshes).
RSP=/tmp/v186_demo_rsp.yaml
/usr/bin/python3 -I -c "import re,yaml,sys
u=re.sub(r'<collision\b.*?</collision>', '', open(sys.argv[1]).read(), flags=re.S)
u=u.replace('filename=\"../meshes/', 'filename=\"file://'+sys.argv[2]+'/model/meshes/')
print(yaml.safe_dump({'/**':{'ros__parameters':{'robot_description':u}}}))" model/urdf/v186_full_robot.urdf "$HERE" > $RSP
# Isolate the preview: another PC on the LAN (domain 0) publishes its own /robot_description.
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
trap 'kill 0' EXIT
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $RSP &
/usr/bin/python3 show.py &
rviz2 -d demo.rviz
