#!/usr/bin/env bash
# perception_master (upper-cam puck survey only) + survey_view.py on a res_record.py session.
#   demo/puck_compare/survey_view.sh ~/craft/bags_robot/puck_res/pucks_1112_0944_1920x1080
# Own ROS domain on localhost, so it does not talk to the robot.
set -e
SESSION=${1:?session dir}
source /opt/ros/lyrical/setup.bash
source ~/craft/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VIEW:-91} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
SHARE=$(ros2 pkg prefix craft_perception)/share/craft_perception
ARGS=(--ros-args --params-file "$SHARE/config/perception.yaml"
      --params-file "$(ros2 pkg prefix craft_bringup)/share/craft_bringup/config/rates.yaml" -p max_input_age_s:=2.0)
for t in d435_yolo apriltag object_detector obstacle_detector puck_axis puck_survey traffic_light lidar_obstacle state_estimator; do
  ARGS+=(-p "schedule.$t.enabled:=false")
done
ros2 run craft_perception perception_master "${ARGS[@]}" > /tmp/survey_view_master.log 2>&1 &
MASTER=$!
trap 'kill -INT $MASTER 2>/dev/null; wait $MASTER 2>/dev/null' EXIT
sleep 4
/usr/bin/python3 "$(dirname "$0")/survey_view.py" "$SESSION"
