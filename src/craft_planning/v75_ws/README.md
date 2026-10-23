# v75_ws

V75 로봇 RViz 표시 + 팔 IK 워크스페이스.

```
v75_ws/src/
├─ v75_robot_description/  URDF(원본), 메시, display.launch.py (관절 슬라이더)
└─ v75_arm_ik/             IK 코어, arm_ik 명령줄 도구, ik_rviz_node, ik_demo.launch.py
```

## 처음 한 번: 설치와 빌드

```bash
sudo apt install ros-$ROS_DISTRO-pinocchio ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-xacro
cd ~/craft/v75_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
```

## 실행

```bash
cd ~/craft/v75_ws && source install/setup.bash

# 1) 슬라이더로 관절을 직접 움직여 보기
ros2 launch v75_robot_description display.launch.py

# 2) IK 결과를 RViz에서 보기
ros2 launch v75_arm_ik ik_demo.launch.py
#    다른 터미널에서 (source install/setup.bash 후) 목표 보내기: [x, y, z (mm), pitch, roll (deg)]
ros2 run v75_arm_ik ik_goal --knob 90                 # 카메라 정렬 → LM 이동 → 잡고 노브 90° 돌리기
ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 0     # 노브 수직 하향 잡기 (바퀴+LM+팔)
ros2 run v75_arm_ik ik_goal --grasp 80 -140 116 -90   # 노브 돌리기
ros2 run v75_arm_ik ik_goal 150 390 195 -30 20       # 팔만 이동

# 3) 터미널에서 IK 계산만
ros2 run v75_arm_ik arm_ik ik 150 390 195 -30 20
```

URDF를 고치면(`src/v75_robot_description/urdf/v75_robot.urdf`) `--symlink-install`로 빌드했기 때문에
다시 빌드하지 않아도 다음 실행부터 RViz와 IK 모두 반영된다.
