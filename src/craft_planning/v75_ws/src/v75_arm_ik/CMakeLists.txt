cmake_minimum_required(VERSION 3.16)
project(v75_arm_ik CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Pinocchio: URDF 로드, FK, 자코비안 (urdfdom, Eigen, Boost를 함께 끌어옴)
find_package(pinocchio REQUIRED)

# 기본 URDF 경로 (실행 시 --urdf, 환경변수 ARM_IK_URDF, ROS 파라미터 urdf 로 바꿀 수 있음).
# URDF 내용은 실행할 때마다 읽으므로, URDF를 수정해도 재빌드할 필요 없음.
# 옆에 v75_robot_description 패키지가 있으면 그 URDF를 그대로 사용 (원본 하나만 관리)
set(_desc_urdf "${CMAKE_CURRENT_SOURCE_DIR}/../v75_robot_description/urdf/v75_robot.urdf")
if(EXISTS "${_desc_urdf}")
  get_filename_component(_default_urdf "${_desc_urdf}" ABSOLUTE)
else()
  set(_default_urdf "${CMAKE_CURRENT_SOURCE_DIR}/urdf/v75_robot.urdf")
endif()
set(ARM_IK_URDF "${_default_urdf}" CACHE FILEPATH "Default URDF used by arm_ik")
message(STATUS "arm_ik default URDF: ${ARM_IK_URDF}")

# 1) 명령줄 도구 (ROS 불필요)
add_executable(arm_ik src/arm_ik.cpp)
target_include_directories(arm_ik PRIVATE include)
target_link_libraries(arm_ik PRIVATE pinocchio::pinocchio)
target_compile_definitions(arm_ik PRIVATE DEFAULT_URDF="${ARM_IK_URDF}")

# 2) ROS2 노드 (ROS 환경에서 colcon 으로 빌드할 때만)
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(ik_rviz_node src/ik_rviz_node.cpp)
  target_include_directories(ik_rviz_node PRIVATE include)
  target_compile_definitions(ik_rviz_node PRIVATE DEFAULT_URDF="${ARM_IK_URDF}")
  target_link_libraries(ik_rviz_node
    pinocchio::pinocchio
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2_ros::tf2_ros
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS})

  # 목표를 한 번 보내는 도구: ros2 run v75_arm_ik ik_goal X Y Z PITCH ROLL
  add_executable(ik_goal src/ik_goal.cpp)
  target_link_libraries(ik_goal rclcpp::rclcpp ${std_msgs_TARGETS})

  install(TARGETS arm_ik ik_rviz_node ik_goal DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
  ament_package()
else()
  message(STATUS "ament_cmake 없음: ROS 노드는 빌드하지 않고 arm_ik 만 빌드")
endif()
