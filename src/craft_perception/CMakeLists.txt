cmake_minimum_required(VERSION 3.8)
project(craft_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(obstacle_detector src/obstacle_detector.cpp)
target_link_libraries(obstacle_detector
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(object_pose_estimator src/object_pose_estimator.cpp)
target_link_libraries(object_pose_estimator
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

add_executable(object_detector src/object_detector.cpp)
target_link_libraries(object_detector
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  state_estimator
  obstacle_detector
  object_pose_estimator
  object_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/aruco_reader.py DESTINATION lib/${PROJECT_NAME} RENAME aruco_reader)
install(PROGRAMS scripts/depth_heatmap.py DESTINATION lib/${PROJECT_NAME} RENAME depth_heatmap)

ament_package()
