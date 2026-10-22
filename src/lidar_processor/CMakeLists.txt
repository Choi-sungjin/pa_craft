cmake_minimum_required(VERSION 3.8)
project(lidar_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(lidar_processor src/lidar_processor.cpp)
target_link_libraries(lidar_processor rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2_ros::tf2_ros)

add_executable(lidar_calibrator src/lidar_calibrator.cpp)
target_link_libraries(lidar_calibrator rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2_ros::tf2_ros)

install(TARGETS lidar_processor lidar_calibrator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/find_lidar_ports.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz udev DESTINATION share/${PROJECT_NAME})

ament_package()
