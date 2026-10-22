cmake_minimum_required(VERSION 3.8)
project(lidar_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(scan_merger src/scan_merger.cpp)
target_compile_features(scan_merger PUBLIC cxx_std_17)
target_include_directories(scan_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(scan_merger PUBLIC ${sensor_msgs_TARGETS})

add_executable(lidar_processor src/lidar_processor.cpp)
target_link_libraries(lidar_processor scan_merger rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2_ros::tf2_ros)

add_executable(lidar_calibrator src/lidar_calibrator.cpp)
target_link_libraries(lidar_calibrator rclcpp::rclcpp ${sensor_msgs_TARGETS} tf2_ros::tf2_ros)

install(TARGETS lidar_processor lidar_calibrator DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/find_lidar_ports.sh DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz udev DESTINATION share/${PROJECT_NAME})
install(TARGETS scan_merger EXPORT scan_merger_export
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(scan_merger_export HAS_LIBRARY_TARGET)
ament_export_dependencies(sensor_msgs)

ament_package()
