cmake_minimum_required(VERSION 3.8)
project(craft_sensors)

# Eigen/OpenCV geometry is ~100x slower unoptimised (uppercam_bridge 570 ms/frame at -O0, 2026-11-13).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# librealsense built with CUDA in ~/.local/librealsense-cuda (rs2::align on the GPU) wins over the ROS package;
# falls back to it.
find_package(realsense2 QUIET PATHS $ENV{HOME}/.local/librealsense-cuda NO_DEFAULT_PATH)
if(NOT realsense2_FOUND)
  find_package(realsense2 REQUIRED)
endif()
message(STATUS "realsense2 from ${realsense2_DIR}")
find_package(craft_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(Eigen3 REQUIRED)

add_executable(realsense_processor src/realsense_processor.cpp)
target_link_libraries(realsense_processor
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  realsense2::realsense2
)


# Upper-cam / LiDAR box sensor bridge (C++ port of demo/corner/uppercam_sensor_bridge.py, 2026-11-12).
add_executable(uppercam_bridge src/uppercam_bridge.cpp src/uppercam_geometry.cpp)
target_include_directories(uppercam_bridge PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(uppercam_bridge
  rclcpp::rclcpp
  ${craft_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  realsense2::realsense2
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
# Offline: the geometry on recorded frames (boxgap read_bag.py format), for checks against the Python estimator.
add_executable(uppercam_geometry_check src/uppercam_geometry_check.cpp src/uppercam_geometry.cpp)
target_include_directories(uppercam_geometry_check PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(uppercam_geometry_check ${OpenCV_LIBS} Eigen3::Eigen)
# Offline: the box finder on dumped /perception/lidar/obstacle messages (exit 1 if a found box is over 2 cm off).
add_executable(uppercam_find_box_check src/uppercam_find_box_check.cpp src/uppercam_geometry.cpp)
target_include_directories(uppercam_find_box_check PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(uppercam_find_box_check ${OpenCV_LIBS} Eigen3::Eigen)

# ROS setup puts /opt/ros/.../librealsense2.so.2.57 (same soname, no CUDA) on LD_LIBRARY_PATH; an old-style
# DT_RPATH (searched before LD_LIBRARY_PATH, unlike RUNPATH) keeps the binaries on the librealsense they were built with.
get_filename_component(REALSENSE_LIB_DIR "${realsense2_DIR}/../.." ABSOLUTE)
set_target_properties(realsense_processor uppercam_bridge PROPERTIES INSTALL_RPATH "${REALSENSE_LIB_DIR}")
target_link_options(realsense_processor PRIVATE -Wl,--disable-new-dtags)
target_link_options(uppercam_bridge PRIVATE -Wl,--disable-new-dtags)

install(TARGETS
  realsense_processor
  uppercam_bridge
  uppercam_geometry_check
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
