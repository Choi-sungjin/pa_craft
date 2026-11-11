# Standalone (not a colcon package): cmake -S . -B build && cmake --build build
# Links slam_toolbox's Karto library directly. Linking ${slam_toolbox_LIBRARIES} instead pulls in every
# dependency down to RViz/Ogre and GNU ld spent >20 min on it on the Jetson (2026-11-01).
cmake_minimum_required(VERSION 3.16)
project(posegraph_tool CXX)
set(CMAKE_CXX_STANDARD 17)
set(ROS /opt/ros/lyrical)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization thread system)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_library(KARTO kartoSlamToolbox PATHS ${ROS}/lib REQUIRED)
add_executable(posegraph_tool posegraph_tool.cpp)
target_include_directories(posegraph_tool PRIVATE ${ROS}/include ${EIGEN3_INCLUDE_DIR})
target_link_libraries(posegraph_tool ${KARTO} rclcpp_lifecycle::rclcpp_lifecycle rclcpp::rclcpp
  Boost::serialization Boost::thread Boost::system tbb)
set_target_properties(posegraph_tool PROPERTIES BUILD_RPATH ${ROS}/lib)
