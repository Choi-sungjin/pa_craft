cmake_minimum_required(VERSION 3.8)
project(craft_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(control_master src/control_master.cpp src/longitudinal_controller.cpp src/lateral_controller.cpp
  src/arm_controller.cpp src/gripper_controller.cpp src/clock_controller.cpp)
target_include_directories(control_master PRIVATE include)
target_compile_features(control_master PRIVATE cxx_std_17)
target_link_libraries(control_master
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  control_master
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_control_controllers test/test_control_controllers.cpp
    src/arm_controller.cpp src/clock_controller.cpp src/lateral_controller.cpp
    src/longitudinal_controller.cpp)
  target_include_directories(test_control_controllers PRIVATE include)
  target_compile_features(test_control_controllers PRIVATE cxx_std_17)
  target_link_libraries(test_control_controllers ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})
endif()

ament_package()
