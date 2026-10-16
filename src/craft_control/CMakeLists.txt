cmake_minimum_required(VERSION 3.8)
project(craft_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(control_master src/control_master.cpp src/longitudinal_controller.cpp src/lateral_controller.cpp)
target_include_directories(control_master PRIVATE include)
target_link_libraries(control_master
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

install(TARGETS
  control_master
  DESTINATION lib/${PROJECT_NAME})

ament_package()
