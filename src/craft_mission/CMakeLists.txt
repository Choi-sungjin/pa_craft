cmake_minimum_required(VERSION 3.8)
project(craft_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_fsm src/mission_fsm.cpp)
target_link_libraries(mission_fsm
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

install(TARGETS
  mission_fsm
  DESTINATION lib/${PROJECT_NAME})

ament_package()
