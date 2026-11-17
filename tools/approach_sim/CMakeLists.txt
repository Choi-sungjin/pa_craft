cmake_minimum_required(VERSION 3.16)
project(approach_sim CXX)
# Builds the robot's own planner and wheel controllers from src/ into one offline simulator.
#   source /opt/ros/lyrical/setup.bash && cmake -S . -B build && cmake --build build -j4
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PNG REQUIRED)
set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)
add_executable(approach_sim approach_sim.cpp
  ${SRC}/craft_planning/src/path_planner.cpp
  ${SRC}/craft_control/src/lateral_controller.cpp
  ${SRC}/craft_control/src/longitudinal_controller.cpp)
target_include_directories(approach_sim PRIVATE ${SRC}/craft_planning/include ${SRC}/craft_control/include)
target_link_libraries(approach_sim PRIVATE ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} yaml-cpp PNG::PNG)
# Diagnostics: the same build with the controller's private state printed (SIM_DEBUG=<case name>).
add_executable(approach_sim_peek approach_sim.cpp
  ${SRC}/craft_planning/src/path_planner.cpp
  ${SRC}/craft_control/src/lateral_controller.cpp
  ${SRC}/craft_control/src/longitudinal_controller.cpp)
target_compile_definitions(approach_sim_peek PRIVATE SIM_PEEK)
target_include_directories(approach_sim_peek PRIVATE ${SRC}/craft_planning/include ${SRC}/craft_control/include)
target_link_libraries(approach_sim_peek PRIVATE ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} yaml-cpp PNG::PNG)
