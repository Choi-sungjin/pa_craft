cmake_minimum_required(VERSION 3.8)
project(craft_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/craft_dashboard.sh scripts/save_map.sh scripts/odom_trail.py DESTINATION lib/${PROJECT_NAME})

ament_package()
