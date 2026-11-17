cmake_minimum_required(VERSION 3.8)
project(craft_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(craft_interfaces REQUIRED)
find_package(craft_perception REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(odom_trail src/odom_trail.cpp)
target_link_libraries(odom_trail rclcpp::rclcpp ${nav_msgs_TARGETS} ${visualization_msgs_TARGETS})
add_executable(box_marker src/box_marker.cpp)
target_link_libraries(box_marker rclcpp::rclcpp ${visualization_msgs_TARGETS} ${craft_interfaces_TARGETS}
  ament_index_cpp::ament_index_cpp yaml-cpp)
add_executable(puck_marker src/puck_marker.cpp)
target_link_libraries(puck_marker rclcpp::rclcpp ${visualization_msgs_TARGETS}
  craft_perception::craft_perception__rosidl_typesupport_cpp
  tf2::tf2 tf2_ros::tf2_ros)
add_executable(global_localize src/global_localize.cpp)
target_link_libraries(global_localize rclcpp::rclcpp tf2::tf2 tf2_ros::tf2_ros ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS} ${std_msgs_TARGETS} yaml-cpp ${OpenCV_LIBS})
foreach(t odom_trail box_marker puck_marker global_localize)
  target_compile_features(${t} PRIVATE cxx_std_17)
  target_compile_options(${t} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
# NumPy evaluates the grid arithmetic without FMA. On this ARM host -O3
# contraction changed y=0.9750000000000001 to 0.975 and crossed the 0.3 m
# NMS boundary (near-search runner-up 0.349375 -> 0.546256, brief_arena1cm).
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(global_localize PRIVATE -ffp-contract=off)
endif()
install(TARGETS odom_trail box_marker puck_marker global_localize DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/craft_dashboard.sh scripts/save_map.sh scripts/odom_trail.py scripts/box_marker.py scripts/puck_marker.py scripts/craft_tui.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_puck_marker test/test_puck_marker.py TIMEOUT 30)
  add_executable(map_tools_cli test/map_tools_cli.cpp)  # for test/map_tools_parity.py
  target_compile_features(map_tools_cli PRIVATE cxx_std_17)
  target_compile_options(map_tools_cli PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(map_tools_cli PRIVATE -ffp-contract=off)
  endif()
  target_link_libraries(map_tools_cli yaml-cpp ${OpenCV_LIBS})
endif()

ament_package()
