cmake_minimum_required(VERSION 3.16)
project(craft_planning)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(craft_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(craft_perception REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PNG REQUIRED)
find_package(Threads REQUIRED)

add_library(planning_modules
  src/state_machine.cpp src/path_planner.cpp src/arm_planner.cpp src/planning_config.cpp)
target_compile_features(planning_modules PUBLIC cxx_std_17)
target_compile_options(planning_modules PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(planning_modules PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE v75_ws/src/v75_arm_ik/include)
target_link_libraries(planning_modules PUBLIC rclcpp::rclcpp ${craft_interfaces_TARGETS}
  ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS} ${sensor_msgs_TARGETS}
  PRIVATE pinocchio::pinocchio yaml-cpp PNG::PNG Threads::Threads
    craft_perception::perception_modules ament_index_cpp::ament_index_cpp)

add_executable(planning_master src/planning_master.cpp)
target_compile_features(planning_master PRIVATE cxx_std_17)
target_compile_options(planning_master PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(planning_master PRIVATE planning_modules
  ament_index_cpp::ament_index_cpp tf2_ros::tf2_ros ${tf2_geometry_msgs_TARGETS}
  craft_perception::craft_perception__rosidl_typesupport_cpp ${std_msgs_TARGETS} ${diagnostic_msgs_TARGETS})
install(TARGETS planning_modules planning_master
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES v75_ws/src/v75_robot_description/urdf/v75_robot.urdf
  DESTINATION share/${PROJECT_NAME}/urdf)

# Keep the original KANT implementation importable for reference and regression tests.
set(reference_python_dir "${CMAKE_CURRENT_BINARY_DIR}/reference_python/${PROJECT_NAME}")
file(MAKE_DIRECTORY "${reference_python_dir}")
foreach(source __init__.py path_planner.py kant_astar_arc.py)
  configure_file("src/${source}" "${reference_python_dir}/${source}" COPYONLY)
endforeach()
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "${reference_python_dir}")
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.stl" PATTERN "*.pgm" PATTERN "*.png" PATTERN "*.yaml" PATTERN "*.txt")
install(DIRECTORY config third_party DESTINATION share/${PROJECT_NAME})
install(FILES README.md IMPLEMENTATION.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(module state_machine path_planner_cpp arm_planner planning_config)
    ament_add_gtest(test_${module} test/test_${module}.cpp)
    target_link_libraries(test_${module} planning_modules)
    target_compile_definitions(test_${module} PRIVATE
      PLANNING_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf"
      CRAFT_PLANNING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endforeach()
  target_link_libraries(test_path_planner_cpp PNG::PNG yaml-cpp)
  ament_add_pytest_test(test_path_planner test/test_path_planner.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR})
  ament_add_pytest_test(test_planning_master_integration test/test_planning_master_integration.py
    TIMEOUT 90 ENV ROS_LOG_DIR=/tmp/craft-planning-test-ros-log)
endif()

ament_package()
