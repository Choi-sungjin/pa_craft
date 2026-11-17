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
find_package(coal REQUIRED)
find_package(urdfdom REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PNG REQUIRED)
find_package(Threads REQUIRED)

add_library(planning_modules
  src/state_machine.cpp src/path_planner.cpp src/arm_planner.cpp src/planning_config.cpp src/grasp_region.cpp
  src/dice_demo.cpp src/mission_logic.cpp src/mission_fsm.cpp)
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
add_executable(dice_demo src/dice_demo_main.cpp)
target_compile_features(dice_demo PRIVATE cxx_std_17)
target_compile_options(dice_demo PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(dice_demo PRIVATE planning_modules ament_index_cpp::ament_index_cpp)
add_executable(mission_master src/mission_master.cpp)
target_compile_features(mission_master PRIVATE cxx_std_17)
target_compile_options(mission_master PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(mission_master PRIVATE planning_modules
  craft_perception::craft_perception__rosidl_typesupport_cpp ${std_msgs_TARGETS})
install(TARGETS planning_modules planning_master dice_demo mission_master
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
add_executable(puck_picker src/puck_picker.cpp)
target_compile_features(puck_picker PRIVATE cxx_std_17)
target_compile_options(puck_picker PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(puck_picker PRIVATE rclcpp::rclcpp ${std_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp)
install(TARGETS puck_picker DESTINATION lib/${PROJECT_NAME})
find_package(Eigen3 REQUIRED)
add_library(puck_execute_math
  src/puck_execute/plan.cpp src/puck_execute/teach.cpp
  src/puck_execute/pick_policy.cpp src/puck_execute/puck_observations.cpp)
target_compile_features(puck_execute_math PUBLIC cxx_std_17)
target_compile_options(puck_execute_math PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(puck_execute_math PUBLIC src/puck_execute third_party)
target_link_libraries(puck_execute_math PUBLIC Eigen3::Eigen yaml-cpp urdfdom::urdf_parser ament_index_cpp::ament_index_cpp
  craft_perception::craft_perception__rosidl_typesupport_cpp)
add_library(puck_execute_core
  src/puck_execute/geometry.cpp src/puck_execute/legacy.cpp src/puck_execute/plan_generator.cpp
  src/puck_execute/runtime.cpp src/puck_execute/camera.cpp src/puck_execute/executor.cpp src/puck_execute/motion.cpp)
target_compile_features(puck_execute_core PUBLIC cxx_std_17)
target_compile_options(puck_execute_core PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(puck_execute_core PUBLIC puck_execute_math coal::coal rclcpp::rclcpp
  ${craft_interfaces_TARGETS} ${sensor_msgs_TARGETS} ${std_msgs_TARGETS})
add_executable(puck_execute src/puck_execute/main.cpp)
target_compile_features(puck_execute PRIVATE cxx_std_17)
target_compile_options(puck_execute PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(puck_execute PRIVATE puck_execute_core)
install(TARGETS puck_execute DESTINATION lib/${PROJECT_NAME})
install(TARGETS puck_execute_math puck_execute_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(PROGRAMS scripts/puck_picker.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES v75_ws/src/v75_robot_description/urdf/v75_robot.urdf
  DESTINATION share/${PROJECT_NAME}/urdf)

# Runtime inputs: install the current taught values without modifying their source copies.
install(DIRECTORY v75_ws/src/v75_robot_description/meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../demo/puck_pick/ DESTINATION share/${PROJECT_NAME}/puck_pick
  FILES_MATCHING PATTERN "teach.yaml" PATTERN "plan*.yaml"
  PATTERN "plan_cam_pucks.json" PATTERN "carousel.yaml"
  PATTERN "runs" EXCLUDE PATTERN "__pycache__" EXCLUDE)

# Keep the original A*+arc (Python) implementation importable for reference and regression tests.
set(reference_python_dir "${CMAKE_CURRENT_BINARY_DIR}/reference_python/${PROJECT_NAME}")
file(MAKE_DIRECTORY "${reference_python_dir}")
foreach(source __init__.py path_planner.py astar_arc.py box_detector.py astar8_spline.py)
  configure_file("src/${source}" "${reference_python_dir}/${source}" COPYONLY)
endforeach()
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "${reference_python_dir}")
install(DIRECTORY maps DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.stl" PATTERN "*.pgm" PATTERN "*.png" PATTERN "*.yaml" PATTERN "*.txt")
install(DIRECTORY config third_party DESTINATION share/${PROJECT_NAME})
install(FILES README.md IMPLEMENTATION.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  add_executable(puck_camera_cli test/puck_camera_cli.cpp)
  target_compile_options(puck_camera_cli PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(puck_camera_cli PRIVATE puck_execute_core)
  add_executable(puck_legacy_cli test/puck_legacy_cli.cpp)
  target_compile_options(puck_legacy_cli PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(puck_legacy_cli PRIVATE puck_execute_core)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(puck_motion_cli test/puck_motion_cli.cpp)
  target_link_libraries(puck_motion_cli PRIVATE puck_execute_core)
  target_compile_options(puck_motion_cli PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME puck_motion_checks COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/puck_motion_checks.py $<TARGET_FILE:puck_motion_cli>)
  add_executable(puck_execute_math_cli test/puck_execute_math_cli.cpp)
  target_compile_options(puck_execute_math_cli PRIVATE -Wall -Wextra -Wpedantic)
  target_link_libraries(puck_execute_math_cli PRIVATE puck_execute_math)

  add_test(NAME puck_execute_parity COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/puck_execute_parity.py $<TARGET_FILE:puck_execute_math_cli>)
  add_test(NAME puck_legacy_parity COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/puck_legacy_parity.py $<TARGET_FILE:puck_legacy_cli>)
  add_test(NAME puck_camera_parity COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/puck_camera_parity.py $<TARGET_FILE:puck_camera_cli>)
  add_test(NAME puck_cli_checks COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/puck_cli_checks.py $<TARGET_FILE:puck_execute>)
  set_tests_properties(puck_execute_parity puck_legacy_parity puck_camera_parity puck_cli_checks PROPERTIES TIMEOUT 180)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(module state_machine path_planner_cpp arm_planner planning_config grasp_region dice_demo mission_logic mission_fsm)
    ament_add_gtest(test_${module} test/test_${module}.cpp)
    target_link_libraries(test_${module} planning_modules)
    target_compile_definitions(test_${module} PRIVATE
      PLANNING_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/v75_ws/src/v75_robot_description/urdf/v75_robot.urdf"
      CRAFT_PLANNING_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  endforeach()
  target_link_libraries(test_path_planner_cpp PNG::PNG yaml-cpp)
  # Offline look at a route through via points on a real map (test/route_check.cpp); not installed.
  add_executable(route_check test/route_check.cpp)
  target_link_libraries(route_check planning_modules)
  ament_add_pytest_test(test_path_planner test/test_path_planner.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR})
  ament_add_pytest_test(test_box_detector test/test_box_detector.py
    APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR})
  ament_add_pytest_test(test_pick_dispatch test/test_pick_dispatch.py TIMEOUT 60)
  ament_add_pytest_test(test_pick_policy test/test_pick_policy.py TIMEOUT 30)
  ament_add_pytest_test(test_puck_picker test/test_puck_picker.py TIMEOUT 30)
  ament_add_pytest_test(test_planning_master_integration test/test_planning_master_integration.py
    TIMEOUT 90 ENV ROS_LOG_DIR=/tmp/craft-planning-test-ros-log)
endif()

ament_package()
