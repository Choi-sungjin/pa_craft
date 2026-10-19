cmake_minimum_required(VERSION 3.14)
project(craft_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc aruco)
find_package(ncnn REQUIRED CONFIG)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg" "msg/DetectionArray.msg"
  "msg/KnobObservation.msg" "msg/KnobObservationArray.msg"
  DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(perception_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(perception_modules
  src/object_detector.cpp src/aruco_reader.cpp src/knob_depth.cpp
  src/knob_lever.cpp src/depth_heatmap.cpp src/shared_frame.cpp
  src/state_estimator.cpp src/obstacle_detector.cpp)
target_compile_features(perception_modules PUBLIC cxx_std_17)
target_include_directories(perception_modules PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_modules PUBLIC ncnn ${OpenCV_LIBS}
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})

add_executable(perception_master src/perception_master.cpp)
target_link_libraries(perception_master PRIVATE perception_modules
  rclcpp::rclcpp ament_index_cpp::ament_index_cpp ${perception_typesupport}
  ${std_msgs_TARGETS} ${diagnostic_msgs_TARGETS})

install(TARGETS perception_master perception_modules
  EXPORT perception_modules_export
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_modules test/test_modules.cpp)
  target_link_libraries(test_perception_modules perception_modules)
  add_executable(ncnn_smoke test/ncnn_smoke.cpp)
  target_link_libraries(ncnn_smoke perception_modules)
endif()

ament_export_targets(perception_modules_export HAS_LIBRARY_TARGET)
ament_export_dependencies(ncnn OpenCV sensor_msgs nav_msgs geometry_msgs rosidl_default_runtime)
ament_package()
