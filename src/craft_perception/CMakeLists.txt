cmake_minimum_required(VERSION 3.14)
project(craft_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lidar_processor REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc)
if(OpenCV_VERSION VERSION_LESS "4.7")
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc aruco)
else()
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc objdetect)
endif()
find_package(ncnn REQUIRED CONFIG)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg" "msg/DetectionArray.msg"
  "msg/KnobObservation.msg" "msg/KnobObservationArray.msg"
  "msg/PuckObservation.msg" "msg/PuckObservationArray.msg"
  DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(perception_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(perception_modules
  src/tag_registry.cpp
  src/yolo_detector.cpp src/apriltag_reader.cpp src/knob_depth.cpp src/knob_lever.cpp
  src/object_detector.cpp src/obstacle_detector.cpp src/puck_axis.cpp
  src/d435_perception.cpp src/c310_perception.cpp src/shared_frame.cpp
  src/state_estimator.cpp src/lidar_obstacle_detector.cpp src/lidar_perception.cpp)
target_compile_features(perception_modules PUBLIC cxx_std_17)
target_include_directories(perception_modules PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(perception_modules PUBLIC lidar_processor::scan_merger ncnn yaml-cpp ${OpenCV_LIBS}
  ${sensor_msgs_TARGETS} ${nav_msgs_TARGETS} ${geometry_msgs_TARGETS})

# TensorRT YOLO backend for the Jetson. auto: build it when TensorRT + CUDA are found.
set(CRAFT_TENSORRT auto CACHE STRING "TensorRT YOLO backend: auto | on | off")
set_property(CACHE CRAFT_TENSORRT PROPERTY STRINGS auto on off)
if(NOT CRAFT_TENSORRT STREQUAL "off")
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h PATH_SUFFIXES include include/aarch64-linux-gnu include/x86_64-linux-gnu
    HINTS /usr /usr/local/TensorRT)
  find_library(TENSORRT_NVINFER nvinfer HINTS /usr/local/TensorRT/lib)
  find_path(CUDA_RUNTIME_INCLUDE_DIR cuda_runtime_api.h HINTS /usr/local/cuda/include)
  find_library(CUDA_CUDART cudart HINTS /usr/local/cuda/lib64)
  if(TENSORRT_INCLUDE_DIR AND TENSORRT_NVINFER AND CUDA_RUNTIME_INCLUDE_DIR AND CUDA_CUDART)
    message(STATUS "craft_perception: TensorRT backend enabled (${TENSORRT_NVINFER})")
    target_sources(perception_modules PRIVATE src/tensorrt_backend.cpp)
    target_compile_definitions(perception_modules PRIVATE CRAFT_WITH_TENSORRT)
    target_include_directories(perception_modules PRIVATE ${TENSORRT_INCLUDE_DIR} ${CUDA_RUNTIME_INCLUDE_DIR})
    target_link_libraries(perception_modules PRIVATE ${TENSORRT_NVINFER} ${CUDA_CUDART})
  elseif(CRAFT_TENSORRT STREQUAL "on")
    message(FATAL_ERROR "CRAFT_TENSORRT=on but TensorRT/CUDA were not found")
  else()
    message(STATUS "craft_perception: TensorRT not found, ncnn backend only")
  endif()
endif()

add_executable(perception_master src/perception_master.cpp)
target_link_libraries(perception_master PRIVATE perception_modules
  rclcpp::rclcpp tf2_ros::tf2_ros ament_index_cpp::ament_index_cpp ${perception_typesupport}
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
ament_export_dependencies(lidar_processor tf2_ros ncnn yaml-cpp OpenCV sensor_msgs nav_msgs geometry_msgs rosidl_default_runtime)
ament_package()
