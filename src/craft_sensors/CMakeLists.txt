cmake_minimum_required(VERSION 3.8)
project(craft_sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio)
find_package(realsense2 REQUIRED)

add_executable(realsense_processor src/realsense_processor.cpp)
target_link_libraries(realsense_processor
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  realsense2::realsense2
)

add_executable(c310_processor src/c310_processor.cpp)
target_include_directories(c310_processor PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(c310_processor
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${OpenCV_LIBS}
)

add_executable(imu_processor src/imu_processor.cpp)
target_link_libraries(imu_processor
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_executable(wheel_encoder_processor src/wheel_encoder_processor.cpp)
target_link_libraries(wheel_encoder_processor
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

add_executable(arm_encoder_processor src/arm_encoder_processor.cpp)
target_link_libraries(arm_encoder_processor
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
)

install(TARGETS
  realsense_processor
  c310_processor
  imu_processor
  wheel_encoder_processor
  arm_encoder_processor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
