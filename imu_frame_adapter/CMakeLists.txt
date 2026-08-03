cmake_minimum_required(VERSION 3.8)
project(imu_frame_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_frame_adapter_node src/imu_frame_adapter_node.cpp)
target_include_directories(imu_frame_adapter_node PUBLIC include)
ament_target_dependencies(imu_frame_adapter_node rclcpp sensor_msgs)

install(TARGETS imu_frame_adapter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_transform test/test_imu_transform.cpp)
  target_include_directories(test_imu_transform PRIVATE include)
endif()

ament_package()
