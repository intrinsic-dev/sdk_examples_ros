cmake_minimum_required(VERSION 3.8)
project(sdk_examples_ros_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sdk_examples_ros_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Intrinsic Dependencies
find_package(absl REQUIRED)
find_package(intrinsic_sdk REQUIRED)

add_subdirectory(src/talker_skill)
add_subdirectory(src/feature_detector)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
