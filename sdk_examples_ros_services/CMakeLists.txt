cmake_minimum_required(VERSION 3.8)
project(sdk_examples_ros_services)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(intrinsic_sdk REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(sdk_examples_ros_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

add_subdirectory(listener_service)
add_subdirectory(feature_detector)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
