find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

### Create a ROS Node
add_library(
  feature_detector_node
  feature_detector_node.cc
)

target_link_libraries(feature_detector_node
  rclcpp::rclcpp
  rclcpp_components::component
  cv_bridge::cv_bridge
  tf2::tf2
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${sdk_examples_ros_interfaces_TARGETS}
  ${OpenCV_LIBS}
)

add_executable(
  feature_detector_node_standalone
  feature_detector_node_standalone.cc
)

target_link_libraries(
  feature_detector_node_standalone
  feature_detector_node
)

install(
  TARGETS feature_detector_node_standalone
  DESTINATION lib/${PROJECT_NAME}
)

# Create the service manifest
intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME feature_detector
  MANIFEST feature_detector.manifest.textproto
  DEFAULT_CONFIGURATION feature_detector_default_config.pbtxt
  PARAMETER_DESCRIPTOR ros_config.proto
  PROTOS_TARGET feature_detector_protos
)

add_executable(feature_detector_main
  feature_detector_main.cpp
)

target_link_libraries(
  feature_detector_main
  feature_detector_node
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${std_msgs_TARGETS}
  listener_service_protos
  intrinsic_sdk::intrinsic_sdk
)

add_dependencies(
  feature_detector_main
  feature_detector_manifest
  feature_detector_default_config
)


# Install the listener service
install(
  TARGETS feature_detector_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install the proto descriptor database
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/feature_detector_protos.desc
        DESTINATION "share/${PROJECT_NAME}/feature_detector"
)

# Install the service manifest
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
        DESTINATION "share/${PROJECT_NAME}/feature_detector"
)
# Install the default configuration
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
        DESTINATION "share/${PROJECT_NAME}/feature_detector"
)
