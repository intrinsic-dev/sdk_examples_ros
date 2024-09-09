# Create the service manifest
intrinsic_sdk_generate_service_manifest(
  SERVICE_NAME listener_service
  MANIFEST listener_service.manifest.textproto
  DEFAULT_CONFIGURATION listener_service_default_config.pbtxt
  PARAMETER_DESCRIPTOR ros_config.proto
  PROTOS_TARGET listener_service_protos
)
add_executable(listener_service_main
  listener_service_main.cpp
  minimal_subscriber.cpp
)

target_link_libraries(
  listener_service_main
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  ${std_msgs_TARGETS}
  listener_service_protos
  intrinsic_sdk::intrinsic_sdk
)

add_dependencies(listener_service_main listener_service_manifest listener_service_default_config)

# Install the listener service
install(
  TARGETS listener_service_main
  DESTINATION lib/${PROJECT_NAME}
)

# Install the proto descriptor database
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/listener_service_protos.desc
        DESTINATION "share/${PROJECT_NAME}/listener_service"
)

# Install the service manifest
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/service_manifest.binarypb
        DESTINATION "share/${PROJECT_NAME}/listener_service"
)
# Install the default configuration
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/default_config.binarypb
        DESTINATION "share/${PROJECT_NAME}/listener_service"
)
