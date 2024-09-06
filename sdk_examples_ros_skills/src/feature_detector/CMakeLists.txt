### Create a ROS Node
add_library(
  feature_detector
  feature_detector.cc
)

target_link_libraries(feature_detector
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${sdk_examples_ros_interfaces_TARGETS}
  absl::status
)

### Create Intrinsic Skill

# Generate skill metadata
intrinsic_sdk_generate_skill(
  SKILL_NAME feature_detector_skill
  SOURCES feature_detector.proto
  MANIFEST feature_detector_skill.manifest.textproto
  PROTOS_TARGET feature_detector_skill_protos
)

# Generate the main function for this skill
intrinsic_sdk_skill_main(
  SKILL_NAME feature_detector_skill
  CREATE_SKILL_HEADER feature_detector_skill.hh
  CREATE_SKILL_FUNCTION "::sdk_examples_ros_skills::FeatureDetectorSkill::CreateSkill"
  MAIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/feature_detector_skill_main.cpp
)

add_executable(feature_detector_skill_main
  feature_detector_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/feature_detector_skill_main.cpp
)

target_include_directories(feature_detector_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(feature_detector_skill_main
  feature_detector_skill_protos
  intrinsic_sdk::intrinsic_sdk
  feature_detector
)

add_dependencies(feature_detector_skill_main feature_detector_skill_skill_config)

install(
  TARGETS feature_detector_skill_main
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/feature_detector_skill_skill_config.pbbin
  DESTINATION "share/${PROJECT_NAME}"
)
