add_library(
  minimal_publisher
  minimal_publisher.cc
)

target_link_libraries(minimal_publisher
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
)

# Create Intrinsic Skill
intrinsic_sdk_generate_skill(
  SKILL_NAME talker_skill
  SOURCES talker_skill.proto
  MANIFEST talker_skill.manifest.textproto)

add_executable(talker_skill_main
  talker_skill.cc
  talker_skill_main.cc
)

target_link_libraries(talker_skill_main
  intrinsic_sdk::intrinsic_sdk
  talker_skill_protos
  minimal_publisher
)

add_dependencies(talker_skill_main talker_skill_skill_config)

install(
  TARGETS talker_skill_main
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/talker_skill_skill_config.pbbin
        DESTINATION "share/${PROJECT_NAME}"
)
