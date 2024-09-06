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
  MANIFEST talker_skill.manifest.textproto
  PROTOS_TARGET talker_skill_protos
)

# Generate the main function for this skill
intrinsic_sdk_skill_main(
  SKILL_NAME talker_skill
  CREATE_SKILL_HEADER talker_skill.hh
  CREATE_SKILL_FUNCTION "::TalkerSkill::CreateSkill"
  MAIN_FILE ${CMAKE_CURRENT_BINARY_DIR}/talker_skill_main.cpp
)

add_executable(talker_skill_main
  talker_skill.cc
  ${CMAKE_CURRENT_BINARY_DIR}/talker_skill_main.cpp
)

target_include_directories(talker_skill_main
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
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

install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/talker_skill_skill_config.pbbin
  DESTINATION "share/${PROJECT_NAME}"
)
