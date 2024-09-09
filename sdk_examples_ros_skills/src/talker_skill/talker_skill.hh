#ifndef TALKER_SKILL_HH
#define TALKER_SKILL_HH

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"


class TalkerSkill final : public intrinsic::skills::SkillInterface {
 public:
  // ---------------------------------------------------------------------------
  // Skill signature (see intrinsic::skills::SkillSignatureInterface)
  // ---------------------------------------------------------------------------

  // Factory method to create an instance of the skill.
  static std::unique_ptr<intrinsic::skills::SkillInterface> CreateSkill();

  // ---------------------------------------------------------------------------
  // Skill execution (see intrinsic::skills::SkillExecuteInterface)
  // ---------------------------------------------------------------------------
  // Called once each time the skill is executed in a process.
  absl::StatusOr<std::unique_ptr<google::protobuf::Message>>
  Execute(const intrinsic::skills::ExecuteRequest& request,
          intrinsic::skills::ExecuteContext& context) override;
};

#endif  // TALKER_SKILL_HH
