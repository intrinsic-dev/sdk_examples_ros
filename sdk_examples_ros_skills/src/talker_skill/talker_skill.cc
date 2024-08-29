#include "talker_skill.hh"
#include "talker_skill.pb.h"
#include "minimal_publisher.hh"

#include <memory>

#include <rclcpp/node_options.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/notification.h"
#include "intrinsic/perception/proto/camera_config.pb.h"
#include "intrinsic/perception/service/proto/camera_server.grpc.pb.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"


using ::com::example::TalkerSkillParams;
using ::com::example::TalkerSkillResult;

using ::intrinsic::skills::ExecuteRequest;
using ::intrinsic::skills::SkillInterface;
using ::intrinsic::skills::ExecuteContext;
using ::intrinsic::WaitForChannelConnected;

class InitRos
{
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

namespace {
InitRos init;
MinimalPublisher minimal_publisher_node_;
}  // namespace

// -----------------------------------------------------------------------------
// Skill signature.
// -----------------------------------------------------------------------------

std::unique_ptr<SkillInterface> TalkerSkill::CreateSkill() {
  return std::make_unique<TalkerSkill>();
}

// -----------------------------------------------------------------------------
// Skill execution.
// -----------------------------------------------------------------------------

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> TalkerSkill::Execute(
    const ExecuteRequest& request, ExecuteContext& context) {

  RCLCPP_INFO(minimal_publisher_node_.get_logger(), "TalkerSkill::Execute");

  // Get parameters.
  INTR_ASSIGN_OR_RETURN(
    auto params, request.params<TalkerSkillParams>());

  minimal_publisher_node_.Publish(params.message());

  auto result = std::make_unique<TalkerSkillResult>();
  return result;
}
