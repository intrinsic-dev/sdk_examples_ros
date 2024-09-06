#ifndef SDK_EXAMPLES_ROS__FEATURE_DETECTOR_SKILL_HH_
#define SDK_EXAMPLES_ROS__FEATURE_DETECTOR_SKILL_HH_

#include <memory>
#include <string>

#include "absl/status/statusor.h"
#include "intrinsic/perception/service/proto/camera_server.grpc.pb.h"
#include "intrinsic/perception/service/proto/camera_server.pb.h"
#include "intrinsic/skills/cc/skill_interface.h"
#include "intrinsic/skills/proto/skill_service.pb.h"

namespace sdk_examples_ros_skills {

class FeatureDetectorSkill final : public intrinsic::skills::SkillInterface {
 public:
  static constexpr char kCameraSlot[] = "camera";

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

 private:
  absl::Status
  ConnectToCamera(
    const intrinsic_proto::resources::ResourceGrpcConnectionInfo& grpc_info,
    const intrinsic_proto::perception::CameraConfig& camera_config,
    std::unique_ptr<intrinsic_proto::perception::CameraServer::Stub>* camera_stub,
    std::string* camera_handle);

  absl::StatusOr<intrinsic_proto::perception::Frame>
  GrabFrame(
    const intrinsic_proto::resources::ResourceGrpcConnectionInfo& grpc_info,
    intrinsic_proto::perception::CameraServer::Stub* camera_stub,
    const std::string& camera_handle);

};

}  // namespace sdk_examples_ros_skills

#endif  // SDK_EXAMPLES_ROS__FEATURE_DETECTOR_SKILL_HH_
