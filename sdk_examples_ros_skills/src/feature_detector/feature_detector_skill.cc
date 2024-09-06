#include "feature_detector_skill.hh"
#include "feature_detector.pb.h"

#include <memory>

#include <rclcpp/node_options.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "absl/log/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/notification.h"
#include "intrinsic/math/proto_conversion.h"
#include "intrinsic/perception/proto/camera_config.pb.h"
#include "intrinsic/perception/proto/image_buffer.pb.h"
#include "intrinsic/perception/service/proto/camera_server.grpc.pb.h"
#include "intrinsic/skills/cc/skill_utils.h"
#include "intrinsic/skills/proto/skill_service.pb.h"
#include "intrinsic/util/status/status_conversion_grpc.h"
#include "intrinsic/util/status/status_macros.h"
#include "intrinsic/math/pose3.h"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "feature_detector.hh"


class InitRos
{
 public:
  InitRos() { rclcpp::init(0, nullptr); }
  ~InitRos() { rclcpp::shutdown(); }
};

namespace {
InitRos init;
sdk_examples_ros_skills::FeatureDetector node(rclcpp::NodeOptions().arguments({"--disable-external-lib-logs"}));
}  // namespace

namespace sdk_examples_ros_skills {

using ::ai::intrinsic::sdk_examples_ros::FeatureDetectorSkillParams;
using ::ai::intrinsic::sdk_examples_ros::FeatureDetectorSkillResult;

using ::intrinsic::FrameName;
using ::intrinsic::Pose3d;
using ::intrinsic::WaitForChannelConnected;
using ::intrinsic::skills::EquipmentPack;
using ::intrinsic::skills::ExecuteContext;
using ::intrinsic::skills::ExecuteRequest;
using ::intrinsic::skills::SkillInterface;
using ::intrinsic::world::Frame;
using ::intrinsic::world::ObjectWorldClient;
using ::intrinsic::world::WorldObject;
using ::intrinsic_proto::perception::CameraConfig;

// -----------------------------------------------------------------------------
// Skill signature.
// -----------------------------------------------------------------------------

std::unique_ptr<SkillInterface> FeatureDetectorSkill::CreateSkill() {
  return std::make_unique<FeatureDetectorSkill>();
}

sensor_msgs::msg::Image
ConvertToRos(
  const ::intrinsic_proto::perception::ImageBuffer & buffer,
  const std::string & camera_name = "camera")
{
  auto message = sensor_msgs::msg::Image();
  message.header.stamp = rclcpp::Clock().now();
  message.header.frame_id = camera_name + "/sensor_link";
  message.width = buffer.dimensions().cols();
  message.height = buffer.dimensions().rows();
  message.step = message.width * 3;
  message.data.resize(message.step * message.height);
  message.encoding = sensor_msgs::image_encodings::RGB8;
  memcpy(message.data.data(), buffer.data().c_str(), message.step * message.height);
  return message;
}

sensor_msgs::msg::CameraInfo
ConvertToRos(const ::intrinsic_proto::perception::CameraConfig & camera_config)
{
  const auto fx = camera_config.intrinsic_params().focal_length_x();
  const auto fy = camera_config.intrinsic_params().focal_length_x();
  const auto ppy = camera_config.intrinsic_params().principal_point_y();
  const auto ppx = camera_config.intrinsic_params().principal_point_x();

  const auto k1 = camera_config.distortion_params().k1();
  const auto k2 = camera_config.distortion_params().k2();
  const auto k3 = camera_config.distortion_params().k3();
  const auto p1 = camera_config.distortion_params().p1();
  const auto p2 = camera_config.distortion_params().p2();

  sensor_msgs::msg::CameraInfo info;
  info.width = camera_config.intrinsic_params().dimensions().cols();
  info.height = camera_config.intrinsic_params().dimensions().rows();
  info.k = {fx, 0, ppx, 0, fy, ppy, 0, 0, 1};
  info.distortion_model = "plumb_bob";
  info.d = {k1, k2, p1, p2, k3};
  info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  info.p = {fx, 0, ppx, 0,
            0, fy, ppy, 0,
            0, 0, 1, 0};
  return info;
}

// -----------------------------------------------------------------------------
// Skill execution.
// -----------------------------------------------------------------------------

absl::StatusOr<std::unique_ptr<google::protobuf::Message>> FeatureDetectorSkill::Execute(
    const ExecuteRequest& request, ExecuteContext& context) {

  // Get parameters.
  INTR_ASSIGN_OR_RETURN(
    auto params, request.params<FeatureDetectorSkillParams>());

  // Get equipment.
  const auto & equipment_pack = context.equipment();
  INTR_ASSIGN_OR_RETURN(const auto camera_equipment, equipment_pack.GetHandle(kCameraSlot));
  intrinsic_proto::perception::CameraConfig camera_config;
  camera_equipment.resource_data().at("CameraConfig").contents().UnpackTo(&camera_config);

  // Connect to the camera over gRPC.
  std::unique_ptr<intrinsic_proto::perception::CameraServer::Stub> camera_stub;
  std::string camera_handle;
  INTR_RETURN_IF_ERROR(ConnectToCamera(
    camera_equipment.connection_info().grpc(), camera_config,
    &camera_stub, &camera_handle));

  ObjectWorldClient& world = context.object_world();
  INTR_ASSIGN_OR_RETURN(WorldObject root_object, world.GetRootObject());
  INTR_ASSIGN_OR_RETURN(WorldObject camera_obj, world.GetObject(camera_equipment));
  INTR_ASSIGN_OR_RETURN(Frame sensor_frame, camera_obj.GetFrame(FrameName{"sensor"}));
  INTR_ASSIGN_OR_RETURN(Pose3d root_t_camera, world.GetTransform(root_object, sensor_frame));

  // Get a frame from the camera.
  INTR_ASSIGN_OR_RETURN(intrinsic_proto::perception::Frame frame,
    GrabFrame(camera_equipment.connection_info().grpc(), camera_stub.get(), camera_handle));

  const auto & camera_name = camera_equipment.name();
  auto ros_image = ConvertToRos(frame.rgb8u(), camera_name);
  auto ros_camera_info = ConvertToRos(camera_config);

  node.PublishImage(camera_name, ros_image, ros_camera_info);

  auto result = std::make_unique<FeatureDetectorSkillResult>();
  if (!params.service().empty())
  {
    auto status = node.Detect(params.service(), ros_image, ros_camera_info, std::chrono::milliseconds(params.service_timeout_ms()));
    if (!status.ok()) {
      return status.status();
    }

    auto service_result = status.value();
    for (const auto & detection : service_result->detections)
    {
      auto *feature = result->add_feature();
      feature->set_id(detection.id);
      feature->mutable_pose()->mutable_position()->set_x(detection.pose.position.x);
      feature->mutable_pose()->mutable_position()->set_y(detection.pose.position.y);
      feature->mutable_pose()->mutable_position()->set_z(detection.pose.position.z);
      feature->mutable_pose()->mutable_orientation()->set_x(detection.pose.orientation.x);
      feature->mutable_pose()->mutable_orientation()->set_y(detection.pose.orientation.y);
      feature->mutable_pose()->mutable_orientation()->set_z(detection.pose.orientation.z);
      feature->mutable_pose()->mutable_orientation()->set_w(detection.pose.orientation.w);

      INTR_ASSIGN_OR_RETURN(Pose3d camera_t_obj, intrinsic_proto::FromProtoNormalized(feature->pose()));
      auto root_t_obj = intrinsic::ToProto(root_t_camera * camera_t_obj);
      *feature->mutable_pose() = root_t_obj;

      LOG(INFO) << feature;
    }
  }
  return result;
}

absl::Status
FeatureDetectorSkill::ConnectToCamera(
  const intrinsic_proto::resources::ResourceGrpcConnectionInfo& grpc_info,
  const intrinsic_proto::perception::CameraConfig& camera_config,
  std::unique_ptr<intrinsic_proto::perception::CameraServer::Stub>* camera_stub,
  std::string* camera_handle)
{
  // Connect to the provided camera.
  const auto & camera_grpc_address = grpc_info.address();
  const auto & camera_server_instance = grpc_info.server_instance();

  grpc::ChannelArguments options;
  constexpr int kMaxReceiveMessageSize{-1};  // Put no limit on the size of a message we can receive.
  options.SetMaxReceiveMessageSize(kMaxReceiveMessageSize);
  auto camera_channel = grpc::CreateCustomChannel(camera_grpc_address, grpc::InsecureChannelCredentials(), options);

  INTR_RETURN_IF_ERROR(
    WaitForChannelConnected(camera_server_instance, camera_channel, absl::InfiniteFuture()));

  *camera_stub = intrinsic_proto::perception::CameraServer::NewStub(camera_channel);

  auto client_context = std::make_unique<grpc::ClientContext>();
  constexpr const auto kCameraClientTimeout = std::chrono::seconds(5);
  client_context->set_deadline(std::chrono::system_clock::now() + kCameraClientTimeout);
  if (!camera_server_instance.empty()) {
    client_context->AddMetadata("x-resource-instance-name", camera_server_instance);
  }

  intrinsic_proto::perception::CreateCameraRequest create_request;
  intrinsic_proto::perception::CreateCameraResponse create_response;
  *create_request.mutable_camera_config() = camera_config;
  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus((*camera_stub)->CreateCamera(client_context.get(), create_request, &create_response)));

  *camera_handle = create_response.camera_handle();

  return absl::OkStatus();
}

absl::StatusOr<intrinsic_proto::perception::Frame>
FeatureDetectorSkill::GrabFrame(
  const intrinsic_proto::resources::ResourceGrpcConnectionInfo& grpc_info,
  intrinsic_proto::perception::CameraServer::Stub* camera_stub,
  const std::string& camera_handle)
{
  std::string camera_server_instance = grpc_info.server_instance();

  auto client_context = std::make_unique<grpc::ClientContext>();
  constexpr const auto kCameraClientTimeout = std::chrono::seconds(5);
  client_context->set_deadline(std::chrono::system_clock::now() + kCameraClientTimeout);
  if (!camera_server_instance.empty()) {
    client_context->AddMetadata("x-resource-instance-name", camera_server_instance);
  }

  intrinsic_proto::perception::GetFrameRequest frame_request;
  frame_request.set_camera_handle(camera_handle);
  frame_request.mutable_timeout()->set_seconds(5);
  frame_request.mutable_post_processing()->set_skip_undistortion(false);
  intrinsic_proto::perception::GetFrameResponse frame_response;
  INTR_RETURN_IF_ERROR(intrinsic::ToAbslStatus(camera_stub->GetFrame(client_context.get(), frame_request, &frame_response)));
  return std::move(*frame_response.mutable_frame());
}

}  // namespace sdk_examples_ros
