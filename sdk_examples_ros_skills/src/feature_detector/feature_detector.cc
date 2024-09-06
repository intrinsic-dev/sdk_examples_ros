#include "feature_detector.hh"
#include <rclcpp/future_return_code.hpp>
#include "sdk_examples_ros_interfaces/srv/detect_features.hpp"

#include "absl/status/statusor.h"

using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

namespace sdk_examples_ros_skills
{

FeatureDetector::FeatureDetector(const rclcpp::NodeOptions & options)
: Node("feature_detctor_skill", options)
{
  this->image_pubs_["all"] =
    this->create_publisher<Image>("all/image_raw", 1);
  this->info_pubs_["all"] =
    this->create_publisher<CameraInfo>("all/camera_info", 1);
}

void
FeatureDetector::PublishImage(
  const std::string & camera_name,
  const Image & image,
  const CameraInfo & info)
{
  this->image_pubs_.try_emplace(
    camera_name, this->create_publisher<Image>(camera_name + "/image_raw", 1));

  this->info_pubs_.try_emplace(
    camera_name,
    this->create_publisher<CameraInfo>(camera_name + "/camera_info", 1));

  RCLCPP_INFO(this->get_logger(),
              "Publishing image on %s", (camera_name + "/image_raw").c_str());
  this->image_pubs_[camera_name]->publish(image);
  this->info_pubs_[camera_name]->publish(info);

  RCLCPP_INFO(this->get_logger(),
              "Publishing image on %s", "all/image_raw");

  this->image_pubs_["all"]->publish(image);
  this->info_pubs_["all"]->publish(info);
}

absl::StatusOr<FeatureDetector::DetectFeaturesSrv::Response::SharedPtr>
FeatureDetector::Detect(
  const std::string & service_name,
  const Image & image,
  const CameraInfo & camera_info,
  const std::chrono::milliseconds & timeout
)
{
  this->detect_features_.try_emplace(
    service_name,
    this->create_client<DetectFeaturesSrv>(service_name));

  auto request = std::make_shared<DetectFeaturesSrv::Request>();
  request->image = image;
  request->camera_info = camera_info;

  auto service = this->detect_features_[service_name];

  if (!service->service_is_ready())
  {
    return absl::NotFoundError(
      "Service [" + service_name + "] is unavailable or not ready");
  }

  auto response = service->async_send_request(request);
  auto spin_result =
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), response, timeout);

  if (spin_result == rclcpp::FutureReturnCode::TIMEOUT)
  {
    return absl::DeadlineExceededError(
      "Service call did not return before timeout.");
  }

  if (spin_result == rclcpp::FutureReturnCode::INTERRUPTED)
  {
    return absl::CancelledError(
      "Service call was interrupted.");
  }

  return response.get();
}

}  // namespace sdk_examples_ros_skills
