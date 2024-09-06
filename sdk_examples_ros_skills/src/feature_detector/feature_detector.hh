#ifndef FEATURE_DETECTOR__FEATURE_DETECTOR_HH_
#define FEATURE_DETECTOR__FEATURE_DETECTOR_HH_

#include <string>
#include <unordered_map>

#include <absl/status/statusor.h>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sdk_examples_ros_interfaces/srv/detect_features.hpp>

namespace sdk_examples_ros_skills
{

/// Class that holds the skill<->ROS Node logic.
class FeatureDetector: public rclcpp::Node
{
 public:
  using DetectFeaturesSrv = sdk_examples_ros_interfaces::srv::DetectFeatures;

  explicit FeatureDetector(const rclcpp::NodeOptions & options);

  void PublishImage(const std::string & camera_name, const sensor_msgs::msg::Image & image, const sensor_msgs::msg::CameraInfo & info);

  absl::StatusOr<DetectFeaturesSrv::Response::SharedPtr>
  Detect(
    const std::string & service_name,
    const sensor_msgs::msg::Image & image,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(1000));

 private:
  // Publisher for camera images
  std::unordered_map<std::string,
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_pubs_;

  // Publisher for camera info
  std::unordered_map<std::string,
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> info_pubs_;

  std::unordered_map<std::string,
    rclcpp::Client<DetectFeaturesSrv>::SharedPtr> detect_features_;
};

}  // namespace sdk_examples_ros_skills

#endif  // FEATURE_DETECTOR__FEATURE_DETECTOR_HH_
