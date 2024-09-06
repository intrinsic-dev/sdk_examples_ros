// Copyright 2024 Intrinsic Innovation LLC
//
// You are hereby granted a non-exclusive, worldwide, royalty-free license to use,
// copy, modify, and distribute this Intrinsic SDK in source code or binary form for use
// in connection with the services and APIs provided by Intrinsic Innovation LLC (“Intrinsic”).
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// If you use this Intrinsic SDK with any Intrinsic services, your use is subject to the Intrinsi
// Platform Terms of Service [https://intrinsic.ai/platform-terms].  If you create works that call
// Intrinsic APIs, you must agree to the terms of service for those APIs separately. This license
// does not grant you any special rights to use the services.
//
// This copyright notice shall be included in all copies or substantial portions of the software.

#ifndef FEATURE_DETECTOR__FEATURE_DETECTOR_NODE_HH_
#define FEATURE_DETECTOR__FEATURE_DETECTOR_NODE_HH_

#include <sdk_examples_ros_interfaces/srv/detect_features.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <rclcpp/rclcpp.hpp>

namespace sdk_examples_ros_services
{

class FeatureDetectorNode: public rclcpp::Node
{
 public:
  using DetectFeaturesSrv = sdk_examples_ros_interfaces::srv::DetectFeatures;

  explicit FeatureDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void DetectFeatures(
      const DetectFeaturesSrv::Request::SharedPtr & request,
      DetectFeaturesSrv::Response::SharedPtr reponse);

 private:
  rclcpp::Service<DetectFeaturesSrv>::SharedPtr service_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
};

}  // namespace sdk_examples_ros_services

#endif  // FEATURE_DETECTOR__FEATURE_DETECTOR_NODE_HH_
