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

#include "feature_detector_node.hh"

#include <opencv2/aruco.hpp>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

#include <functional>
#include <memory>
#include <opencv2/aruco/dictionary.hpp>
#include <thread>
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace sdk_examples_ros_services
{

FeatureDetectorNode::FeatureDetectorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("feature_detector_node", options)
{

  this->service_ = this->create_service<DetectFeaturesSrv>("/detect_features",
            std::bind(&FeatureDetectorNode::DetectFeatures, this, std::placeholders::_1, std::placeholders::_2));

  this->debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/aruco_debug", 1);
  this->debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/aruco_markers", 1);
}

void
FeatureDetectorNode::DetectFeatures(
      const DetectFeaturesSrv::Request::SharedPtr & request,
      DetectFeaturesSrv::Response::SharedPtr response)
{
  RCLCPP_INFO(this->get_logger(), "FeatureDetectorNode::DetectFeatures");

  auto cv_image = cv_bridge::toCvCopy(request->image);

  auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_16h5);
  auto detectorParams = cv::aruco::DetectorParameters::create();
  detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  std::vector<std::vector<cv::Point2f>> rejectedCandidates;

  cv::aruco::detectMarkers(cv_image->image,
      dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);

  RCLCPP_INFO(get_logger(), "Found %zu markers", markerIds.size());

  // Output a debug image with the markers indicated
  cv::Mat output_image = cv_image->image.clone();
  cv::aruco::drawDetectedMarkers(output_image, rejectedCandidates, cv::noArray());
  cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = request->image.header.stamp;
  out_msg.header.frame_id = request->image.header.frame_id;
  out_msg.image = output_image;
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  this->debug_image_pub_->publish(*out_msg.toImageMsg().get());

  if (markerIds.empty())
  {
    return;
  }

  // Compute the poses of the markers relative to the camera
  RCLCPP_INFO(get_logger(), "Estimating poses");

  auto k = request->camera_info.k;
  cv::Matx33d K(k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8]);

  auto d = cv::Mat::zeros(1, 5, CV_64F);

  std::vector<cv::Vec3d> rvecs;
  std::vector<cv::Vec3d> tvecs;
  cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.055, K, d, rvecs, tvecs);

  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();

  RCLCPP_INFO(get_logger(), "Populating response");
  for (size_t ii = 0; ii < markerIds.size(); ++ii)
  {
    sdk_examples_ros_interfaces::msg::DetectionResult feature;
    feature.id = markerIds[ii];

    // Convert rotation vector to quaternion
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Rodrigues(rvecs[ii], rot);
    tf2::Matrix3x3 tf_rot(
      rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
      rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
      rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
    tf2::Quaternion quat;
    tf_rot.getRotation(quat);

    feature.pose.position.x = tvecs[ii][0];
    feature.pose.position.y = tvecs[ii][1];
    feature.pose.position.z = tvecs[ii][2];
    feature.pose.orientation.x = quat.x();
    feature.pose.orientation.y = quat.y();
    feature.pose.orientation.z = quat.z();
    feature.pose.orientation.w = quat.w();

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = request->image.header.stamp;
    marker.header.frame_id = request->image.header.frame_id;
    marker.ns = "aruco_markers";
    marker.id = 10*ii + feature.id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.lifetime = rclcpp::Duration::from_seconds(0);
    marker.color.r = 255.0/255.0;
    marker.color.g = 100.0/255.0;
    marker.color.b = 100.0/255.0;
    marker.color.a = 1.0;
    marker.scale.x = 0.055;
    marker.scale.y = 0.055;
    marker.scale.z = 0.01;
    marker.pose = feature.pose;
    markers->markers.push_back(marker);

    /*
    RCLCPP_INFO(this->get_logger(), "[%i] [%f, %f, %f]", feature.id, tvecs[ii][0], tvecs[ii][1], tvecs[ii][2]);
    RCLCPP_INFO(this->get_logger(), "[%i] [%f, %f, %f]", feature.id, rvecs[ii][0], rvecs[ii][1], rvecs[ii][2]);
    RCLCPP_INFO(this->get_logger(), "[%i] [%f, %f, %f, %f]", feature.id, quat.x(), quat.y(), quat.z(), quat.w());
    */

    response->detections.push_back(feature);
  }

  this->debug_marker_pub_->publish(std::move(markers));
}

}  // namespace sdk_examples_ros_services

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(sdk_examples_ros_services::FeatureDetectorNode)
