#include <rclcpp/executors/single_threaded_executor.hpp>

#include "feature_detector_node.hh"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<sdk_examples_ros_services::FeatureDetectorNode>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
