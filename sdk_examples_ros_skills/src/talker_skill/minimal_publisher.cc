#include "minimal_publisher.hh"

MinimalPublisher::MinimalPublisher()
: rclcpp::Node("minimal_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
}

void MinimalPublisher::Publish(const std::string & message)
{
  auto ros_message = std_msgs::msg::String();
  ros_message.data = message;
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", ros_message.data.c_str());
  publisher_->publish(ros_message);
}
