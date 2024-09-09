#ifndef MINIMAL_SUBSCRIBER_HPP_
#define MINIMAL_SUBSCRIBER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Create a MinimalSubscriber node that has a single subscription
// The main function will instantiate the Node
class MinimalSubscriber : public rclcpp::Node
{
  public:
    // Constructor
    explicit MinimalSubscriber(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  private:
    // Callback to be fired when new messages arrive on the "chatter" topic
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

    // The subscription pointer
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // MINIMAL_SUBSCRIBER_HPP_
