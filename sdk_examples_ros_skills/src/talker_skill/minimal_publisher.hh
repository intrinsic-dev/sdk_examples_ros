#ifndef MINIMAL_PUBLISHER_HH_
#define MINIMAL_PUBLISHER_HH_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher();

    void Publish(const std::string & message);

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif  // MINIMAL_PUBLISHER_HH_
