#include <fstream>
#include <memory>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "intrinsic/resources/proto/runtime_context.pb.h"
#include "rclcpp/rclcpp.hpp"

#include "ros_config.pb.h"
#include "minimal_subscriber.hpp"


intrinsic_proto::config::RuntimeContext
GetRuntimeContext() {
    intrinsic_proto::config::RuntimeContext runtime_context;
    std::ifstream runtime_context_file;
    runtime_context_file.open("/etc/intrinsic/runtime_config.pb", std::ios::binary);
    if (!runtime_context.ParseFromIstream(&runtime_context_file)) {
        // Return default context for running locally
        std::cerr << "Warning: using default RuntimeContext\n";
    }
    return runtime_context;
}

void StartZenohBridge(const std::string & zenoh_router_address) {
    // Start the zenoh bridge
  auto zenoh_pkg_location = ament_index_cpp::get_package_prefix("zenoh_bridge_dds");
  if (!zenoh_pkg_location.empty()) {
    std::string cmd = zenoh_pkg_location + "/lib/zenoh_bridge_dds/zenoh_bridge_dds" + \
        " -m client" + \
        " -e " + zenoh_router_address + \
        " --no-multicast-scouting &";
    std::system(cmd.c_str());
  }
}

int main(int argc, char * argv[])
{
  auto runtime_context = GetRuntimeContext();

  ros::RosConfig ros_config;
  if (!runtime_context.config().UnpackTo(&ros_config)) {
      return 1;
  }

  // Get ROS arguments
  StartZenohBridge(ros_config.zenoh_router_address());
  int ros_argc = ros_config.ros_args_size();
  std::vector<const char *> ros_argv;

  // Insert --ros-args at beginning
  ros_argv.emplace_back("--ros-args");

  // Copy all other arguments
  for (int i = 0; i < ros_argc; ++i) {
      const auto & str = ros_config.ros_args(i);
      ros_argv.emplace_back(str.c_str());
      std::cerr << "ROS argument: " << ros_argv.back()<< "\n";
  }

  rclcpp::init(ros_argc + 1, ros_argv.data());
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
