#include <socketcan_interface/dispatcher.hpp>
#include <socketcan_interface/socketcan.hpp>
#include <canopen_chain_node/ros_chain.hpp>

// using namespace can;
using namespace canopen;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RosChain>();

  if (!node->setup()) {
    return 1;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
