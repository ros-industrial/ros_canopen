// #include <socketcan_interface/dispatcher.hpp>
// #include <socketcan_interface/socketcan.hpp>
#include <canopen_chain_node/ros_chain.hpp>

// using namespace can;
using namespace canopen;

int main(int argc, char ** argv)
{
  // ros::init(argc, argv, "canopen_chain_node_node");
  // ros::NodeHandle nh;
  // ros::NodeHandle nh_priv("~");

  rclcpp::init(argc, argv);
  // RosChain chain();
  auto node = std::make_shared<RosChain>();

  if (!node->setup()) {
    return 1;
  }

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
