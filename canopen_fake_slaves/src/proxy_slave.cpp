#include "canopen_fake_slaves/proxy_slave.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::InitOptions options;
  options.shutdown_on_signal = true;
  rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::All);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto canopen_slave = std::make_shared<ros2_canopen::BasicSlave>("proxy_slave");
  executor.add_node(canopen_slave->get_node_base_interface());
  executor.spin();
  return 0;
}
