#include "canopen_402_driver/lifecycle_cia402_driver.hpp"

using namespace ros2_canopen;

LifecycleCia402Driver::LifecycleCia402Driver(rclcpp::NodeOptions node_options)
: LifecycleCanopenDriver(node_options)
{
  node_canopen_402_driver_ =
    std::make_shared<node_interfaces::NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>>(this);
  node_canopen_driver_ =
    std::static_pointer_cast<node_interfaces::NodeCanopenDriverInterface>(node_canopen_402_driver_);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleCia402Driver)
