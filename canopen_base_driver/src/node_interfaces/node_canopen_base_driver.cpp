#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver_impl.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template class ros2_canopen::node_interfaces::NodeCanopenBaseDriver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopenBaseDriver<rclcpp_lifecycle::LifecycleNode>;