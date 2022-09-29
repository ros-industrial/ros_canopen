#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver_impl.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template class ros2_canopen::node_interfaces::NodeCanopen402Driver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>;