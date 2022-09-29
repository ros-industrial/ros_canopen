#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver_impl.hpp"

using namespace ros2_canopen::node_interfaces;

template class ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>;