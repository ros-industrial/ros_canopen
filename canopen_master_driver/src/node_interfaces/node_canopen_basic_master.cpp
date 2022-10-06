#include "canopen_master_driver/node_interfaces/node_canopen_basic_master.hpp"
#include "canopen_master_driver/node_interfaces/node_canopen_basic_master_impl.hpp"

using namespace ros2_canopen::node_interfaces;


template class ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>;