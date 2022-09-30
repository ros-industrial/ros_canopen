#include "canopen_master_driver/master_driver.hpp"

namespace ros2_canopen
{

    ros2_canopen::MasterDriver::MasterDriver(
        const rclcpp::NodeOptions &node_options) :
        CanopenMaster(node_options)
    {
        node_canopen_basic_master_ = std::make_shared<node_interfaces::NodeCanopenBasicMaster<rclcpp::Node>>(this);
        node_canopen_master_ = std::static_pointer_cast<node_interfaces::NodeCanopenMasterInterface>(node_canopen_basic_master_);
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::MasterDriver)