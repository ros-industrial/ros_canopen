#include "canopen_master_driver/lifecycle_master_driver.hpp"

namespace ros2_canopen
{

    ros2_canopen::LifecycleMasterDriver::LifecycleMasterDriver(
        const rclcpp::NodeOptions &node_options) :
        LifecycleCanopenMaster(node_options)
    {
        node_canopen_master_ = std::make_shared<node_interfaces::NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>>(this);
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleMasterDriver)