#include "ros2_canopen/basic_device_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;
using namespace ros2_canopen;

void BasicDeviceNode::nmt_listener()
{
    while (configured.load())
    {
        auto f = driver->async_request_nmt();
        f.wait();
        on_nmt(f.get());
    }
}

void BasicDeviceNode::rdpo_listener()
{
    while (configured.load())
    {
        auto f = driver->async_request_rpdo();
        f.wait();
        on_rpdo(f.get());
    }
}

CallbackReturn
BasicDeviceNode::on_configure(const rclcpp_lifecycle::State &state)
{
    configured.store(true);
    nmt_state_publisher_future = std::async(std::launch::async, std::bind(&ros2_canopen::BasicDeviceNode::nmt_listener, this));
    rpdo_publisher_future = std::async(std::launch::async, std::bind(&ros2_canopen::BasicDeviceNode::rdpo_listener, this));
    return on_configure_app(state);
}

CallbackReturn
BasicDeviceNode::on_activate(const rclcpp_lifecycle::State &state)
{
    return on_activate_app(state);
}

CallbackReturn
BasicDeviceNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    return on_deactivate_app(state);
}

CallbackReturn
BasicDeviceNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    configured.store(false);
    nmt_state_publisher_future.wait();
    rpdo_publisher_future.wait();
    return on_cleanup_app(state);
}

CallbackReturn
BasicDeviceNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    return on_shuttdown_app(state);
}
