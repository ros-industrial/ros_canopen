#include "canopen_core/master_node.hpp"

using namespace ros2_canopen;

void CanopenMaster::init()
{
    RCLCPP_INFO(this->get_logger(), "INIT");
    node_canopen_master_->init();
    node_canopen_master_->configure();
    node_canopen_master_->activate();
}
void CanopenMaster::shutdown()
{
    node_canopen_master_->shutdown();
}

std::shared_ptr<lely::canopen::AsyncMaster> 
CanopenMaster::get_master()
{
    return node_canopen_master_->get_master();
}

std::shared_ptr<lely::ev::Executor> 
CanopenMaster::get_executor()
{
    return node_canopen_master_->get_executor();
}

void LifecycleCanopenMaster::init()
{
    node_canopen_master_->init();
}

void LifecycleCanopenMaster::shutdown()
{
    node_canopen_master_->shutdown();
}

std::shared_ptr<lely::canopen::AsyncMaster> 
LifecycleCanopenMaster::get_master()
{
    return node_canopen_master_->get_master();
}

std::shared_ptr<lely::ev::Executor> 
LifecycleCanopenMaster::get_executor()
{
    return node_canopen_master_->get_executor();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_configure(const rclcpp_lifecycle::State &state)
{
    node_canopen_master_->configure();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_activate(const rclcpp_lifecycle::State &state)
{
    node_canopen_master_->activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_deactivate(const rclcpp_lifecycle::State &state)
{
    node_canopen_master_->deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_cleanup(const rclcpp_lifecycle::State &state)
{
    node_canopen_master_->cleanup();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_shutdown(const rclcpp_lifecycle::State &state)
{
    node_canopen_master_->shutdown();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}