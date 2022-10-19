#include "canopen_core/driver_node.hpp"

using namespace ros2_canopen;
void CanopenDriver::init()
{
    node_canopen_driver_->init();
    node_canopen_driver_->configure();
    bool success = false;
    for(int i = 0; i < 5; i++)
    {
        try 
        {
            node_canopen_driver_->demand_set_master();
            success = true;
            break;
        }
        catch(std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get demand set master result becauss %s. Retrying.", e.what());
        }
    }
    if(!success)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to get demand set master result. Exiting...");
        throw DriverException("Failed to get demand set master result. Exiting...");
    }
    node_canopen_driver_->activate();
}

void CanopenDriver::shutdown()
{
    node_canopen_driver_->shutdown();
}


void CanopenDriver::set_master(
    std::shared_ptr<lely::ev::Executor> exec,
    std::shared_ptr<lely::canopen::AsyncMaster> master)
{
    node_canopen_driver_->set_master(exec, master);
}

void LifecycleCanopenDriver::init()
{
    node_canopen_driver_->init();
}

void LifecycleCanopenDriver::shutdown()
{
    node_canopen_driver_->shutdown();
}

void LifecycleCanopenDriver::set_master(
    std::shared_ptr<lely::ev::Executor> exec,
    std::shared_ptr<lely::canopen::AsyncMaster> master)
{
    node_canopen_driver_->set_master(exec, master);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_configure(const rclcpp_lifecycle::State &state)
{
    node_canopen_driver_->configure();
    bool success = false;
    for(int i = 0; i < 5; i++)
    {
        try 
        {
            node_canopen_driver_->demand_set_master();
            success = true;
            break;
        }
        catch(std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "Failed to get demand set master result becauss %s. Retrying.", e.what());
        }
    }
    if(!success)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to get demand set master result. Exiting...");
        throw DriverException("Failed to get demand set master result. Exiting...");
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_activate(const rclcpp_lifecycle::State &state)
{
    node_canopen_driver_->activate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_deactivate(const rclcpp_lifecycle::State &state)
{
    node_canopen_driver_->deactivate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_cleanup(const rclcpp_lifecycle::State &state)
{
    node_canopen_driver_->cleanup();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenDriver::on_shutdown(const rclcpp_lifecycle::State &state)
{
    node_canopen_driver_->shutdown();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}