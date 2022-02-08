#include "canopen_402/mc_device_node.hpp"

using namespace ros2_canopen;
using namespace canopen_402;
using namespace std::placeholders;

void MCDeviceNode::handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        motor_->handleInit();
    }
}

void MCDeviceNode::handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        motor_->handleHalt();
    }
}

void MCDeviceNode::handle_set_mode_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Position)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Position);
        }
        response->success = true;
    }
}

void MCDeviceNode::handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Velocity)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Velocity);
        }
        response->success = true;
    }
}

void MCDeviceNode::handle_set_mode_torque(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Torque)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Torque);
        }
        response->success = true;
    }
}

void MCDeviceNode::handle_set_target(
    const ros2_canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    ros2_canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
    if (active.load())
    {
        response->success = motor_->setTarget(request->target);
    }
}

void MCDeviceNode::register_services()
{
    handle_init_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/init").c_str(),
        std::bind(&MCDeviceNode::handle_init, this, _1, _2)
    );

    handle_halt_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/halt").c_str(),
        std::bind(&MCDeviceNode::handle_halt, this, _1, _2)
    );

    handle_set_mode_position_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/position_mode").c_str(),
        std::bind(&MCDeviceNode::handle_set_mode_position, this, _1, _2)
    );

    handle_set_mode_velocity_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/velocity_mode").c_str(),
        std::bind(&MCDeviceNode::handle_set_mode_velocity, this, _1, _2)
    );

    handle_set_mode_torque_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/torque_mode").c_str(),
        std::bind(&MCDeviceNode::handle_set_mode_torque, this, _1, _2)
    );

    handle_set_target_service = this->create_service<ros2_canopen_interfaces::srv::COTargetDouble>(
        std::string(this->get_name()).append("/target").c_str(),
        std::bind(&MCDeviceNode::handle_set_target, this, _1, _2)
        
    );
}