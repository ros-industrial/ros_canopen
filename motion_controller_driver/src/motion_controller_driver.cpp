#include "motion_controller_driver/motion_controller_driver.hpp"

using namespace ros2_canopen;
using namespace std::placeholders;

void MotionControllerDriver::handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        motor_->handleInit();
        response->success = true;
    }
}

void MotionControllerDriver::handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        motor_->handleHalt();
    }
}

void MotionControllerDriver::handle_set_mode_position(
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

void MotionControllerDriver::handle_set_mode_velocity(
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

void MotionControllerDriver::handle_set_mode_torque(
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

void MotionControllerDriver::handle_set_target(
    const ros2_canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    ros2_canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
    if (active.load())
    {
        response->success = motor_->setTarget(request->target);
    }
}

void MotionControllerDriver::register_services()
{
    handle_init_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/init").c_str(),
        std::bind(&MotionControllerDriver::handle_init, this, _1, _2));

    handle_halt_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/halt").c_str(),
        std::bind(&MotionControllerDriver::handle_halt, this, _1, _2));

    handle_set_mode_position_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/position_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_position, this, _1, _2));

    handle_set_mode_velocity_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/velocity_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_velocity, this, _1, _2));

    handle_set_mode_torque_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/torque_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_torque, this, _1, _2));

    handle_set_target_service = this->create_service<ros2_canopen_interfaces::srv::COTargetDouble>(
        std::string(this->get_name()).append("/target").c_str(),
        std::bind(&MotionControllerDriver::handle_set_target, this, _1, _2)

    );
}
void MotionControllerDriver::init(ev::Executor &exec,
          canopen::AsyncMaster &master,
          uint8_t node_id) noexcept
{
    RCLCPP_INFO(this->get_logger(), "Intitialising MotionControllerDriver");
    ProxyDeviceDriver::init(exec, master, node_id);
    mc_driver_ = std::make_shared<MCDeviceDriver>(exec, master, node_id);
    motor_ = std::make_shared<Motor402>(std::string("motor"), mc_driver_);
    register_services();
    timer_ = this->create_wall_timer(
        20ms, std::bind(&MotionControllerDriver::run, this));
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::MotionControllerDriver)