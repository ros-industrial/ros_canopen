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
        mc_driver_->validate_objs();
        response->success = true;
    }
}

void MotionControllerDriver::handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        motor_->handleRecover();
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
            return;
        }

        response->success = false;
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
            return;
        }
        response->success = false;
    }
}

void MotionControllerDriver::handle_set_mode_cyclic_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Position)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Position);
            return;
        }
        response->success = false;
    }
}

void MotionControllerDriver::handle_set_mode_cyclic_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (active.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Velocity)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Velocity);
            return;
        }
        response->success = false;
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

    handle_recover_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/recover").c_str(),
        std::bind(&MotionControllerDriver::handle_recover, this, _1, _2));

    handle_set_mode_position_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/position_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_position, this, _1, _2));

    handle_set_mode_velocity_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/velocity_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_velocity, this, _1, _2));

    handle_set_mode_cyclic_velocity_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/cyclic_velocity_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_cyclic_velocity, this, _1, _2));

    handle_set_mode_cyclic_position_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/cyclic_position_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_cyclic_position, this, _1, _2));

    handle_set_mode_torque_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/torque_mode").c_str(),
        std::bind(&MotionControllerDriver::handle_set_mode_torque, this, _1, _2));

    handle_set_target_service = this->create_service<ros2_canopen_interfaces::srv::COTargetDouble>(
        std::string(this->get_name()).append("/target").c_str(),
        std::bind(&MotionControllerDriver::handle_set_target, this, _1, _2));
}

void MotionControllerDriver::init(ev::Executor &exec,
                                  canopen::AsyncMaster &master,
                                  uint8_t node_id) noexcept
{
    RCLCPP_INFO(this->get_logger(), "Intitialising MotionControllerDriver");
    ProxyDeviceDriver::init(exec, master, node_id);
    driver.reset();
    mc_driver_ = std::make_shared<MCDeviceDriver>(exec, master, node_id);
    driver = std::static_pointer_cast<LelyBridge>(mc_driver_);
    motor_ = std::make_shared<Motor402>(std::string("motor"), mc_driver_);
    register_services();
    timer_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = this->create_wall_timer(
        2000ms, std::bind(&MotionControllerDriver::run, this), timer_group);
    driver->Boot();
    active.store(true);
    RCLCPP_INFO(this->get_logger(), "Intialised with nodeid %hhu.", node_id);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::MotionControllerDriver)
