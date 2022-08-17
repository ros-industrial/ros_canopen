#include "canopen_402_driver/lifecycle_canopen_402_driver.hpp"

using namespace ros2_canopen;
using namespace std::placeholders;

void LifecycleMotionControllerDriver::handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        motor_->handleInit();
        mc_driver_->validate_objs();
        response->success = true;
    }
}

void LifecycleMotionControllerDriver::handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        motor_->handleRecover();
        response->success = true;
    }
}

void LifecycleMotionControllerDriver::handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        motor_->handleHalt();
    }
}

void LifecycleMotionControllerDriver::handle_set_mode_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Position)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Position);
            return;
        }

        response->success = false;
    }
}

void LifecycleMotionControllerDriver::handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Velocity)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Velocity);
            return;
        }
        response->success = false;
    }
}

void LifecycleMotionControllerDriver::handle_set_mode_cyclic_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Position)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Position);
            return;
        }
        response->success = false;
    }
}

void LifecycleMotionControllerDriver::handle_set_mode_cyclic_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Velocity)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Velocity);
            return;
        }
        response->success = false;
    }
}

void LifecycleMotionControllerDriver::handle_set_mode_torque(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Torque)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Torque);
        }
        response->success = true;
    }
}

void LifecycleMotionControllerDriver::handle_set_target(
    const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
    if (activated_.load())
    {
        response->success = motor_->setTarget(request->target);
    }
}

void LifecycleMotionControllerDriver::publish()
{
    std_msgs::msg::Float64 pos_msg;
    std_msgs::msg::Float64 speed_msg;
    pos_msg.data = mc_driver_->get_position();
    speed_msg.data = mc_driver_->get_speed();
    publish_actual_position->publish(pos_msg);
    publish_actual_speed->publish(speed_msg);
}

void LifecycleMotionControllerDriver::register_ros_interface()
{
    RCLCPP_INFO(this->get_logger(), "register_ros_interface");
    LifecycleProxyDriver::register_ros_interface();
    publish_actual_position = this->create_publisher<std_msgs::msg::Float64>("~/actual_position", 10);

    publish_actual_speed = this->create_publisher<std_msgs::msg::Float64>("~/actual_speed", 10);
    handle_init_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/init").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_init, this, _1, _2));

    handle_halt_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/halt").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_halt, this, _1, _2));

    handle_recover_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/recover").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_recover, this, _1, _2));

    handle_set_mode_position_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/position_mode").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_set_mode_position, this, _1, _2));

    handle_set_mode_velocity_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/velocity_mode").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_set_mode_velocity, this, _1, _2));

    handle_set_mode_cyclic_velocity_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/cyclic_velocity_mode").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_set_mode_cyclic_velocity, this, _1, _2));

    handle_set_mode_cyclic_position_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/cyclic_position_mode").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_set_mode_cyclic_position, this, _1, _2));

    handle_set_mode_torque_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/torque_mode").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_set_mode_torque, this, _1, _2));

    handle_set_target_service = this->create_service<canopen_interfaces::srv::COTargetDouble>(
        std::string(this->get_name()).append("/target").c_str(),
        std::bind(&LifecycleMotionControllerDriver::handle_set_target, this, _1, _2));
}

void LifecycleMotionControllerDriver::start_timers()
{
    ros2_canopen::LifecycleProxyDriver::start_timers();
    RCLCPP_INFO(this->get_logger(), "start_timers_start");
    std::this_thread::sleep_for(10ms);
    motor_->registerDefaultModes();
    mc_driver_->validate_objs();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms_),
        std::bind(&LifecycleMotionControllerDriver::run,
                  this),
        this->timer_cbg_);
    RCLCPP_INFO(this->get_logger(), "start_timers_end");
}

void LifecycleMotionControllerDriver::stop_timers()
{
    timer_->cancel();
}

bool LifecycleMotionControllerDriver::add()
{
    RCLCPP_INFO(this->get_logger(), "add_start");
    std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::MCDeviceDriver>>> prom;
    prom = std::make_shared<std::promise<std::shared_ptr<ros2_canopen::MCDeviceDriver>>>();
    std::future<std::shared_ptr<ros2_canopen::MCDeviceDriver>> f = prom->get_future();
    master_->GetExecutor().post(
        [this, prom]()
        {
            RCLCPP_INFO(this->get_logger(), "in_executor_start");
            std::scoped_lock<std::mutex> lock(this->driver_mutex_);
            mc_driver_ = std::make_shared<MCDeviceDriver>(*exec_, *master_, node_id_);
            RCLCPP_INFO(this->get_logger(), "inexecutor_boot");
            mc_driver_->Boot();
            RCLCPP_INFO(this->get_logger(), "inexecutor_set");
            prom->set_value(mc_driver_);
            RCLCPP_INFO(this->get_logger(), "in_executor_end");
        });
    RCLCPP_INFO(this->get_logger(), "wait_future");
    auto future_status = f.wait_for(this->non_transmit_timeout_);
    if (future_status != std::future_status::ready)
    {
        RCLCPP_INFO(this->get_logger(), "timed_out");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "get_future");
    mc_driver_ = f.get();
    RCLCPP_INFO(this->get_logger(), "set_driver");
    motor_ = std::make_shared<Motor402>(mc_driver_);
    driver_ = std::static_pointer_cast<LelyBridge>(mc_driver_);
    RCLCPP_INFO(this->get_logger(), "add_end");
    return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleMotionControllerDriver::on_configure(const rclcpp_lifecycle::State &state)
{
    return LifecycleProxyDriver::on_configure(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleMotionControllerDriver::on_activate(const rclcpp_lifecycle::State &state)
{
    return LifecycleProxyDriver::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleMotionControllerDriver::on_deactivate(const rclcpp_lifecycle::State &state)
{
    return LifecycleProxyDriver::on_deactivate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleMotionControllerDriver::on_cleanup(const rclcpp_lifecycle::State &state)
{
    return LifecycleProxyDriver::on_cleanup(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleMotionControllerDriver::on_shutdown(const rclcpp_lifecycle::State &state)
{
    return LifecycleProxyDriver::on_shutdown(state);
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleMotionControllerDriver)
