#ifndef NODE_CANOPEN_402_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_402_DRIVER_IMPL_HPP_

#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
#include "canopen_core/driver_error.hpp"

#include <optional>

using namespace ros2_canopen::node_interfaces;
using namespace std::placeholders;

template <class NODETYPE>
NodeCanopen402Driver<NODETYPE>::NodeCanopen402Driver(NODETYPE *node) : ros2_canopen::node_interfaces::NodeCanopenProxyDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::init(bool called_from_base)
{
	RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::init(bool called_from_base)
{
	NodeCanopenProxyDriver<rclcpp::Node>::init(false);
	publish_joint_state = this->node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
	handle_init_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/init").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_init, this, _1, _2));

	handle_halt_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/halt").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_halt, this, _1, _2));

	handle_recover_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/recover").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_recover, this, _1, _2));

	handle_set_mode_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/position_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_position, this, _1, _2));

	handle_set_mode_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/velocity_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_velocity, this, _1, _2));

	handle_set_mode_cyclic_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/cyclic_velocity_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_cyclic_velocity, this, _1, _2));

	handle_set_mode_cyclic_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/cyclic_position_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_cyclic_position, this, _1, _2));

	handle_set_mode_torque_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/torque_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_set_mode_torque, this, _1, _2));

	handle_set_target_service = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
		std::string(this->node_->get_name()).append("/target").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp::Node>::handle_set_target, this, _1, _2));
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
	NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(false);
	publish_joint_state = this->node_->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
	handle_init_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/init").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_init, this, _1, _2));

	handle_halt_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/halt").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_halt, this, _1, _2));

	handle_recover_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/recover").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_recover, this, _1, _2));

	handle_set_mode_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/position_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_position, this, _1, _2));

	handle_set_mode_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/velocity_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_velocity, this, _1, _2));

	handle_set_mode_cyclic_velocity_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/cyclic_velocity_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_cyclic_velocity, this, _1, _2));

	handle_set_mode_cyclic_position_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/cyclic_position_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_cyclic_position, this, _1, _2));

	handle_set_mode_torque_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/torque_mode").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_mode_torque, this, _1, _2));

	handle_set_target_service = this->node_->create_service<canopen_interfaces::srv::COTargetDouble>(
		std::string(this->node_->get_name()).append("/target").c_str(),
		std::bind(&NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::handle_set_target, this, _1, _2));
}

template <>
void NodeCanopen402Driver<rclcpp_lifecycle::LifecycleNode>::configure(bool called_from_base)
{
	NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::configure(false);
	auto period = this->config_["period"].as<uint32_t>();
    std::optional<double> scale_pos_to_dev;
    std::optional<double> scale_pos_from_dev;
    std::optional<double> scale_vel_to_dev;
    std::optional<double> scale_vel_from_dev;
    try
    {
        scale_pos_to_dev = std::optional(this->config_["scale_pos_to_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
        scale_pos_from_dev = std::optional(this->config_["scale_pos_from_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
        scale_vel_to_dev = std::optional(this->config_["scale_vel_to_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
        scale_vel_from_dev = std::optional(this->config_["scale_vel_from_dev"].as<double>());
    }
    catch (...)
    {
    }

    // auto period = this->config_["scale_eff_to_dev"].as<double>();
    // auto period = this->config_["scale_eff_from_dev"].as<double>();
    scale_pos_to_dev_ = scale_pos_to_dev.value_or(1000.0);
    scale_pos_from_dev_ = scale_pos_from_dev.value_or(0.001);
    scale_vel_to_dev_ = scale_vel_to_dev.value_or(1000.0);
    scale_vel_from_dev_ = scale_vel_from_dev.value_or(0.001);
    RCLCPP_INFO(this->node_->get_logger(), "scale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ %f\nscale_vel_from_dev_ %f\n",
        scale_pos_to_dev_,
        scale_pos_from_dev_,
        scale_vel_to_dev_,
        scale_vel_from_dev_
    );
	period_ms_ = period;
}

template <>
void NodeCanopen402Driver<rclcpp::Node>::configure(bool called_from_base)
{
	NodeCanopenProxyDriver<rclcpp::Node>::configure(false);
	auto period = this->config_["period"].as<uint32_t>();
    std::optional<double> scale_pos_to_dev;
    std::optional<double> scale_pos_from_dev;
    std::optional<double> scale_vel_to_dev;
    std::optional<double> scale_vel_from_dev;
    try
    {
        scale_pos_to_dev = std::optional(this->config_["scale_pos_to_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
        scale_pos_from_dev = std::optional(this->config_["scale_pos_from_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
        scale_vel_to_dev = std::optional(this->config_["scale_vel_to_dev"].as<double>());
    }
    catch (...)
    {
    }
    try
    {
        scale_vel_from_dev = std::optional(this->config_["scale_vel_from_dev"].as<double>());
    }
    catch (...)
    {
    }

    // auto period = this->config_["scale_eff_to_dev"].as<double>();
    // auto period = this->config_["scale_eff_from_dev"].as<double>();
    scale_pos_to_dev_ = scale_pos_to_dev.value_or(1000.0);
    scale_pos_from_dev_ = scale_pos_from_dev.value_or(0.001);
    scale_vel_to_dev_ = scale_vel_to_dev.value_or(1000.0);
    scale_vel_from_dev_ = scale_vel_from_dev.value_or(0.001);
    RCLCPP_INFO(this->node_->get_logger(), "scale_pos_to_dev_ %f\nscale_pos_from_dev_ %f\nscale_vel_to_dev_ %f\nscale_vel_from_dev_ %f\n",
        scale_pos_to_dev_,
        scale_pos_from_dev_,
        scale_vel_to_dev_,
        scale_vel_from_dev_
    );
	period_ms_ = period;
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::activate(bool called_from_base)
{
	motor_->registerDefaultModes();
	mc_driver_->validate_objs();
	timer_ = this->node_->create_wall_timer(
		std::chrono::milliseconds(period_ms_),
		std::bind(&NodeCanopen402Driver<NODETYPE>::run,
				  this),
		this->timer_cbg_);
	NodeCanopenProxyDriver<NODETYPE>::activate(false);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::deactivate(bool called_from_base)
{
	NodeCanopenProxyDriver<NODETYPE>::deactivate(false);
	timer_->cancel();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::run()
{
	motor_->handleRead();
	motor_->handleWrite();
	publish();
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::publish()
{
	sensor_msgs::msg::JointState js_msg;
	js_msg.name.push_back(this->node_->get_name());
	js_msg.position.push_back(mc_driver_->get_position() * scale_pos_from_dev_);
	js_msg.velocity.push_back(mc_driver_->get_speed() * scale_vel_from_dev_);
	js_msg.effort.push_back(0.0);
	publish_joint_state->publish(js_msg);
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::add_to_master()
{
	std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::LelyMotionControllerBridge>>> prom;
	prom = std::make_shared<std::promise<std::shared_ptr<ros2_canopen::LelyMotionControllerBridge>>>();
	std::future<std::shared_ptr<ros2_canopen::LelyMotionControllerBridge>> f = prom->get_future();
	this->exec_->post(
		[this, prom]()
		{
			std::scoped_lock<std::mutex> lock(this->driver_mutex_);
			mc_driver_ = std::make_shared<LelyMotionControllerBridge>(*(this->exec_), *(this->master_), this->node_id_, this->node_->get_name());
			mc_driver_->Boot();
			prom->set_value(mc_driver_);
		});
	auto future_status = f.wait_for(this->non_transmit_timeout_);
	if (future_status != std::future_status::ready)
	{
		RCLCPP_ERROR(this->node_->get_logger(), "Adding timed out.");
		throw DriverException("add_to_master: Adding timed out.");
	}
	this->mc_driver_ = f.get();
	this->motor_ = std::make_shared<Motor402>(mc_driver_);
	this->lely_driver_ = std::static_pointer_cast<LelyDriverBridge>(mc_driver_);
	this->driver_ = std::static_pointer_cast<lely::canopen::BasicDriver>(mc_driver_);
	if (!this->mc_driver_->IsReady())
	{
		RCLCPP_WARN(this->node_->get_logger(), "Wait for device to boot.");
		try
		{
			this->mc_driver_->wait_for_boot();
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->node_->get_logger(), e.what());
			std::string msg;
			msg.append("add_to_master: ");
			msg.append(e.what());
			throw DriverException(msg);
		}
	}
	RCLCPP_INFO(this->node_->get_logger(), "Driver booted and ready.");
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_init(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        bool temp = motor_->handleInit();
        mc_driver_->validate_objs();
        response->success = temp;
    }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_recover(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        response->success = motor_->handleRecover();
    }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_halt(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        response->success = motor_->handleHalt();
    }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Position)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Position);
            return;
        }

        response->success = false;
    }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Velocity)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Velocity);
            return;
        }
        response->success = false;
    }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_position(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Position)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Position);
            return;
        }
        response->success = false;
    }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_cyclic_velocity(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Velocity)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Velocity);
            return;
        }
        response->success = false;
    }
}
template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_mode_torque(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Torque)
        {
            response->success = motor_->enterModeAndWait(MotorBase::Profiled_Torque);
        }
        response->success = true;
    }
}

template <class NODETYPE>
void NodeCanopen402Driver<NODETYPE>::handle_set_target(
    const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
    canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response)
{
    if (this->activated_.load())
    {
        auto mode = motor_->getMode();
        double target;
        if (
            (mode == MotorBase::Profiled_Position) or 
            (mode == MotorBase::Cyclic_Synchronous_Position)
        )
        {
            target = request->target * scale_pos_to_dev_;
        }
        else if(
            (mode == MotorBase::Velocity) or
            (mode == MotorBase::Profiled_Velocity) or
            (mode == MotorBase::Cyclic_Synchronous_Velocity)
        )
        {
            target = request->target * scale_vel_to_dev_;
        }
        else
        {
            target = request->target;
        }

        response->success = motor_->setTarget(target);
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::init_motor() {
    if (this->activated_.load())
    {
        bool temp = motor_->handleInit();
        mc_driver_->validate_objs();
        return temp;
    }else{
        RCLCPP_INFO(this->node_->get_logger(), "Initialisation failed.");
        return false;
    }

}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::recover_motor() {
    if (this->activated_.load())
    {
        return motor_->handleRecover();
    }else{
        return false;
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::halt_motor() {
    if (this->activated_.load())
    {
        return motor_->handleHalt();
    }else{
        return false;
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_mode_position() {
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Position)
        {
            return motor_->enterModeAndWait(MotorBase::Profiled_Position);
        }else{
            return false;
        }
    }else{
        return false;
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_mode_velocity() {
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Velocity)
        {
            return motor_->enterModeAndWait(MotorBase::Velocity);
        }else{
            return false;
        }
    }else{
        return false;
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_mode_cyclic_position() {
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Position)
        {
            return motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Position);
        }else{
            return false;
        }
    }else{
        return false;
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_mode_cyclic_velocity() {
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Cyclic_Synchronous_Velocity)
        {
            return motor_->enterModeAndWait(MotorBase::Cyclic_Synchronous_Velocity);
        }else{
            return false;
        }
    }else{
        return false;
    }
}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_mode_torque() {
    if (this->activated_.load())
    {
        if (motor_->getMode() != MotorBase::Profiled_Torque)
        {
            return motor_->enterModeAndWait(MotorBase::Profiled_Torque);
        }else{
            return false;
        }
    }else{
        return false;
    }}

template<class NODETYPE>
bool NodeCanopen402Driver<NODETYPE>::set_target(double target) {
    if (this->activated_.load())
    {
        auto mode = motor_->getMode();
        double scaled_target;
        if (
            (mode == MotorBase::Profiled_Position) or 
            (mode == MotorBase::Cyclic_Synchronous_Position)
        )
        {
            scaled_target = target * scale_pos_to_dev_;
        }
        else if(
            (mode == MotorBase::Velocity) or
            (mode == MotorBase::Profiled_Velocity) or
            (mode == MotorBase::Cyclic_Synchronous_Velocity)
        )
        {
            scaled_target = target * scale_vel_to_dev_;
        }
        else
        {
            scaled_target = target;
        }
        //RCLCPP_INFO(this->node_->get_logger(), "Scaled target %f", scaled_target);
        return motor_->setTarget(scaled_target);
    }
    else
    {
        return false;
    }
}

#endif