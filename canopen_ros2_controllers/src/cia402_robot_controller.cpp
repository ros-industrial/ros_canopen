#include "canopen_ros2_controllers/cia402_robot_controller.hpp"

namespace cia402_robot_controller
{

enum CommandInterface
{
  INIT,
  INIT_FEEDBACK,
  HALT,
  HALT_FEEDBACK,
  RECOVER,
  RECOVER_FEEDBACK,
  OPERATION_MODE,
  OPERATION_MODE_FEEDBACK,
  ADD_OP
};

Cia402RobotController::Cia402RobotController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn Cia402RobotController::on_init()
{
  try
  {
    declare_parameters();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void Cia402RobotController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn Cia402RobotController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }
  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Cia402RobotController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.operation_mode == 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'operation_mode' parameter was not correctly specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.command_poll_freq <= 0)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "'command_poll_freq' parameter was not correctly specified");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interfaces_config_.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto joint_name : params_.joints)
  {
    command_interfaces_config_.names.push_back(joint_name + "/" + "init");
    command_interfaces_config_.names.push_back(joint_name + "/" + "init_feedback");
    command_interfaces_config_.names.push_back(joint_name + "/" + "halt");
    command_interfaces_config_.names.push_back(joint_name + "/" + "halt_feedback");
    command_interfaces_config_.names.push_back(joint_name + "/" + "recover");
    command_interfaces_config_.names.push_back(joint_name + "/" + "recover_feedback");
    command_interfaces_config_.names.push_back(joint_name + "/" + "operation_mode");
    command_interfaces_config_.names.push_back(joint_name + "/" + "operation_mode_feedback");
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Cia402RobotController::command_interface_configuration() const
{
  return command_interfaces_config_;
}

controller_interface::InterfaceConfiguration Cia402RobotController::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn Cia402RobotController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  timer_ = this->get_node()->create_wall_timer(
    std::chrono::milliseconds(params_.command_poll_freq),
    std::bind(&Cia402RobotController::activate, this));
  return controller_interface::CallbackReturn::SUCCESS;
}

void Cia402RobotController::activate()
{
  int jn = 0;
  for (auto joint_name : params_.joints)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Initialise '%s'", joint_name.c_str());
    // Initialise joint
    ////////////////////////////////////////////////////////////////////////////////////////
    RCLCPP_INFO(
      get_node()->get_logger(), "Using %s to init",
      command_interfaces_[jn + INIT].get_full_name().c_str());
    command_interfaces_[jn + INIT].set_value(1.0);
    while (std::isnan(command_interfaces_[jn + INIT_FEEDBACK].get_value()) && rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(params_.command_poll_freq));
    }
    if (command_interfaces_[jn + INIT_FEEDBACK].get_value() != 1.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Init of '%s' failed", joint_name.c_str());
      // return controller_interface::CallbackReturn::ERROR;
    }

    command_interfaces_[jn + INIT].set_value(std::numeric_limits<double>::quiet_NaN());
    command_interfaces_[jn + INIT_FEEDBACK].set_value(std::numeric_limits<double>::quiet_NaN());
    // rclcpp::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(
      get_node()->get_logger(), "Setting operation mode '%li' of '%s'", params_.operation_mode,
      joint_name.c_str());
    // Set operation mode for joint
    // Some devices may require setting operation mode before init.
    ////////////////////////////////////////////////////////////////////////////////////////

    command_interfaces_[jn + OPERATION_MODE].set_value(params_.operation_mode);
    while (std::isnan(command_interfaces_[jn + OPERATION_MODE_FEEDBACK].get_value()) &&
           rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(params_.command_poll_freq));
    }
    if (command_interfaces_[jn + OPERATION_MODE_FEEDBACK].get_value() != 1.0)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Setting operation mode '%li' of '%s' failed",
        params_.operation_mode, joint_name.c_str());
      // return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[jn + OPERATION_MODE].set_value(std::numeric_limits<double>::quiet_NaN());
    command_interfaces_[jn + OPERATION_MODE_FEEDBACK].set_value(
      std::numeric_limits<double>::quiet_NaN());
    jn += ADD_OP;
  }
  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  timer_->cancel();
}

controller_interface::CallbackReturn Cia402RobotController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  int jn = 0;
  for (auto joint_name : params_.joints)
  {
    // Halt joint
    ////////////////////////////////////////////////////////////////////////////////////////
    command_interfaces_[jn + HALT].set_value(1.0);
    while (std::isnan(command_interfaces_[jn + HALT_FEEDBACK].get_value()) && rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(params_.command_poll_freq));
    }
    if (command_interfaces_[jn + HALT_FEEDBACK].get_value() != 1.0)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Halting of '%s' failed", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    command_interfaces_[jn + HALT].set_value(std::numeric_limits<double>::quiet_NaN());
    command_interfaces_[jn + HALT_FEEDBACK].set_value(std::numeric_limits<double>::quiet_NaN());

    // Set operation mode for joint
    // Some devices may require setting operation mode before init.
    ////////////////////////////////////////////////////////////////////////////////////////

    command_interfaces_[jn + OPERATION_MODE].set_value(0.0);
    while (std::isnan(command_interfaces_[jn + OPERATION_MODE_FEEDBACK].get_value()) &&
           rclcpp::ok())
    {
      rclcpp::sleep_for(std::chrono::milliseconds(params_.command_poll_freq));
    }
    if (command_interfaces_[jn + OPERATION_MODE_FEEDBACK].get_value() != 1.0)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Setting operation mode '%li' of '%s' failed",
        params_.operation_mode, joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[jn + OPERATION_MODE].set_value(std::numeric_limits<double>::quiet_NaN());
    command_interfaces_[jn + OPERATION_MODE_FEEDBACK].set_value(
      std::numeric_limits<double>::quiet_NaN());
    jn += ADD_OP;
  }
  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  RCLCPP_INFO(get_node()->get_logger(), "deactivate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Cia402RobotController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return controller_interface::return_type::OK;
}
}  // namespace cia402_robot_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  cia402_robot_controller::Cia402RobotController, controller_interface::ControllerInterface)
