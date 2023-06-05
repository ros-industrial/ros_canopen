// Copyright (c) 2023, Fraunhofer IPA
// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "canopen_ros2_control/robot_system.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

using namespace canopen_ros2_control;

// auto robot_system_logger = rclcpp::get_logger("robot_system_interface");

hardware_interface::CallbackReturn RobotSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  robot_system_logger = rclcpp::get_logger(info_.name + "_interface");
  RCLCPP_INFO(robot_system_logger, "Registering hardware interface '%s'", info_.name.c_str());

  // Check bus config is specified.
  if (info_.hardware_parameters.find("bus_config") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      robot_system_logger, "No bus_config parameter provided for '%s' hardware interface.",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }
  bus_config_ = info_.hardware_parameters["bus_config"];
  RCLCPP_INFO(
    robot_system_logger, "'%s' has bus config: '%s'", info_.name.c_str(), bus_config_.c_str());

  // Check master config is specified.
  if (info_.hardware_parameters.find("master_config") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      robot_system_logger, "No master_config parameter provided for '%s' hardware interface.",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }
  master_config_ = info_.hardware_parameters["master_config"];
  RCLCPP_INFO(
    robot_system_logger, "'%s' has master config: '%s'", info_.name.c_str(),
    master_config_.c_str());

  // Check master bin is specified.
  if (info_.hardware_parameters.find("master_bin") != info_.hardware_parameters.end())
  {
    master_bin_ = info_.hardware_parameters["master_bin"];
    if (master_bin_ == "\"\"")
    {
      master_bin_ = "";
    }
    RCLCPP_INFO(
      robot_system_logger, "'%s' has master bin: '%s'", info_.name.c_str(), master_bin_.c_str());
  }
  else
  {
    master_bin_ = "";
  }

  // Check can_interface_name is specified.
  if (info_.hardware_parameters.find("can_interface_name") == info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      robot_system_logger, "No can_interface_name parameter provided for '%s' hardware interface.",
      info_.name.c_str());
    return CallbackReturn::ERROR;
  }
  can_interface_ = info_.hardware_parameters["can_interface_name"];
  RCLCPP_INFO(
    robot_system_logger, "'%s' has can interface: '%s'", info_.name.c_str(),
    can_interface_.c_str());

  ros2_canopen::ConfigurationManager config(bus_config_);
  config.init_config();

  // Load joint data
  for (auto joint : info.joints)
  {
    auto driver_type =
      config.get_entry<std::string>(joint.parameters["device_name"], "driver").value();
    if (driver_type == "ros2_canopen::Cia402Driver")
    {
      auto data = Cia402Data();
      if (data.init_data(joint, config.dump_device(joint.parameters["device_name"])))
      {
        robot_motor_data_.push_back(data);
      }
      {
        robot_motor_data_.push_back(data);
      }
    }
    else
    {
      RCLCPP_ERROR(
        robot_system_logger, "Driver type '%s' not supported for joint '%s'", driver_type.c_str(),
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  executor_ =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 2);
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  spin_thread_ = std::make_unique<std::thread>(&RobotSystem::spin, this);
  init_thread_ = std::make_unique<std::thread>(&RobotSystem::initDeviceContainer, this);

  if (init_thread_->joinable())
  {
    init_thread_->join();
  }
  else
  {
    RCLCPP_ERROR(robot_system_logger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn RobotSystem::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto & data : robot_motor_data_)
  {
    if (!data.driver->init_motor())
    {
      RCLCPP_ERROR(robot_system_logger, "Failed to activate '%s'", data.joint_name.c_str());
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn RobotSystem::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  for (auto & data : robot_motor_data_)
  {
    if (!data.driver->halt_motor())
    {
      RCLCPP_ERROR(robot_system_logger, "Failed to deactivate '%s'", data.joint_name.c_str());
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn RobotSystem::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  clean();
  return CallbackReturn::SUCCESS;
}
hardware_interface::CallbackReturn RobotSystem::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  clean();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Iterate over joints in xacro
  for (canopen_ros2_control::Cia402Data & data : robot_motor_data_)
  {
    data.export_state_interface(state_interfaces);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Iterate over joints in xacro
  for (canopen_ros2_control::Cia402Data & data : robot_motor_data_)
  {
    data.export_command_interface(command_interfaces);
  }
  return command_interfaces;
}

hardware_interface::return_type RobotSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Iterate over joints
  for (canopen_ros2_control::Cia402Data & data : robot_motor_data_)
  {
    data.read_state();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (canopen_ros2_control::Cia402Data & data : robot_motor_data_)
  {
    data.write_target();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // register interfaces to start per device
  for (auto interface : start_interfaces)
  {
    auto it = std::find_if(
      robot_motor_data_.begin(), robot_motor_data_.end(),
      [interface](Cia402Data & data)
      {
        return std::find(data.interfaces.begin(), data.interfaces.end(), interface) !=
               data.interfaces.end();
      });
    if (it != robot_motor_data_.end())
    {
      it->interfaces_to_start.push_back(
        hardware_interface::CommandInterface(interface).get_interface_name());
    }
  }

  // register interfaces to stop per device
  for (auto interface : stop_interfaces)
  {
    auto it = std::find_if(
      robot_motor_data_.begin(), robot_motor_data_.end(),
      [interface](Cia402Data & data)
      {
        return std::find(data.interfaces.begin(), data.interfaces.end(), interface) !=
               data.interfaces.end();
      });
    if (it != robot_motor_data_.end())
    {
      it->interfaces_to_stop.push_back(
        hardware_interface::CommandInterface(interface).get_interface_name());
    }
  }

  // perform switching
  for (auto & data : robot_motor_data_)
  {
    if (!data.perform_switch())
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

void RobotSystem::initDeviceContainer()
{
  // Init device container
  device_container_->init(
    this->can_interface_, this->master_config_, this->bus_config_, this->master_bin_);

  // Get all registered drivers.
  auto drivers = device_container_->get_registered_drivers();

  // Iterate over all drivers and allocate them to the correct joint.
  for (auto & data : robot_motor_data_)
  {
    // Find correct driver for joint via node id.
    auto driver = std::find_if(
      drivers.begin(), drivers.end(),
      [&data](const std::pair<int, std::shared_ptr<ros2_canopen::CanopenDriverInterface>> & driver)
      { return driver.first == data.node_id; });

    if (driver == drivers.end())
    {
      RCLCPP_ERROR(
        device_container_->get_logger(), "Could not find driver for joint '%s' with node id '%d'",
        data.joint_name.c_str(), data.node_id);
      continue;
    }

    // Allocate driver to joint.
    if (
      device_container_->get_driver_type(driver->first).compare("ros2_canopen::Cia402Driver") == 0)
    {
      data.driver = std::static_pointer_cast<ros2_canopen::Cia402Driver>(driver->second);
    }
  }
  RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
}

void RobotSystem::spin()
{
  executor_->spin();
  executor_->remove_node(device_container_);
  RCLCPP_INFO(device_container_->get_logger(), "Stopped spinning RobotSystem ROS2 executor");
}

void RobotSystem::clean()
{
  printf("Cancel exectutor...");
  executor_->cancel();
  printf("Join spin thread...");
  spin_thread_->join();

  printf("Reset variables...");
  device_container_.reset();
  executor_.reset();

  init_thread_->join();
  init_thread_.reset();

  executor_.reset();
  spin_thread_.reset();
  robot_motor_data_.clear();
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(canopen_ros2_control::RobotSystem, hardware_interface::SystemInterface)
