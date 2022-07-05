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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-06-29
 *
 */
//----------------------------------------------------------------------

#include <limits>
#include <vector>

#include "canopen_ros2_control/canopen_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
    auto const kLogger = rclcpp::get_logger("CanopenSystem");
}

namespace canopen_ros2_control
{

CanopenSystem::~CanopenSystem() {

    executor_->cancel();
    printf("Joining...");
    spin_thread_->join();
    printf("Joined!");


    device_manager_.reset();
    executor_.reset();

    init_thread_->join();
    init_thread_.reset();


    executor_.reset();
    spin_thread_.reset();
}

hardware_interface::CallbackReturn CanopenSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(kLogger, "bus_config: '%s'", info_.hardware_parameters["bus_config"].c_str());
  RCLCPP_INFO(kLogger, "master_config: '%s'", info_.hardware_parameters["master_config"].c_str());
  RCLCPP_INFO(kLogger, "can_interface_name: '%s'", info_.hardware_parameters["can_interface_name"].c_str());
  RCLCPP_INFO(kLogger, "master_bin: '%s'", info_.hardware_parameters["master_bin"].c_str());

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_manager_ = std::make_shared<DeviceManager>(executor_);
  executor_->add_node(device_manager_);

  // threads
  init_thread_ = std::make_unique<std::thread>(&CanopenSystem::initDeviceManager, this);
  spin_thread_ = std::make_unique<std::thread>(&CanopenSystem::spin, this);


  return CallbackReturn::SUCCESS;
}

void CanopenSystem::spin() {

    executor_->spin();
    executor_->remove_node(device_manager_);

    RCLCPP_INFO(kLogger, "Exiting spin thread...");
}

void CanopenSystem::initDeviceManager() {

    std::string tmp_master_bin  = (info_.hardware_parameters["master_bin"] == "\"\"" ) ? "" : info_.hardware_parameters["master_bin"];


    if(device_manager_->init(info_.hardware_parameters["can_interface_name"],
                             info_.hardware_parameters["master_config"],
                             info_.hardware_parameters["bus_config"],
                             tmp_master_bin))
    {
        RCLCPP_INFO(device_manager_->get_logger(), "Initialisation successful.");
    }
    else
    {
        RCLCPP_INFO(device_manager_->get_logger(), "Initialisation failed.");
    }

}

std::vector<hardware_interface::StateInterface> CanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> CanopenSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CanopenSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands


  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type CanopenSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): read robot states
  device_manager_->get_m

//  RCLCPP_INFO(kLogger, "read...");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanopenSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'

  return hardware_interface::return_type::OK;
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_control::CanopenSystem, hardware_interface::SystemInterface)
