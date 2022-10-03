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
 * \date    2022-08-01
 *
 */
//----------------------------------------------------------------------


#include "canopen_ros2_control/cia402_system.hpp"
#include "canopen_402_driver/canopen_402_driver.hpp" // for MotionControllerDriver

namespace {
    auto const kLogger = rclcpp::get_logger("Cia402System");
}

namespace canopen_ros2_control
{

Cia402System::~Cia402System() {

}

hardware_interface::CallbackReturn Cia402System::on_init(
  const hardware_interface::HardwareInfo & info)
{

    if (CanopenSystem::on_init(info) != CallbackReturn::SUCCESS) {
        return CallbackReturn::ERROR;
    }


  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Cia402System::export_state_interfaces()
{

  std::vector<hardware_interface::StateInterface> state_interfaces;

  // underlying base class export first
  state_interfaces = CanopenSystem::export_state_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {

      if(info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
      {
          // skip adding motor canopen interfaces
          continue;
      }
      const uint8_t node_id = static_cast<uint8_t >(std::stoi(info_.joints[i].parameters["node_id"]));

      // actual position
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "actual_position",
                                                                       &motor_data_[node_id].actual_position));
      // actual speed
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "actual_speed",
              &motor_data_[node_id].actual_speed));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Cia402System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // underlying base class export first
  command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++) {

      if(info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
      {
          // skip adding canopen interfaces
          continue;
      }

      const uint8_t node_id = static_cast<uint8_t >(std::stoi(info_.joints[i].parameters["node_id"]));

      // init
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "init_cmd",
                                                                       &motor_data_[node_id].init.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "init_resp",
                                                                           &motor_data_[node_id].init.resp));

      // halt
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "halt_cmd",
                                                                           &motor_data_[node_id].halt.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "halt_resp",
                                                                           &motor_data_[node_id].halt.resp));

      // recover
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "recover_cmd",
                                                                           &motor_data_[node_id].recover.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "recover_resp",
                                                                           &motor_data_[node_id].recover.resp));

      // set position mode
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "position_mode_cmd",
                                                                           &motor_data_[node_id].position_mode.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "position_mode_resp",
                                                                           &motor_data_[node_id].position_mode.resp));

      // set velocity mode
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "velocity_mode_cmd",
                                                                           &motor_data_[node_id].velocity_mode.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "velocity_mode_resp",
                                                                           &motor_data_[node_id].velocity_mode.resp));

      // set cyclic velocity mode
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "cyclic_velocity_mode_cmd",
                                                                           &motor_data_[node_id].cyclic_velocity_mode.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "cyclic_velocity_mode_resp",
                                                                           &motor_data_[node_id].cyclic_velocity_mode.resp));
      // set cyclic position mode
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "cyclic_position_mode_cmd",
                                                                           &motor_data_[node_id].cyclic_position_mode.ons_cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "cyclic_position_mode_resp",
                                                                           &motor_data_[node_id].cyclic_position_mode.resp));

  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Cia402System::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to receive commands

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Cia402System::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Cia402System::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): read robot states

   auto ret_val = CanopenSystem::read(time, period);

   auto data = canopen_data_;

    for(auto it = data.begin(); it!=data.end(); ++it) {
        auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::MotionControllerDriver>(
                device_container_->get_node(it->first));

        // get position
        motor_data_[it->first].actual_position = motion_controller_driver->get_position();
        // get speed
        motor_data_[it->first].actual_speed = motion_controller_driver->get_speed();

    }

    return ret_val;
}

hardware_interface::return_type Cia402System::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(anyone): write robot's commands'

    auto ret_val = CanopenSystem::write(time, period);

    return ret_val;
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_control::Cia402System, hardware_interface::SystemInterface)
