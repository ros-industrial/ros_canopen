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
////    state_interfaces.emplace_back(hardware_interface::StateInterface(
////      // TODO(anyone): insert correct interfaces
////      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
////
      if(info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
      {
          // skip adding canopen interfaces
          continue;
      }
////      const uint8_t node_id = static_cast<uint8_t >(std::stoi(info_.joints[i].parameters["node_id"]));
//////      RCLCPP_INFO(kLogger, "node id on export state interface for joint: '%s' is '%s'", info_.joints[i].name.c_str(), info_.joints[i].parameters["node_id"].c_str());
////
////      // rpdo index
////      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/index",
////              &canopen_data_[node_id].rpdo_data.index));
////
////      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/subindex",
////                                                                       &canopen_data_[node_id].rpdo_data.subindex));
////
////      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/type",
////                                                                       &canopen_data_[node_id].rpdo_data.type));
////
////      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/data",
////                                                                       &canopen_data_[node_id].rpdo_data.data));
////
////      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "nmt/state",
////                                                                       &canopen_data_[node_id].nmt_state.state));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Cia402System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
//  for (uint i = 0; i < info_.joints.size(); i++) {
//    command_interfaces.emplace_back(hardware_interface::CommandInterface(
//      // TODO(anyone): insert correct interfaces
//      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
//
//      if(info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
//      {
//          // skip adding canopen interfaces
//          continue;
//      }
//
//      const uint8_t node_id = static_cast<uint8_t >(std::stoi(info_.joints[i].parameters["node_id"]));
//
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/index",
//                                                                       &canopen_data_[node_id].tpdo_data.index));
//
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/subindex",
//                                                                       &canopen_data_[node_id].tpdo_data.subindex));
//
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/type",
//                                                                       &canopen_data_[node_id].tpdo_data.type));
//
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/data",
//                                                                       &canopen_data_[node_id].tpdo_data.data));
//
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/ons",
//                                                                           &canopen_data_[node_id].tpdo_data.one_shot));
//
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "nmt/reset",
//                                                                         &canopen_data_[node_id].nmt_state.reset_ons));
//      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "nmt/start",
//                                                                         &canopen_data_[node_id].nmt_state.start_ons));
//
//  }

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

   auto data = get_canopen_data();

    for(auto it = data.begin(); it!=data.end(); ++it) {
        auto motion_controller_driver = std::static_pointer_cast<ros2_canopen::MotionControllerDriver>(
                get_device_manager()->get_node(it->first));

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
