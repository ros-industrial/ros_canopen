// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "canopen_ros2_control/canopen_system.hpp"

#include <limits>
#include <vector>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
auto const kLogger = rclcpp::get_logger("CanopenSystem");
}

namespace canopen_ros2_control
{

void CanopenSystem::clean() {
  executor_->cancel();
  printf("Joining...");
  spin_thread_->join();
  printf("Joined!");

  device_container_.reset();
  executor_.reset();

  init_thread_->join();
  init_thread_.reset();

  executor_.reset();
  spin_thread_.reset();
}

CanopenSystem::~CanopenSystem() {
  clean();
}

hardware_interface::CallbackReturn CanopenSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(kLogger, "bus_config: '%s'", info_.hardware_parameters["bus_config"].c_str());
  RCLCPP_INFO(kLogger, "master_config: '%s'", info_.hardware_parameters["master_config"].c_str());
  RCLCPP_INFO(kLogger, "can_interface_name_name: '%s'", info_.hardware_parameters["can_interface_name_name"].c_str());
  RCLCPP_INFO(kLogger, "master_bin: '%s'", info_.hardware_parameters["master_bin"].c_str());

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenSystem::on_configure(
  const rclcpp_lifecycle::State &previous_state)
{
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  device_container_ = std::make_shared<ros2_canopen::DeviceContainer>(executor_);
  executor_->add_node(device_container_);

  // threads
    spin_thread_ = std::make_unique<std::thread>(&CanopenSystem::spin, this);
    init_thread_ = std::make_unique<std::thread>(&CanopenSystem::initDeviceContainer, this);

  // actually wait for init phase to end
  if (init_thread_->joinable())
  {
    init_thread_->join();
  }
  else
  {
    RCLCPP_ERROR(kLogger, "Could not join init thread!");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanopenSystem::on_cleanup(
  const rclcpp_lifecycle::State &previous_state) {
  clean();
  return CallbackReturn::SUCCESS;

}

hardware_interface::CallbackReturn CanopenSystem::on_shutdown(
  const rclcpp_lifecycle::State &previous_state) {

  clean();
  return CallbackReturn::SUCCESS;
}

void CanopenSystem::spin() {

  executor_->spin();
  executor_->remove_node(device_container_);

  RCLCPP_INFO(kLogger, "Exiting spin thread...");
}

void CanopenSystem::initDeviceContainer() {
    std::string tmp_master_bin  = (info_.hardware_parameters["master_bin"] == "\"\"" ) ? "" : info_.hardware_parameters["master_bin"];

  device_container_->init(info_.hardware_parameters["can_interface_name"],
                            info_.hardware_parameters["master_config"],
                            info_.hardware_parameters["bus_config"],
                            tmp_master_bin);
  auto drivers = device_container_->get_registered_drivers();
  RCLCPP_INFO(kLogger, "Number of registered drivers: '%zu'", device_container_->count_drivers());
  for (auto it = drivers.begin(); it != drivers.end(); it++){
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(it->second);

    auto nmt_state_cb = [&](canopen::NmtState nmt_state, uint8_t id){
      canopen_data_[id].nmt_state.set_state(nmt_state);
    };
    // register callback
    proxy_driver->register_nmt_state_cb(nmt_state_cb);

    auto rpdo_cb = [&](ros2_canopen::COData data, uint8_t id){
      canopen_data_[id].rpdo_data.set_data(data);
    };
    // register callback
    proxy_driver->register_rpdo_cb(rpdo_cb);

    RCLCPP_INFO(kLogger, "\nRegistered driver:\n    name: '%s'\n    node_id: '%u'", it->second->get_node_base_interface()->get_name(), it->first);
    }

    RCLCPP_INFO(device_container_->get_logger(), "Initialisation successful.");
  }

std::vector<hardware_interface::StateInterface> CanopenSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      // TODO(anyone): insert correct interfaces
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));

      if(info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
      {
          // skip adding canopen interfaces
          continue;
      }
      const uint8_t node_id = static_cast<uint8_t >(std::stoi(info_.joints[i].parameters["node_id"]));
//      RCLCPP_INFO(kLogger, "node id on export state interface for joint: '%s' is '%s'", info_.joints[i].name.c_str(), info_.joints[i].parameters["node_id"].c_str());

      // rpdo index
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/index",
              &canopen_data_[node_id].rpdo_data.index));

      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/subindex",
                                                                       &canopen_data_[node_id].rpdo_data.subindex));

      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/type",
                                                                       &canopen_data_[node_id].rpdo_data.type));

      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "rpdo/data",
                                                                       &canopen_data_[node_id].rpdo_data.data));

      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, "nmt/state",
                                                                       &canopen_data_[node_id].nmt_state.state));
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

      if(info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
      {
          // skip adding canopen interfaces
          continue;
      }

      const uint8_t node_id = static_cast<uint8_t >(std::stoi(info_.joints[i].parameters["node_id"]));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/index",
                                                                       &canopen_data_[node_id].tpdo_data.index));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/subindex",
                                                                       &canopen_data_[node_id].tpdo_data.subindex));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/type",
                                                                       &canopen_data_[node_id].tpdo_data.type));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/data",
                                                                       &canopen_data_[node_id].tpdo_data.data));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "tpdo/ons",
                                                                           &canopen_data_[node_id].tpdo_data.one_shot));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "nmt/reset",
                                                                         &canopen_data_[node_id].nmt_state.reset_ons));

      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, "nmt/start",
                                                                         &canopen_data_[node_id].nmt_state.start_ons));
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

  // nmt state is set via Ros2ControlNmtState::set_state within nmt_state_cb

  // rpdo is set via RORos2ControlCOData::set_data within rpdo_cb


    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanopenSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'
  auto drivers = device_container_->get_registered_drivers();
  for(auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it){
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

      // reset node nmt
      if(it->second.nmt_state.reset_command()){
          proxy_driver->reset_node_nmt_command();
      }

      // start nmt
      if(it->second.nmt_state.start_command()){
        proxy_driver->start_node_nmt_command();
      }

      // tpdo data one shot mechanism
      if(it->second.tpdo_data.write_command()){
          it->second.tpdo_data.prepare_data();
          proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace canopen_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_control::CanopenSystem, hardware_interface::SystemInterface)
