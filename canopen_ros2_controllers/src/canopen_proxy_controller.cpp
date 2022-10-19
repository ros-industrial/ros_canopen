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

#include "canopen_ros2_controllers/canopen_proxy_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"

static constexpr int kLoopPeriodMS = 100;
static constexpr double kCommandValue = 1.0;

namespace
{  // utility


using ControllerCommandMsg = canopen_ros2_controllers::CanopenProxyController::ControllerCommandMsg;

// called from RT control loop
void reset_controller_command_msg(
  std::shared_ptr<ControllerCommandMsg> & msg, const std::string & joint_name)
{
  msg->index = 0u;
  msg->subindex = 0u;
  msg->type = 0u;
  msg->data = 0u;
}
bool propagate_controller_command_msg(
  std::shared_ptr<ControllerCommandMsg> & msg)
{
  // TODO (livanov93): add better logic to decide if a message
  //  should be propagated to the bus
  // check if it is 8 = uint8_t or 16 = uint16_t or 32 = uint32_t
  return msg->type == 8 || msg->type == 16 || msg->type == 32;
}
}  // namespace

namespace canopen_ros2_controllers
{
CanopenProxyController::CanopenProxyController()
: controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn CanopenProxyController::on_init()
{

  try {
    // use auto declare
    auto_declare<std::string>("joint", joint_name_);
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CanopenProxyController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto error_if_empty = [&](const auto & parameter, const char * parameter_name) {
      if (parameter.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'%s' parameter was empty", parameter_name);
        return true;
      }
      return false;
    };

  auto get_string_array_param_and_error_if_empty =
    [&](std::vector<std::string> & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string_array();
      return error_if_empty(parameter, parameter_name);
    };

  auto get_string_param_and_error_if_empty =
    [&](std::string & parameter, const char * parameter_name) {
      parameter = get_node()->get_parameter(parameter_name).as_string();
      return error_if_empty(parameter, parameter_name);
    };

  if (
    get_string_param_and_error_if_empty(joint_name_, "joint"))
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  // Command Subscriber and callbacks
  auto callback_cmd = [&](const std::shared_ptr<ControllerCommandMsg> msg) -> void {
      input_cmd_.writeFromNonRT(msg);

    };
  tpdo_subscriber_ = get_node()->create_subscription<ControllerCommandMsg>(
    "~/tpdo", rclcpp::SystemDefaultsQoS(), callback_cmd);

  std::shared_ptr<ControllerCommandMsg> msg = std::make_shared<ControllerCommandMsg>();
  reset_controller_command_msg(msg, joint_name_);

  input_cmd_.writeFromNonRT(msg);

  try {
    // nmt state publisher
    nmt_state_pub_ =
      get_node()->create_publisher<ControllerNMTStateMsg>(
      "~/nmt_state",
      rclcpp::SystemDefaultsQoS());
    nmt_state_rt_publisher_ = std::make_unique<ControllerNmtStateRTPublisher>(nmt_state_pub_);

    // rpdo publisher
    rpdo_pub_ =
      get_node()->create_publisher<ControllerCommandMsg>("~/rpdo", rclcpp::SystemDefaultsQoS());
    rpdo_rt_publisher_ = std::make_unique<ControllerRPDOPRTublisher>(rpdo_pub_);

  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  nmt_state_rt_publisher_->lock();
  nmt_state_rt_publisher_->msg_.data = std::string();
  nmt_state_rt_publisher_->unlock();

  rpdo_rt_publisher_->lock();
  rpdo_rt_publisher_->msg_.index = 0u;
  rpdo_rt_publisher_->msg_.subindex = 0u;
  rpdo_rt_publisher_->msg_.type = 0u;
  rpdo_rt_publisher_->msg_.data = 0u;
  rpdo_rt_publisher_->unlock();

  // init services

  // NMT reset
  auto on_nmt_state_reset = [&](const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {

      command_interfaces_[CommandInterfaces::NMT_RESET].set_value(kCommandValue);

      while (!std::isnan(command_interfaces_[CommandInterfaces::NMT_RESET].get_value()))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMS));
      }

      // report success
      response->success = static_cast<bool>(command_interfaces_[CommandInterfaces::NMT_RESET_FBK].get_value());
      // reset to nan
      command_interfaces_[CommandInterfaces::NMT_RESET_FBK].set_value(std::numeric_limits<double>::quiet_NaN());
    };

  auto service_profile = rmw_qos_profile_services_default;
  service_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  service_profile.depth = 1;
  nmt_state_reset_service_ = get_node()->create_service<ControllerStartResetSrvType>(
    "~/nmt_reset_node", on_nmt_state_reset,
    service_profile);

  // NMT start
  auto on_nmt_state_start = [&](const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response) {

      command_interfaces_[CommandInterfaces::NMT_START].set_value(kCommandValue);

      while (!std::isnan(command_interfaces_[CommandInterfaces::NMT_START].get_value()))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(kLoopPeriodMS));
      }

      // report success
      response->success = static_cast<bool>(command_interfaces_[CommandInterfaces::NMT_START_FBK].get_value());
      // reset to nan
      command_interfaces_[CommandInterfaces::NMT_START_FBK].set_value(std::numeric_limits<double>::quiet_NaN());
    };
  nmt_state_start_service_ = get_node()->create_service<ControllerStartResetSrvType>(
    "~/nmt_start_node", on_nmt_state_start,
    service_profile);

  // SDO read
  auto on_sdo_read = [&](
    const canopen_interfaces::srv::CORead::Request::SharedPtr request,
    canopen_interfaces::srv::CORead::Response::SharedPtr response) {};

  sdo_read_service_ = get_node()->create_service<ControllerSDOReadSrvType>(
    "~/sdo_read", on_sdo_read,
    service_profile);

  // SDO write
  auto on_sdo_write = [&](
    const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
    canopen_interfaces::srv::COWrite::Response::SharedPtr response) {

    };

  sdo_write_service_ = get_node()->create_service<ControllerSDOWriteSrvType>(
    "~/sdo_write", on_sdo_write,
    service_profile);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CanopenProxyController::command_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(9);
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/index");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/subindex");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/type");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/data");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/ons");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/reset");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/reset_fbk");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/start");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/start_fbk");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CanopenProxyController::state_interface_configuration()
const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.reserve(5);
  state_interfaces_config.names.push_back(joint_name_ + "/" + "rpdo/index");
  state_interfaces_config.names.push_back(joint_name_ + "/" + "rpdo/subindex");
  state_interfaces_config.names.push_back(joint_name_ + "/" + "rpdo/type");
  state_interfaces_config.names.push_back(joint_name_ + "/" + "rpdo/data");
  state_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/state");

  return state_interfaces_config;
}

controller_interface::CallbackReturn CanopenProxyController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set default value in command
  reset_controller_command_msg(*(input_cmd_.readFromRT)(), joint_name_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CanopenProxyController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  // instead of a loop
  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
    command_interfaces_[i].set_value(std::numeric_limits<double>::quiet_NaN());
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type CanopenProxyController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // nmt state is retrieved in SystemInterface via cb and is exposed here
  if (nmt_state_rt_publisher_) {

      auto message = std_msgs::msg::String();
      auto nmt_state =  static_cast<int>(state_interfaces_[StateInterfaces::NMT_STATE].get_value());

      switch (static_cast<canopen::NmtState>(nmt_state))
      {
          case canopen::NmtState::BOOTUP:
              message.data = "BOOTUP";
              break;
          case canopen::NmtState::PREOP:
              message.data = "PREOP";
              break;
          case canopen::NmtState::RESET_COMM:
              message.data = "RESET_COMM";
              break;
          case canopen::NmtState::RESET_NODE:
              message.data = "RESET_NODE";
              break;
          case canopen::NmtState::START:
              message.data = "START";
              break;
          case canopen::NmtState::STOP:
              message.data = "STOP";
              break;
          case canopen::NmtState::TOGGLE:
              message.data = "TOGGLE";
              break;
          default:
              RCLCPP_ERROR(get_node()->get_logger(), "Unknown NMT State.");
              message.data = "ERROR";
              break;
      }

      if (nmt_state_actual_ != message.data && nmt_state_rt_publisher_->trylock()){
          // publish on change only
          nmt_state_actual_ = std::string(message.data);
          nmt_state_rt_publisher_->msg_.data = nmt_state_actual_;
          nmt_state_rt_publisher_->unlockAndPublish();
      }
  }

  // exposing rpdo data via real-time publisher
  if (rpdo_rt_publisher_ && rpdo_rt_publisher_->trylock()) {
    rpdo_rt_publisher_->msg_.index =
      static_cast<uint16_t>(state_interfaces_[StateInterfaces::RPDO_INDEX].get_value());
    rpdo_rt_publisher_->msg_.subindex =
      static_cast<uint8_t>(state_interfaces_[StateInterfaces::RPDO_SUBINDEX].get_value());
    rpdo_rt_publisher_->msg_.type =
      static_cast<uint8_t>(state_interfaces_[StateInterfaces::RPDO_TYPE].get_value());
    rpdo_rt_publisher_->msg_.data =
      static_cast<uint32_t>(state_interfaces_[StateInterfaces::RPDO_DATA].get_value());

    rpdo_rt_publisher_->unlockAndPublish();
  }

  // tpdo data is the main controller data retrieved via subscription
  auto current_cmd = input_cmd_.readFromRT();
  if (!current_cmd || !(*current_cmd)) {
    return controller_interface::return_type::OK;
  } else if (propagate_controller_command_msg(*current_cmd)) {
    command_interfaces_[CommandInterfaces::TPDO_INDEX].set_value((*current_cmd)->index);
    command_interfaces_[CommandInterfaces::TPDO_SUBINDEX].set_value((*current_cmd)->subindex);
    command_interfaces_[CommandInterfaces::TPDO_TYPE].set_value((*current_cmd)->type);
    command_interfaces_[CommandInterfaces::TPDO_DATA].set_value((*current_cmd)->data);
    // tpdo data one shot mechanism
    command_interfaces_[CommandInterfaces::TPDO_ONS].set_value(kCommandValue);
  }

  return controller_interface::return_type::OK;
}

}  // namespace canopen_ros2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_controllers::CanopenProxyController, controller_interface::ControllerInterface)
