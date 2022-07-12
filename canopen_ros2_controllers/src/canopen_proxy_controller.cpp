// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "canopen_ros2_controllers/canopen_proxy_controller.hpp"

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace
{  // utility

// TODO(destogl): remove this when merged upstream
// Changed services history QoS to keep all so we don't lose any client service calls
static constexpr rmw_qos_profile_t rmw_qos_profile_services_hist_keep_all = {
  RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  1,  // message queue depth
  RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false};

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

enum CommandInterfaces{
    TPDO_INDEX,
    TPDO_SUBINDEX,
    TPDO_TYPE,
    TPDO_DATA,
    TPDO_ONS,
    NMT_RESET,
    NMT_START,
};

enum StateInterfaces{
    RPDO_INDEX,
    RPDO_SUBINDEX,
    RPDO_TYPE,
    RPDO_DATA,
    NMT_STATE,
};

}  // namespace

namespace canopen_ros2_controllers
{
CanopenProxyController::CanopenProxyController() : controller_interface::ControllerInterface() {}

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
          get_string_param_and_error_if_empty(joint_name_, "joint")) {
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
      get_node()->create_publisher<ControllerNMTStateMsg>("~/nmt_state", rclcpp::SystemDefaultsQoS());
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
                                std_srvs::srv::Trigger::Response::SharedPtr response){

  };
  nmt_state_reset_service_ = get_node()->create_service<ControllerStartResetSrvType>(
          "~/nmt_reset_node", on_nmt_state_reset,
          rmw_qos_profile_services_hist_keep_all);

  // NMT start
  auto on_nmt_state_start = [&](const std_srvs::srv::Trigger::Request::SharedPtr request,
                                std_srvs::srv::Trigger::Response::SharedPtr response){

  };
  nmt_state_start_service_ = get_node()->create_service<ControllerStartResetSrvType>(
          "~/nmt_start_node", on_nmt_state_start,
          rmw_qos_profile_services_hist_keep_all);

  // SDO read
  auto on_sdo_read = [&](
          const canopen_interfaces::srv::CORead::Request::SharedPtr request,
          canopen_interfaces::srv::CORead::Response::SharedPtr response){};

  sdo_read_service_ = get_node()->create_service<ControllerSDOReadSrvType>(
          "~/sdo_read", on_sdo_read,
          rmw_qos_profile_services_hist_keep_all);

  // SDO write
  auto on_sdo_write = [&](
          const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
          canopen_interfaces::srv::COWrite::Response::SharedPtr response){

  };

  sdo_write_service_ = get_node()->create_service<ControllerSDOWriteSrvType>(
          "~/sdo_write", on_sdo_write,
          rmw_qos_profile_services_hist_keep_all);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CanopenProxyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.reserve(7);
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/index");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/subindex");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/type");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/data");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "tpdo/ons");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/reset");
  command_interfaces_config.names.push_back(joint_name_ + "/" + "nmt/start");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration CanopenProxyController::state_interface_configuration() const
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
  auto current_cmd = input_cmd_.readFromRT();

  // instead of a loop
//  for (size_t i = 0; i < command_interfaces_.size(); ++i) {
//    if (!std::isnan((*current_cmd)->displacements[i])) {
//      if (*(control_mode_.readFromRT()) == control_mode_type::SLOW) {
//        (*current_cmd)->displacements[i] /= 2;
//      }
//      command_interfaces_[i].set_value((*current_cmd)->displacements[i]);
//
//      (*current_cmd)->displacements[i] = std::numeric_limits<double>::quiet_NaN();
//    }
//  }

  if (nmt_state_rt_publisher_ && nmt_state_rt_publisher_->trylock()) {
    nmt_state_rt_publisher_->msg_.data = std::to_string(state_interfaces_[StateInterfaces::NMT_STATE].get_value());

    nmt_state_rt_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace canopen_ros2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  canopen_ros2_controllers::CanopenProxyController, controller_interface::ControllerInterface)
