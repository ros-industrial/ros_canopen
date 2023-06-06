//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
using namespace std::chrono_literals;

template <>
void ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node>::demand_set_master()
{
  RCLCPP_DEBUG(node_->get_logger(), "demand_set_master_start");
  if (!configured_.load())
  {
    throw ros2_canopen::DriverException("Set Master: driver is not configured");
  }
  std::string init_service_name = container_name_ + "/init_driver";
  RCLCPP_DEBUG(node_->get_logger(), "Service: %s", init_service_name.c_str());
  rclcpp::Client<canopen_interfaces::srv::CONode>::SharedPtr demand_set_master_client;
  // demand_set_master_client;
  demand_set_master_client = node_->create_client<canopen_interfaces::srv::CONode>(
    init_service_name, rmw_qos_profile_services_default, client_cbg_);

  while (!demand_set_master_client->wait_for_service(non_transmit_timeout_))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Interrupted while waiting for init_driver service. Exiting.");
    }
    RCLCPP_INFO(node_->get_logger(), "init_driver service not available, waiting again...");
  }
  auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
  request->nodeid = node_id_;

  auto future_result = demand_set_master_client->async_send_request(request);

  auto future_status = future_result.wait_for(non_transmit_timeout_);
  RCLCPP_DEBUG(node_->get_logger(), "demand_set_master end");

  if (future_status == std::future_status::ready)
  {
    future_result.get();
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not get result.");
    throw DriverException("Could not get result.");
  }
}

template <>
void ros2_canopen::node_interfaces::NodeCanopenDriver<
  rclcpp_lifecycle::LifecycleNode>::demand_set_master()
{
  RCLCPP_DEBUG(node_->get_logger(), "demand_set_master_start");
  if (!configured_.load())
  {
    throw ros2_canopen::DriverException("Set Master: driver is not configured");
  }
  std::string init_service_name = container_name_ + "/init_driver";
  rclcpp::Client<canopen_interfaces::srv::CONode>::SharedPtr demand_set_master_client;
  // demand_set_master_client;
  demand_set_master_client = node_->create_client<canopen_interfaces::srv::CONode>(
    init_service_name, rmw_qos_profile_services_default, client_cbg_);

  while (!demand_set_master_client->wait_for_service(non_transmit_timeout_))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Interrupted while waiting for init_driver service. Exiting.");
    }
    RCLCPP_INFO(node_->get_logger(), "init_driver service not available, waiting again...");
  }
  auto request = std::make_shared<canopen_interfaces::srv::CONode::Request>();
  request->nodeid = node_id_;

  auto future_result = demand_set_master_client->async_send_request(request);

  auto future_status = future_result.wait_for(non_transmit_timeout_);
  RCLCPP_DEBUG(node_->get_logger(), "demand_set_master end");

  if (future_status == std::future_status::ready)
  {
    future_result.get();
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "Could not get result.");
    throw DriverException("Could not get result.");
  }
}

template class ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp_lifecycle::LifecycleNode>;
