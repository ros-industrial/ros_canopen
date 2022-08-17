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
#include "canopen_proxy_driver/lifecycle_canopen_proxy_driver.hpp"

namespace ros2_canopen
{
  void LifecycleProxyDriver::on_nmt(canopen::NmtState nmt_state)
  {
    if (this->activated_.load())
    {
      auto message = std_msgs::msg::String();

      switch (nmt_state)
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
        RCLCPP_ERROR(this->get_logger(), "Unknown NMT State.");
        message.data = "ERROR";
        break;
      }
      RCLCPP_INFO(
          this->get_logger(),
          "Slave %hhu: Switched NMT state to %s",
          this->driver_->get_id(),
          message.data.c_str());

      nmt_state_publisher->publish(message);
    }
  }

  void LifecycleProxyDriver::on_tpdo(const canopen_interfaces::msg::COData::SharedPtr msg)
  {
    if (this->activated_.load())
    {
      COData data = {msg->index, msg->subindex, msg->data, static_cast<CODataTypes>(msg->type)};
      driver_->tpdo_transmit(data);
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Could transmit PDO because driver not activated.");
  }

  void LifecycleProxyDriver::on_rpdo(COData d)
  {
    if (this->activated_.load())
    {
      RCLCPP_INFO(
          this->get_logger(),
          "Slave %hhu: Sent PDO index %hu, subindex %hhu, data %x",
          this->driver_->get_id(),
          d.index_,
          d.subindex_,
          d.data_);
      auto message = canopen_interfaces::msg::COData();
      message.index = d.index_;
      message.subindex = d.subindex_;
      message.data = d.data_;
      message.type = static_cast<uint8_t>(d.type_);
      rpdo_publisher->publish(message);
    }
  }

  void LifecycleProxyDriver::on_nmt_state_reset(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (this->activated_.load())
    {
      driver_->nmt_command(canopen::NmtCommand::RESET_NODE);
      response->success = true;
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Could not reset device via NMT because driver not activated.");
    response->success = false;
  }

  void LifecycleProxyDriver::on_nmt_state_start(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    if (this->activated_.load())
    {
      driver_->nmt_command(canopen::NmtCommand::START);
      response->success = true;
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Could not start device via NMT because driver not activated.");
    response->success = false;
  }

  void LifecycleProxyDriver::on_sdo_read(
      const canopen_interfaces::srv::CORead::Request::SharedPtr request,
      canopen_interfaces::srv::CORead::Response::SharedPtr response)
  {
    if (this->activated_.load())
    {
      RCLCPP_INFO(
          this->get_logger(), "Slave %hhu: SDO Read Call index=0x%x subindex=%hhu bits=%hhu",
          this->driver_->get_id(), request->index, request->subindex, request->type);

      // Only allow one SDO request concurrently
      std::scoped_lock<std::mutex> lk(sdo_mtex);
      // Prepare Data read
      COData data = {request->index, request->subindex, 0U, static_cast<CODataTypes>(request->type)};
      // Send read request
      auto f = driver_->async_sdo_read(data);
      // Wait for response
      f.wait();
      // Process response
      try
      {
        response->data = f.get().data_;
        response->success = true;
      }
      catch (std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
        response->success = false;
      }
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Could not read from SDO because driver not activated.");
    response->success = false;
  }

  void LifecycleProxyDriver::on_sdo_write(
      const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
      canopen_interfaces::srv::COWrite::Response::SharedPtr response)
  {
    if (this->activated_.load())
    {
      RCLCPP_INFO(
          this->get_logger(), "Slave %hhu: SDO Write Call index=0x%x subindex=%hhu bits=%hhu data=%u",
          this->driver_->get_id(), request->index, request->subindex, request->type, request->data);

      // Only allow one SDO request concurrently
      std::scoped_lock<std::mutex> lk(sdo_mtex);

      // Prepare Data
      COData d =
          {request->index, request->subindex, request->data, static_cast<CODataTypes>(request->type)};
      // Send write request
      auto f = driver_->async_sdo_write(d);
      // Wait for request to complete
      f.wait();

      // Process response
      try
      {
        response->success = f.get();
      }
      catch (std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), e.what());
        response->success = false;
      }
      return;
    }
    RCLCPP_ERROR(this->get_logger(), "Could not write to SDO because driver not activated.");
    response->success = false;
  }

  void LifecycleProxyDriver::register_ros_interface()
  {
    ros2_canopen::LifecycleDriverInterface::register_ros_interface();
    nmt_state_publisher = this->create_publisher<std_msgs::msg::String>(
        std::string(
            this->get_name())
            .append("/nmt_state")
            .c_str(),
        10);
    tpdo_subscriber = this->create_subscription<canopen_interfaces::msg::COData>(
        std::string(this->get_name()).append("/tpdo").c_str(),
        10,
        std::bind(&LifecycleProxyDriver::on_tpdo, this, std::placeholders::_1));

    rpdo_publisher = this->create_publisher<canopen_interfaces::msg::COData>(
        std::string(this->get_name()).append("/rpdo").c_str(), 10);

    nmt_state_reset_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/nmt_reset_node").c_str(),
        std::bind(
            &ros2_canopen::LifecycleProxyDriver::on_nmt_state_reset,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    nmt_state_start_service = this->create_service<std_srvs::srv::Trigger>(
        std::string(this->get_name()).append("/nmt_start_node").c_str(),
        std::bind(
            &ros2_canopen::LifecycleProxyDriver::on_nmt_state_start,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    sdo_read_service = this->create_service<canopen_interfaces::srv::CORead>(
        std::string(this->get_name()).append("/sdo_read").c_str(),
        std::bind(
            &ros2_canopen::LifecycleProxyDriver::on_sdo_read,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    sdo_write_service = this->create_service<canopen_interfaces::srv::COWrite>(
        std::string(this->get_name()).append("/sdo_write").c_str(),
        std::bind(
            &ros2_canopen::LifecycleProxyDriver::on_sdo_write,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleProxyDriver::on_configure(const rclcpp_lifecycle::State &state)
  {
    return LifecycleBaseDriver::on_configure(state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleProxyDriver::on_activate(const rclcpp_lifecycle::State &state)
  {
    return LifecycleBaseDriver::on_activate(state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleProxyDriver::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    return LifecycleBaseDriver::on_deactivate(state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleProxyDriver::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    return LifecycleBaseDriver::on_cleanup(state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  LifecycleProxyDriver::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    return LifecycleBaseDriver::on_shutdown(state);
  }

} // namespace ros2_canopen

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_canopen::LifecycleProxyDriver)
