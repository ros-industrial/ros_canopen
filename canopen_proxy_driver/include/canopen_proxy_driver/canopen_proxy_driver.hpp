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

#ifndef CANOPEN_PROXY_DRIVER__CANOPEN_PROXY_DRIVER_HPP_
#define CANOPEN_PROXY_DRIVER__CANOPEN_PROXY_DRIVER_HPP_
#include <string>

#include "canopen_proxy_driver/visibility_control.h"
#include "canopen_base_driver/canopen_base_driver.hpp"

namespace ros2_canopen
{
class ProxyDriver : public BaseDriver
{
  //publishers, subscriptions and services
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nmt_state_publisher;
  rclcpp::Publisher<canopen_interfaces::msg::COData>::SharedPtr rpdo_publisher;
  rclcpp::Subscription<canopen_interfaces::msg::COData>::SharedPtr tpdo_subscriber;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_reset_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr nmt_state_start_service;
  rclcpp::Service<canopen_interfaces::srv::CORead>::SharedPtr sdo_read_service;
  rclcpp::Service<canopen_interfaces::srv::COWrite>::SharedPtr sdo_write_service;

  std::mutex sdo_mtex;

  // expose for ros2 control purposes
public:
    /**
     * @brief Send Reset Node NMT command
     * 
     * @return true 
     * @return false 
     */
    bool reset_node_nmt_command(){
        driver->nmt_command(canopen::NmtCommand::RESET_NODE);
        return true;
    }

    /**
     * @brief Send Start Node NMT command
     * 
     * @return true 
     * @return false 
     */
    bool start_nmt_command(){
        driver->nmt_command(canopen::NmtCommand::START);
        return true;
    }

  /**
   * @brief Transmit Object via TPDO
   * 
   * @param data 
   */
    void tpdo_transmit(COData &data){
            driver->tpdo_transmit(data);
    }

    /**
     * @brief Register a callback for NMT state change
     * 
     * @param nmt_state_cb 
     */
    void register_nmt_state_cb( std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb){
        nmt_state_cb_ = std::move(nmt_state_cb);
    }

    /**
     * @brief Register a callback for RPDO
     * 
     * @param rpdo_cb 
     */
    void register_rpdo_cb( std::function<void(COData, uint8_t)> rpdo_cb){
        rpdo_cb_ = std::move(rpdo_cb);
    }

    // nmt state callback
    std::function<void(canopen::NmtState, uint8_t)> nmt_state_cb_;
    // rpdo callback
    std::function<void(COData, uint8_t)> rpdo_cb_;

protected:

  /**
   * @brief Callback function that is called on NMT state change
   * 
   * @param [in] nmt_state    New state
   */
  void on_nmt(canopen::NmtState nmt_state);

  /**
   * @brief Callback function that is called on TPDO subscription event
   * 
   * Transforms the message received into an COData object and calls
   * the tpdo transmission.
   * 
   * @param [in] msg      Message received
   */
  void on_tpdo(const canopen_interfaces::msg::COData::SharedPtr msg);

  /**
   * @brief Callback function that is called on reception of an Object via RPDO
   * 
   * Publishes the received data via the RPDO topic of the driver and
   * calls the rpdo callback if it was registered earlier.
   * 
   * @param [in] d      Object data received
   */
  void on_rpdo(COData d);


  /**
   * @brief Callback function for nmt_state_reset service
   * 
   * @param request 
   * @param response 
   */
  void on_nmt_state_reset(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Callback for nmt_state_start service
   * 
   * @param request 
   * @param response 
   */
  void on_nmt_state_start(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Callback for sdo_read service
   * 
   * @param request 
   * @param response 
   */
  void on_sdo_read(
    const canopen_interfaces::srv::CORead::Request::SharedPtr request,
    canopen_interfaces::srv::CORead::Response::SharedPtr response);

  /**
   * @brief Callback for sdo_write service
   * 
   * @param request 
   * @param response 
   */
  void on_sdo_write(
    const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
    canopen_interfaces::srv::COWrite::Response::SharedPtr response);

public:
  /**
   * @brief Construct a new Proxy Driver object
   * 
   * @param options 
   */
  explicit ProxyDriver(const rclcpp::NodeOptions & options)
  : BaseDriver(options), nmt_state_cb_(nullptr), rpdo_cb_(nullptr) {}

  /**
   * @brief Initialize driver
   * 
   * @param [in] exec       Executor
   * @param [in] master     Master
   * @param [in] node_id    Target device's CANopen node id 
   * @param [in] config     Pointer to Configuration Manager
   */
  void init(
    ev::Executor & exec,
    canopen::AsyncMaster & master,
    uint8_t node_id,
    std::shared_ptr<ros2_canopen::ConfigurationManager> config) noexcept override
  {
    BaseDriver::init(exec, master, node_id, config);
    nmt_state_publisher = this->create_publisher<std_msgs::msg::String>(
      std::string(
        this->get_name()).append("/nmt_state").c_str(), 10);
    tpdo_subscriber = this->create_subscription<canopen_interfaces::msg::COData>(
      std::string(this->get_name()).append("/tpdo").c_str(),
      10,
      std::bind(&ProxyDriver::on_tpdo, this, std::placeholders::_1));

    rpdo_publisher = this->create_publisher<canopen_interfaces::msg::COData>(
      std::string(this->get_name()).append("/rpdo").c_str(), 10);

    nmt_state_reset_service = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()).append("/nmt_reset_node").c_str(),
      std::bind(
        &ros2_canopen::ProxyDriver::on_nmt_state_reset,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    nmt_state_start_service = this->create_service<std_srvs::srv::Trigger>(
      std::string(this->get_name()).append("/nmt_start_node").c_str(),
      std::bind(
        &ros2_canopen::ProxyDriver::on_nmt_state_start,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    sdo_read_service = this->create_service<canopen_interfaces::srv::CORead>(
      std::string(this->get_name()).append("/sdo_read").c_str(),
      std::bind(
        &ros2_canopen::ProxyDriver::on_sdo_read,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    sdo_write_service = this->create_service<canopen_interfaces::srv::COWrite>(
      std::string(this->get_name()).append("/sdo_write").c_str(),
      std::bind(
        &ros2_canopen::ProxyDriver::on_sdo_write,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }
};
}  // namespace ros2_canopen

#endif  // CANOPEN_PROXY_DRIVER__CANOPEN_PROXY_DRIVER_HPP_