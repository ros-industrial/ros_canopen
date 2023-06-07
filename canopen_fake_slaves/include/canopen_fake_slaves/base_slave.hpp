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

#ifndef SLAVE_HPP
#define SLAVE_HPP
#include <atomic>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_canopen
{
class BaseSlave : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit BaseSlave(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(
      node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    this->declare_parameter("node_id", 2);
    this->declare_parameter("slave_config", "slave.eds");
    this->declare_parameter("can_interface_name", "vcan0");
    this->activated.store(false);
  }

  virtual ~BaseSlave()
  {
    if (this->run_thread.joinable())
    {
      run_thread.join();
    }
  }

  virtual void run() = 0;

protected:
  std::thread run_thread;
  int node_id_;
  std::string slave_config_;
  std::string can_interface_name_;
  std::atomic<bool> activated;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &)
  {
    this->activated.store(false);
    RCLCPP_INFO(this->get_logger(), "Reaching inactive state.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &)
  {
    this->activated.store(true);
    get_parameter("node_id", node_id_);
    get_parameter("slave_config", slave_config_);
    get_parameter("can_interface_name", can_interface_name_);
    run_thread = std::thread(std::bind(&BaseSlave::run, this));
    RCLCPP_INFO(this->get_logger(), "Reaching active state.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &)
  {
    this->activated.store(false);
    RCLCPP_INFO(this->get_logger(), "Reaching inactive state.");
    if (run_thread.joinable())
    {
      run_thread.join();
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &)
  {
    this->activated.store(false);
    RCLCPP_INFO(this->get_logger(), "Reaching unconfigured state.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }
};
}  // namespace ros2_canopen

#endif
