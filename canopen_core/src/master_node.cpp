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

#include "canopen_core/master_node.hpp"

using namespace ros2_canopen;

void CanopenMaster::init()
{
  node_canopen_master_->init();
  node_canopen_master_->configure();
  node_canopen_master_->activate();
}
void CanopenMaster::shutdown() { node_canopen_master_->shutdown(); }

std::shared_ptr<lely::canopen::AsyncMaster> CanopenMaster::get_master()
{
  return node_canopen_master_->get_master();
}

std::shared_ptr<lely::ev::Executor> CanopenMaster::get_executor()
{
  return node_canopen_master_->get_executor();
}

void LifecycleCanopenMaster::init() { node_canopen_master_->init(); }

void LifecycleCanopenMaster::shutdown() { node_canopen_master_->shutdown(); }

std::shared_ptr<lely::canopen::AsyncMaster> LifecycleCanopenMaster::get_master()
{
  return node_canopen_master_->get_master();
}

std::shared_ptr<lely::ev::Executor> LifecycleCanopenMaster::get_executor()
{
  return node_canopen_master_->get_executor();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_configure(const rclcpp_lifecycle::State & state)
{
  node_canopen_master_->configure();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_activate(const rclcpp_lifecycle::State & state)
{
  node_canopen_master_->activate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_deactivate(const rclcpp_lifecycle::State & state)
{
  node_canopen_master_->deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_cleanup(const rclcpp_lifecycle::State & state)
{
  node_canopen_master_->cleanup();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleCanopenMaster::on_shutdown(const rclcpp_lifecycle::State & state)
{
  node_canopen_master_->shutdown();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
