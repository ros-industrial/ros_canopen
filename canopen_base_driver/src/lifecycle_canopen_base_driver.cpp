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
#include <memory>
#include "canopen_base_driver/lifecycle_canopen_base_driver.hpp"

using namespace lely;
using namespace ros2_canopen;
using namespace std::chrono_literals;

void LifecycleBaseDriver::nmt_listener()
{
  while (rclcpp::ok())
  {
    std::future<lely::canopen::NmtState> f;
    {
      std::scoped_lock<std::mutex> lock (this->driver_mutex_);
      f = driver_->async_request_nmt();
    }
    while (f.wait_for(non_transmit_timeout_) != std::future_status::ready)
    {
      if (!this->activated_.load())
        return;
    }
    on_nmt(f.get());
  }
}

void LifecycleBaseDriver::rdpo_listener()
{
  while (rclcpp::ok())
  {
    std::future<ros2_canopen::COData> f;
    {
      std::scoped_lock<std::mutex> lock (this->driver_mutex_);
      f = driver_->async_request_rpdo();
    }

    while (f.wait_for(non_transmit_timeout_) != std::future_status::ready)
    {
      if (!this->activated_.load())
        return;
    }

    on_rpdo(f.get());
  }
}


bool LifecycleBaseDriver::add()
{
  std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::LelyBridge>>> prom;
  prom = std::make_shared<std::promise<std::shared_ptr<ros2_canopen::LelyBridge>>>();
  std::future<std::shared_ptr<ros2_canopen::LelyBridge>> f = prom->get_future();
  master_->GetExecutor().post([this, prom]()
                              {
                std::scoped_lock<std::mutex> lock (this->driver_mutex_);
                driver_ =
                  std::make_shared<ros2_canopen::LelyBridge>(*exec_, *master_, node_id_);
                driver_->Boot();
                prom->set_value(driver_); });
  auto future_status = f.wait_for(this->non_transmit_timeout_);
  if (future_status != std::future_status::ready)
  {
    return false;
  }
  driver_ = f.get();
  return true;
}

bool LifecycleBaseDriver::remove()
{
  std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
  auto f = prom->get_future();
  exec_->post([this, prom]()
              { 
                                driver_.reset(); 
                                prom->set_value(); });
  auto future_status = f.wait_for(this->non_transmit_timeout_);

  if (future_status != std::future_status::ready)
  {
    return false;
  }
  return true;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleBaseDriver::on_configure(const rclcpp_lifecycle::State &state)
{
  return LifecycleDriverInterface::on_configure(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleBaseDriver::on_activate(const rclcpp_lifecycle::State &state)
{
  return LifecycleDriverInterface::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleBaseDriver::on_deactivate(const rclcpp_lifecycle::State &state)
{
  return LifecycleDriverInterface::on_deactivate(state);
}
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleBaseDriver::on_cleanup(const rclcpp_lifecycle::State &state)
{
  return LifecycleDriverInterface::on_cleanup(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
LifecycleBaseDriver::on_shutdown(const rclcpp_lifecycle::State &state)
{
  return LifecycleDriverInterface::on_shutdown(state);
}