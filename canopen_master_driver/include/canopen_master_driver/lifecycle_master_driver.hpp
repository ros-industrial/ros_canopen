//    Copyright 2022 Harshavadan Deshpande
//                   Christoph Hellmann Santos
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
#ifndef LIFECYCLE_MASTER_DRIVER_HPP
#define LIFECYCLE_MASTER_DRIVER_HPP

#include <atomic>
#include <memory>
#include <thread>

#include "canopen_core/master_node.hpp"
#include "canopen_master_driver/node_interfaces/node_canopen_basic_master.hpp"

namespace ros2_canopen
{
/**
 * @brief Lifecycle Master Node
 *
 * This class implements the Lifecycle master interface.
 * It uses the Lely Master Bridge and exposes a ROS node
 * interface.
 *
 */
class LifecycleMasterDriver : public ros2_canopen::LifecycleCanopenMaster
{
  std::shared_ptr<node_interfaces::NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>>
    node_canopen_basic_master_;

public:
  explicit LifecycleMasterDriver(const rclcpp::NodeOptions & node_options);
};

}  // namespace ros2_canopen

#endif
