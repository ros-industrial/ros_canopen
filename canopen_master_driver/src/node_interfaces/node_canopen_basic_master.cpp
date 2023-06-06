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

#include "canopen_master_driver/node_interfaces/node_canopen_basic_master.hpp"
#include "canopen_master_driver/node_interfaces/node_canopen_basic_master_impl.hpp"

using namespace ros2_canopen::node_interfaces;

template class ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp::Node>;
template class ros2_canopen::node_interfaces::NodeCanopenBasicMaster<
  rclcpp_lifecycle::LifecycleNode>;
