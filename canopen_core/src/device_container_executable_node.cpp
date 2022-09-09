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

#include "canopen_core/device_container_node.hpp"

using namespace ros2_canopen;

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_container = std::make_shared<DeviceContainerNode>(exec);
    std::thread spinThread([&device_container]()
                        { 
                            if(device_container->init())
                            {
                                RCLCPP_INFO(device_container->get_logger(), "Initialisation successful.");
                            }
                            else
                            {
                                RCLCPP_INFO(device_container->get_logger(), "Initialisation failed.");
                            }
                        });
    exec->add_node(device_container);
    exec->spin();
    spinThread.join();
    return 0;
}
