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

#include <lely/coapp/fiber_driver.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/slave.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace lely;

namespace ros2_canopen {
    // Base class for driver plugin
    // Pluginlib API does allows only default constructors
    class CANopenDriverWrapper : public rclcpp::Node{
    public:
        CANopenDriverWrapper(const std::string& node_name,
            const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) :
            rclcpp::Node(node_name, node_options) {}

        virtual void init(ev::Executor& exec,
            canopen::AsyncMaster& master,
            uint8_t node_id) noexcept = 0;
    };


    class MasterDevice : public rclcpp::Node, public canopen::AsyncMaster {
    public:
        MasterDevice(io::Timer& timer,
            io::CanChannel& chan,
            std::string& dcf_txt,
            u_int32_t id,
            const std::string& devName)
                : rclcpp::Node(devName),
                canopen::AsyncMaster(timer, chan, dcf_txt, "", id) {}

        // ROS interfaces
    };


    class SlaveDevice : public rclcpp::Node, public canopen::BasicSlave {
    public:
        SlaveDevice(io::Timer& timer,
            io::CanChannel& chan,
            std::string& dcf_txt,
            u_int32_t id,
            const std::string& devName)
                : rclcpp::Node(devName),
                canopen::BasicSlave(timer, chan, dcf_txt, "", id) {}

        // ROS interfaces
    };
}   // end ros2_canopen namespace
