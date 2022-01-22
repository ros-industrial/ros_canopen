#ifndef ROS2_CANOPEN_NODE_HPP
#define ROS2_CANOPEN_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <utility>
#include <fstream>
#include <map>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>

#include <yaml-cpp/yaml.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_canopen_interfaces/srv/master_nmt.hpp"
#include "ros2_canopen_interfaces/srv/master_read_sdo8.hpp"
#include "ros2_canopen_interfaces/srv/master_read_sdo16.hpp"
#include "ros2_canopen_interfaces/srv/master_read_sdo32.hpp"
#include "ros2_canopen_interfaces/srv/master_write_sdo8.hpp"
#include "ros2_canopen_interfaces/srv/master_write_sdo16.hpp"
#include "ros2_canopen_interfaces/srv/master_write_sdo32.hpp"
#include "ros2_canopen/basicdevice.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;

namespace ros2_canopen
{
    class ROSCANopen_Node : public rclcpp_lifecycle::LifecycleNode
    {
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        std::shared_ptr<io::IoGuard> io_guard;
        std::shared_ptr<io::Context> ctx;
        std::shared_ptr<io::Poll> poll;
        std::shared_ptr<ev::Loop> loop;
        std::shared_ptr<ev::Executor> exec;
        std::shared_ptr<io::CanController> ctrl;
        std::shared_ptr<io::CanChannel> chan;
        std::shared_ptr<io::Timer> can_timer;
        std::shared_ptr<canopen::AsyncMaster> can_master;
        std::shared_ptr<ros2_canopen::BasicDevice> basicdevice;
        std::shared_ptr<std::string> nmt_status;

        std::string can_interface_name;
        std::string dcf_path;
        std::string yaml_path;
        std::map<int, std::string> drivers;

        std::unique_ptr<std::thread> canopen_loop_thread;

        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterNmt>> master_nmt_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterReadSdo8>> master_read_sdo8_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterReadSdo16>> master_read_sdo16_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterReadSdo32>> master_read_sdo32_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterWriteSdo8>> master_write_sdo8_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterWriteSdo16>> master_write_sdo16_service;
        std::shared_ptr<rclcpp::Service<ros2_canopen_interfaces::srv::MasterWriteSdo32>> master_write_sdo32_service;
        
        std::atomic<bool> active;
        std::mutex master_mutex;

        //Service Callback Declarations
        void master_nmt(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterNmt::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterNmt::Response> response);

        void master_read_sdo8(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo8::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo8::Response> response);

        void master_read_sdo16(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo16::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo16::Response> response);

        void master_read_sdo32(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo32::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo32::Response> response);

        void master_write_sdo8(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo8::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo8::Response> response);

        void master_write_sdo16(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo16::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo16::Response> response);

        void master_write_sdo32(
            const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo32::Request> request,
            std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo32::Response> response);

        //helper functions
        void run();
        void read_yaml();

        template <typename T>
        bool write_sdo(uint8_t nodeid, uint16_t index, uint8_t subindex, T data)
        {
            if (active.load())
            {
                ev_exec_t *exe = *exec;
                lely::canopen::SdoFuture<void> f;
                {
                    std::lock_guard<std::mutex> guard(master_mutex);
                    f = can_master->AsyncWrite<T>(exe, nodeid, index, subindex, std::move(data), 100ms);
                }
                while (!f.is_ready())
                {
                    std::this_thread::sleep_for(10ms);
                }
                auto res = f.get();
                if (res.has_error())
                {
                    return false;
                }
                else
                {
                    return true;
                }
            }
            else
            {
                return false;
            }
        }

        template <typename T>
        bool read_sdo(uint8_t nodeid, uint16_t index, uint8_t subindex, std::shared_ptr<T> data)
        {
            if (active.load())
            {
                ev_exec_t *exe = *exec;
                lely::canopen::SdoFuture<T> f;
                {
                    std::lock_guard<std::mutex> guard(master_mutex);
                    f = can_master->AsyncRead<T>(exe, nodeid, index, subindex, 100ms);
                }
                while (!f.is_ready())
                {
                    std::this_thread::sleep_for(10ms);
                }
                auto res = f.get();
                if (res.has_error())
                {
                    auto error = res.error();
                    return false;
                }
                else
                {
                    *data = res.value();
                    return true;
                }
            }
            else
            {
                return false;
            }
        }

        //Lifecycle Callback Functions
        CallbackReturn on_configure(const rclcpp_lifecycle::State &state);
        CallbackReturn on_activate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);

    public:
        //Constructor Declarations
        ROSCANopen_Node(const std::string &node_name, bool intra_process_comms = false) : rclcpp_lifecycle::LifecycleNode(
                                                                                              node_name,
                                                                                              rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
        {
            this->declare_parameter<std::string>("can_interface_name", "vcan0");
            this->declare_parameter<std::string>("dcf_path", "");
            this->declare_parameter<std::string>("yaml_path", "");
            can_interface_name = "";
            dcf_path = "";
            yaml_path = "";
        }
    };
}

#endif