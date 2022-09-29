#include "canopen_core/device_container.hpp"
#include "gtest/gtest.h"
#include <chrono>
using namespace std::chrono_literals;
using namespace ros2_canopen;

TEST(ComponentLoad, test_device_container_configure)
{
    rclcpp::init(0, nullptr);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_container = std::make_shared<DeviceContainer>(exec);
    exec->add_node(device_container);

    device_container->set_parameter(Parameter("bus_config", "bus.yml"));
    device_container->set_parameter(Parameter("master_config", "master.dcf"));
    device_container->set_parameter(Parameter("can_interface", "can0"));

    exec->spin_some(100ms);

    device_container->configure();

    auto time_now = std::chrono::steady_clock::now();
    auto time_future = time_now + 100ms;
    while(time_future > std::chrono::steady_clock::now())
    {
        exec->spin_some(100ms);
    }
    rclcpp::shutdown();

        
}

TEST(ComponentLoad, test_load_master)
{
    rclcpp::init(0, nullptr);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_container = std::make_shared<DeviceContainer>(exec);
    exec->add_node(device_container);

    device_container->set_parameter(Parameter("bus_config", "bus.yml"));
    device_container->set_parameter(Parameter("master_config", "master.dcf"));
    device_container->set_parameter(Parameter("can_interface", "vcan0"));

    exec->spin_some(100ms);

    device_container->configure();
    device_container->load_master();

    std::thread spinner (
        [exec](){
            exec->spin();
            RCLCPP_INFO(rclcpp::get_logger("test"), "Executor done.");
        }
    );
    std::this_thread::sleep_for(500ms);
    device_container->shutdown();
    rclcpp::shutdown();
    spinner.join();
    RCLCPP_INFO(rclcpp::get_logger("test"), "rclcpp::shutdown");
}



TEST(ComponentLoad, test_load_component_2)
{
    rclcpp::init(0, nullptr);
    auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto device_container = std::make_shared<DeviceContainer>(exec);
    exec->add_node(device_container);

    device_container->set_parameter(Parameter("bus_config", "bus.yml"));
    device_container->set_parameter(Parameter("master_config", "master.dcf"));
    device_container->set_parameter(Parameter("can_interface", "vcan0"));

    

    std::thread spinner (
        [exec](){
            exec->spin();
            RCLCPP_INFO(rclcpp::get_logger("test"), "Executor done.");
        }
    );
    device_container->configure();
    device_container->load_master();
    EXPECT_THROW(device_container->load_drivers(), std::system_error);
    std::this_thread::sleep_for(500ms);
    device_container->shutdown();
    rclcpp::shutdown();
    spinner.join();
        
}