#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "device.hpp"

class DeviceManager : public rclcpp_components::ComponentManager {
public:
    DeviceManager(
        std::weak_ptr<rclcpp::Executor> executor =
        std::weak_ptr<rclcpp::executors::MultiThreadedExecutor>(),
        std::string node_name = "DeviceManager",
        const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()
            .start_parameter_services(false)
            .start_parameter_event_publisher(false)) :
            rclcpp_components::ComponentManager(executor, node_name, node_options), 
            poly_loader_("ros2_canopen_core", "ros2_canopen::CANopenDriverWrapper") {

        this->declare_parameter("can_interface_name");
        this->declare_parameter("dcf_txt");
        this->declare_parameter("dcf_config");    
    }

    bool init();

private:
    std::map<uint32_t, std::shared_ptr<ros2_canopen::CANopenDriverWrapper>> drivers_;
    pluginlib::ClassLoader<ros2_canopen::CANopenDriverWrapper> poly_loader_;
    std::shared_ptr<ros2_canopen::MasterDevice> can_master_;
    std::shared_ptr<ev::Executor> exec_;

    bool load_driver(std::string& device_name, uint32_t node_id);   // can make a ROS service for this

    bool init_devices_from_config(io::Timer& timer,
        io::CanChannel& chan,    // assuming linux
        ev::Executor& exec,
        std::string& dcf_txt, std::string& dcf_config);
};