
#ifndef MC_DEVICE_NODE_HPP
#define MC_DEVICE_NODE_HPP

#include "std_srvs/srv/trigger.hpp"
#include "ros2_canopen_interfaces/srv/co_target_double.hpp"
#include "proxy_device_driver/proxy_device_driver.hpp"
#include "motion_controller_driver/motor.hpp"

using namespace std::chrono_literals;
using namespace ros2_canopen;
namespace ros2_canopen
{
    /**
     * @brief ROS2 node for a ProxyDevice
     *
     * This class provides a ros2 node for a simple Proxy
     * device that forwards nmt, pdo and sdo.
     */
    class MotionControllerDriver : public ProxyDeviceDriver
    {
    private:
        std::shared_ptr<MCDeviceDriver> mc_driver_;
        std::shared_ptr<Motor402> motor_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_halt_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_position_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_torque_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_velocity_service;
        rclcpp::Service<ros2_canopen_interfaces::srv::COTargetDouble>::SharedPtr handle_set_target_service;

        void register_services();

    public:
        explicit MotionControllerDriver(const rclcpp::NodeOptions &options)
            : ProxyDeviceDriver(options)
        {
            
        }

        void run()
        {
            motor_->handleRead();
            motor_->handleWrite();
        }

        void init(ev::Executor &exec,
                  canopen::AsyncMaster &master,
                  uint8_t node_id) noexcept override;

    protected:
        virtual void on_rpdo(COData data) override
        {
            RCLCPP_INFO(this->get_logger(), "on_rpo not implemented");
        }

    private:
        std::atomic<bool> active;
        void handle_init(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        void handle_halt(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        void handle_set_mode_position(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        void handle_set_mode_velocity(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        void handle_set_mode_torque(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        void handle_set_target(
            const ros2_canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
            ros2_canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response);
    };

}

#endif