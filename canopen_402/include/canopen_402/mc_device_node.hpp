
#ifndef MC_DEVICE_NODE_HPP
#define MC_DEVICE_NODE_HPP

#include "std_srvs/srv/trigger.hpp"
#include "ros2_canopen_interfaces/srv/co_target_double.hpp"
#include "ros2_canopen_core/proxy_device_node.hpp"
#include "canopen_402/motor.hpp"

using namespace std::chrono_literals;
using namespace ros2_canopen;
namespace canopen_402
{
    /**
     * @brief ROS2 node for a ProxyDevice
     *
     * This class provides a ros2 node for a simple Proxy
     * device that forwards nmt, pdo and sdo.
     */
    class MCDeviceNode : public ProxyDeviceNode
    {
    private:
        std::shared_ptr<MCDeviceDriver> mc_driver;
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
        explicit MCDeviceNode(
            const std::string &node_name,
            std::shared_ptr<MCDeviceDriver> driver,
            std::shared_ptr<Motor402> motor,
            bool intra_process_comms = false)
            : ProxyDeviceNode(
                  node_name,
                  std::static_pointer_cast<BasicDeviceDriver>(driver),
                  intra_process_comms)
        {
            mc_driver = driver;
            motor_ = motor;
            register_services();
        }

        void run()
        {
            //RCLCPP_INFO(this->get_logger(), "Running");
            motor_->handleRead();
            motor_->handleWrite();
        }

    protected:
        virtual void on_rpdo(COData data) override
        {
            RCLCPP_INFO(this->get_logger(), "on_rpo not implemented");
        }
        virtual CallbackReturn on_configure_app(const rclcpp_lifecycle::State &state) override
        {
            RCLCPP_INFO(this->get_logger(), "on_configure_app not implemented");
            return CallbackReturn::SUCCESS;
        }
        virtual CallbackReturn on_activate_app(const rclcpp_lifecycle::State &state) override
        {
            RCLCPP_INFO(this->get_logger(), "Activating node");
            motor_->registerDefaultModes();
            mc_driver->validate_objs();

            RCLCPP_INFO(this->get_logger(), "Registered Default modes");
            timer_ = this->create_wall_timer(
                500ms, std::bind(&MCDeviceNode::run, this));

            RCLCPP_INFO(this->get_logger(), "Store Active");
            active.store(true);
            RCLCPP_INFO(this->get_logger(), "Done Activate");
            return CallbackReturn::SUCCESS;
        }
        virtual CallbackReturn on_deactivate_app(const rclcpp_lifecycle::State &state) override
        {
            active.store(false);
            timer_->cancel();
            return CallbackReturn::SUCCESS;
        }
        virtual CallbackReturn on_cleanup_app(const rclcpp_lifecycle::State &state) override
        {
            RCLCPP_INFO(this->get_logger(), "on_cleanup_app not implemented");
            return CallbackReturn::SUCCESS;
        }
        virtual CallbackReturn on_shuttdown_app(const rclcpp_lifecycle::State &state) override
        {
            RCLCPP_INFO(this->get_logger(), "on_shuttdown_app not implemented");
            return CallbackReturn::SUCCESS;
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

};

#endif