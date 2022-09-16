
#ifndef MC_DEVICE_NODE_HPP
#define MC_DEVICE_NODE_HPP

#include "canopen_402_driver/visibility_control.h"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "canopen_proxy_driver/lifecycle_canopen_proxy_driver.hpp"
#include "canopen_402_driver/motor.hpp"

using namespace std::chrono_literals;
using namespace ros2_canopen;
using namespace canopen_402;
namespace ros2_canopen
{
    /**
     * @brief ROS2 node for a Motion Controller
     *
     * This class provides a ros2 node for a CIA 402
     * device.
     */
    class LifecycleMotionControllerDriver : public LifecycleProxyDriver
    {
    private:
        std::shared_ptr<MCDeviceDriver> mc_driver_;
        std::shared_ptr<Motor402> motor_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_init_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_halt_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_recover_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_position_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_torque_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_velocity_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_velocity_service;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handle_set_mode_cyclic_position_service;
        rclcpp::Service<canopen_interfaces::srv::COTargetDouble>::SharedPtr handle_set_target_service;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publish_joint_state;
        uint32_t period_ms_;
        virtual bool add() override;
        virtual void register_ros_interface() override;
        virtual void start_timers() override;
        virtual void stop_timers() override;

    public:
        explicit LifecycleMotionControllerDriver(const rclcpp::NodeOptions &options)
            : LifecycleProxyDriver(options)
        {
        }

        void run()
        {
            motor_->handleRead();
            motor_->handleWrite();
            // motor_->handleDiag();
            publish();
        }

        void init() override
        {
            LifecycleProxyDriver::init();
            period_ms_ = 20;
        }

    protected:
        // void read_config() override
        // {
        //     RCLCPP_INFO(this->get_logger(), "read_config_start");
        //     auto period = this->config_->get_entry<uint32_t>(std::string(this->get_name()), std::string("period"));
        //     RCLCPP_INFO(this->get_logger(), "read_config_start");
        //     if (!period.has_value())
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "ERROR: Bus Configuration does not set period for %s", this->get_name());
        //     }
        //     RCLCPP_INFO(this->get_logger(), "read_config_start");
        //     period_ms_ = period.value();
        //     RCLCPP_INFO(this->get_logger(), "read_config_end");
        // }

        virtual void on_rpdo(COData data) override
        {
            RCLCPP_INFO(this->get_logger(), "on_rpo not implemented");
        }

        /**
         * @brief Configures the driver
         *
         * Read parameters
         * Initialise objects
         *
         * @param state
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &state);

        /**
         * @brief Activates the driver
         *
         * Add driver to masters event loop
         *
         * @param state
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &state);

        /**
         * @brief Deactivates the driver
         *
         * Remove driver from masters event loop
         *
         * @param state
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &state);

        /**
         * @brief Cleanup the driver
         *
         * Delete objects
         *
         * @param state
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &state);

        /**
         * @brief Shutdowns the driver
         *
         * Delete objects
         *
         * @param state
         * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &state);

    private:

        /**
         * @brief Service Callback to initialise device
         *
         * Calls Motor402::handleInit function. Brings motor to enabled
         * state and homes it.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_init(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to recover device
         *
         * Calls Motor402::handleRecover function. Resets faults and brings
         * motor to enabled state.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_recover(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to halt device
         *
         * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
         * Motor state depends on devices configuration specifically object
         * 0x605A.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_halt(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set profiled position mode
         *
         * Calls Motor402::enterModeAndWait with Profiled Position Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Profiled Position Mode.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_set_mode_position(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set profiled velocity mode
         *
         * Calls Motor402::enterModeAndWait with Profiled Velocity Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Profiled Velocity Mode.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_set_mode_velocity(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set cyclic position mode
         *
         * Calls Motor402::enterModeAndWait with Cyclic Position Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Cyclic Position Mode.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_set_mode_cyclic_position(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set cyclic velocity mode
         *
         * Calls Motor402::enterModeAndWait with Cyclic Velocity Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Cyclic Velocity Mode.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_set_mode_cyclic_velocity(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set profiled torque mode
         *
         * Calls Motor402::enterModeAndWait with Profiled Torque Mode as
         * Target Operation Mode. If successful, the motor was transitioned
         * to Profiled Torque Mode.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_set_mode_torque(
            const std_srvs::srv::Trigger::Request::SharedPtr request,
            std_srvs::srv::Trigger::Response::SharedPtr response);

        /**
         * @brief Service Callback to set target
         *
         * Calls Motor402::setTarget and sets the requested target value. Note
         * that the resulting movement is dependent on the Operation Mode and the
         * drives state.
         *
         * @param [in] request
         * @param [out] response
         */
        void handle_set_target(
            const canopen_interfaces::srv::COTargetDouble::Request::SharedPtr request,
            canopen_interfaces::srv::COTargetDouble::Response::SharedPtr response);

        /**
         * @brief Publishes actual position and speed
         *
         */
        void publish();
    };

}

#endif
