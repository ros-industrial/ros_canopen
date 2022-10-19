#ifndef NODE_CANOPEN_402_DRIVER
#define NODE_CANOPEN_402_DRIVER

#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "canopen_402_driver/motor.hpp"
#include "canopen_402_driver/lely_motion_controller_bridge.hpp"
#include "canopen_interfaces/srv/co_target_double.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace ros2_canopen
{
    namespace node_interfaces
    {

        template <class NODETYPE>
        class NodeCanopen402Driver : public NodeCanopenProxyDriver<NODETYPE>
        {
            static_assert(
                std::is_base_of<rclcpp::Node, NODETYPE>::value ||
                    std::is_base_of<rclcpp_lifecycle::LifecycleNode, NODETYPE>::value,
                "NODETYPE must derive from rclcpp::Node or rclcpp_lifecycle::LifecycleNode");

        protected:
            std::shared_ptr<LelyMotionControllerBridge> mc_driver_;
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
            double scale_pos_to_dev_;
            double scale_pos_from_dev_;
            double scale_vel_to_dev_;
            double scale_vel_from_dev_;

            void publish();
            void run();

        public:
            NodeCanopen402Driver(NODETYPE *node);

            virtual void init(bool called_from_base) override;
            virtual void configure(bool called_from_base) override;
            virtual void activate(bool called_from_base) override;
            virtual void deactivate(bool called_from_base) override;
            virtual void add_to_master() override;

            virtual double get_speed()
            {
                return this->mc_driver_->get_speed() * scale_pos_from_dev_;
            }
            
            virtual double get_position()
            {
                return this->mc_driver_->get_position() * scale_vel_from_dev_;
            }

            virtual uint16_t get_mode(){
                return motor_->getMode();
            }

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
             * @brief Method to initialise device
             *
             * Calls Motor402::handleInit function. Brings motor to enabled
             * state and homes it.
             *
             * @param [in] void
             *
             * @return  bool
             * Indicates initialisation procedure result
             */
            bool init_motor();

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
             * @brief Method to recover device
             *
             * Calls Motor402::handleRecover function. Resets faults and brings
             * motor to enabled state.
             *
             * @param [in] void
             *
             * @return bool
             */
            bool recover_motor();

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
             * @brief Method to halt device
             *
             * Calls Motor402::handleHalt function. Calls Quickstop. Resulting
             * Motor state depends on devices configuration specifically object
             * 0x605A.
             *
             * @param [in] void
             *
             * @return bool
             */
            bool halt_motor();

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
             * @brief Method to set profiled position mode
             *
             * Calls Motor402::enterModeAndWait with Profiled Position Mode as
             * Target Operation Mode. If successful, the motor was transitioned
             * to Profiled Position Mode.
             *
             * @param [in] void
             *
             * @return bool
             */
            bool set_mode_position();

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
             * @brief Method to set profiled velocity mode
             *
             * Calls Motor402::enterModeAndWait with Profiled Velocity Mode as
             * Target Operation Mode. If successful, the motor was transitioned
             * to Profiled Velocity Mode.
             *
             * @param [in] void
             *
             * @return bool
             */
             bool set_mode_velocity();

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
             * @brief Method to set cyclic position mode
             *
             * Calls Motor402::enterModeAndWait with Cyclic Position Mode as
             * Target Operation Mode. If successful, the motor was transitioned
             * to Cyclic Position Mode.
             *
             * @param [in] void
             *
             * @return bool
             */
             bool set_mode_cyclic_position();

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
             * @brief Method to set cyclic velocity mode
             *
             * Calls Motor402::enterModeAndWait with Cyclic Velocity Mode as
             * Target Operation Mode. If successful, the motor was transitioned
             * to Cyclic Velocity Mode.
             *
             * @param [in] void
             *
             * @return bool
             */
             bool set_mode_cyclic_velocity();

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
             * @brief Method to set profiled torque mode
             *
             * Calls Motor402::enterModeAndWait with Profiled Torque Mode as
             * Target Operation Mode. If successful, the motor was transitioned
             * to Profiled Torque Mode.
             *
             * @param [in] void
             *
             * @return bool
             */
             bool set_mode_torque();

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
             * @brief Method to set target
             *
             * Calls Motor402::setTarget and sets the requested target value. Note
             * that the resulting movement is dependent on the Operation Mode and the
             * drives state.
             *
             * @param [in] double target value
             *
             * @return bool
             */
             bool set_target(double target);
        };
    }
}

#endif