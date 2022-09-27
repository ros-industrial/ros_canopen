#ifndef NODE_CANOPEN_BASIC_MASTER_HPP_
#define NODE_CANOPEN_BASIC_MASTER_HPP_

#include "canopen_core/node_interfaces/node_canopen_master.hpp"
#include "canopen_master_driver/lely_master_bridge.hpp"

namespace ros2_canopen
{
    namespace node_interfaces
    {
        template <class NODETYPE>
        void NodeCanopenBasicMaster<NODETYPE>::on_sdo_read(
            const std::shared_ptr<canopen_interfaces::srv::COReadID::Request> request,
            std::shared_ptr<canopen_interfaces::srv::COReadID::Response> response)
        {
            if (node_canopen_master_->activated.load())
            {
                ros2_canopen::CODataTypes datatype = static_cast<ros2_canopen::CODataTypes>(request->type);
                ros2_canopen::COData data = {request->index, request->subindex, 0U, datatype};
                std::future<COData> f = this->master_->async_read_sdo(request->nodeid, data);
                f.wait();
                try
                {
                    response->data = f.get().data_;
                    response->success = true;
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), e.what());
                    response->success = false;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "LifecycleMasterNode is not in active state. SDO read service is not available.");
                response->success = false;
            }
        }
        template <class NODETYPE>
        void LifecycleMasterNode<NODETYPE>::on_sdo_write(
            const std::shared_ptr<canopen_interfaces::srv::COWriteID::Request> request,
            std::shared_ptr<canopen_interfaces::srv::COWriteID::Response> response)
        {
            if (node_canopen_master_->activated.load())
            {
                ros2_canopen::CODataTypes datatype = static_cast<ros2_canopen::CODataTypes>(request->type);
                ros2_canopen::COData data = {request->index, request->subindex, request->data, datatype};
                std::future<bool> f = this->master_->async_write_sdo(request->nodeid, data);
                f.wait();
                try
                {
                    response->success = f.get();
                }
                catch (std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), e.what());
                    response->success = false;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "LifecycleMasterNode is not in active state. SDO write service is not available.");
                response->success = false;
            }
        }
        template <>
        void NodeCanopenBasicMaster<rclcpp::Node>::init(bool called_from_base)
        {
            // declare services
            sdo_read_service = this->node_->create_service<canopen_interfaces::srv::COReadID>(
                std::string(this->node_->get_name()).append("/sdo_read").c_str(),
                std::bind(
                    &ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp::Node>::on_sdo_read,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            sdo_write_service = this->node_->create_service<canopen_interfaces::srv::COWriteID>(
                std::string(this->node_->get_name()).append("/sdo_write").c_str(),
                std::bind(
                    &ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp::Node>::on_sdo_write,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }

        template <>
        void NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
        {
            // declare services
            sdo_read_service = this->node_->create_service<canopen_interfaces::srv::COReadID>(
                std::string(this->node_->get_name()).append("/sdo_read").c_str(),
                std::bind(
                    &ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>::on_sdo_read,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

            sdo_write_service = this->node_->create_service<canopen_interfaces::srv::COWriteID>(
                std::string(this->node_->get_name()).append("/sdo_write").c_str(),
                std::bind(
                    &ros2_canopen::node_interfaces::NodeCanopenBasicMaster<rclcpp_lifecycle::LifecycleNode>::on_sdo_write,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));
        }
    }
}

#endif