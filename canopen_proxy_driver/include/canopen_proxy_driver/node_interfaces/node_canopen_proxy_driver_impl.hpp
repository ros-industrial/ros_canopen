#ifndef NODE_CANOPEN_PROXY_DRIVER_IMPL_HPP_
#define NODE_CANOPEN_PROXY_DRIVER_IMPL_HPP_

#include "canopen_proxy_driver/node_interfaces/node_canopen_proxy_driver.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template <class NODETYPE>
NodeCanopenProxyDriver<NODETYPE>::NodeCanopenProxyDriver(NODETYPE *node) : ros2_canopen::node_interfaces::NodeCanopenBaseDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::init(bool called_from_base)
{
	RCLCPP_ERROR(this->node_->get_logger(), "Not init implemented.");
}

template <>
void NodeCanopenProxyDriver<rclcpp::Node>::init(bool called_from_base)
{
	nmt_state_publisher = this->node_->create_publisher<std_msgs::msg::String>(
		std::string(
			this->node_->get_name())
			.append("/nmt_state")
			.c_str(),
		10);
	tpdo_subscriber = this->node_->create_subscription<canopen_interfaces::msg::COData>(
		std::string(this->node_->get_name()).append("/tpdo").c_str(),
		10,
		std::bind(&NodeCanopenProxyDriver<rclcpp::Node>::on_tpdo, this, std::placeholders::_1));

	rpdo_publisher = this->node_->create_publisher<canopen_interfaces::msg::COData>(
		std::string(this->node_->get_name()).append("/rpdo").c_str(), 10);

	nmt_state_reset_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/nmt_reset_node").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp::Node>::on_nmt_state_reset,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	nmt_state_start_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/nmt_start_node").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp::Node>::on_nmt_state_start,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	sdo_read_service = this->node_->create_service<canopen_interfaces::srv::CORead>(
		std::string(this->node_->get_name()).append("/sdo_read").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp::Node>::on_sdo_read,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	sdo_write_service = this->node_->create_service<canopen_interfaces::srv::COWrite>(
		std::string(this->node_->get_name()).append("/sdo_write").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp::Node>::on_sdo_write,
			this,
			std::placeholders::_1,
			std::placeholders::_2));
}

template <>
void NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::init(bool called_from_base)
{
	nmt_state_publisher = this->node_->create_publisher<std_msgs::msg::String>(
		std::string(
			this->node_->get_name())
			.append("/nmt_state")
			.c_str(),
		10);
	tpdo_subscriber = this->node_->create_subscription<canopen_interfaces::msg::COData>(
		std::string(this->node_->get_name()).append("/tpdo").c_str(),
		10,
		std::bind(&NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::on_tpdo, this, std::placeholders::_1));

	rpdo_publisher = this->node_->create_publisher<canopen_interfaces::msg::COData>(
		std::string(this->node_->get_name()).append("/rpdo").c_str(), 10);

	nmt_state_reset_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/nmt_reset_node").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::on_nmt_state_reset,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	nmt_state_start_service = this->node_->create_service<std_srvs::srv::Trigger>(
		std::string(this->node_->get_name()).append("/nmt_start_node").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::on_nmt_state_start,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	sdo_read_service = this->node_->create_service<canopen_interfaces::srv::CORead>(
		std::string(this->node_->get_name()).append("/sdo_read").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::on_sdo_read,
			this,
			std::placeholders::_1,
			std::placeholders::_2));

	sdo_write_service = this->node_->create_service<canopen_interfaces::srv::COWrite>(
		std::string(this->node_->get_name()).append("/sdo_write").c_str(),
		std::bind(
			&NodeCanopenProxyDriver<rclcpp_lifecycle::LifecycleNode>::on_sdo_write,
			this,
			std::placeholders::_1,
			std::placeholders::_2));
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_nmt(canopen::NmtState nmt_state)
{
	if (this->activated_.load())
	{
		auto message = std_msgs::msg::String();

		switch (nmt_state)
		{
		case canopen::NmtState::BOOTUP:
			message.data = "BOOTUP";
			break;
		case canopen::NmtState::PREOP:
			message.data = "PREOP";
			break;
		case canopen::NmtState::RESET_COMM:
			message.data = "RESET_COMM";
			break;
		case canopen::NmtState::RESET_NODE:
			message.data = "RESET_NODE";
			break;
		case canopen::NmtState::START:
			message.data = "START";
			break;
		case canopen::NmtState::STOP:
			message.data = "STOP";
			break;
		case canopen::NmtState::TOGGLE:
			message.data = "TOGGLE";
			break;
		default:
			RCLCPP_ERROR(this->node_->get_logger(), "Unknown NMT State.");
			message.data = "ERROR";
			break;
		}
		RCLCPP_INFO(
			this->node_->get_logger(),
			"Slave %hhu: Switched NMT state to %s",
			this->lely_driver_->get_id(),
			message.data.c_str());

		nmt_state_publisher->publish(message);
	}
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_tpdo(const canopen_interfaces::msg::COData::SharedPtr msg)
{
	ros2_canopen::COData data = {msg->index, msg->subindex, msg->data, static_cast<CODataTypes>(msg->type)};
	if (!tpdo_transmit(data))
	{
		RCLCPP_ERROR(this->node_->get_logger(), "Could transmit PDO because driver not activated.");
	}
}

template <class NODETYPE>
bool NodeCanopenProxyDriver<NODETYPE>::tpdo_transmit(ros2_canopen::COData& data)
{
	if (this->activated_.load())
	{
		this->lely_driver_->tpdo_transmit(data);
		return true;
	}
	return false;
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_rpdo(ros2_canopen::COData d)
{
	if (this->activated_.load())
	{
		RCLCPP_INFO(
			this->node_->get_logger(),
			"Slave %hhu: Sent PDO index %hu, subindex %hhu, data %x",
			this->lely_driver_->get_id(),
			d.index_,
			d.subindex_,
			d.data_);
		auto message = canopen_interfaces::msg::COData();
		message.index = d.index_;
		message.subindex = d.subindex_;
		message.data = d.data_;
		message.type = static_cast<uint8_t>(d.type_);
		rpdo_publisher->publish(message);
	}
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_nmt_state_reset(
	const std_srvs::srv::Trigger::Request::SharedPtr request,
	std_srvs::srv::Trigger::Response::SharedPtr response)
{
	response->success = reset_node_nmt_command();
}

template <class NODETYPE>
bool NodeCanopenProxyDriver<NODETYPE>::reset_node_nmt_command()
{
	if (this->activated_.load())
	{
		this->lely_driver_->nmt_command(canopen::NmtCommand::RESET_NODE);
		return true;
	}
	RCLCPP_ERROR(this->node_->get_logger(), "Could not reset device via NMT because driver not activated.");
	return false;
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_nmt_state_start(
	const std_srvs::srv::Trigger::Request::SharedPtr request,
	std_srvs::srv::Trigger::Response::SharedPtr response)
{
	response->success = start_node_nmt_command();
}

template <class NODETYPE>
bool NodeCanopenProxyDriver<NODETYPE>::start_node_nmt_command()
{
	if (this->activated_.load())
	{
		this->lely_driver_->nmt_command(canopen::NmtCommand::START);
		return true;
	}
	RCLCPP_ERROR(this->node_->get_logger(), "Could not start device via NMT because driver not activated.");
	return false;
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_sdo_read(
	const canopen_interfaces::srv::CORead::Request::SharedPtr request,
	canopen_interfaces::srv::CORead::Response::SharedPtr response)
{
	ros2_canopen::COData data = {request->index, request->subindex, 0U, static_cast<CODataTypes>(request->type)};
	response->success = sdo_read(data);
	response->data = data.data_;
}

template <class NODETYPE>
bool NodeCanopenProxyDriver<NODETYPE>::sdo_read(ros2_canopen::COData& data)
{
	if (this->activated_.load())
	{
		RCLCPP_INFO(
			this->node_->get_logger(), "Slave %hhu: SDO Read Call index=0x%x subindex=%hhu bits=%hhu",
			this->lely_driver_->get_id(), data.index_, data.subindex_, data.type_);

		// Only allow one SDO request concurrently
		std::scoped_lock<std::mutex> lk(sdo_mtex);
		// Send read request
		auto f = this->lely_driver_->async_sdo_read(data);
		// Wait for response
		f.wait();
		// Process response
		try
		{
			data.data_ = f.get().data_;
		}
		catch (std::exception &e)
		{
			RCLCPP_ERROR(this->node_->get_logger(), e.what());
			return false;
		}
		return true;
	}
	RCLCPP_ERROR(this->node_->get_logger(), "Could not read from SDO because driver not activated.");
	return false;
}

template <class NODETYPE>
void NodeCanopenProxyDriver<NODETYPE>::on_sdo_write(
	const canopen_interfaces::srv::COWrite::Request::SharedPtr request,
	canopen_interfaces::srv::COWrite::Response::SharedPtr response)
{
	ros2_canopen::COData data =
		{request->index, request->subindex, request->data, static_cast<CODataTypes>(request->type)};
	response->success = sdo_write(data);
}

template <class NODETYPE>
bool NodeCanopenProxyDriver<NODETYPE>::sdo_write(ros2_canopen::COData &data)
{
	if (this->activated_.load())
	{
		RCLCPP_INFO(
			this->node_->get_logger(), "Slave %hhu: SDO Write Call index=0x%x subindex=%hhu bits=%hhu data=%u",
			this->lely_driver_->get_id(), data.index_, data.subindex_, data.type_, data.data_);

		// Only allow one SDO request concurrently
		std::scoped_lock<std::mutex> lk(sdo_mtex);

		// Send write request
		auto f = this->lely_driver_->async_sdo_write(data);
		// Wait for request to complete
		f.wait();

		// Process response
		try
		{
			f.get();
		}
		catch (std::exception &e)
		{
			RCLCPP_ERROR(this->node_->get_logger(), e.what());
			return false;
		}
		return true;
	}
	RCLCPP_ERROR(this->node_->get_logger(), "Could not write to SDO because driver not activated.");
	return false;
}

#endif