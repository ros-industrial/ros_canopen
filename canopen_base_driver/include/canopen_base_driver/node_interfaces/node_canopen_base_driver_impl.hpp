#ifndef NODE_CANOPEN_BASE_DRIVER_IMPL
#define NODE_CANOPEN_BASE_DRIVER_IMPL
#include "canopen_base_driver/node_interfaces/node_canopen_base_driver.hpp"
#include "canopen_core/driver_error.hpp"

using namespace ros2_canopen::node_interfaces;

template <class NODETYPE>
NodeCanopenBaseDriver<NODETYPE>::NodeCanopenBaseDriver(NODETYPE *node) : ros2_canopen::node_interfaces::NodeCanopenDriver<NODETYPE>(node)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::init(bool called_from_base)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::configure(bool called_from_base)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::activate(bool called_from_base)
{
	nmt_state_publisher_thread_ =
		std::thread(std::bind(&NodeCanopenBaseDriver<NODETYPE>::nmt_listener, this));

	rpdo_publisher_thread_ =
		std::thread(std::bind(&NodeCanopenBaseDriver<NODETYPE>::rdpo_listener, this));
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::deactivate(bool called_from_base)
{
	nmt_state_publisher_thread_.join();
	rpdo_publisher_thread_.join();
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::cleanup(bool called_from_base)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::shutdown(bool called_from_base)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::add_to_master()
{
	std::shared_ptr<std::promise<std::shared_ptr<ros2_canopen::LelyDriverBridge>>> prom;
	prom = std::make_shared<std::promise<std::shared_ptr<ros2_canopen::LelyDriverBridge>>>();
	std::future<std::shared_ptr<ros2_canopen::LelyDriverBridge>> f = prom->get_future();
	this->exec_->post([this, prom]()
					  {
                        std::scoped_lock<std::mutex> lock (this->driver_mutex_);
                        this->lely_driver_ =
                            std::make_shared<ros2_canopen::LelyDriverBridge>(*(this->exec_), *(this->master_), this->node_id_, this->node_->get_name());
						this->driver_ = std::static_pointer_cast<lely::canopen::BasicDriver>(this->lely_driver_);
                        prom->set_value(lely_driver_); });

	auto future_status = f.wait_for(this->non_transmit_timeout_);
	if (future_status != std::future_status::ready)
	{
		RCLCPP_ERROR(this->node_->get_logger(), "Adding timed out.");
		throw DriverException("add_to_master: adding timed out");
	}
	this->lely_driver_ = f.get();
	this->driver_ = std::static_pointer_cast<lely::canopen::BasicDriver>(this->lely_driver_);
	if (!this->lely_driver_->IsReady())
	{
		RCLCPP_WARN(this->node_->get_logger(), "Wait for device to boot.");
		try
		{
			this->lely_driver_->wait_for_boot();
		}
		catch (const std::exception &e)
		{
			RCLCPP_ERROR(this->node_->get_logger(), e.what());
		}
	}
	RCLCPP_INFO(this->node_->get_logger(), "Driver booted and ready.");
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::remove_from_master()
{
	std::shared_ptr<std::promise<void>> prom = std::make_shared<std::promise<void>>();
	auto f = prom->get_future();
	this->exec_->post([this, prom]()
					  { 
                        this->driver_.reset();
						this->lely_driver_.reset();
                        prom->set_value(); });

	auto future_status = f.wait_for(this->non_transmit_timeout_);
	if (future_status != std::future_status::ready)
	{
		throw DriverException("remove_from_master: removing timed out");
	}
}
template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::nmt_listener()
{
	while (rclcpp::ok())
	{
		std::future<lely::canopen::NmtState> f;
		{
			std::scoped_lock<std::mutex> lock(this->driver_mutex_);
			f = this->lely_driver_->async_request_nmt();
		}
		while (f.wait_for(this->non_transmit_timeout_) != std::future_status::ready)
		{
			if (!this->activated_.load())
				return;
		}
		try{
			auto state = f.get();
			if (nmt_state_cb_)
			{
				nmt_state_cb_(state, this->lely_driver_->get_id());
			}
			on_nmt(state);
		}
		catch (const std::future_error &e)
		{
			break;
		}
	}
}
template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::on_nmt(canopen::NmtState nmt_state)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::on_rpdo(COData data)
{
}

template <class NODETYPE>
void NodeCanopenBaseDriver<NODETYPE>::rdpo_listener()
{
	while (rclcpp::ok())
	{
		std::future<ros2_canopen::COData> f;
		{
			std::scoped_lock<std::mutex> lock(this->driver_mutex_);
			f = lely_driver_->async_request_rpdo();
		}

		while (f.wait_for(this->non_transmit_timeout_) != std::future_status::ready)
		{
			if (!this->activated_.load())
				return;
		}
		try
		{
			auto rpdo = f.get();
			if (rpdo_cb_)
			{
				rpdo_cb_(rpdo, this->lely_driver_->get_id());
			}
			on_rpdo(f.get());
		}
		catch(const std::future_error& e)
		{
			break;
		}
		

	}
}

#endif
