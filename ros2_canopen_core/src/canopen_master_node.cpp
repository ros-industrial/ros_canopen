/*
 *  Copyright 2022 Christoph Hellmann Santos
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include "ros2_canopen_core/canopen_master_node.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;
using namespace ros2_canopen;

void CANopenNode::master_nmt(
	const std::shared_ptr<ros2_canopen_interfaces::srv::CONmtID::Request> request,
	std::shared_ptr<ros2_canopen_interfaces::srv::CONmtID::Response> response)
{
	if (active.load())
	{
		canopen::NmtCommand command = static_cast<canopen::NmtCommand>(request->nmtcommand);
		switch (command)
		{
		case canopen::NmtCommand::ENTER_PREOP:
		case canopen::NmtCommand::RESET_COMM:
		case canopen::NmtCommand::RESET_NODE:
		case canopen::NmtCommand::START:
		case canopen::NmtCommand::STOP:
			can_master->Command(command, request->nodeid);
			response->success = true;
			break;
		default:
			response->success = false;
			break;
		}
	}
	else
	{
		response->success = false;
	}
}
void CANopenNode::master_read_sdo(
	const std::shared_ptr<ros2_canopen_interfaces::srv::COReadID::Request> request,
	std::shared_ptr<ros2_canopen_interfaces::srv::COReadID::Response> response)
{
	ros2_canopen::CODataTypes datatype = static_cast<ros2_canopen::CODataTypes>(request->type);
	switch (datatype)
	{
	case ros2_canopen::CODataTypes::COData8:
		this->master_read<uint8_t>(request, response);
		break;
	case ros2_canopen::CODataTypes::COData16:
		this->master_read<uint16_t>(request, response);
		break;
	case ros2_canopen::CODataTypes::COData32:
		this->master_read<uint32_t>(request, response);
		break;
	default:
		response->success = false;
		break;
	}
}

void CANopenNode::master_write_sdo(
	const std::shared_ptr<ros2_canopen_interfaces::srv::COWriteID::Request> request,
	std::shared_ptr<ros2_canopen_interfaces::srv::COWriteID::Response> response)
{
	ros2_canopen::CODataTypes datatype = static_cast<ros2_canopen::CODataTypes>(request->type);
	switch (datatype)
	{
	case ros2_canopen::CODataTypes::COData8:
		this->master_write<uint8_t>(request, response);
		break;
	case ros2_canopen::CODataTypes::COData16:
		this->master_write<uint16_t>(request, response);
		break;
	case ros2_canopen::CODataTypes::COData32:
		this->master_write<uint32_t>(request, response);
		break;
	default:
		response->success = false;
		break;
	}
}

void CANopenNode::master_set_heartbeat(
	const std::shared_ptr<ros2_canopen_interfaces::srv::COHeartbeatID::Request> request,
	std::shared_ptr<ros2_canopen_interfaces::srv::COHeartbeatID::Response> response)
{
	if (active.load())
	{
		WriteSdoCoTask<uint16_t> write_task(*exec);
		write_task.set_data(can_master, request->nodeid, 0x1017, 0x0, request->heartbeat);
		auto f = write_task.get_future();
		{
			std::scoped_lock<std::mutex> lk(*master_mutex);
			exec->post(write_task);
		}
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
		RCLCPP_ERROR(this->get_logger(), "Couldn't set heartbeat because node not active");
		response->success = false;
	}
}

void CANopenNode::run()
{

	can_master->Reset();
	register_drivers();

	this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

	// Signal to ROS Node that drivers were success fully registered.
	this->registration_done.set_value();

	RCLCPP_INFO(this->get_logger(), "Done configuring.");
	while (configured)
	{
		// Get future from current active promise
		auto active_f = this->active_p.get_future();
		// Wait for future to become ready
		active_f.wait();

		// activate devices
		this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

		RCLCPP_INFO(this->get_logger(), "Done activating.");
		// while node is active do the work.
		while (active.load())
		{
			// get lock on canopen master executor
			std::scoped_lock<std::mutex> lk(*master_mutex);
			// do work for at max 5ms
			loop->run_one_for(500ms);
		}
		this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
	}
	this->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);

	this->deregister_drivers();
	this->drivers = std::map<int, std::string>();
	this->ctx->shutdown();
}

void CANopenNode::register_drivers()
{
	for (auto it = this->drivers.begin(); it != this->drivers.end(); ++it)
	{
		uint8_t id = (uint8_t)it->first;
		std::string name = it->second;
		if (name.compare("BasicDevice") == 0)
		{
			auto dev = std::make_shared<ros2_canopen::ProxyDevice>();
			this->devices->insert({id, dev});
			dev->registerDriver(exec, can_master, id);
			this->executor_->add_node(dev->get_node());
		}
		else
		{

			try
			{
				std::shared_ptr<ros2_canopen::CANopenDevice> dev = poly_loader.createSharedInstance(name.c_str());
				this->devices->insert({id, dev});
				dev->registerDriver(exec, can_master, id);
				this->executor_->add_node(dev->get_node());
			}
			catch (pluginlib::PluginlibException &ex)
			{
				printf("The plugin failed to load for some reason. Error: %s\n", ex.what());
			}
		}
	}
}

void CANopenNode::deregister_drivers()
{
	for (auto it = this->devices->begin(); it != this->devices->end(); ++it)
	{
		this->executor_->remove_node(it->second->get_node());
		it->second->get_node()->~NodeBaseInterface();
		this->devices->erase(it);
	}
}

void CANopenNode::read_yaml()
{
	YAML::Node node = YAML::LoadFile(yaml_path.c_str());
	for (
		YAML::const_iterator it_devices = node.begin();
		it_devices != node.end();
		it_devices++)
	{
		// Get toplevel node name
		std::string device_name = it_devices->first.as<std::string>();
		// Check that this is not master
		if (device_name.find("master") == std::string::npos)
		{
			// Device config
			YAML::Node config = it_devices->second;
			// Save in map
			int node_id = config["node_id"].as<int>();
			std::string driver = config["driver"].as<std::string>();
			drivers.insert({node_id, driver});
		}
	}
}

void CANopenNode::register_services()
{
	// Create service for master_nmt
	this->master_nmt_service = this->create_service<ros2_canopen_interfaces::srv::CONmtID>(
		std::string("~/set_nmt").c_str(),
		std::bind(&CANopenNode::master_nmt,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2));
	// Create service for read sdo
	this->master_read_sdo_service = this->create_service<ros2_canopen_interfaces::srv::COReadID>(
		std::string("~/read_sdo").c_str(),
		std::bind(&CANopenNode::master_read_sdo,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2));

	// Create service for write sdo
	this->master_write_sdo_service = this->create_service<ros2_canopen_interfaces::srv::COWriteID>(
		std::string("~/write_sdo").c_str(),
		std::bind(&CANopenNode::master_write_sdo,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2));

	this->master_set_hearbeat_service = this->create_service<ros2_canopen_interfaces::srv::COHeartbeatID>(
		std::string("~/set_heartbeat").c_str(),
		std::bind(&CANopenNode::master_set_heartbeat,
				  this,
				  std::placeholders::_1,
				  std::placeholders::_2));
}

template <typename FutureT, typename WaitTimeT>
std::future_status
wait_for_result(
	FutureT &future,
	WaitTimeT time_to_wait)
{
	auto end = std::chrono::steady_clock::now() + time_to_wait;
	std::chrono::milliseconds wait_period(100);
	std::future_status status = std::future_status::timeout;
	do
	{
		auto now = std::chrono::steady_clock::now();
		auto time_left = end - now;
		if (time_left <= std::chrono::seconds(0))
		{
			break;
		}
		status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
	} while (rclcpp::ok() && status != std::future_status::ready);
	return status;
}

CallbackReturn
CANopenNode::change_state(const std::uint8_t transition, std::chrono::seconds time_out)
{
	//Iterate through nodes and activate specified tranisiton
	for (auto it = this->devices->begin(); it != this->devices->end(); ++it)
	{
		std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node = it->second->get_node();
		std::string qualitfied_node_name = node->get_fully_qualified_name();
		std::string change_state_name = qualitfied_node_name.append("/change_state");
		std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> change_state_ =
			this->create_client<lifecycle_msgs::srv::ChangeState>(change_state_name);

		std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> request =
			std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
		request->transition.id = transition;

		if (!change_state_->wait_for_service(time_out))
		{
			RCLCPP_ERROR(
				get_logger(),
				"Service %s is not available.",
				change_state_->get_service_name());
			return CallbackReturn::FAILURE;
		}

		auto future_result = change_state_->async_send_request(request);
		auto future_status = wait_for_result(future_result, time_out);

		if (future_status != std::future_status::ready)
		{
			RCLCPP_ERROR(
				get_logger(), "Server time out while setting %hhu for node %s", transition, qualitfied_node_name.c_str());
			return CallbackReturn::FAILURE;
		}

		// We have an answer, let's print our success.
		if (!future_result.get()->success)
		{
			RCLCPP_WARN(
				get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
			return CallbackReturn::FAILURE;
		}
	}
	return CallbackReturn::SUCCESS;
}

CallbackReturn
CANopenNode::on_configure(const rclcpp_lifecycle::State &state)
{
	std::scoped_lock<std::mutex> lk(lifecycle_mutex);
	// Wait for master Thread to terminate.
	if (master_thread_running.valid())
		master_thread_running.wait();

	this->active.store(false);
	this->configured.store(true);
	this->get_parameter("can_interface_name", can_interface_name);
	this->get_parameter("dcf_path", dcf_path);
	this->get_parameter("yaml_path", yaml_path);

	read_yaml();
	// Start loop
	io_guard = std::make_shared<io::IoGuard>();
	ctx = std::make_shared<io::Context>();
	poll = std::make_shared<io::Poll>(*ctx);
	loop = std::make_shared<ev::Loop>(poll->get_poll());
	exec = std::make_shared<ev::Executor>(loop->get_executor());
	can_timer = std::make_shared<io::Timer>(*poll, *exec, CLOCK_MONOTONIC);

	ctrl = std::make_shared<io::CanController>(can_interface_name.c_str());
	chan = std::make_shared<io::CanChannel>(*poll, *exec);

	// Open CAN channel
	chan->open(*ctrl);

	// Create Master from DCF
	//@Todo: Probably read from parameter server
	can_master = std::make_shared<canopen::AsyncMaster>(*can_timer, *chan, dcf_path.c_str(), "", 1);
	master_thread_running = std::async(std::launch::async, std::bind(&CANopenNode::run, this));

	// Create new promise
	active_p = std::promise<void>();

	return CallbackReturn::SUCCESS;
}

CallbackReturn
CANopenNode::on_activate(const rclcpp_lifecycle::State &state)
{
	std::scoped_lock<std::mutex> lk(lifecycle_mutex);
	// Wait for Drivers to be registered - in case it is called to fast.
	registration_done.get_future().wait();
	// Set active to true
	this->active.store(true);
	// Signal to master thread that we are active, to start loop.
	active_p.set_value();
	return CallbackReturn::SUCCESS;
}

CallbackReturn
CANopenNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
	// Recreate promise so that master can wait for new future.
	active_p = std::promise<void>();
	// Stop master loop.
	this->active.store(false);
	return CallbackReturn::SUCCESS;
}

CallbackReturn
CANopenNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
	this->configured.store(false);
	return CallbackReturn::SUCCESS;
}

CallbackReturn
CANopenNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
	this->active.store(false);
	this->configured.store(true);
	return CallbackReturn::SUCCESS;
}

using namespace ros2_canopen;
int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
	auto canopen_node = std::make_shared<CANopenNode>("canopen_master", executor);
	executor->add_node(canopen_node->get_node_base_interface());
	executor->spin();
	rclcpp::shutdown();
	return 0;
}