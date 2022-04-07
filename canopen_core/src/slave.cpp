//    Copyright 2022 Christoph Hellmann Santos
// 
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.


#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>

#include <future>
#include <atomic>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

using namespace lely;
using namespace std::chrono_literals;

class RPDOTestSlave : public canopen::BasicSlave
{
public:
	using BasicSlave::BasicSlave;
	uint32_t counter = 0;

	void
	countup_task()
	{
		(*this)[0x4001][0] = counter;
		this->TpdoEvent(1);
		counter++;
		this->SubmitWait(10ms, nullptr, std::bind(&RPDOTestSlave::countup_task, this));
	}
};

class ROS2CANopenTestNode : public rclcpp_lifecycle::LifecycleNode
{
	uint8_t id;
	std::string eds;
	std::string ifname;
	std::string test;
	std::atomic<bool> active;
	std::future<void> slave_done;
	std::mutex a;
	std::thread t;


public:
	explicit ROS2CANopenTestNode(const std::string &node_name, bool intra_process_comms = false)
		: rclcpp_lifecycle::LifecycleNode(node_name,
										  rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
	{
		this->declare_parameter<uint8_t>("slave_id", 2);
		this->declare_parameter<std::string>("eds", "/home/christoph/ws_ros2/src/ros2_canopen/canopen_core/ressources/simple.eds");
		this->declare_parameter<std::string>("can_ifname", "vcan0");
		this->declare_parameter<std::string>("test", "simple");
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_configure(const rclcpp_lifecycle::State &)
	{
		active.store(false);
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_activate(const rclcpp_lifecycle::State &state)
	{
		get_parameter("slave_id", id);
		get_parameter("eds", eds);
		get_parameter("can_ifname", ifname);
		get_parameter("test", test);

		active.store(true);

		if(test.compare("simple") == 0){
			t = std::thread(std::bind(&ROS2CANopenTestNode::run_simple, this));
		}
		else if(test.compare("pdo_counter") == 0){
			t = std::thread(std::bind(&ROS2CANopenTestNode::run_pdo_counter, this));
		}

		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_deactivate(const rclcpp_lifecycle::State &state)
	{
		active.store(false);
		t.join();
		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_cleanup(const rclcpp_lifecycle::State &)
	{

		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	on_shutdown(const rclcpp_lifecycle::State &state)
	{

		return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	}

private:
	void run_simple()
	{
		io::IoGuard io_guard;
		io::Context ctx;
		io::Poll poll(ctx);
		ev::Loop loop(poll.get_poll());
		auto exec = loop.get_executor();
		io::Timer timer(poll, exec, CLOCK_MONOTONIC);
		io::CanController ctrl(ifname.c_str());
		io::CanChannel chan(poll, exec);
		chan.open(ctrl);

		canopen::BasicSlave slave(timer, chan, eds.c_str(), "", id);
		slave.Reset();
		while(active.load()){
			
			loop.run_one_for(10ms);
		}
		ctx.shutdown();
	}
	void run_pdo_counter()
	{
		io::IoGuard io_guard;
		io::Context ctx;
		io::Poll poll(ctx);
		ev::Loop loop(poll.get_poll());
		auto exec = loop.get_executor();
		io::Timer timer(poll, exec, CLOCK_MONOTONIC);
		io::CanController ctrl(ifname.c_str());
		io::CanChannel chan(poll, exec);
		chan.open(ctrl);

		RPDOTestSlave slave(timer, chan, eds.c_str(), "", id);
		//slave.submit_counter();
		ev::Task task(std::bind(&RPDOTestSlave::countup_task, &slave));
		exec.post(task);
		slave.Reset();
		while(active.load()){
			
			loop.run_one_for(10ms);
		}
		ctx.shutdown();
	}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;
	auto canopen_slave = std::make_shared<ROS2CANopenTestNode>("canopen_test_slave");
	executor.add_node(canopen_slave->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
