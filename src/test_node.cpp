/**
 * @file test_node.cpp
 * @author Christoph Hellmann Santos
 * @brief Testnode for integrating lelycore in a ros2 node
 * @version 0.0.1
 * @date 2022-01-19
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <fstream>
#include <map>

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/master.hpp>
#include <lely/coapp/fiber_driver.hpp>

#include <yaml-cpp/yaml.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

//#include "ros2_canopen/cia402_driver.hpp"

#define ROS2_CONTROL_DEMO_HARDWARE_PUBLIC

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

using namespace lely;

class CANopen_Device_Driver : canopen::FiberDriver
{
public:
  using FiberDriver::FiberDriver;
  CANopen_Device_Driver(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id) : FiberDriver(exec, master, id)
  {
  }

private:
  void
  OnBoot(canopen::NmtState state, char es,
         const std::string &whatisit) noexcept override
  {
    // What to do when device signaled that it booted.
  }

  void
  OnConfig(std::function<void(std::error_code ec)> res) noexcept override
  {
    // Seems to be called before or during boot of slave device. Clean up all local data related to slave (mainly registers?).
  }

  void
  OnState(canopen::NmtState state) noexcept override
  {
    // When NmtState of device changes
    switch (state)
    {
    case canopen::NmtState::PREOP:
      break;

    case canopen::NmtState::START:
      break;
    }
  }

  void
  OnRpdoWrite(uint16_t idx, uint8_t subidx) noexcept override
  {
    // Handle data sent via RPDO
  }

  void
  OnEmcy(uint16_t eec, uint8_t er, uint8_t *msef) noexcept override
  {
    // Handle emergency message
  }
};

class ROSCANopen_Node : public rclcpp_lifecycle::LifecycleNode
{

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::shared_ptr<io::IoGuard> io_guard;
  std::shared_ptr<io::Context> ctx;
  std::shared_ptr<io::Poll> poll;
  std::shared_ptr<ev::Loop> loop;
  std::shared_ptr<ev::Executor> exec;
  std::shared_ptr<io::CanController> ctrl;
  std::shared_ptr<io::CanChannel> chan;
  std::shared_ptr<io::Timer> can_timer;
  std::shared_ptr<canopen::AsyncMaster> can_master;
  std::shared_ptr<CANopen_Device_Driver> canopen_device_driver;
  std::shared_ptr<std::string> nmt_status;

  std::string can_interface_name;
  std::string dcf_path;
  std::string yaml_path;
  std::map<int, std::string> drivers;

public:
  explicit ROSCANopen_Node(const std::string &node_name, bool intra_process_comms = false)
      : rclcpp_lifecycle::LifecycleNode(
            node_name,
            rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    this->declare_parameter<std::string>("can_interface_name", "vcan0");
    this->declare_parameter<std::string>("dcf_path", "");
    this->declare_parameter<std::string>("yaml_path", "");
    can_interface_name = "";
    dcf_path = "";
    yaml_path = "";
  }

  void run_one()
  {
    loop->run_one();
  }

  void read_yaml()
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

  CallbackReturn
  on_configure(const rclcpp_lifecycle::State &state)
  {
    this->get_parameter("can_interface_name", can_interface_name);
    this->get_parameter("dcf_path", dcf_path);
    this->get_parameter("yaml_path", yaml_path);

    read_yaml();

    io_guard = std::make_shared<io::IoGuard>();
    ctx = std::make_shared<io::Context>();
    poll = std::make_shared<io::Poll>(*ctx);
    loop = std::make_shared<ev::Loop>(poll->get_poll());
    exec = std::make_shared<ev::Executor>(loop->get_executor());
    can_timer = std::make_shared<io::Timer>(*poll, *exec, CLOCK_MONOTONIC);

    //@Todo: Probably read from parameter server
    ctrl = std::make_shared<io::CanController>(can_interface_name.c_str());
    chan = std::make_shared<io::CanChannel>(*poll, *exec);

    //Open CAN channel
    chan->open(*ctrl);

    //Create Master from DCF
    //@Todo: Probably read from parameter server
    can_master = std::make_shared<canopen::AsyncMaster>(*can_timer, *chan, dcf_path.c_str(), "", 1);

    // @Todo: Add Devices via Pluginlib?
    // For now simply add a driver
    //canopen_device_driver = std::make_shared<CANopen_Device_Driver>(*exec, *can_master, 2, nmt_status);

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &state)
  {
    // devices
    can_master->Reset();
    // run loop every 10ms
    timer_ = this->create_wall_timer(
        10ms, std::bind(&ROSCANopen_Node::run_one, this));

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &state)
  {
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &state)
  {
    timer_.reset();
    ctx->shutdown();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &state)
  {
    ctx->shutdown();
    return CallbackReturn::SUCCESS;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto cia402_node = std::make_shared<ROSCANopen_Node>("test_node");
  executor.add_node(cia402_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}