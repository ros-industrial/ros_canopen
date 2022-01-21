/**
 * @file test_node.cpp
 * @author Christoph Hellmann Santos
 * @brief Testnode for integrating lelycore in a ros2 node
 * @version 0.0.1
 * @date 2022-01-19
 */
#include "ros2_canopen/ros2_canopen_node.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using namespace lely;
using namespace ros2_canopen;

void ROSCANopen_Node::master_nmt(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterNmt::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterNmt::Response> response)
{
  if (active.load())
  {
    std::lock_guard<std::mutex> guard(master_mutex);
    canopen::NmtCommand command = static_cast<canopen::NmtCommand>(request->nmtcommand);
    can_master->Command(command, request->nodeid);
    response->success = true;
  }
  else
  {
    response->success = false;
  }
}

void ROSCANopen_Node::master_read_sdo16(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo16::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterReadSdo16::Response> response)
{
  if (active.load())
  {
    ev_exec_t *exe = *exec;
    lely::canopen::SdoFuture<uint16_t> f;
    {
      std::lock_guard<std::mutex> guard(master_mutex);
      f = can_master->AsyncRead<uint16_t>(exe, request->nodeid, request->index, request->subindex, 100ms);
    }
    while (!f.is_ready())
    {
      std::this_thread::sleep_for(10ms);
    }
    auto res = f.get();
    if (res.has_error())
    {
      response->success = false;
    }
    else
    {
      response->success = true;
      response->data = f.get().value();
    }
  }
  else
  {
    response->success = false;
  }
}

void ROSCANopen_Node::master_write_sdo8(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo8::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo8::Response> response)
{
  response->success = write_sdo<uint8_t>(request->nodeid, request->index, request->subindex, request->data);
}

void ROSCANopen_Node::master_write_sdo16(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo16::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo16::Response> response)
{
  response->success = write_sdo<uint16_t>(request->nodeid, request->index, request->subindex, request->data);
}

void ROSCANopen_Node::master_write_sdo32(
    const std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo32::Request> request,
    std::shared_ptr<ros2_canopen_interfaces::srv::MasterWriteSdo32::Response> response)
{
  response->success = write_sdo<uint32_t>(request->nodeid, request->index, request->subindex, request->data);
}

void ROSCANopen_Node::run()
{
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
  can_master->Reset();
  while (active.load())
  {
    std::lock_guard<std::mutex> guard(master_mutex);
    //@Todo make configurable via parameter
    loop->run_for(20ms);
  }
}

void ROSCANopen_Node::read_yaml()
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
ROSCANopen_Node::on_configure(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  this->get_parameter("can_interface_name", can_interface_name);
  this->get_parameter("dcf_path", dcf_path);
  this->get_parameter("yaml_path", yaml_path);

  //Create service for master_nmt
  this->master_nmt_service = this->create_service<ros2_canopen_interfaces::srv::MasterNmt>(
      "master_nmt",
      std::bind(&ROSCANopen_Node::master_nmt,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  //Create service for read sdo
  this->master_read_sdo16_service = this->create_service<ros2_canopen_interfaces::srv::MasterReadSdo16>(
      "master_read_sdo",
      std::bind(&ROSCANopen_Node::master_read_sdo16,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  //Create service for write sdo
  this->master_write_sdo8_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo8>(
      "master_write8_sdo",
      std::bind(&ROSCANopen_Node::master_write_sdo8,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_write_sdo16_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo16>(
      "master_write16_sdo",
      std::bind(&ROSCANopen_Node::master_write_sdo16,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
  this->master_write_sdo32_service = this->create_service<ros2_canopen_interfaces::srv::MasterWriteSdo32>(
      "master_write32_sdo",
      std::bind(&ROSCANopen_Node::master_write_sdo32,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

  read_yaml();

  // @Todo: Add Devices via Pluginlib?
  // For now simply add a driver
  //basicdevice = std::make_shared<ros2_canopen::BasicDevice>();
  //basicdevice->registerDriver(*exec, *can_master, 2);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_activate(const rclcpp_lifecycle::State &state)
{
  this->active.store(true);
  // devices
  // run loop every 10ms
  //timer_ = this->create_wall_timer(
  //    500ms, std::bind(&ROSCANopen_Node::run_one, this));
  canopen_loop_thread = std::make_unique<std::thread>(std::bind(&ROSCANopen_Node::run, this));
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_deactivate(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  canopen_loop_thread->join();
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_cleanup(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  return CallbackReturn::SUCCESS;
}

CallbackReturn
ROSCANopen_Node::on_shutdown(const rclcpp_lifecycle::State &state)
{
  this->active.store(false);
  canopen_loop_thread->join();
  return CallbackReturn::SUCCESS;
}

using namespace ros2_canopen;
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