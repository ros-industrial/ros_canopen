#include "canopen_chain_node/sync_node.hpp"


void SyncNode::report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  canopen::LayerReport r;
  read(r);
  diag(r);
  if (getLayerState() != canopen::Layer::Off && r.bounded<canopen::LayerStatus::Unbounded>()) { // valid
    stat.summary(r.get(), r.reason());
    for (std::vector<std::pair<std::string, std::string>>::const_iterator it = r.values().begin();
      it != r.values().end(); ++it)
    {
      stat.add(it->first, it->second);
    }
    if (!r.bounded<canopen::LayerStatus::Warn>()) {
      canopen::LayerStatus s;
      recover(s);
    }
  } else {
    stat.summary(stat.WARN, "sync not initilized");
    canopen::LayerStatus s;
    init(s);
  }
}

SyncNode::SyncNode(const std::string & node_name)
: canopen::LayerStack("SyncNodeLayer"), Node(node_name)
{
  get_parameters();
  init_sync_layer();
  init_diagnostics();
}

SyncNode::SyncNode()
: SyncNode("canopen_sync_node")
{
}

void SyncNode::get_parameters()
{
  if (!get_parameter_or("overflow", sync_overflow_, 0)) {
    RCLCPP_WARN(this->get_logger(),
      "sync overflow was not specified, so overflow is disabled per default");
  }
  if (sync_overflow_ == 1 || sync_overflow_ > 240) {
    throw std::runtime_error("Sync overflow " + std::to_string(sync_overflow_) + "is invalid");
  }

  if (!get_parameter_or("device", can_device_, std::string("can0"))) {
    RCLCPP_WARN(this->get_logger(),
      "CAN device not specified, using can0");
  }

  // TODO(sam): figure out how to read int vectors
  std::vector<std::string> monitored_nodes_str;
  get_parameter_or_set("monitored_nodes", monitored_nodes_str, {});
  monitored_nodes_ = {};
  for (auto & monitored_node_str : monitored_nodes_str) {
    monitored_node_str = monitored_node_str.substr(monitored_node_str.find(':') + 1);
    monitored_nodes_.push_back(std::stoi(monitored_node_str));
  }

  std::vector<std::string> ignored_nodes_str;
  get_parameter_or_set("ignored_nodes", ignored_nodes_str, {});
  ignored_nodes_ = {};
  for (auto & ignored_node_str : ignored_nodes_str) {
    ignored_node_str = ignored_node_str.substr(ignored_node_str.find(':') + 1);
    ignored_nodes_.push_back(std::stoi(ignored_node_str));
  }

  if (!get_parameter_or("sync_id", sync_id_, 0x080)) {
    RCLCPP_WARN(this->get_logger(),
      "sync_id not specified, using 0x080");
  }

  if (!get_parameter_or("heartbeat_msg", heartbeat_msg_, std::string("77f#05"))) {
    RCLCPP_WARN(this->get_logger(),
      "heartbeat_msg not specified, using 77f#05");
  }

  if (!get_parameter_or("interval_ms", sync_ms_, 10)) {
    RCLCPP_WARN(this->get_logger(),
      "interval_ms not specified, using 10");
  }
  if (sync_ms_ <= 0) {
    throw std::runtime_error("sync interval " + std::to_string(sync_ms_) + " is invalid");
  }

  get_parameter_or_set("hardware_id", hardware_id_, std::string("none"));
}

void SyncNode::init_sync_layer()
{
  can::SocketCANDriverSharedPtr driver = std::make_shared<can::SocketCANDriver>();
  canopen::SyncProperties sync_properties(can::MsgHeader(sync_id_), sync_ms_, sync_overflow_);
  std::shared_ptr<canopen::BCMsync> sync = std::make_shared<canopen::BCMsync>(
    can_device_, driver, sync_properties);

  if (monitored_nodes_.size() != 0) {
    sync->setMonitored(monitored_nodes_);
  } else {
    // TODO(sam): figure out if this does anything...
    // can::Frame f = can::toframe(heartbeat_msg_);
    // if(f.isValid() && (f.id & ~canopen::BCMsync::ALL_NODES_MASK) == canopen::BCMsync::HEARTBEAT_ID){
    // monitored_nodes.push_back(f.id & canopen::BCMsync::ALL_NODES_MASK);
    sync->setIgnored(ignored_nodes_);
  }

  add(std::make_shared<canopen::CANLayer>(driver, can_device_, false));
  add(sync);
}

void SyncNode::init_diagnostics()
{
  diag_updater_.setHardwareID(hardware_id_);

  diag_updater_.add("sync", this, &SyncNode::report_diagnostics);
  diag_timer_ =
    this->create_wall_timer(1s, std::bind(&diagnostic_updater::Updater::update, &diag_updater_));
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SyncNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
