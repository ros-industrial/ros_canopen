#include <canopen_chain_node/ros_chain.hpp>

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

using namespace can;

namespace canopen
{

template<typename Tpub, int dt>
PublishFuncType RosChain::createPublisher(
  const std::string & name, ObjectStorageSharedPtr storage,
  const std::string & key, const bool force)
{
  using data_type = typename ObjectStorage::DataType<dt>::type;
  using entry_type = ObjectStorage::Entry<data_type>;

  entry_type entry = storage->entry<data_type>(key);
  if (!entry.valid()) {
    return 0;
  }

  auto pub = this->create_publisher<Tpub>(name);

  typedef const data_type (entry_type::* getter_type)(void);
  const getter_type getter =
    force ? static_cast<getter_type>(&entry_type::get) :
    static_cast<getter_type>(&entry_type::get_cached);

  return [force, pub, entry, getter]() mutable {
           Tpub msg;
           msg.data = (const typename Tpub::_data_type &)(entry.*getter)();
           pub->publish(msg);
         };
}

PublishFuncType RosChain::createPublishFunc(
  const std::string & name, canopen::NodeSharedPtr node,
  const std::string & key, bool force)
{
  ObjectStorageSharedPtr storage = node->getStorage();

  switch (ObjectDict::DataTypes(storage->dict_->get(key)->data_type)) {

    case ObjectDict::DEFTYPE_INTEGER8:
      return createPublisher<std_msgs::msg::Int8, ObjectDict::DEFTYPE_INTEGER8>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_INTEGER16:
      return createPublisher<std_msgs::msg::Int16, ObjectDict::DEFTYPE_INTEGER16>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_INTEGER32:
      return createPublisher<std_msgs::msg::Int32, ObjectDict::DEFTYPE_INTEGER32>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_INTEGER64:
      return createPublisher<std_msgs::msg::Int64, ObjectDict::DEFTYPE_INTEGER64>(
        name, storage, key, force);

    case ObjectDict::DEFTYPE_UNSIGNED8:
      return createPublisher<std_msgs::msg::UInt8, ObjectDict::DEFTYPE_UNSIGNED8>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_UNSIGNED16:
      return createPublisher<std_msgs::msg::UInt16, ObjectDict::DEFTYPE_UNSIGNED16>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_UNSIGNED32:
      return createPublisher<std_msgs::msg::UInt32, ObjectDict::DEFTYPE_UNSIGNED32>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_UNSIGNED64:
      return createPublisher<std_msgs::msg::UInt64, ObjectDict::DEFTYPE_UNSIGNED64>(
        name, storage, key, force);

    case ObjectDict::DEFTYPE_REAL32:
      return createPublisher<std_msgs::msg::Float32, ObjectDict::DEFTYPE_REAL32>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_REAL64:
      return createPublisher<std_msgs::msg::Float64, ObjectDict::DEFTYPE_REAL64>(
        name, storage, key, force);

    case ObjectDict::DEFTYPE_VISIBLE_STRING:
      return createPublisher<std_msgs::msg::String, ObjectDict::DEFTYPE_VISIBLE_STRING>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_OCTET_STRING:
      return createPublisher<std_msgs::msg::String, ObjectDict::DEFTYPE_DOMAIN>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_UNICODE_STRING:
      return createPublisher<std_msgs::msg::String, ObjectDict::DEFTYPE_UNICODE_STRING>(
        name, storage, key, force);
    case ObjectDict::DEFTYPE_DOMAIN:
      return createPublisher<std_msgs::msg::String, ObjectDict::DEFTYPE_DOMAIN>(
        name, storage, key, force);

    default: return 0;
  }
}

void RosChain::logState(const can::State & s)
{
  can::DriverInterfaceSharedPtr interface = interface_;
  std::string msg;
  if (interface && !interface->translateError(s.internal_error, msg)) {
    msg = "Undefined";
  }

  // TODO(sam): get this log working
  // RCLCPP_INFO(this->get_logger(),
  //   "Current state : %s device error: %s internal_error: %d (%s)",
  //   s.driver_state, s.error_code.message(),
  //   s.internal_error, msg.c_str());
}

void RosChain::run()
{
  running_ = true;
  time_point abs_time = boost::chrono::high_resolution_clock::now();
  while (running_) {
    LayerStatus s;
    try {
      read(s);
      write(s);
      // TODO(sam): Throttle logs
      if (!s.bounded<LayerStatus::Warn>()) {
        RCLCPP_ERROR(this->get_logger(), s.reason());
      } else if (!s.bounded<LayerStatus::Ok>()) {
        RCLCPP_WARN(this->get_logger(), s.reason());
      }
    } catch (const canopen::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), boost::diagnostic_information(e));
    }
    if (!sync_) {
      abs_time += update_duration_;
      boost::this_thread::sleep_until(abs_time);
    }
  }
}

void RosChain::handle_init(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Initializing XXX");

  boost::mutex::scoped_lock lock(mutex_);
  if (getLayerState() > Off) {
    response->success = true;
    response->message = "already initialized";
  }
  thread_.reset(new boost::thread(&RosChain::run, this));
  LayerReport status;
  try {
    init(status);
    response->success = status.bounded<LayerStatus::Ok>();
    response->message = status.reason();
    if (!status.bounded<LayerStatus::Warn>()) {
      diag(status);
      response->message = status.reason();
      return;
    } else {
      heartbeat_timer_.restart();
      return;
    }
  } catch (const std::exception & e) {
    std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), info);
    response->message = info;
    status.error(response->message);
  } catch (...) {
    response->message = "Unknown exception";
    status.error(response->message);
  }

  response->success = false;
  shutdown(status);
}

void RosChain::handle_recover(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Recovering XXX");
  boost::mutex::scoped_lock lock(mutex_);
  response->success = false;

  if (getLayerState() > Init) {
    LayerReport status;
    try {
      if (!reset_errors_before_recover_ ||
        emcy_handlers_->callFunc<LayerStatus::Warn>(&EMCYHandler::resetErrors, status))
      {
        recover(status);
      }
      if (!status.bounded<LayerStatus::Warn>()) {
        diag(status);
      }
      response->success = status.bounded<LayerStatus::Warn>();
      response->message = status.reason();
    } catch (const std::exception & e) {
      std::string info = boost::diagnostic_information(e);
      RCLCPP_ERROR(this->get_logger(), info);
      response->message = info;
    } catch (...) {
      response->message = "Unknown exception";
    }
  } else {
    response->message = "not running";
  }
}

void RosChain::handleWrite(LayerStatus & status, const LayerState & current_state)
{
  LayerStack::handleWrite(status, current_state);
  if (current_state > Shutdown) {
    for (const PublishFuncType & func: publishers_) {
      func();
    }
  }
}

void RosChain::handleShutdown(LayerStatus & status)
{
  boost::mutex::scoped_lock lock(diag_mutex_);
  heartbeat_timer_.stop();
  LayerStack::handleShutdown(status);
  if (running_) {
    running_ = false;
    thread_->interrupt();
    thread_->join();
    thread_.reset();
  }
}

void RosChain::handle_shutdown(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Shuting down XXX");
  boost::mutex::scoped_lock lock(mutex_);
  response->success = true;
  if (getLayerState() > Init) {
    LayerStatus s;
    halt(s);
    shutdown(s);
  } else {
    response->message = "not running";
  }
}

void RosChain::handle_halt(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Halting down XXX");
  boost::mutex::scoped_lock lock(mutex_);
  response->success = true;
  if (getLayerState() > Init) {
    LayerStatus s;
    halt(s);
  } else {
    response->message = "not running";
  }
}

void RosChain::handle_get_object(
  const std::shared_ptr<canopen_msgs::srv::GetObject::Request> request,
  std::shared_ptr<canopen_msgs::srv::GetObject::Response> response)
{
  std::map<std::string, canopen::NodeSharedPtr>::iterator it =
    nodes_lookup_.find(request->node);
  if (it == nodes_lookup_.end()) {
    response->message = "node not found";
  } else {
    try {
      response->value = it->second->getStorage()->getStringReader(
        canopen::ObjectDict::Key(request->object), request->cached)();
      response->success = true;
    } catch (std::exception & e) {
      response->message = boost::diagnostic_information(e);
    }
  }
}

void RosChain::handle_set_object(
  const std::shared_ptr<canopen_msgs::srv::SetObject::Request> request,
  std::shared_ptr<canopen_msgs::srv::SetObject::Response> response)
{
  std::map<std::string, canopen::NodeSharedPtr>::iterator it =
    nodes_lookup_.find(request->node);
  if (it == nodes_lookup_.end()) {
    response->message = "node not found";
  } else {
    try {
      it->second->getStorage()->getStringWriter(
        canopen::ObjectDict::Key(request->object), request->cached)(request->value);
      response->success = true;
    } catch (std::exception & e) {
      response->message = boost::diagnostic_information(e);
    }
  }
}

bool RosChain::setup_bus()
{
  std::string can_device;
  std::string driver_plugin;
  std::string master_alloc;
  bool loopback;

  if (!get_parameter_or("bus.device", can_device, std::string("can0"))) {
    RCLCPP_WARN(this->get_logger(),
      "CAN device not specified, using can0");
  }
  RCLCPP_INFO(this->get_logger(), "can device: %s", can_device.c_str());

  if (!get_parameter_or("bus.driver_plugin", driver_plugin,
    std::string("socketcan_interface/SocketCANInterface")))
  {
    RCLCPP_WARN(this->get_logger(),
      "driver_plugin not specified, using can::SocketCANInterface");
  }
  RCLCPP_INFO(this->get_logger(), "driver_plugin: %s", driver_plugin.c_str());

  if (!get_parameter_or("bus.loopback", loopback, false)) {
    RCLCPP_WARN(this->get_logger(),
      "loopback not specified, socket will not loop back messages");
  }
  RCLCPP_INFO(this->get_logger(), "loopback: %d", loopback);

  try {
    interface_ = driver_loader_.createInstance(driver_plugin);
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(this->get_logger(), ex.what());
    return false;
  }

  state_listener_ = interface_->createStateListenerM(this, &RosChain::logState);

  if (!get_parameter_or("bus.master_allocator", master_alloc,
    std::string("canopen_master/SimpleMasterAllocator")))
  {
    RCLCPP_WARN(this->get_logger(),
      "master_allocator not specified, using canopen_master/SimpleMasterAllocator");
  }
  RCLCPP_INFO(this->get_logger(), "master_allocator: %s", master_alloc.c_str());

  try {
    master_ = master_allocator_.allocateInstance(master_alloc, can_device, interface_);
  } catch (const std::exception & e) {
    std::string info = boost::diagnostic_information(e);
    RCLCPP_ERROR(this->get_logger(), info);
    return false;
  }

  if (!master_) {
    RCLCPP_ERROR(this->get_logger(), "Could not allocate master");
    return false;
  }

  add(std::make_shared<CANLayer>(interface_, can_device, loopback));

  return true;
}

bool RosChain::setup_sync()
{
  int sync_ms;
  int sync_overflow;

  if (!get_parameter_or("sync.interval_ms", sync_ms, 0)) {
    RCLCPP_WARN(this->get_logger(),
      "sync interval was not specified, so sync is disabled per default");
  }
  RCLCPP_INFO(this->get_logger(), "interval_ms: %d", sync_ms);
  if (sync_ms < 0) {
    RCLCPP_ERROR(this->get_logger(),
      "sync interval " + std::to_string(sync_ms) + " is invalid");
    return false;
  }

  int update_ms = sync_ms;
  if (sync_ms == 0) {
    get_parameter_or("sync.update_ms", update_ms, 10);
    RCLCPP_INFO(this->get_logger(), "update_ms: %d", update_ms);
  }
  if (update_ms == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "update interval " + std::to_string(update_ms) + " is invalid");
    return false;
  } else {
    update_duration_ = boost::chrono::milliseconds(update_ms);
  }

  if (sync_ms) {
    if (!get_parameter_or("sync.overflow", sync_overflow, 0)) {
      RCLCPP_WARN(this->get_logger(),
        "sync overflow was not specified, so overflow is disabled per default");
    }
    thread_.reset(new boost::thread(&RosChain::run, this));
    LayerReport status;
    try{
        init(status);
        res.success = status.bounded<LayerStatus::Ok>();
        res.message = status.reason();
        if(!status.bounded<LayerStatus::Warn>()){
            diag(status);
            res.message = status.reason();
        }else{
            heartbeat_timer_.restart();
            return true;
        }
    }
    catch( const std::exception &e){
        std::string info = boost::diagnostic_information(e);
        ROS_ERROR_STREAM(info);
        res.message = info;
        status.error(res.message);
    }
    catch(...){
        res.message = "Unknown exception";
        status.error(res.message);
    }
    add(sync_);
  }
  return true;
}

bool RosChain::setup_heartbeat()
{
  std::string msg;
  double rate = 0;

  bool got_any = false;

  if (get_parameter_or("heartbeat.msg", msg, std::string("77f#05"))) {
    got_any = true;
  } else {
    RCLCPP_WARN(this->get_logger(),
      "heartbeat_msg not specified, using 77f#05");
  }
  RCLCPP_INFO(this->get_logger(), "heartbeat_msg: %s", msg.c_str());

  if (get_parameter_or("heartbeat.rate", rate, 10.0)) {
    got_any = true;
  } else {
    RCLCPP_WARN(this->get_logger(),
      "heartbeat_rate not specified, using 10.0");
  }
  RCLCPP_INFO(this->get_logger(), "heartbeat_rate: %f", rate);

  if (!got_any) {
    RCLCPP_INFO(this->get_logger(), "not producing heartbeat!");
    return true; // nothing todo
  }

  if (rate <= 0) {
    RCLCPP_ERROR(this->get_logger(),
      "rate " + std::to_string(rate) + " is invalid");
    return false;
  }

  hb_sender_.frame = can::toframe(msg);

  if (!hb_sender_.frame.isValid()) {
    // ROS_ERROR_STREAM("Message '"<< msg << "' is invalid");
    RCLCPP_ERROR(this->get_logger(),
      "heartbeat_msg " + msg + " is invalid");
    return false;
  }

  hb_sender_.interface = interface_;

    res.success = false;
    shutdown(status);

    return true;
}

// std::pair<std::string, bool> parseObjectName(std::string obj_name){
//     size_t pos = obj_name.find('!');
//     bool force = pos != std::string::npos;
//     if(force) obj_name.erase(pos);
//     return std::make_pair(obj_name, force);
// }

// bool addLoggerEntries(XmlRpc::XmlRpcValue merged, const std::string param, uint8_t level, Logger &logger){
//     if(merged.hasMember(param)){
//         try{
//             XmlRpc::XmlRpcValue objs = merged[param];
//             for(int i = 0; i < objs.size(); ++i){
//                 std::pair<std::string, bool> obj_name = parseObjectName(objs[i]);
//
//                 if(!logger.add(level, obj_name.first, obj_name.second)){
//                     ROS_ERROR_STREAM("Could not create logger for '" << obj_name.first << "'");
//                     return false;
//                 }
//             }
//         }
//         catch(...){
//             ROS_ERROR_STREAM("Could not parse " << param << " parameter");
//             return false;
//         }
//     }
//     return true;
// }

bool RosChain::setup_nodes()
{
  nodes_.reset(new canopen::LayerGroupNoDiag<canopen::Node>("301 layer"));
  add(nodes_);

  emcy_handlers_.reset(new canopen::LayerGroupNoDiag<canopen::EMCYHandler>("EMCY layer"));

  std::string default_eds_pkg;
  if (!get_parameter_or("defaults.eds_pkg", default_eds_pkg, std::string(""))) {
    RCLCPP_WARN(this->get_logger(),
      "default eds_pkg not specified");
  }
  RCLCPP_INFO(this->get_logger(), "default eds_pkg: %s", default_eds_pkg.c_str());

  std::string default_eds_file;
  if (!get_parameter_or("defaults.eds_file", default_eds_file, std::string(""))) {
    RCLCPP_WARN(this->get_logger(),
      "default eds_file not specified");
  }
  RCLCPP_INFO(this->get_logger(), "default eds_file: %s", default_eds_file.c_str());

  std::vector<std::string> nodes;
  if (!get_parameter_or("nodes", nodes, {})) {
    RCLCPP_ERROR(this->get_logger(),
      "no nodes were spiecified!");
    return false;
  }

  for (auto & node_name : nodes) {
    int node_id;
    if (!get_parameter_or(node_name + ".id", node_id, 4)) {
      RCLCPP_ERROR(this->get_logger(),
        "no node id was spiecified for " + node_name);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), node_name + " node id: %d", node_id);

    std::string eds_file;
    if (!get_parameter_or(node_name + ".eds_file", eds_file, default_eds_file)) {
      RCLCPP_WARN(this->get_logger(),
        "eds_file not specified for " + node_name + ", using " + eds_file);
    }
    RCLCPP_INFO(this->get_logger(), node_name + " eds_file: %s", default_eds_file.c_str());

    std::string eds_pkg;
    if (!get_parameter_or(node_name + ".eds_pkg", eds_pkg, default_eds_pkg)) {
      RCLCPP_WARN(this->get_logger(),
        "eds_pkg not specified for " + node_name + ", using " + eds_pkg);
    } else {
      RCLCPP_INFO(this->get_logger(), node_name + " eds_pkg: %s", eds_pkg.c_str());
    }

    std::string eds_pkg_share_directory = "";
    std::string eds_full_path = eds_file;
    if (!eds_pkg.empty()) {
      try {
        eds_pkg_share_directory =
          ament_index_cpp::get_package_share_directory(eds_pkg);
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(),
          "eds_pkg share directory not found!");
        return false;
      }

      eds_full_path =
        (boost::filesystem::path(eds_pkg_share_directory) / eds_file).make_preferred().native();
    }
    RCLCPP_INFO(this->get_logger(), node_name + " eds full path: %s",
      eds_full_path.c_str());

    ObjectDict::Overlay overlay;
    // TODO(sam): parse overlay

    // if(merged.hasMember("dcf_overlay")){
    //     XmlRpc::XmlRpcValue dcf_overlay = merged["dcf_overlay"];
    //     if(dcf_overlay.getType() != XmlRpc::XmlRpcValue::TypeStruct){
    //         ROS_ERROR_STREAM("dcf_overlay is no struct");
    //         return false;
    //     }
    //     for(XmlRpc::XmlRpcValue::iterator ito = dcf_overlay.begin(); ito!= dcf_overlay.end(); ++ito){
    //         if(ito->second.getType() != XmlRpc::XmlRpcValue::TypeString){
    //             ROS_ERROR_STREAM("dcf_overlay '" << ito->first << "' must be string");
    //             return false;
    //         }
    //         overlay.push_back(ObjectDict::Overlay::value_type(ito->first, ito->second));
    //     }
    // }

    auto exists =
      [this](const std::string & name) -> bool
      {
        struct stat buffer;
        return stat(name.c_str(), &buffer) == 0;
      };

    if (!exists(eds_full_path)) {
      RCLCPP_ERROR(this->get_logger(), node_name + " eds file: %s does not exist!",
        eds_full_path.c_str());
      return false;
    }

    ObjectDictSharedPtr dict = ObjectDict::fromFile(eds_full_path, overlay);
    if (!dict) {
      RCLCPP_ERROR(this->get_logger(),
        "EDS '" + eds_file + "' could not be parsed");
      return false;
    }

    canopen::NodeSharedPtr node = std::make_shared<canopen::Node>(interface_, dict, node_id, sync_);
    LoggerSharedPtr logger = std::make_shared<Logger>(node);

    // TODO(sam): figure out what this is supposed to do...
    if (!nodeAdded(node, logger)) {
      return false;
    }

    // if(!addLoggerEntries(merged, "log", diagnostic_updater::DiagnosticStatusWrapper::OK, *logger)) return false;
    // if(!addLoggerEntries(merged, "log_warn", diagnostic_updater::DiagnosticStatusWrapper::WARN, *logger)) return false;
    // if(!addLoggerEntries(merged, "log_error", diagnostic_updater::DiagnosticStatusWrapper::ERROR, *logger)) return false;

    loggers_.push_back(logger);
    diag_updater_.add(node_name, std::bind(&Logger::log, logger, std::placeholders::_1));


    std::vector<std::string> publish;
    if (!get_parameter_or(node_name + ".publish", publish, {})) {
      RCLCPP_INFO(this->get_logger(),
        "no objects to be published were spiecified");
    }

    for (auto & object : publish) {
      std::string object_name = object.substr(object.find(":") + 1);
      bool force = false;
      if (object_name.back() == '!') {
        force = true;
        object_name.pop_back();
      }

      RCLCPP_INFO(this->get_logger(),
        "%s object to be published: %s, force: %d", node_name.c_str(),
        object_name.c_str(), force);

      PublishFuncType pub =
        createPublishFunc(node_name + "_" + object_name,
          node, object_name, force);

      if (!pub) {
        RCLCPP_ERROR(this->get_logger(),
          "%s could not create publisher for object: '%s'", node_name.c_str(),
          object_name.c_str());
        return false;
      }

      publishers_.push_back(pub);
    }

    nodes_->add(node);
    nodes_lookup_.insert(std::make_pair(node_name, node));
    std::shared_ptr<canopen::EMCYHandler> emcy =
      std::make_shared<canopen::EMCYHandler>(interface_, node->getStorage());
    emcy_handlers_->add(emcy);
    logger->add(emcy);
    nodes_lookup_.insert(std::make_pair(node_name, node));
  }

  return true;
}

bool RosChain::nodeAdded(const canopen::NodeSharedPtr & node, const LoggerSharedPtr & logger)
{
  return true;
}

void RosChain::report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  boost::mutex::scoped_lock lock(diag_mutex_);
  LayerReport r;
  if (getLayerState() == Off) {
    stat.summary(stat.WARN, "Not initailized");
  } else if (!running_) {
    stat.summary(stat.ERROR, "Thread is not running");
  } else {
    diag(r);
    if (r.bounded<LayerStatus::Unbounded>()) { // valid
      stat.summary(r.get(), r.reason());
      for (std::vector<std::pair<std::string, std::string>>::const_iterator it =
        r.values().begin(); it != r.values().end(); ++it)
      {
        stat.add(it->first, it->second);
      }
    }
  }
}

void RosChain::report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
    boost::mutex::scoped_lock lock(diag_mutex_);
    LayerReport r;
    if(getLayerState() == Off){
        stat.summary(stat.WARN,"Not initialized");
    }else if(!running_){
        stat.summary(stat.ERROR,"Thread is not running");
    }else{
        diag(r);
        if(r.bounded<LayerStatus::Unbounded>()){ // valid
            stat.summary(r.get(), r.reason());
            for(std::vector<std::pair<std::string, std::string> >::const_iterator it = r.values().begin(); it != r.values().end(); ++it){
                stat.add(it->first, it->second);
            }
        }
    }
}

RosChain::RosChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv)
: LayerStack("ROS stack"),driver_loader_("socketcan_interface", "can::DriverInterface"),
  master_allocator_("canopen_master", "canopen::Master::Allocator"),
  nh_(nh), nh_priv_(nh_priv),
  diag_updater_(nh_,nh_priv_),
  running_(false),
  reset_errors_before_recover_(false)
{
}

bool RosChain::setup()
{
  boost::mutex::scoped_lock lock(mutex_);
  bool okay = setup_chain();
  if (okay) {add(emcy_handlers_);}
  return okay;
}

bool RosChain::setup_chain()
{
  std::string hardware_id;
  get_parameter_or_set("hardware_id", hardware_id, std::string("none"));
  get_parameter_or_set("reset_errors_before_recover",
    reset_errors_before_recover_, false);

    diag_timer_ = nh_.createTimer(ros::Duration(diag_updater_.getPeriod()/2.0),std::bind(&diagnostic_updater::Updater::update, &diag_updater_));

    ros::NodeHandle nh_driver(nh_, "driver");

  srv_init_ = create_service<std_srvs::srv::Trigger>(
    "init", std::bind(
      &RosChain::handle_init, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_recover_ = create_service<std_srvs::srv::Trigger>(
    "recover", std::bind(
      &RosChain::handle_recover, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_halt_ = create_service<std_srvs::srv::Trigger>(
    "halt", std::bind(
      &RosChain::handle_halt, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_shutdown_ = create_service<std_srvs::srv::Trigger>(
    "shutdown", std::bind(
      &RosChain::handle_shutdown, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_get_object_ = create_service<canopen_msgs::srv::GetObject>(
    "get_object", std::bind(
      &RosChain::handle_get_object, this,
      std::placeholders::_1, std::placeholders::_2));

  srv_set_object_ = create_service<canopen_msgs::srv::SetObject>(
    "set_object", std::bind(
      &RosChain::handle_set_object, this,
      std::placeholders::_1, std::placeholders::_2));

  return setup_bus() && setup_sync() && setup_heartbeat() && setup_nodes();
}

RosChain::~RosChain(){
    try{
        LayerStatus s;
        halt(s);
        shutdown(s);
    }catch(...){ ROS_ERROR("CATCH"); }
}

}
