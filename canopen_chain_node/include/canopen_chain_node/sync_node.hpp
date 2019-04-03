#ifndef CANOPEN_CHAIN_NODE__SYNC_NODE_HPP
#define CANOPEN_CHAIN_NODE__SYNC_NODE_HPP

#include <fstream>
#include <chrono>
#include <canopen_master/bcm_sync.hpp>
#include <socketcan_interface/string.hpp>
#include <canopen_master/can_layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

using namespace std::chrono_literals;

class SyncNode : public rclcpp::Node, public canopen::LayerStack
{
public:
  explicit SyncNode(const std::string & node_name);
  SyncNode();

private:
  void get_parameters();
  void init_sync_layer();
  void init_diagnostics();
  void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);

  int sync_overflow_;
  std::string can_device_;
  std::vector<int> monitored_nodes_;
  std::vector<int> ignored_nodes_;
  int sync_id_;
  std::string heartbeat_msg_;
  int sync_ms_;
  std::string hardware_id_;

  diagnostic_updater::Updater diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

};

#endif // CANOPEN_CHAIN_NODE__SYNC_NODE_HPP
