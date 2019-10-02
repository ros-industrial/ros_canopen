// Copyright (c) 2016-2019, Mathias Lüdtke, Samuel Lindgren
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef CANOPEN_CHAIN_NODE__SYNC_NODE_HPP_
#define CANOPEN_CHAIN_NODE__SYNC_NODE_HPP_

#include <canopen_master/bcm_sync.hpp>
#include <socketcan_interface/string.hpp>
#include <canopen_master/can_layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <fstream>
#include <chrono>
#include <string>
#include <vector>

using namespace std::literals::chrono_literals;

class SyncNode
: public rclcpp::Node,
  public canopen::LayerStack
{
public:
  explicit SyncNode(const std::string & node_name);
  SyncNode();

private:
  void get_parameters();
  void init_sync_layer();
  void init_diagnostics();
  void report_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  int sync_overflow_;
  std::string can_device_;
  std::vector<int64_t> monitored_nodes_;
  std::vector<int64_t> ignored_nodes_;
  int sync_id_;
  std::string heartbeat_msg_;
  int sync_ms_;
  std::string hardware_id_;

  diagnostic_updater::Updater diag_updater_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

#endif  // CANOPEN_CHAIN_NODE__SYNC_NODE_HPP_
