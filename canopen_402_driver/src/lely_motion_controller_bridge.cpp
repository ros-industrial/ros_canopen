#include "canopen_402_driver/lely_motion_controller_bridge.hpp"

using namespace ros2_canopen;

LelyMotionControllerBridge::LelyMotionControllerBridge(
  ev_exec_t * exec, canopen::AsyncMaster & master, uint8_t id, std::string name, std::string eds,
  std::string bin)
: LelyDriverBridge(exec, master, id, name, eds, bin)
{
  sync = true;
}
