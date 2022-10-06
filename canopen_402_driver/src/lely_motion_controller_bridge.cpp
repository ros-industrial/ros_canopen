#include "canopen_402_driver/lely_motion_controller_bridge.hpp"

using namespace ros2_canopen;

LelyMotionControllerBridge::LelyMotionControllerBridge(ev_exec_t *exec, canopen::AsyncMaster &master, uint8_t id, std::string name)
    : LelyDriverBridge(exec, master, id, name)
{
    sync = true;
}
