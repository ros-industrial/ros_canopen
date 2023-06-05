#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <algorithm>
#include <atomic>
#include <bitset>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "canopen_402_driver/default_homing_mode.hpp"
#include "canopen_402_driver/mode_forward_helper.hpp"
#include "canopen_402_driver/profiled_position_mode.hpp"
#include "canopen_base_driver/lely_driver_bridge.hpp"

namespace ros2_canopen
{

typedef ModeForwardHelper<MotorBase::Profiled_Velocity, int32_t, 0x60FF, 0, 0> ProfiledVelocityMode;
typedef ModeForwardHelper<MotorBase::Profiled_Torque, int16_t, 0x6071, 0, 0> ProfiledTorqueMode;
typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Position, int32_t, 0x607A, 0, 0>
  CyclicSynchronousPositionMode;
typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Velocity, int32_t, 0x60FF, 0, 0>
  CyclicSynchronousVelocityMode;
typedef ModeForwardHelper<MotorBase::Cyclic_Synchronous_Torque, int16_t, 0x6071, 0, 0>
  CyclicSynchronousTorqueMode;
typedef ModeForwardHelper<
  MotorBase::Velocity, int16_t, 0x6042, 0,
  (1 << Command402::CW_Operation_mode_specific0) | (1 << Command402::CW_Operation_mode_specific1) |
    (1 << Command402::CW_Operation_mode_specific2)>
  VelocityMode;
typedef ModeForwardHelper<
  MotorBase::Interpolated_Position, int32_t, 0x60C1, 0x01,
  (1 << Command402::CW_Operation_mode_specific0)>
  InterpolatedPositionMode;

class Motor402 : public MotorBase
{
public:
  Motor402(
    std::shared_ptr<LelyDriverBridge> driver, ros2_canopen::State402::InternalState switching_state)
  : MotorBase(), switching_state_(switching_state), monitor_mode_(true), state_switch_timeout_(5)
  {
    this->driver = driver;
  }

  virtual bool setTarget(double val);
  virtual bool enterModeAndWait(uint16_t mode);
  virtual bool isModeSupported(uint16_t mode);
  virtual uint16_t getMode();
  bool readState();
  void handleDiag();
  /**
   * @brief Initialise the drive
   *
   * This function initialises the drive. This means, it first
   * attempts to bring the device to operational state (CIA402)
   * and then executes the chosen homing method.
   *
   */
  bool handleInit();
  /**
   * @brief Read objects of the drive
   *
   * This function should be called regularly. It reads the status word
   * from the device and translates it into the devices state.
   *
   */
  void handleRead();
  /**
   * @brief Writes objects to the drive
   *
   * This function should be called regularly. It writes the new command
   * word to the drive
   *
   */
  void handleWrite();
  /**
   * @brief Shutdowns the drive
   *
   * This function shuts down the drive by bringing it into
   * SwitchOn disabled state.
   *
   */
  bool handleShutdown();
  /**
   * @brief Executes a quickstop
   *
   * The function executes a quickstop.
   *
   */
  bool handleHalt();

  /**
   * @brief Recovers the device from fault
   *
   * This function tries to reset faults and
   * put the device back to operational state.
   *
   */
  bool handleRecover();

  /**
   * @brief Register a new operation mode for the drive
   *
   * This function will register an operation mode for the drive.
   * It will check if the mode is supported by the drive by reading
   * 0x6508 object.
   *
   * @tparam T
   * @tparam Args
   * @param mode
   * @param args
   * @return true
   * @return false
   */
  template <typename T, typename... Args>
  bool registerMode(uint16_t mode, Args &&... args)
  {
    return mode_allocators_
      .insert(std::make_pair(
        mode,
        [args..., mode, this]()
        {
          if (isModeSupportedByDevice(mode)) registerMode(mode, ModeSharedPtr(new T(args...)));
        }))
      .second;
  }

  /**
   * @brief Tries to register the standard operation modes defined in cia402
   *
   */
  virtual void registerDefaultModes()
  {
    registerMode<ProfiledPositionMode>(MotorBase::Profiled_Position, driver);
    registerMode<VelocityMode>(MotorBase::Velocity, driver);
    registerMode<ProfiledVelocityMode>(MotorBase::Profiled_Velocity, driver);
    registerMode<ProfiledTorqueMode>(MotorBase::Profiled_Torque, driver);
    registerMode<DefaultHomingMode>(MotorBase::Homing, driver);
    registerMode<InterpolatedPositionMode>(MotorBase::Interpolated_Position, driver);
    registerMode<CyclicSynchronousPositionMode>(MotorBase::Cyclic_Synchronous_Position, driver);
    registerMode<CyclicSynchronousVelocityMode>(MotorBase::Cyclic_Synchronous_Velocity, driver);
    registerMode<CyclicSynchronousTorqueMode>(MotorBase::Cyclic_Synchronous_Torque, driver);
  }

  double get_speed() const { return (double)this->driver->universal_get_value<int32_t>(0x606C, 0); }
  double get_position() const
  {
    return (double)this->driver->universal_get_value<int32_t>(0x6064, 0);
  }

private:
  virtual bool isModeSupportedByDevice(uint16_t mode);
  void registerMode(uint16_t id, const ModeSharedPtr & m);

  ModeSharedPtr allocMode(uint16_t mode);

  bool switchMode(uint16_t mode);
  bool switchState(const State402::InternalState & target);

  std::atomic<uint16_t> status_word_;
  uint16_t control_word_;
  std::mutex cw_mutex_;
  std::atomic<bool> start_fault_reset_;
  std::atomic<State402::InternalState> target_state_;

  State402 state_handler_;

  std::mutex map_mutex_;
  std::unordered_map<uint16_t, ModeSharedPtr> modes_;
  typedef std::function<void()> AllocFuncType;
  std::unordered_map<uint16_t, AllocFuncType> mode_allocators_;

  ModeSharedPtr selected_mode_;
  uint16_t mode_id_;
  std::condition_variable mode_cond_;
  std::mutex mode_mutex_;
  const State402::InternalState switching_state_;
  const bool monitor_mode_;
  const std::chrono::seconds state_switch_timeout_;

  std::shared_ptr<LelyDriverBridge> driver;
  const uint16_t status_word_entry_index = 0x6041;
  const uint16_t control_word_entry_index = 0x6040;
  const uint16_t op_mode_display_index = 0x6061;
  const uint16_t op_mode_index = 0x6060;
  const uint16_t supported_drive_modes_index = 0x6502;
};

}  // namespace ros2_canopen

#endif
