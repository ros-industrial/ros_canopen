#include "canopen_402_driver/motor.hpp"
using namespace ros2_canopen;

bool Motor402::setTarget(double val)
{
  if (state_handler_.getState() == State402::Operation_Enable)
  {
    std::scoped_lock lock(mode_mutex_);
    return selected_mode_ && selected_mode_->setTarget(val);
  }
  return false;
}
bool Motor402::isModeSupported(uint16_t mode)
{
  return mode != MotorBase::Homing && allocMode(mode);
}

bool Motor402::enterModeAndWait(uint16_t mode)
{
  bool okay = mode != MotorBase::Homing && switchMode(mode);
  return okay;
}

uint16_t Motor402::getMode()
{
  std::scoped_lock lock(mode_mutex_);
  return selected_mode_ ? selected_mode_->mode_id_ : (uint16_t)MotorBase::No_Mode;
}

bool Motor402::isModeSupportedByDevice(uint16_t mode)
{
  uint32_t supported_modes =
    driver->universal_get_value<uint32_t>(supported_drive_modes_index, 0x0);
  bool supported = supported_modes & (1 << (mode - 1));
  bool below_max = mode <= 32;
  bool above_min = mode > 0;
  return below_max && above_min && supported;
}
void Motor402::registerMode(uint16_t id, const ModeSharedPtr & m)
{
  std::scoped_lock map_lock(map_mutex_);
  if (m && m->mode_id_ == id) modes_.insert(std::make_pair(id, m));
}

ModeSharedPtr Motor402::allocMode(uint16_t mode)
{
  ModeSharedPtr res;
  if (isModeSupportedByDevice(mode))
  {
    std::scoped_lock map_lock(map_mutex_);
    std::unordered_map<uint16_t, ModeSharedPtr>::iterator it = modes_.find(mode);
    if (it != modes_.end())
    {
      res = it->second;
    }
  }
  return res;
}

bool Motor402::switchMode(uint16_t mode)
{
  if (mode == MotorBase::No_Mode)
  {
    std::scoped_lock lock(mode_mutex_);
    selected_mode_.reset();
    try
    {  // try to set mode
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);
    }
    catch (...)
    {
    }
    return true;
  }

  ModeSharedPtr next_mode = allocMode(mode);
  if (!next_mode)
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode is not supported.");
    return false;
  }

  if (!next_mode->start())
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Could not  start mode.");
    return false;
  }

  {  // disable mode handler
    std::scoped_lock lock(mode_mutex_);

    if (mode_id_ == mode && selected_mode_ && selected_mode_->mode_id_ == mode)
    {
      // nothing to do
      return true;
    }

    selected_mode_.reset();
  }

  if (!switchState(switching_state_)) return false;

  driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode);

  bool okay = false;

  {  // wait for switch
    std::unique_lock lock(mode_mutex_);

    std::chrono::steady_clock::time_point abstime =
      std::chrono::steady_clock::now() + std::chrono::seconds(5);
    if (monitor_mode_)
    {
      while (mode_id_ != mode && mode_cond_.wait_until(lock, abstime) == std::cv_status::no_timeout)
      {
      }
    }
    else
    {
      while (mode_id_ != mode && std::chrono::steady_clock::now() < abstime)
      {
        lock.unlock();                                                    // unlock inside loop
        driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);  // poll
        std::this_thread::sleep_for(std::chrono::milliseconds(20));       // wait some time
        lock.lock();
      }
    }

    if (mode_id_ == mode)
    {
      selected_mode_ = next_mode;
      okay = true;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode switch timed out.");
      driver->universal_set_value<int8_t>(op_mode_index, 0x0, mode_id_);
    }
  }

  if (!switchState(State402::Operation_Enable)) return false;

  return okay;
}

bool Motor402::switchState(const State402::InternalState & target)
{
  std::chrono::steady_clock::time_point abstime =
    std::chrono::steady_clock::now() + state_switch_timeout_;
  State402::InternalState state = state_handler_.getState();
  target_state_ = target;
  while (state != target_state_)
  {
    std::unique_lock lock(cw_mutex_);
    State402::InternalState next = State402::Unknown;
    bool success = Command402::setTransition(control_word_, state, target_state_, &next);
    if (!success)
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Could not set transition.");
      return false;
    }
    lock.unlock();
    if (state != next && !state_handler_.waitForNewState(abstime, state))
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Transition timed out.");
      return false;
    }
  }
  return state == target;
}

bool Motor402::readState()
{
  uint16_t old_sw, sw = driver->universal_get_value<uint16_t>(
                     status_word_entry_index, 0x0);  // TODO: added error handling
  old_sw = status_word_.exchange(sw);

  state_handler_.read(sw);

  std::unique_lock lock(mode_mutex_);
  uint16_t new_mode;
  new_mode = driver->universal_get_value<int8_t>(op_mode_display_index, 0x0);
  // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode %hhi",new_mode);

  if (selected_mode_ && selected_mode_->mode_id_ == new_mode)
  {
    if (!selected_mode_->read(sw))
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode handler has error.");
    }
  }
  if (new_mode != mode_id_)
  {
    mode_id_ = new_mode;
    mode_cond_.notify_all();
  }
  if (selected_mode_ && selected_mode_->mode_id_ != new_mode)
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Mode does not match.");
  }
  if (sw & (1 << State402::SW_Internal_limit))
  {
    if (old_sw & (1 << State402::SW_Internal_limit))
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Internal limit active");
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Internal limit active");
    }
  }

  return true;
}
void Motor402::handleRead() { readState(); }
void Motor402::handleWrite()
{
  std::scoped_lock lock(cw_mutex_);
  control_word_ |= (1 << Command402::CW_Halt);
  if (state_handler_.getState() == State402::Operation_Enable)
  {
    std::scoped_lock lock(mode_mutex_);
    Mode::OpModeAccesser cwa(control_word_);
    bool okay = false;
    if (selected_mode_ && selected_mode_->mode_id_ == mode_id_)
    {
      okay = selected_mode_->write(cwa);
    }
    else
    {
      cwa = 0;
    }
    if (okay)
    {
      control_word_ &= ~(1 << Command402::CW_Halt);
    }
  }
  if (start_fault_reset_.exchange(false))
  {
    RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Fault reset");
    this->driver->universal_set_value<uint16_t>(
      control_word_entry_index, 0x0, control_word_ & ~(1 << Command402::CW_Fault_Reset));
  }
  else
  {
    // RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Control Word %s",
    // std::bitset<16>{control_word_}.to_string());
    this->driver->universal_set_value<uint16_t>(control_word_entry_index, 0x0, control_word_);
  }
}
void Motor402::handleDiag()
{
  uint16_t sw = status_word_;
  State402::InternalState state = state_handler_.getState();

  switch (state)
  {
    case State402::Not_Ready_To_Switch_On:
    case State402::Switch_On_Disabled:
    case State402::Ready_To_Switch_On:
    case State402::Switched_On:
      std::cout << "Motor operation is not enabled" << std::endl;
    case State402::Operation_Enable:
      break;

    case State402::Quick_Stop_Active:
      std::cout << "Quick stop is active" << std::endl;
      break;
    case State402::Fault:
    case State402::Fault_Reaction_Active:
      std::cout << "Motor has fault" << std::endl;
      break;
    case State402::Unknown:
      std::cout << "State is unknown" << std::endl;
      break;
  }

  if (sw & (1 << State402::SW_Warning))
  {
    std::cout << "Warning bit is set" << std::endl;
  }
  if (sw & (1 << State402::SW_Internal_limit))
  {
    std::cout << "Internal limit active" << std::endl;
  }
}
bool Motor402::handleInit()
{
  for (std::unordered_map<uint16_t, AllocFuncType>::iterator it = mode_allocators_.begin();
       it != mode_allocators_.end(); ++it)
  {
    (it->second)();
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Read State");
  if (!readState())
  {
    std::cout << "Could not read motor state" << std::endl;
    return false;
  }
  {
    std::scoped_lock lock(cw_mutex_);
    control_word_ = 0;
    start_fault_reset_ = true;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Enable");
  if (!switchState(State402::Operation_Enable))
  {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }

  ModeSharedPtr m = allocMode(MotorBase::Homing);
  if (!m)
  {
    std::cout << "Homeing mode not supported" << std::endl;
    return true;  // homing not supported
  }

  HomingMode * homing = dynamic_cast<HomingMode *>(m.get());

  if (!homing)
  {
    std::cout << "Homing mode has incorrect handler" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Switch to homing");
  if (!switchMode(MotorBase::Homing))
  {
    std::cout << "Could not enter homing mode" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Execute homing");
  if (!homing->executeHoming())
  {
    std::cout << "Homing failed" << std::endl;
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Init: Switch no mode");
  if (!switchMode(MotorBase::No_Mode))
  {
    std::cout << "Could not enter no mode" << std::endl;
    return false;
  }
  return true;
}
bool Motor402::handleShutdown()
{
  switchMode(MotorBase::No_Mode);
  return switchState(State402::Switch_On_Disabled);
}
bool Motor402::handleHalt()
{
  State402::InternalState state = state_handler_.getState();
  std::scoped_lock lock(cw_mutex_);

  // do not demand quickstop in case of fault
  if (state == State402::Fault_Reaction_Active || state == State402::Fault) return false;

  if (state != State402::Operation_Enable)
  {
    target_state_ = state;
  }
  else
  {
    target_state_ = State402::Quick_Stop_Active;
    if (!Command402::setTransition(control_word_, state, State402::Quick_Stop_Active, 0))
    {
      std::cout << "Could not quick stop" << std::endl;
      return false;
    }
  }
  return true;
}
bool Motor402::handleRecover()
{
  start_fault_reset_ = true;
  {
    std::scoped_lock lock(mode_mutex_);
    if (selected_mode_ && !selected_mode_->start())
    {
      std::cout << "Could not restart mode." << std::endl;
      return false;
    }
  }
  if (!switchState(State402::Operation_Enable))
  {
    std::cout << "Could not enable motor" << std::endl;
    return false;
  }
  return true;
}
