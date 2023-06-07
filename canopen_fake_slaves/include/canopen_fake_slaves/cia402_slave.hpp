//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#ifndef CIA402_SLAVE_HPP
#define CIA402_SLAVE_HPP
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <atomic>
#include <mutex>
#include <thread>

#include "canopen_fake_slaves/base_slave.hpp"
#include "canopen_fake_slaves/motion_generator.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace lely;
using namespace std::chrono_literals;

namespace ros2_canopen
{

class CIA402MockSlave : public canopen::BasicSlave
{
public:
  explicit CIA402MockSlave(
    io::TimerBase & timer, io::CanChannelBase & chan, const ::std::string & dcf_txt,
    const ::std::string & dcf_bin = "", uint8_t id = 0xff)
  : canopen::BasicSlave(timer, chan, dcf_txt, dcf_bin, id)
  {
    state.store(InternalState::Not_Ready_To_Switch_On);
    status_word = 0x0;
    operation_mode.store(No_Mode);
    control_cycle_period = 0.01;
    actual_position = 0.0;
  }

  virtual ~CIA402MockSlave()
  {
    if (profiled_position_mode.joinable())
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Joined profiled_position_mode thread.");
      profiled_position_mode.join();
    }
    if (cyclic_position_mode.joinable())
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Joined cyclic_position_mode thread.");
      cyclic_position_mode.join();
    }
    if (interpolated_position_mode.joinable())
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Joined interpolated_position_mode thread.");
      interpolated_position_mode.join();
    }
  }

protected:
  enum InternalState
  {
    Unknown = 0,
    Start = 0,
    Not_Ready_To_Switch_On = 1,
    Switch_On_Disabled = 2,
    Ready_To_Switch_On = 3,
    Switched_On = 4,
    Operation_Enable = 5,
    Quick_Stop_Active = 6,
    Fault_Reaction_Active = 7,
    Fault = 8,
  };

  enum StatusWord
  {
    SW_Ready_To_Switch_On = 0,
    SW_Switched_On = 1,
    SW_Operation_enabled = 2,
    SW_Fault = 3,
    SW_Voltage_enabled = 4,
    SW_Quick_stop = 5,
    SW_Switch_on_disabled = 6,
    SW_Warning = 7,
    SW_Manufacturer_specific0 = 8,
    SW_Remote = 9,
    SW_Target_reached = 10,
    SW_Internal_limit = 11,
    SW_Operation_mode_specific0 = 12,
    SW_Operation_mode_specific1 = 13,
    SW_Manufacturer_specific1 = 14,
    SW_Manufacturer_specific2 = 15
  };
  enum ControlWord
  {
    CW_Switch_On = 0,
    CW_Enable_Voltage = 1,
    CW_Quick_Stop = 2,
    CW_Enable_Operation = 3,
    CW_Operation_mode_specific0 = 4,
    CW_Operation_mode_specific1 = 5,
    CW_Operation_mode_specific2 = 6,
    CW_Fault_Reset = 7,
    CW_Halt = 8,
    CW_Operation_mode_specific3 = 9,
    // CW_Reserved1=10,
    CW_Manufacturer_specific0 = 11,
    CW_Manufacturer_specific1 = 12,
    CW_Manufacturer_specific2 = 13,
    CW_Manufacturer_specific3 = 14,
    CW_Manufacturer_specific4 = 15,
  };

  enum OperationMode
  {
    No_Mode = 0,
    Profiled_Position = 1,
    Velocity = 2,
    Profiled_Velocity = 3,
    Profiled_Torque = 4,
    Reserved = 5,
    Homing = 6,
    Interpolated_Position = 7,
    Cyclic_Synchronous_Position = 8,
    Cyclic_Synchronous_Velocity = 9,
    Cyclic_Synchronous_Torque = 10,
  };
  std::atomic<bool> is_relative;
  std::atomic<bool> is_running;
  std::atomic<bool> is_halt;
  std::atomic<bool> is_new_set_point;
  std::atomic<int8_t> operation_mode;
  std::atomic<int8_t> old_operation_mode;

  std::mutex w_mutex;
  uint16_t status_word;
  uint16_t control_word;
  std::atomic<InternalState> state;

  std::thread profiled_position_mode;
  std::thread profiled_velocity_mode;
  std::thread cyclic_position_mode;
  std::thread cyclic_velocity_mode;
  std::thread interpolated_position_mode;

  double cycle_time;

  std::mutex in_mode_mutex;
  double actual_position;
  double actual_speed;
  double acceleration;
  double control_cycle_period;

  void run_profiled_position_mode()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "run_profiled_position_mode");
    double profile_speed = static_cast<double>(((uint32_t)(*this)[0x6081][0])) / 1000;
    double profile_accerlation = static_cast<double>(((uint32_t)(*this)[0x6083][0])) / 1000;
    double actual_position = static_cast<double>(((int32_t)(*this)[0x6064][0])) / 1000.0;
    double target_position = static_cast<double>(((int32_t)(*this)[0x607A][0])) / 1000.0;
    double actual_speed = static_cast<double>(((int32_t)(*this)[0x606C][0])) / 1000.0;
    RCLCPP_INFO(
      rclcpp::get_logger("cia402_slave"), "Profile_Speed %f, Profile Acceleration: %f",
      profile_speed, profile_accerlation);

    while ((state.load() == InternalState::Operation_Enable) &&
           (operation_mode.load() == Profiled_Position) && (rclcpp::ok()))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      target_position = static_cast<double>(((int32_t)(*this)[0x607A][0])) / 1000.0;
      if (target_position != actual_position)
      {
        clear_status_bit(SW_Operation_mode_specific0);
        clear_status_bit(SW_Target_reached);
        {
          std::scoped_lock<std::mutex> lock(w_mutex);
          (*this)[0x6041][0] = status_word;
          this->TpdoEvent(1);
        }
        is_new_set_point.store(false);
        RCLCPP_INFO(
          rclcpp::get_logger("cia402_slave"), "Move from %f to %f", actual_position,
          target_position);
        {
          MotionGenerator gen(profile_accerlation, profile_speed, actual_position);

          while (!gen.getFinished())
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            actual_position = gen.update(target_position);
            actual_speed = gen.getVelocity();
            (*this)[0x6064][0] = (int32_t)(actual_position * 1000);
            (*this)[0x606C][0] = (int32_t)(actual_speed * 1000);
          }
        }
        RCLCPP_INFO(
          rclcpp::get_logger("cia402_slave"), "Reached target position %f", actual_position);
        clear_status_bit(SW_Operation_mode_specific0);
        set_status_bit(SW_Target_reached);
        {
          std::scoped_lock<std::mutex> lock(w_mutex);
          (*this)[0x6041][0] = status_word;
          this->TpdoEvent(1);
        }
      }
    }
  }
  void run_cyclic_position_mode()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "run_cyclic_position_mode");
    int32_t min_pos = (int32_t)(*this)[0x607D][1];
    int32_t max_pos = (int32_t)(*this)[0x607D][2];
    uint8_t int_period = (*this)[0x60C2][1];
    int32_t offset = (*this)[0x60B0][0];
    int8_t index = (*this)[0x60C2][2];

    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Lower Software Limit: %d", min_pos);
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Upper Software Limit: %d", max_pos);
    RCLCPP_INFO(
      rclcpp::get_logger("cia402_slave"), "Control Cycle: %hhu + 10^%hhd", int_period, index);
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Offset: %d", offset);

    double cp_min_position = min_pos / 1000;
    double cp_max_position = max_pos / 1000;
    double cp_interpolation_period = int_period * std::pow(10.0, index);
    double cp_offset = (double)(offset / 1000.0);
    int ccp_millis = (int)(control_cycle_period * std::pow(10.0, 3));
    int32_t act_pos;
    while ((state.load() == InternalState::Operation_Enable) &&
           (operation_mode.load() == Cyclic_Synchronous_Position) && (rclcpp::ok()))
    {
      act_pos = (*this)[0x607A][0];
      double target_position = (act_pos) / 1000 - cp_offset;      // m
      double position_delta = target_position - actual_position;  // m
      double speed = position_delta / cp_interpolation_period;    // m/s
      double increment = control_cycle_period * speed;            // m
      (*this)[0x606C][0] = (int32_t)speed * 1000;
      if (
        (target_position < cp_max_position) && (target_position > cp_min_position) &&
        (std::abs(position_delta) > 0.001))
      {
        while ((std::abs(actual_position - target_position) > 0.001) && (rclcpp::ok()))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(ccp_millis));
          actual_position += increment;
          (*this)[0x6064][0] = (int32_t)actual_position * 1000;
          if (std::abs(actual_position - target_position) < 0.001)
          {
            RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Reached Target %f", target_position);
          }
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(ccp_millis));
    }
  }

  void run_interpolated_position_mode()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "run_interpolated_position_mode");
    // Retrieve parameters from the object dictionary
    double interpolation_period = static_cast<double>((uint8_t)(*this)[0x60C2][1]);
    double target_position = static_cast<double>((int32_t)(*this)[0x60C1][1]);

    // int32_t offset = (*this)[0x60B0][0];

    // Convert parameters to SI units
    interpolation_period *= std::pow(10.0, static_cast<double>((int8_t)(*this)[0x60C2][2]));
    target_position /= 1000.0;
    double actual_position = static_cast<double>((int32_t)(*this)[0x6064][0]) / 1000.0;

    RCLCPP_INFO(
      rclcpp::get_logger("cia402_slave"), "Interpolation Period: %f", interpolation_period);
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Target position: %f", target_position);

    while ((state.load() == InternalState::Operation_Enable) &&
           (operation_mode.load() == Interpolated_Position) && (rclcpp::ok()))
    {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(interpolation_period * 1000)));

      target_position = static_cast<double>((int32_t)(*this)[0x60C1][1]) / 1000.0;

      if (target_position != actual_position)
      {
        double position_delta = target_position - actual_position;
        double position_increment = position_delta / 20;
        clear_status_bit(SW_Operation_mode_specific0);
        clear_status_bit(SW_Target_reached);
        {
          std::scoped_lock<std::mutex> lock(w_mutex);
          (*this)[0x6041][0] = status_word;
          this->TpdoEvent(1);
        }

        while ((std::abs(actual_position - target_position) > 0.001) && (rclcpp::ok()))
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          actual_position += position_increment;
          (*this)[0x6064][0] = static_cast<int32_t>(actual_position * 1000);
          (*this)[0x606C][0] = static_cast<int32_t>(position_increment * 1000);
        }

        actual_position = target_position;

        RCLCPP_DEBUG(rclcpp::get_logger("cia402_slave"), "Reached target: %f", actual_position);
        clear_status_bit(SW_Operation_mode_specific0);
        set_status_bit(SW_Target_reached);
        {
          std::lock_guard<std::mutex> lock(w_mutex);
          (*this)[0x6041][0] = status_word;
          this->TpdoEvent(1);
        }
      }
    }
  }

  void run_position_mode() {}

  void set_new_status_word_and_state()
  {
    switch (state.load())
    {
      case InternalState::Not_Ready_To_Switch_On:
        on_not_ready_to_switch_on();
        break;
      case InternalState::Switch_On_Disabled:
        on_switch_on_disabled();
        break;
      case InternalState::Ready_To_Switch_On:
        on_ready_to_switch_on();
        break;
      case InternalState::Switched_On:
        on_switched_on();
        break;
      case InternalState::Operation_Enable:
        on_operation_enabled();
        break;
      case InternalState::Quick_Stop_Active:
        on_quickstop_active();
        break;
      case InternalState::Fault_Reaction_Active:
        break;
      case InternalState::Fault:
        RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Fault");
        break;
      default:
        break;
    }
  }

  void set_status_bit(int bit)
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    status_word |= 1UL << bit;
  }

  void clear_status_bit(int bit)
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    status_word &= ~(1UL << bit);
  }

  void set_switch_on_disabled()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Switch_On_Disabled");
    state.store(InternalState::Switch_On_Disabled);
    clear_status_bit(SW_Ready_To_Switch_On);
    clear_status_bit(SW_Switched_On);
    clear_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Switch_on_disabled);
  }

  void set_ready_to_switch_on()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Ready_To_Switch_On");
    state.store(InternalState::Ready_To_Switch_On);
    set_status_bit(SW_Ready_To_Switch_On);
    clear_status_bit(SW_Switched_On);
    clear_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
  }

  void set_switch_on()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Switched_On");
    state.store(InternalState::Switched_On);
    set_status_bit(SW_Ready_To_Switch_On);
    set_status_bit(SW_Switched_On);
    clear_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
  }

  void set_operation_enabled()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Operation_Enable");
    state.store(InternalState::Operation_Enable);
    set_status_bit(SW_Ready_To_Switch_On);
    set_status_bit(SW_Switched_On);
    set_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    set_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
  }

  void set_quick_stop()
  {
    RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Quick_Stop_Active");
    state.store(InternalState::Quick_Stop_Active);
    set_status_bit(SW_Ready_To_Switch_On);
    set_status_bit(SW_Switched_On);
    set_status_bit(SW_Operation_enabled);
    clear_status_bit(SW_Fault);
    clear_status_bit(SW_Quick_stop);
    clear_status_bit(SW_Switch_on_disabled);
  }

  void on_not_ready_to_switch_on() { set_switch_on_disabled(); }

  void on_switch_on_disabled()
  {
    if (is_shutdown())
    {
      set_ready_to_switch_on();
    }
  }

  void on_ready_to_switch_on()
  {
    if (is_disable_voltage())
    {
      set_switch_on_disabled();
    }
    if (is_switch_on())
    {
      set_switch_on();
    }
    if (is_faul_reset())
    {
      set_ready_to_switch_on();
    }
  }

  void on_switched_on()
  {
    if (is_disable_voltage())
    {
      set_switch_on_disabled();
    }
    if (is_shutdown())
    {
      set_ready_to_switch_on();
    }
    if (is_enable_operation())
    {
      set_operation_enabled();
    }
  }

  void on_operation_enabled()
  {
    if (is_disable_voltage())
    {
      set_switch_on_disabled();
    }
    if (is_shutdown())
    {
      set_ready_to_switch_on();
    }
    if (is_switch_on())
    {
      set_switch_on();
    }
    if (is_quickstop())
    {
      set_quick_stop();
    }
    {
      std::scoped_lock<std::mutex> lock(w_mutex);
      is_relative.store(((control_word >> 6) & 1U) == 1U);
      is_halt.store(((control_word >> 8) & 1U) == 1U);
      is_new_set_point.store(((control_word >> 4) & 1U) == 1U);
    }

    if (old_operation_mode.load() != operation_mode.load())
    {
      if (profiled_position_mode.joinable())
      {
        RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Joined profiled_position_mode thread.");
        profiled_position_mode.join();
      }
      if (cyclic_position_mode.joinable())
      {
        RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Joined cyclic_position_mode thread.");
        cyclic_position_mode.join();
      }
      if (interpolated_position_mode.joinable())
      {
        RCLCPP_INFO(
          rclcpp::get_logger("cia402_slave"), "Joined interpolated_position_mode thread.");
        interpolated_position_mode.join();
      }
      old_operation_mode.store(operation_mode.load());
      switch (operation_mode.load())
      {
        case Cyclic_Synchronous_Position:
          start_sync_pos_mode();
          break;
        case Profiled_Position:
          start_profile_pos_mode();
          break;
        case Interpolated_Position:
          start_interpolated_pos_mode();
          break;
        default:
          break;
      }
    }
  }

  void start_sync_pos_mode()
  {
    cyclic_position_mode = std::thread(std::bind(&CIA402MockSlave::run_cyclic_position_mode, this));
  }

  void start_profile_pos_mode()
  {
    profiled_position_mode =
      std::thread(std::bind(&CIA402MockSlave::run_profiled_position_mode, this));
  }

  void start_interpolated_pos_mode()
  {
    interpolated_position_mode =
      std::thread(std::bind(&CIA402MockSlave::run_interpolated_position_mode, this));
  }

  void on_quickstop_active()
  {
    if (is_enable_operation())
    {
      set_operation_enabled();
    }
    if (is_disable_voltage())
    {
      set_switch_on_disabled();
    }
  }

  bool is_shutdown()
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool qs_set = ((control_word >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_unset = ((control_word >> CW_Switch_On) & 1U) == 0U;

    if (fr_unset && qs_set && ev_set && so_unset)
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Received Shutdown.");
      return true;
    }
    return false;
  }

  bool is_disable_voltage()
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool ev_unset = ((control_word >> CW_Enable_Voltage) & 1U) == 0U;

    if (fr_unset && ev_unset)
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Received Disable Voltage.");
      return true;
    }
    return false;
  }

  bool is_switch_on()
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool eo_unset = ((control_word >> CW_Enable_Operation) & 1U) == 0U;
    bool qs_set = ((control_word >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_set = ((control_word >> CW_Switch_On) & 1U) == 1U;
    if (fr_unset && eo_unset && qs_set && ev_set && so_set)
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Received Switch On.");
      return true;
    }
    return false;
  }

  bool is_enable_operation()
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool eo_set = ((control_word >> CW_Enable_Operation) & 1U) == 1U;
    bool qs_set = ((control_word >> CW_Quick_Stop) & 1U) == 1U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    bool so_set = ((control_word >> CW_Switch_On) & 1U) == 1U;
    if (fr_unset && eo_set && qs_set && ev_set && so_set)
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Received Enable Operation.");
      return true;
    }
    return false;
  }

  bool is_quickstop()
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_unset = ((control_word >> CW_Fault_Reset) & 1U) == 0U;
    bool qs_unset = ((control_word >> CW_Quick_Stop) & 1U) == 0U;
    bool ev_set = ((control_word >> CW_Enable_Voltage) & 1U) == 1U;
    if (fr_unset && qs_unset && ev_set)
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Received Quick Stop.");
      return true;
    }
    return false;
  }

  bool is_faul_reset()
  {
    std::scoped_lock<std::mutex> lock(w_mutex);
    bool fr_set = ((control_word >> CW_Fault_Reset) & 1U) == 1U;
    if (fr_set)
    {
      RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Received Fault Reset.");
      return true;
    }
    return false;
  }
  // This function gets called every time a value is written to the local object
  // dictionary by an SDO or RPDO.
  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override
  {
    // System State
    if (idx == 0x6040 && subidx == 0)
    {
      {
        std::scoped_lock<std::mutex> lock(w_mutex);
        control_word = (*this)[0x6040][0];
      }
      set_new_status_word_and_state();
      {
        std::scoped_lock<std::mutex> lock(w_mutex);
        (*this)[0x6041][0] = status_word;
        this->TpdoEvent(1);
      }
    }
    // Operation Mode
    if (idx == 0x6060 && subidx == 0)
    {
      int8_t mode = (*this)[0x6060][0];
      switch (mode)
      {
        case No_Mode:
        case Profiled_Position:
        case Velocity:
        case Profiled_Velocity:
        case Profiled_Torque:
        case Reserved:
        case Homing:
        case Interpolated_Position:
        case Cyclic_Synchronous_Position:
        case Cyclic_Synchronous_Velocity:
        case Cyclic_Synchronous_Torque:
          operation_mode.store(mode);
          break;
        default:
          std::cout << "Error: Master tried to set unknown operation mode." << std::endl;
      }
      // RCLCPP_INFO(rclcpp::get_logger("cia402_slave"), "Switched to mode %hhi.", mode);
      (*this)[0x6061][0] = (int8_t)(mode);
      this->TpdoEvent(1);
    }
  }
};

class CIA402Slave : public BaseSlave
{
public:
  explicit CIA402Slave(const std::string & node_name, bool intra_process_comms = false)
  : BaseSlave(node_name, intra_process_comms)
  {
  }

  void run() override
  {
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl(can_interface_name_.c_str());
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    auto sigset_ = lely::io::SignalSet(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset_.insert(SIGHUP);
    sigset_.insert(SIGINT);
    sigset_.insert(SIGTERM);

    sigset_.submit_wait(
      [&](int /*signo*/)
      {
        // If the signal is raised again, terminate immediately.
        sigset_.clear();

        // Perform a clean shutdown.
        ctx.shutdown();
      });

    ros2_canopen::CIA402MockSlave slave(timer, chan, slave_config_.c_str(), "", node_id_);
    slave.Reset();

    RCLCPP_INFO(this->get_logger(), "Created cia402 slave for node_id %i.", node_id_);

    loop.run();
    ctx.shutdown();
    RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
    rclcpp::shutdown();
  }
};
}  // namespace ros2_canopen
#endif
