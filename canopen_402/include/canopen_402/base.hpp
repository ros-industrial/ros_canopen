#ifndef CANOPEN_402_BASE_H
#define CANOPEN_402_BASE_H

#include <canopen_master/canopen.hpp>

namespace canopen
{

class State402
{
public:
  enum StatusWord
  {
    SW_Ready_To_Switch_On=0,
    SW_Switched_On=1,
    SW_Operation_enabled=2,
    SW_Fault=3,
    SW_Voltage_enabled=4,
    SW_Quick_stop=5,
    SW_Switch_on_disabled=6,
    SW_Warning=7,
    SW_Manufacturer_specific0=8,
    SW_Remote=9,
    SW_Target_reached=10,
    SW_Internal_limit=11,
    SW_Operation_mode_specific0=12,
    SW_Operation_mode_specific1=13,
    SW_Manufacturer_specific1=14,
    SW_Manufacturer_specific2=15
  };
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
  InternalState getState();
  InternalState read(uint16_t sw);
  bool waitForNewState(const time_point &abstime, InternalState &state);
  State402() : state_(Unknown) {}
private:
  boost::condition_variable cond_;
  boost::mutex mutex_;
  InternalState state_;
};

class MotorBase
: public canopen::Layer
{
protected:
  MotorBase(const std::string & name)
  : Layer(name)
  {}
public:
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

  virtual bool setTarget(double val) = 0;
  virtual bool enterModeAndWait(uint16_t mode) = 0;
  virtual bool isModeSupported(uint16_t mode) = 0;
  virtual uint16_t getMode() = 0;
  virtual void registerDefaultModes(ObjectStorageSharedPtr storage) {}

  // NOTE(sam): Keep this?
  virtual bool switchState(const State402::InternalState & target) {}

  typedef std::shared_ptr<MotorBase> MotorBaseSharedPtr;

  class Allocator
  {
public:
    virtual MotorBaseSharedPtr allocate(
      const std::string &name,
      ObjectStorageSharedPtr storage,
      const canopen::Settings &settings) = 0;
    virtual ~Allocator() {}
  };
};
using MotorBaseSharedPtr = MotorBase::MotorBaseSharedPtr;

}

#endif
